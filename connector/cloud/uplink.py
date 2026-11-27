"""Enrolled host service. All control, timers and socket ownership stay on asyncio."""
import argparse
import asyncio
import getpass
import json
import os
import random
import re
import stat
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import aiohttp
from yarl import URL

from listener import Snapshot, listen
from native import Controller

# Cloud recordings (connector/README.md "Cloud recordings"): the relay asks for the segment list or for one
# clip; this host gets both from the local connector over the pinned LAN link and uploads a clip only to the
# signed Cloud Storage URL the relay issued. Footage never goes anywhere else.
CLIP_UPLOAD_SCHEME, CLIP_UPLOAD_HOST = 'https', 'storage.googleapis.com'
SEGMENT = re.compile(r'dashcam_\d{6}_\d{8}_\d{6}')
MAX_LISTED = 1500   # about 210 KB of JSON: under the relay's 256 KB uplink message limit
CLIP_ERRORS = {400: 'The clip request was not valid', 404: 'That segment is no longer on the Jetson',
               409: 'That segment is still recording', 416: 'The clip starts after the end of that segment',
               422: 'The Jetson could not make that clip',
               503: 'The Jetson is busy exporting another clip', 504: 'The clip took too long to make'}


def private_read(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd) as stream:
        st = os.fstat(fd)
        if not stat.S_ISREG(st.st_mode) or st.st_uid != os.getuid() or st.st_mode & 0o077:
            raise ValueError('Credential must be owner-only')
        return json.load(stream)


def private_write(path, value):
    with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as stream:
        json.dump(value, stream)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(stream.name, path)


def origin(value):
    url = URL(value)
    if url.scheme != 'https' or not url.host or url.user or url.query or url.fragment or url.path != '/':
        raise ValueError('Expected HTTPS relay origin')
    return str(url.origin())


async def enroll(http, path, relay):
    relay = origin(relay)
    pairing = urlsplit(getpass.getpass('Local dashcam pairing link: '))
    if (pairing.scheme, pairing.netloc) != ('dashcam', 'pair'):
        raise ValueError('Expected local pairing link')
    args = parse_qs(pairing.query, strict_parsing=True)
    local = {k: args[k][0] for k in ('host', 'port', 'pin', 'token', 'id')}
    aiohttp.Fingerprint(bytes.fromhex(local['pin']))
    code = getpass.getpass('One-time enrollment code: ')
    async with http.post(relay + '/v1/enroll', json={'code': code}, allow_redirects=False) as r:
        r.raise_for_status()
        data = await r.json()
    config = data['config']
    url = 'https://identitytoolkit.googleapis.com/v1/accounts:signInWithCustomToken'
    async with http.post(url, params={'key': config['apiKey']},
                         json={'token': data['token'], 'returnSecureToken': True}) as r:
        r.raise_for_status()
        credential = await r.json()
    private_write(path, dict(vehicle=data['vehicle'], relay=relay, local=local, project=config['projectId'],
                             apiKey=config['apiKey'], refresh=credential['refreshToken']))


async def refresh(http, path, cfg):
    async with http.post('https://securetoken.googleapis.com/v1/token', params={'key': cfg['apiKey']},
                         data={'grant_type': 'refresh_token', 'refresh_token': cfg['refresh']}) as r:
        r.raise_for_status()
        result = await r.json()
    if cfg['refresh'] != result['refresh_token']:
        cfg['refresh'] = result['refresh_token']
        private_write(path, cfg)
    return result['id_token'], int(result['expires_in'])


class Resources:
    def __init__(self, http, cfg, token):
        self.http, self.cfg, self.token = http, cfg, token
        self.core = Controller(cfg['vehicle'], self.reconcile)
        self.attempt = 0
        self.upload_bytes = {1: 0, 2: 0}
        self.last_report = None
        self.socket = None
        self.connection = None
        self.readers = {}
        self.tasks = set()
        self.register = asyncio.Event()
        self.registration = None
        self.watcher = None
        self.mode = None
        self.version = -1
        self.failure = asyncio.get_running_loop().create_future()
        self.exporting = False

    def task(self, coro, wanted=None):
        """wanted() says whether the task's work is still current when it ends; None means always."""
        task = asyncio.create_task(coro)
        self.tasks.add(task)
        task.add_done_callback(lambda done: self.finished(done, wanted))
        return task

    def finished(self, task, wanted=None):
        self.tasks.discard(task)
        for bit, reader in list(self.readers.items()):
            if reader is task:
                del self.readers[bit]   # a later demand for this channel starts a fresh reader
        if task.cancelled():
            return
        error = task.exception()
        # A task whose attempt or channel was released ended because the session did: the
        # normal end of an on-demand view. Only work that is still wanted restarts the uplink
        # (with backoff); otherwise the watcher keeps running and sees the next request at once.
        if wanted is not None and not wanted():
            return
        if not self.failure.done():
            self.failure.set_result(error or ConnectionError('Stream ended'))

    def current(self, attempt, bit=0):
        state = self.core.state()
        return state.attempt == attempt and self.attempt == attempt and (not bit or bool(state.active & bit))

    def reconcile(self, state):
        if state.attempt != self.attempt:
            if self.connection:
                self.connection.cancel()
            self.socket = None
            self.attempt = state.attempt
            self.registration = None
            attempt = state.attempt
            self.connection = self.task(self.connect(attempt), lambda: self.current(attempt)) if attempt else None
        report = (state.attempt, state.active)
        if report != self.last_report:
            self.last_report = report
            print(json.dumps(dict(event='uplink_target', attempt=state.attempt,
                                  telemetryReaders=int(bool(state.active & 1)), videoReaders=int(bool(state.active & 2)),
                                  telemetryBytes=self.upload_bytes[1], videoBytes=self.upload_bytes[2])), flush=True)
        registration = (state.generation, state.requested)
        if self.registration != registration:
            self.registration = registration
            self.register.set()
        for bit, route in ((1, 'stream'), (2, 'video')):
            if state.active & bit and bit not in self.readers:
                self.readers[bit] = self.task(self.read(bit, route, state.attempt),
                                              lambda b=bit, a=state.attempt: self.current(a, b))
            elif not state.active & bit and bit in self.readers:
                self.readers.pop(bit).cancel()

    async def read(self, bit, route, attempt):
        local = self.cfg['local']
        # Always the Low preview (no quality=hd): original HD frames (16-56 Mbit/s) stay on the LAN;
        # mobile uplink and relay egress cannot carry them (connector/README.md, "HD").
        url = URL.build(scheme='wss', host=local['host'], port=int(local['port']), path='/v1/' + route)
        async with self.http.ws_connect(url, ssl=aiohttp.Fingerprint(bytes.fromhex(local['pin'])),
                                        headers={'Authorization': 'Bearer ' + local['token']},
                                        max_msg_size=262168) as ws:
            async for msg in ws:
                state = self.core.state()
                if state.attempt != attempt or not state.active & bit:
                    return
                if bit == 1 and msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    if data.get('device_id') != local['id'] or data.get('type') != 'snapshot':
                        raise ValueError('Local connector identity mismatch')
                    await asyncio.wait_for(self.socket.send_str(msg.data), 2)
                    self.upload_bytes[bit] += len(msg.data.encode())
                elif bit == 2 and msg.type == aiohttp.WSMsgType.BINARY:
                    await asyncio.wait_for(self.socket.send_bytes(msg.data), 2)
                    self.upload_bytes[bit] += len(msg.data)
                else:
                    break

    def local_url(self, path):
        local = self.cfg['local']
        return URL.build(scheme='https', host=local['host'], port=int(local['port']), path=path)

    def local_request(self):
        local = self.cfg['local']
        return dict(ssl=aiohttp.Fingerprint(bytes.fromhex(local['pin'])),
                    headers={'Authorization': 'Bearer ' + local['token']})

    async def list_recordings(self, ws, request):
        reply = dict(type='recordings', id=request.get('id'))
        try:
            async with self.http.get(self.local_url('/v1/recordings'), **self.local_request()) as r:
                if r.status == 404:
                    reply['error'] = "This Jetson's connector does not offer recordings"
                else:
                    r.raise_for_status()
                    reply['segments'] = (await r.json())['segments'][:MAX_LISTED]
        except Exception as exc:   # noqa: BLE001 - reported to the viewer, never fatal here
            reply['error'] = 'Could not read the recordings on the Jetson (' + type(exc).__name__ + ')'
        await asyncio.wait_for(ws.send_json(reply), 10)

    async def export_clip(self, ws, request):
        reply = dict(type='clip_result', id=request.get('id'))
        try:
            upload = URL(request['upload_url'])
            segment = request['segment']
            if upload.scheme != CLIP_UPLOAD_SCHEME or upload.host != CLIP_UPLOAD_HOST or not SEGMENT.fullmatch(segment):
                raise ValueError('refused')
            body = {k: request[k] for k in ('start_s', 'duration_s', 'quality', 'captions')}
        except (KeyError, TypeError, ValueError):
            reply['error'] = 'The clip request was not valid'
            await asyncio.wait_for(ws.send_json(reply), 10)
            return
        if self.exporting:
            reply['error'] = CLIP_ERRORS[503]
            await asyncio.wait_for(ws.send_json(reply), 10)
            return
        self.exporting = True
        try:
            async with self.http.post(self.local_url(f'/v1/recordings/{segment}.mkv/clip'), json=body,
                                      timeout=aiohttp.ClientTimeout(total=700), **self.local_request()) as r:
                if r.status != 200:
                    reply['error'] = CLIP_ERRORS.get(r.status, f'The Jetson answered HTTP {r.status}')
                else:
                    clip = await r.read()   # at most a few tens of MB: one 3-minute 720p clip
                    reply['captions'] = r.headers.get('X-Clip-Captions') == 'burned'
            if 'error' not in reply:
                async with self.http.put(upload, data=clip, headers={'Content-Type': 'video/mp4'},
                                         timeout=aiohttp.ClientTimeout(total=900)) as up:
                    if up.status not in (200, 201):
                        raise ConnectionError(f'upload HTTP {up.status}')
                reply.update(ok=True, bytes=len(clip))
                print(json.dumps(dict(event='clip_uploaded', id=reply['id'], bytes=len(clip))), flush=True)
        except Exception as exc:   # noqa: BLE001 - reported to the viewer, never fatal here
            reply['error'] = 'The clip could not be made or uploaded (' + type(exc).__name__ + ')'
        finally:
            self.exporting = False
        await asyncio.wait_for(ws.send_json(reply), 10)

    async def register_loop(self, ws, attempt):
        while self.attempt == attempt:
            try:
                await asyncio.wait_for(self.register.wait(), 60)
            except asyncio.TimeoutError:
                await ws.send_json(dict(type='heartbeat'))
                continue
            self.register.clear()
            state = self.core.state()
            if state.attempt != attempt:
                return
            await ws.send_json(dict(type='register', attempt=attempt, generation=state.generation))

    async def connect(self, attempt):
        url = URL(origin(self.cfg['relay'])).with_scheme('wss').with_path('/v1/uplink')
        async with self.http.ws_connect(url.with_query(vehicle=self.cfg['vehicle']),
                                        headers={'Authorization': 'Bearer ' + self.token},
                                        max_msg_size=16384) as ws:   # export requests carry a signed URL
            if self.core.state().attempt != attempt:
                return
            self.socket = ws
            sender = self.task(self.register_loop(ws, attempt), lambda: self.current(attempt))
            try:
                while True:
                    msg = await ws.receive(timeout=150)
                    if msg.type != aiohttp.WSMsgType.TEXT:
                        break
                    grant = json.loads(msg.data)
                    if self.core.state().attempt != attempt:
                        return
                    if grant['type'] == 'control' and self.watcher is None:
                        fields = grant['fields']
                        if fields['desiredMode'] != 'ready_while_powered':
                            # Hand back to a fresh rules-enforced snapshot before releasing the socket.
                            self.watcher = self.task(self.watch())
                            continue
                        if fields['expiresAt'] is not None:
                            fields['expiresAt'] = datetime.fromtimestamp(fields['expiresAt'], timezone.utc)
                        self.core.snapshot(Snapshot(grant['path'], fields, True))
                    elif grant['type'] == 'list_recordings':
                        # Never ends the session: a failure is reported to the relay instead.
                        self.task(self.list_recordings(ws, grant), lambda: False)
                    elif grant['type'] == 'export_clip':
                        self.task(self.export_clip(ws, grant), lambda: False)
                    elif grant['type'] == 'grant' and grant['attempt'] == attempt:
                        self.core.confirm(attempt, grant['generation'], grant['channels'])
                        if (grant['mode'] == self.mode == 'ready_while_powered' and self.watcher
                                and self.version == grant['version'] and self.core.state().generation == grant['generation']):
                            self.watcher.cancel()
                            self.watcher = None
            finally:
                sender.cancel()

    async def watch(self):
        async for event in listen(self.cfg['project'], self.cfg['vehicle'], self.token):
            old_version = self.version
            self.core.snapshot(event)
            self.mode = event.fields['desiredMode']
            self.version = event.fields['configVersion']
            if old_version >= 0 and self.version != old_version:
                self.register.set()
            if event.fields.get('requestId'):
                print(json.dumps(dict(event='device_control', requestId=event.fields['requestId'],
                                      generation=event.fields['demandGeneration'])), flush=True)

    async def run(self, lifetime):
        self.watcher = self.task(self.watch())
        try:
            error = await asyncio.wait_for(self.failure, max(1, lifetime - 120))
            raise error
        finally:
            for task in self.tasks:
                task.cancel()
            await asyncio.gather(*self.tasks, return_exceptions=True)
            self.core.close()


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['enroll', 'run'])
    parser.add_argument('--state', required=True, type=Path)
    parser.add_argument('--relay')
    args = parser.parse_args()
    timeout = aiohttp.ClientTimeout(total=20, sock_connect=10)
    async with aiohttp.ClientSession(timeout=timeout, trust_env=False) as http:
        if args.command == 'enroll':
            await enroll(http, args.state, args.relay)
            return
        backoff = 1
        while True:
            started = time.monotonic()
            try:
                cfg = private_read(args.state)
                token, ttl = await refresh(http, args.state, cfg)
                await Resources(http, cfg, token).run(ttl)
            except Exception as exc:
                print('Uplink reconnect:', type(exc).__name__, flush=True)
            if time.monotonic() - started > 60:
                backoff = 1
            await asyncio.sleep(random.uniform(backoff / 2, backoff))
            backoff = min(60, backoff * 2)


if __name__ == '__main__':
    asyncio.run(main())
