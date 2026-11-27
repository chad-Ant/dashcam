"""Single-instance Cloud Run relay. Firestore owns authorization and demand."""
import asyncio
import json
import math
import os
import re
import secrets
import time
from dataclasses import dataclass, field
from datetime import timedelta
from pathlib import Path

import firebase_admin
from aiohttp import WSMsgType, web
from firebase_admin import auth
from google.cloud import firestore

from store import Conflict, Store, vehicle_id

WEB = Path(__file__).parent / 'web'
COOKIE = '__Host-dashcam'
MAX_PACKET = 262144 + 24
VEHICLE_FULL = 1013   # viewer close code: every lease slot is taken; retry later
# Cloud recordings (connector/README.md "Cloud recordings"): viewers ask for the segment list or a clip; the
# relay forwards to the vehicle's uplink and, for a clip, hands it a signed upload URL, then gives the viewer
# signed play and download links. Owners and viewers alike may use it.
REQUEST_ID = re.compile(r'[A-Za-z0-9_-]{1,64}')
SEGMENT = re.compile(r'dashcam_\d{6}_\d{8}_\d{6}')
CLIP_QUALITIES = ('small', '720p')
MAX_CLIP_SECONDS = 180
# How long the vehicle has to answer before the viewer is told it did not. A 3 min 720p clip took 127 s on the
# Jetson plus its upload; the list takes well under a second. Tests shorten these.
REQUEST_SECONDS = {'recordings': 60, 'clip': 10 * 60}
VEHICLE_GONE = 'The vehicle disconnected before it answered. Try again.'
SIGNING_FAILED = 'The relay could not sign the clip links. Try again in a moment.'


class ClipStorage:
    """Signed V4 URLs for clips in one bucket. The relay's service account signs through the IAM signBlob API
    (it holds Service Account Token Creator on itself), so no key file exists. A bucket lifecycle rule deletes
    clips after a day."""
    UPLOAD_SECONDS = 15 * 60
    LINK_SECONDS = 60 * 60

    def __init__(self, bucket):
        import google.auth
        from google.cloud import storage
        self.credentials, project = google.auth.default(scopes=['https://www.googleapis.com/auth/cloud-platform'])
        self.bucket = storage.Client(project=project, credentials=self.credentials).bucket(bucket)

    def _sign(self, name, **options):
        if not self.credentials.valid:   # the access token also authorises the IAM signBlob call
            import google.auth.transport.requests
            self.credentials.refresh(google.auth.transport.requests.Request())
        return self.bucket.blob(name).generate_signed_url(
            version='v4', service_account_email=self.credentials.service_account_email,
            access_token=self.credentials.token, **options)

    @staticmethod
    def object_name(vehicle, segment, start, quality):
        return f'clips/{vehicle}/{segment}_{int(start)}s_{quality}_{secrets.token_hex(6)}.mp4'

    def upload_url(self, name):
        return self._sign(name, method='PUT', content_type='video/mp4', expiration=timedelta(seconds=self.UPLOAD_SECONDS))

    def download_urls(self, name, filename):
        expiry = timedelta(seconds=self.LINK_SECONDS)
        return (self._sign(name, method='GET', expiration=expiry),
                self._sign(name, method='GET', expiration=expiry,
                           response_disposition=f'attachment; filename="{filename}"'))


def trace(event, **fields):
    print(json.dumps(dict(event=event, **fields)), flush=True)


@web.middleware
async def errors(request, handler):
    try:
        response = await handler(request)
    except PermissionError:
        raise web.HTTPForbidden() from None
    except (ValueError, KeyError, TypeError):
        raise web.HTTPBadRequest() from None
    except Conflict:
        raise web.HTTPConflict() from None
    response.headers['Cache-Control'] = 'no-store'
    response.headers['X-Content-Type-Options'] = 'nosniff'
    response.headers['Referrer-Policy'] = 'no-referrer'
    return response


def same_origin(request):
    if request.headers.get('Origin') != 'https://' + request.host:
        raise PermissionError


async def identity(request, device=False):
    bearer = request.headers.get('Authorization', '')
    if bearer.startswith('Bearer '):
        verifier, token = auth.verify_id_token, bearer[7:]
    else:
        if device:
            raise PermissionError
        if request.method != 'GET' or request.headers.get('Upgrade') or 'Origin' in request.headers:
            same_origin(request)
        verifier, token = auth.verify_session_cookie, request.cookies.get(COOKIE, '')
    try:
        claims = await asyncio.to_thread(verifier, token, check_revoked=True)
    except (ValueError, firebase_admin.exceptions.FirebaseError):
        raise PermissionError from None
    provider = claims.get('firebase', {}).get('sign_in_provider')
    if provider != ('custom' if device else 'google.com'):
        raise PermissionError
    return claims


@dataclass(eq=False)
class Viewer:
    ws: web.WebSocketResponse
    vehicle: str
    request_id: str
    channels: list = field(default_factory=list)
    deadline: float = 0
    pending: dict = field(default_factory=dict)
    wake: asyncio.Event = field(default_factory=asyncio.Event)
    first: bool = False
    sent: int = 0
    replies: list = field(default_factory=list)

    def offer(self, channel, payload):
        if channel in self.channels and time.monotonic() < self.deadline:
            self.pending[channel] = payload
            self.wake.set()

    def reply(self, value):
        """A recordings answer for this viewer only; the send loop is the socket's single writer."""
        self.replies.append(json.dumps(value))
        self.wake.set()

    async def send(self):
        try:
            while True:
                await self.wake.wait()
                self.wake.clear()
                while self.replies:
                    data = self.replies.pop(0)
                    await asyncio.wait_for(self.ws.send_str(data), 2)
                    self.sent += len(data)
                for channel in list(self.pending):
                    data = self.pending.pop(channel)
                    if channel not in self.channels or time.monotonic() >= self.deadline:
                        continue
                    send = self.ws.send_bytes(data) if channel == 'video' else self.ws.send_str(data)
                    await asyncio.wait_for(send, 2)
                    self.sent += len(data)
        except (TimeoutError, ConnectionError, asyncio.TimeoutError):
            await self.ws.close()


class Relay:
    def __init__(self, store, config, clips=None):
        self.store, self.config, self.clips = store, config, clips
        self.viewers = set()
        self.devices = {}
        self.releasing = set()
        # token sent to the device -> dict(viewer, id, kind, device, timer[, object, filename]). Every entry ends
        # exactly once, through finish(): the vehicle's answer, its socket closing or being replaced, the deadline,
        # or the viewer leaving (which ends it silently).
        self.requests = {}

    def finish(self, token, reply=None):
        """Ends a pending request: forgets it, stops its deadline and, if given, sends @p reply to its viewer."""
        request = self.requests.pop(token, None)
        if request is None:
            return None
        if request.get('timer'):
            request['timer'].cancel()
        if reply is not None and request['viewer'] in self.viewers:
            request['viewer'].reply(dict(reply, id=request['id']))
        return request

    def fail_device(self, ws, error=VEHICLE_GONE):
        """The vehicle socket @p ws is gone or replaced: its unanswered requests fail now, not at their deadline."""
        for token in [t for t, r in self.requests.items() if r['device'] is ws]:
            self.finish(token, dict(type='error', error=error))

    def expire(self, token):
        request = self.requests.get(token)
        if request is not None:
            trace('recordings_timeout', requestId=request['viewer'].request_id, kind=request['kind'])
            self.finish(token, dict(type='error', error='The vehicle did not answer in time. Try again.'))

    async def recording_request(self, viewer, key, body):
        """A viewer's 'recordings' or 'clip' request: checked here, carried out by the vehicle."""
        rid = body.get('id')
        if not isinstance(rid, str) or not REQUEST_ID.fullmatch(rid):
            raise ValueError('request id')

        def fail(error):
            viewer.reply(dict(type='error', id=rid, error=error))
        device = self.devices.get(key)
        if device is None or device.closed:
            return fail('The vehicle is not connected yet. Try again in a moment.')
        token = secrets.token_hex(12)
        if body['type'] == 'recordings':
            request = dict(kind='recordings')
            message = dict(type='list_recordings', id=token)
        else:
            if self.clips is None:
                return fail('Clip export is not set up on this relay')
            if any(r['viewer'] is viewer and r['kind'] == 'clip' for r in self.requests.values()):
                return fail('Wait for the clip being exported to finish')
            segment, quality, captions = body.get('segment'), body.get('quality'), body.get('captions', False)
            try:
                start, duration = float(body.get('start_s')), float(body.get('duration_s'))
            except (TypeError, ValueError):
                start = duration = math.nan
            if (not isinstance(segment, str) or not SEGMENT.fullmatch(segment) or quality not in CLIP_QUALITIES
                    or not isinstance(captions, bool) or not math.isfinite(start) or not math.isfinite(duration)
                    or not 0 <= start <= 3600 or not 1 <= duration <= MAX_CLIP_SECONDS):
                return fail('The clip request was not valid')
            name = self.clips.object_name(key, segment, start, quality)
            try:
                upload = await asyncio.to_thread(self.clips.upload_url, name)
            except Exception as e:   # IAM signBlob, token refresh, network: the request fails, the sockets stay
                trace('clip_signing_failed', requestId=viewer.request_id, stage='upload', error=type(e).__name__)
                return fail(SIGNING_FAILED)
            if self.devices.get(key) is not device or device.closed:   # it went away while we signed
                return fail('The vehicle is not connected yet. Try again in a moment.')
            request = dict(kind='clip', object=name, filename=f'{segment}_{int(start)}s_{int(duration)}s_{quality}.mp4')
            message = dict(type='export_clip', id=token, segment=segment, start_s=start, duration_s=duration,
                           quality=quality, captions=captions, upload_url=upload)
        request.update(viewer=viewer, id=rid, device=device,
                       timer=asyncio.get_running_loop().call_later(REQUEST_SECONDS[request['kind']], self.expire, token))
        self.requests[token] = request
        if request['kind'] == 'clip':
            viewer.reply(dict(type='clip_status', id=rid, status='exporting'))
        trace('recordings_request', requestId=viewer.request_id, vehicle=key, kind=body['type'])
        try:
            await asyncio.wait_for(device.send_json(message), 5)
        except (ConnectionError, asyncio.TimeoutError, RuntimeError):
            self.finish(token, dict(type='error', error='The vehicle is not connected yet. Try again in a moment.'))

    async def recording_response(self, ws, body):
        """The vehicle's answer, to the viewer that asked (if still connected). Only the socket the request
        went to may answer it: a late reply from a replaced socket is ignored."""
        token = body.get('id')
        request = self.requests.get(token) if isinstance(token, str) else None
        if request is None or request['device'] is not ws:
            return
        if body.get('error') or (request['kind'] == 'clip' and not body.get('ok')):
            return self.finish(token, dict(type='error', error=str(body.get('error') or 'The clip failed')[:300]))
        if request['kind'] == 'recordings':
            segments = body.get('segments')
            return self.finish(token, dict(type='recordings', segments=segments if isinstance(segments, list) else []))
        try:
            play, download = await asyncio.to_thread(self.clips.download_urls, request['object'], request['filename'])
        except Exception as e:   # never let a signing failure reach the vehicle's socket loop
            trace('clip_signing_failed', requestId=request['viewer'].request_id, stage='download', error=type(e).__name__)
            return self.finish(token, dict(type='error', error=SIGNING_FAILED))
        if self.finish(token, dict(type='clip', url=play, download=download, bytes=body.get('bytes'),
                                   captions=bool(body.get('captions')), expires_s=self.clips.LINK_SECONDS,
                                   filename=request['filename'])):
            trace('clip_ready', requestId=request['viewer'].request_id, bytes=body.get('bytes'))

    async def release(self, key, uid, session):
        doc = await self.store.update(key, uid, session=session)
        await self.push_control(key, doc)

    async def drain(self, app):
        if self.releasing:
            await asyncio.gather(*self.releasing, return_exceptions=True)

    async def push_control(self, key, doc):
        ws = self.devices.get(key)
        if ws is None or ws.closed:
            return
        fields = {k: doc[k] for k in ('desiredMode', 'configVersion', 'demandGeneration', 'channels')}
        fields['expiresAt'] = doc['expiresAt'].timestamp() if doc['expiresAt'] else None
        try:
            await asyncio.wait_for(ws.send_json(dict(type='control', path=f'vehicles/{key}/control/current',
                                                     fields=fields)), 2)
        except (ConnectionError, asyncio.TimeoutError):
            await ws.close()

    async def session(self, request):
        same_origin(request)
        result = web.Response(status=204)
        if request.method == 'DELETE':
            result.del_cookie(COOKIE, path='/', secure=True, httponly=True, samesite='Strict')
            return result
        claims = await identity(request)
        if time.time() - claims['auth_time'] > 300 or 'Authorization' not in request.headers:
            raise PermissionError
        cookie = await asyncio.to_thread(auth.create_session_cookie, request.headers['Authorization'][7:],
                                         expires_in=timedelta(hours=2))
        result.set_cookie(COOKIE, cookie.decode() if isinstance(cookie, bytes) else cookie, path='/',
                          secure=True, httponly=True, samesite='Strict', max_age=7200)
        return result

    async def vehicles(self, request):
        user = await identity(request)
        return web.json_response(await self.store.vehicles(user['uid']))

    async def policy(self, request):
        user = await identity(request)
        key = vehicle_id(request.match_info['key'])
        grant = (await self.store.grant(user['uid'], key).get()).to_dict()
        if not grant:
            raise PermissionError
        if request.method == 'PUT':
            body = await request.json()
            doc = await self.store.update(key, user['uid'], mode=body['mode'], version=body['version'])
            await self.push_control(key, doc)
        else:
            doc = (await self.store.vehicle(key).collection('control').document('current').get()).to_dict()
        return web.json_response(dict(mode=doc['desiredMode'], version=doc['configVersion'],
                                      online=key in self.devices))

    async def issue(self, request):
        user = await identity(request)
        code = await self.store.issue(request.match_info['key'], user['uid'])
        return web.json_response(dict(code=code))

    async def exchange(self, request):
        body = await request.json()
        code = body['code']
        if not isinstance(code, str) or len(code) != 32:
            raise PermissionError
        key, uid = await self.store.exchange(code)
        token = await asyncio.to_thread(auth.create_custom_token, uid)
        return web.json_response(dict(vehicle=key, token=token.decode(), config=self.config))

    async def viewer(self, request):
        user = await identity(request)
        key = vehicle_id(request.match_info['key'])
        if not (await self.store.grant(user['uid'], key).get()).exists:
            raise PermissionError
        ws = web.WebSocketResponse(max_msg_size=1024)
        await ws.prepare(request)
        v = Viewer(ws, key, secrets.token_hex(12))
        self.viewers.add(v)
        start = time.monotonic()
        trace('viewer_authorized', requestId=v.request_id, vehicle=key)
        writer = asyncio.create_task(v.send())
        try:
            await ws.send_json(dict(type='connecting', requestId=v.request_id))
            while True:
                msg = await ws.receive(timeout=20)
                if msg.type != WSMsgType.TEXT:
                    break
                body = json.loads(msg.data)
                if body.get('type') in ('recordings', 'clip'):
                    await identity(request)   # still signed in, and still granted below
                    if not (await self.store.grant(user['uid'], key).get()).exists:
                        raise PermissionError
                    await self.recording_request(v, key, body)
                    continue
                channels = body['channels']
                if channels not in (['telemetry'], ['telemetry', 'video']):
                    break
                await identity(request)
                since = time.monotonic()
                doc = await self.store.update(key, user['uid'], session=v.request_id, channels=channels)
                v.deadline = min(since + 30, since + user['exp'] - time.time())
                if channels != v.channels:
                    trace('demand_committed', requestId=v.request_id, generation=doc['demandGeneration'])
                v.channels = channels
                await self.push_control(key, doc)
        except Conflict:
            # A viewer update conflicts only when the vehicle's lease set is full. That is
            # retryable (1013 Try Again Later), unlike the access failures below (1008).
            await ws.close(code=VEHICLE_FULL)
        except (PermissionError, ValueError, KeyError, TypeError, asyncio.TimeoutError):
            await ws.close(code=1008)
        finally:
            self.viewers.discard(v)
            for token in [t for t, r in self.requests.items() if r['viewer'] is v]:
                self.finish(token)   # nobody to tell; a late answer from the vehicle is then ignored
            writer.cancel()
            release = asyncio.create_task(self.release(key, user['uid'], v.request_id))
            self.releasing.add(release)
            release.add_done_callback(self.releasing.discard)
            await asyncio.shield(release)
            await asyncio.gather(writer, return_exceptions=True)
            trace('viewer_closed', requestId=v.request_id, seconds=round(time.monotonic()-start, 2),
                  bytes=v.sent)
        return ws

    async def uplink(self, request):
        user = await identity(request, device=True)
        key = vehicle_id(request.query.get('vehicle'))
        await self.store.device_control(key, user['uid'])
        ws = web.WebSocketResponse(max_msg_size=MAX_PACKET)
        await ws.prepare(request)
        old = self.devices.get(key)
        self.devices[key] = ws
        if old:
            self.fail_device(old)   # its pending answers can no longer arrive here
            await old.close()
        granted, until, generation, version = [], 0, None, None
        last_traffic = last_check = time.monotonic()
        try:
            while self.devices.get(key) is ws:
                if time.monotonic() - last_traffic >= 150:
                    break
                if time.monotonic() - last_check >= 10:
                    await identity(request, device=True)
                    doc = await self.store.device_control(key, user['uid'])
                    last_check = time.monotonic()
                    if (generation, version) != (doc['demandGeneration'], doc['configVersion']):
                        await self.push_control(key, doc)
                    if doc['demandGeneration'] == generation and doc['expiresAt']:
                        until = doc['expiresAt'].timestamp()
                    else:
                        granted = []
                try:
                    msg = await ws.receive(timeout=5)
                except asyncio.TimeoutError:
                    continue
                if msg.type not in (WSMsgType.TEXT, WSMsgType.BINARY):
                    break
                last_traffic = time.monotonic()
                if msg.type == WSMsgType.TEXT:
                    body = json.loads(msg.data)
                    if body.get('type') == 'register':
                        doc = await self.store.device_control(key, user['uid'])
                        generation = body['generation']
                        version = doc['configVersion']
                        until = doc['expiresAt'].timestamp() if doc['expiresAt'] else 0
                        if doc['demandGeneration'] != generation:
                            break
                        granted = doc['channels'] if time.time() < until else []
                        if not granted and doc['desiredMode'] != 'ready_while_powered':
                            break
                        await ws.send_json(dict(type='grant', attempt=body['attempt'], generation=generation,
                                                channels=granted, mode=doc['desiredMode'],
                                                version=doc['configVersion']))
                        await self.push_control(key, doc)
                        trace('relay_confirmed', requestId=doc.get('requestId'), generation=generation)
                        continue
                    if body.get('type') == 'heartbeat':
                        await ws.send_json(dict(type='heartbeat'))
                        continue
                    if body.get('type') in ('recordings', 'clip_result'):
                        await self.recording_response(ws, body)
                        continue
                    if body.get('type') != 'snapshot':
                        break
                    channel = 'telemetry'
                else:
                    channel = 'video'
                if channel in granted and time.time() < until and self.devices.get(key) is ws:
                    for viewer in self.viewers:
                        if viewer.vehicle == key:
                            viewer.offer(channel, msg.data)
                            if channel == 'telemetry' and not viewer.first and body.get('runtime_fresh'):
                                viewer.first = True
                                trace('first_fresh_sample', requestId=viewer.request_id)
        except (PermissionError, ValueError, KeyError, TypeError, ConnectionError):
            pass
        finally:
            if self.devices.get(key) is ws:
                del self.devices[key]
            self.fail_device(ws)
            await ws.close()
        return ws


def application(store, config, clips=None):
    app = web.Application(middlewares=[errors], client_max_size=4096)
    relay = Relay(store, config, clips)
    app.on_cleanup.append(relay.drain)
    async def static(request):
        return web.FileResponse(WEB / (request.match_info.get('name') or 'index.html'),
                                headers={'X-Frame-Options': 'DENY'})
    async def dashboard(request):
        name = request.match_info.get('name', 'index.html')
        return web.FileResponse(WEB.parent.parent / 'web' / name, headers={'X-Frame-Options': 'DENY'})

    async def configuration(request):
        return web.json_response(config)
    app.router.add_get('/', static)
    app.router.add_get(r'/{name:fleet\.js|style\.css}', static)
    app.router.add_get('/dashboard', dashboard)
    app.router.add_get(r'/{name:app\.js|app\.css|icon\.svg}', dashboard)
    app.router.add_get('/v1/config', configuration)
    app.router.add_post('/v1/session', relay.session)
    app.router.add_delete('/v1/session', relay.session)
    app.router.add_get('/v1/vehicles', relay.vehicles)
    app.router.add_get('/v1/vehicles/{key}/policy', relay.policy)
    app.router.add_put('/v1/vehicles/{key}/policy', relay.policy)
    app.router.add_post('/v1/vehicles/{key}/enrollment', relay.issue)
    app.router.add_post('/v1/enroll', relay.exchange)
    app.router.add_get('/v1/view/{key}', relay.viewer)
    app.router.add_get('/v1/uplink', relay.uplink)
    return app


async def production():
    if any(os.getenv(k) for k in ('FIRESTORE_EMULATOR_HOST', 'FIREBASE_AUTH_EMULATOR_HOST')):
        raise RuntimeError('Emulator settings in staging environment')
    config = json.loads(os.environ['FIREBASE_WEB_CONFIG'])
    firebase_admin.initialize_app(options={'projectId': config['projectId']})
    db = firestore.AsyncClient(project=config['projectId'])
    bucket = os.getenv('CLIP_BUCKET')
    return application(Store(db), config, ClipStorage(bucket) if bucket else None)


if __name__ == '__main__':
    web.run_app(production(), port=int(os.getenv('PORT', '8080')), access_log=None)
