"""Single-instance Cloud Run relay. Firestore owns authorization and demand."""
import asyncio
import json
import os
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

    def offer(self, channel, payload):
        if channel in self.channels and time.monotonic() < self.deadline:
            self.pending[channel] = payload
            self.wake.set()

    async def send(self):
        try:
            while True:
                await self.wake.wait()
                self.wake.clear()
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
    def __init__(self, store, config):
        self.store, self.config = store, config
        self.viewers = set()
        self.devices = {}
        self.releasing = set()

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
        except (PermissionError, Conflict, ValueError, KeyError, TypeError, asyncio.TimeoutError):
            await ws.close(code=1008)
        finally:
            self.viewers.discard(v)
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
            await ws.close()
        return ws


def application(store, config):
    app = web.Application(middlewares=[errors], client_max_size=4096)
    relay = Relay(store, config)
    app.on_cleanup.append(relay.drain)
    async def static(request):
        return web.FileResponse(WEB / (request.match_info.get('name') or 'index.html'),
                                headers={'X-Frame-Options': 'DENY'})
    async def configuration(request):
        return web.json_response(config)
    app.router.add_get('/', static)
    app.router.add_get('/{name:app.js|style.css}', static)
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
    return application(Store(db), config)


if __name__ == '__main__':
    web.run_app(production(), port=int(os.getenv('PORT', '8080')), access_log=None)
