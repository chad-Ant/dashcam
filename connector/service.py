#!/usr/bin/env python3
"""Read-only TLS monitoring agent. Run on the Jetson host, outside Docker."""
import argparse
import asyncio
import copy
import datetime as dt
import hashlib
import hmac
import ipaddress
import json
import math
import os
from pathlib import Path
import secrets
import socket
import ssl
import stat
import time
import uuid
from urllib.parse import urlencode

from aiohttp import web, WSMsgType
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import NameOID
from health import Health

MAX_PACKET = 16384
STALE_SECONDS = 2.0


def encode(data):
    return json.dumps(data, allow_nan=False, separators=(',', ':'))


def initialize(directory, name):
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    if (directory / 'identity.json').exists():
        raise ValueError('Identity already exists; reuse it instead of replacing paired credentials')
    key = ec.generate_private_key(ec.SECP256R1())
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'Dashcam Monitor')])
    now = dt.datetime.now(dt.timezone.utc)
    cert = (x509.CertificateBuilder().subject_name(subject).issuer_name(subject)
            .public_key(key.public_key()).serial_number(x509.random_serial_number())
            .not_valid_before(now - dt.timedelta(days=1)).not_valid_after(now + dt.timedelta(days=3650))
            .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
            .sign(key, hashes.SHA256()))
    files = {
        'key.pem': key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                     serialization.NoEncryption()),
        'cert.pem': cert.public_bytes(serialization.Encoding.PEM),
        'identity.json': encode({'id': str(uuid.uuid4()), 'name': name,
                                 'token': secrets.token_urlsafe(32)}).encode(),
    }
    for name, content in files.items():
        fd = os.open(directory / name, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'wb') as f:
            f.write(content)


def load_identity(directory):
    if os.name == 'posix':
        for name in ('key.pem', 'identity.json'):
            if (directory / name).stat().st_mode & 0o077:
                raise ValueError(f'{name} must be accessible only to the service user (chmod 600)')
    identity = json.loads((directory / 'identity.json').read_text())
    if not isinstance(identity.get('token'), str) or len(identity['token']) < 32:
        raise ValueError('Invalid device credential')
    cert = x509.load_pem_x509_certificate((directory / 'cert.pem').read_bytes())
    identity['pin'] = cert.fingerprint(hashes.SHA256()).hex()
    return identity


class State:
    def __init__(self, identity, storage):
        self.identity = identity
        self.source = None
        self.health = {}
        self.collector = Health(storage)
        self.clients = set()
        self.seq = 0
        self.session = str(uuid.uuid4())
        self.invalid_packets = 0
        self.tasks = []

    def accept(self, data):
        try:
            if len(data) > MAX_PACKET:
                raise ValueError('oversize')
            source = json.loads(data, parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite')))
            stamp = source['source_mono_s']
            if (source['schema'] != 1 or source['runtime'] != 'dashcam_v0_4' or
                isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or
                not math.isfinite(stamp) or not 0 <= time.monotonic() - stamp <= STALE_SECONDS or
                not isinstance(source['vehicle'], dict) or not isinstance(source['adas'], dict) or
                not isinstance(source['recording'], bool) or not isinstance(source['bridge_fresh'], bool)):
                raise ValueError('invalid source')
            if self.source and stamp <= self.source['source_mono_s']:
                return  # queued or reordered datagram, never rejuvenate it
            self.source = source
        except (ValueError, KeyError, TypeError, UnicodeError, RecursionError):
            self.invalid_packets += 1

    def snapshot(self):
        age = None if self.source is None else max(0, int((time.monotonic() - self.source['source_mono_s']) * 1000))
        fresh = age is not None and age <= int(STALE_SECONDS * 1000)
        runtime = copy.deepcopy(self.source) if fresh else None
        health_age = time.monotonic() - self.health.get('sample_mono_s', -math.inf)
        health = self.health if 0 <= health_age <= 3 else {}
        self.seq += 1
        return {'schema': 1, 'type': 'snapshot', 'device_id': self.identity['id'],
                'session': self.session, 'seq': self.seq, 'runtime_fresh': fresh,
                'runtime_age_ms': age, 'runtime': runtime, 'system': health,
                'invalid_packets': self.invalid_packets}


class Receiver(asyncio.DatagramProtocol):
    def __init__(self, state):
        self.state = state

    def datagram_received(self, data, _addr):
        self.state.accept(data)


def create_app(state):
    @web.middleware
    async def authenticate(request, handler):
        expected = 'Bearer ' + state.identity['token']
        supplied = request.headers.get('Authorization', '')
        if not hmac.compare_digest(supplied.encode(), expected.encode()):
            raise web.HTTPUnauthorized(text='Device credential required')
        return await handler(request)

    app = web.Application(middlewares=[authenticate], client_max_size=4096)

    async def identity(_request):
        return web.json_response({'schema': 1, 'id': state.identity['id'], 'name': state.identity['name'],
                                  'capabilities': {'vehicle': True, 'system': True, 'lane': False,
                                                   'driver': False, 'controls': False}}, dumps=encode)

    async def snapshot(_request):
        return web.json_response(state.snapshot(), dumps=encode)

    async def stream(request):
        if len(state.clients) >= 4:
            raise web.HTTPServiceUnavailable(text='Maximum viewers reached')
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=4096, compress=False)
        # Reserve a slot before the upgrade awaits, so concurrent handshakes
        # cannot exceed the limit.
        state.clients.add(ws)
        try:
            await ws.prepare(request)
            async def receive():
                async for msg in ws:
                    if msg.type in (WSMsgType.TEXT, WSMsgType.BINARY):
                        await ws.close(code=1008, message=b'Read-only channel')
                        break
            reader = asyncio.create_task(receive())
            try:
                while not ws.closed and not reader.done():
                    await asyncio.wait_for(ws.send_str(encode(state.snapshot())), timeout=2)
                    await asyncio.sleep(.2)
            finally:
                reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        except (ConnectionError, asyncio.TimeoutError):
            pass
        finally:
            state.clients.discard(ws)
            if ws.prepared:
                await ws.close()
        return ws

    async def shutdown(_app):
        await asyncio.gather(*(ws.close(code=1001) for ws in tuple(state.clients)), return_exceptions=True)

    app.router.add_get('/v1/identity', identity)
    app.router.add_get('/v1/snapshot', snapshot)
    app.router.add_get('/v1/stream', stream)
    app.on_shutdown.append(shutdown)
    return app


async def collect_health(state):
    while True:
        state.health = await asyncio.to_thread(state.collector.sample)
        await asyncio.sleep(1)


async def simulate(state):
    seq = 0
    while True:
        seq += 1
        state.accept(encode({'schema': 1, 'runtime': 'dashcam_v0_4', 'seq': seq,
                            'source_mono_s': time.monotonic(), 'recording': True,
                            'frames': seq * 6, 'fps': 30, 'bridge_fresh': True, 'bridge_age_ms': 30,
                            'vehicle': {'speed_kmh': 40 + 10 * math.sin(seq / 25), 'speed_source': 'SIMULATED',
                                        'latitude': 10.7725, 'longitude': 106.6581, 'heading_deg': 90,
                                        'rpm': 1800, 'coolant_c': 85, 'fuel_percent': 60,
                                        'satellites': 12, 'acceleration_ms2': .2},
                            'adas': {'lane_available': False, 'driver_available': False}}))
        await asyncio.sleep(.2)


async def run(args, identity):
    state = State(identity, args.storage)
    transport, socket_path, advertisement = None, None, None
    try:
        if args.simulate:
            identity['name'] += ' [SIMULATOR]'
            state.tasks.append(asyncio.create_task(simulate(state)))
        else:
            socket_path = Path(args.socket)
            socket_path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            # A stale socket can be left by SIGKILL. Never unlink another type
            # of file or an active receiver.
            if socket_path.exists():
                if not stat.S_ISSOCK(socket_path.lstat().st_mode):
                    raise ValueError('IPC path exists and is not a socket')
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as probe:
                    try:
                        probe.connect(str(socket_path))
                    except ConnectionRefusedError:
                        socket_path.unlink()
                    else:
                        raise ValueError('A monitoring agent already owns this socket')
            transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
                lambda: Receiver(state), local_addr=str(socket_path), family=socket.AF_UNIX)
            socket_path.chmod(0o600)
        state.tasks.append(asyncio.create_task(collect_health(state)))
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(args.state_dir / 'cert.pem', args.state_dir / 'key.pem')
        if args.advertise_address:
            from zeroconf import IPVersion, ServiceInfo
            from zeroconf.asyncio import AsyncZeroconf
            address = ipaddress.IPv4Address(args.advertise_address)
            info = ServiceInfo('_dashcam._tcp.local.', identity['id'] + '._dashcam._tcp.local.',
                               addresses=[address.packed], port=args.port,
                               properties={'id': identity['id'], 'schema': '1'},
                               server='dashcam-' + identity['id'][:8] + '.local.')
            zc = AsyncZeroconf(interfaces=[str(address)], ip_version=IPVersion.V4Only)
            await zc.async_register_service(info)
            advertisement = zc, info
        runner = web.AppRunner(create_app(state), access_log=None, keepalive_timeout=10,
                               shutdown_timeout=3)
        await runner.setup()
        try:
            await web.TCPSite(runner, args.bind, args.port, ssl_context=context).start()
            print(f'Dashcam monitor listening on {args.bind}:{args.port}; read-only; simulator={args.simulate}', flush=True)
            await asyncio.Event().wait()
        finally:
            await runner.cleanup()
    finally:
        for task in state.tasks:
            task.cancel()
        await asyncio.gather(*state.tasks, return_exceptions=True)
        if transport:
            transport.close()
            socket_path.unlink(missing_ok=True)
        if advertisement:
            zc, info = advertisement
            await zc.async_unregister_service(info)
            await zc.async_close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['init', 'pair', 'serve'])
    p.add_argument('--state-dir', type=Path, default=Path(__file__).parent / '.state')
    p.add_argument('--name', default='Jetson dashcam')
    p.add_argument('--host', help='LAN address/hostname to put in the pairing link')
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8443)
    p.add_argument('--socket', default='/run/dashcam-monitor/telemetry.sock')
    p.add_argument('--storage', default='/user/output/footage')
    p.add_argument('--advertise-address', help='Host LAN IPv4 for mDNS; omit for manual pairing only')
    p.add_argument('--simulate', action='store_true')
    args = p.parse_args()
    if not 1 <= args.port <= 65535:
        p.error('port must be 1..65535')
    if args.command == 'init':
        initialize(args.state_dir, args.name)
        print('Created private device identity. Run pair --host <Jetson LAN IP> to display the pairing link.')
        return
    identity = load_identity(args.state_dir)
    if args.command == 'pair':
        if not args.host:
            p.error('--host is required for pairing')
        print('Private pairing link (contains the device credential):')
        print('dashcam://pair?' + urlencode({'host': args.host, 'port': args.port, 'pin': identity['pin'],
                                            'token': identity['token'], 'id': identity['id'], 'name': identity['name']}))
    else:
        try:
            asyncio.run(run(args, identity))
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
