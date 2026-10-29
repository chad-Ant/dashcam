#!/usr/bin/env python3
"""Read-only TLS monitoring agent. Run on the Jetson host, outside Docker."""
import argparse
import asyncio
import base64
import copy
import datetime as dt
import hashlib
import hmac
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import secrets
import socket
import ssl
import stat
import struct
import time
import uuid
from urllib.parse import quote, urlencode

from aiohttp import web, WSMsgType
from cryptography import x509
from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
from health import Health

MAX_PACKET = 16384
STALE_SECONDS = 2.0
MAX_VIEWERS = 4  # per channel: /v1/stream and /v1/video each
# Camera preview, little-endian: magic, header length, then v0.4 datagram = capture CLOCK_MONOTONIC us,
# source sequence; client message = agent frame sequence, age ms at send time; then width, height, JPEG.
VIDEO_MAGIC = b'DCV1'
VIDEO_HEADER = struct.Struct('<4sIQIHH')
MAX_VIDEO_JPEG = 262144
VIDEO_NEW_PRODUCER_SECONDS = 5.0  # capture time going back this far = another v0.4 run
VIDEO_SEND_MAX_AGE = 1.0
VIDEO_MIN_INTERVAL = .09  # <= 10 fps per viewer
DEMAND_INTERVAL = .5  # v0.4 converts while the demand file is < 3 s old
CA_NETWORKS = [ipaddress.ip_network(n) for n in (
    '10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16', '127.0.0.0/8', '169.254.0.0/16',
    '::1/128', 'fd00::/8', 'fe80::/10')]
# Fixed, never widened by SANs: a stolen CA key cannot sign for any public name or address.
CA_DOMAINS = ['local', 'lan', 'home.arpa', 'localhost']
LABEL = re.compile(r'[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?')
COOKIE = '__Host-dcsession'
COOKIE_VALUE = re.compile(r'v1\.([0-9]{1,12})\.([A-Za-z0-9_-]{22})\.([A-Za-z0-9_-]{43})')
SECURITY_HEADERS = {
    'Content-Security-Policy': "default-src 'none'; script-src 'self'; style-src 'self'; img-src 'self' data: blob:; "
                               "connect-src 'self'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'",
    'X-Content-Type-Options': 'nosniff', 'Referrer-Policy': 'no-referrer', 'X-Frame-Options': 'DENY',
    'Cross-Origin-Opener-Policy': 'same-origin', 'Cross-Origin-Resource-Policy': 'same-origin',
    'Permissions-Policy': 'geolocation=(), camera=(), microphone=()', 'Cache-Control': 'no-store'}
STATIC = {'/': ('index.html', 'text/html; charset=utf-8'), '/index.html': ('index.html', 'text/html; charset=utf-8'),
          '/app.js': ('app.js', 'text/javascript; charset=utf-8'), '/app.css': ('app.css', 'text/css; charset=utf-8'),
          '/icon.svg': ('icon.svg', 'image/svg+xml')}


def encode(data):
    return json.dumps(data, allow_nan=False, separators=(',', ':'))


def b64(data):
    return base64.urlsafe_b64encode(data).rstrip(b'=').decode()


def colon_hex(cert):
    return ':'.join(f'{b:02X}' for b in cert.fingerprint(hashes.SHA256()))


def parse_san(value):
    try:
        address = ipaddress.ip_address(value)
    except ValueError:
        name = value.lower()
        if len(name) > 253 or not all(LABEL.fullmatch(label) for label in name.split('.')):
            raise ValueError(f'Invalid SAN {value!r}: expected an IP address or a DNS name '
                             '(no wildcard, scheme, port or path)')
        return name
    if getattr(address, 'scope_id', None):
        raise ValueError(f'Invalid SAN {value!r}: IPv6 zone IDs cannot be put in a certificate')
    return address


def permitted(san, constraints):
    if isinstance(san, str):
        return any(isinstance(c, x509.DNSName) and (san == c.value or san.endswith('.' + c.value))
                   for c in constraints)
    return any(isinstance(c, x509.IPAddress) and c.value.version == san.version and san in c.value
               for c in constraints)


def check_sans(values, sans, ca):
    constraints = ca.extensions.get_extension_for_class(x509.NameConstraints).value.permitted_subtrees
    for value, san in zip(values, sans):
        if not permitted(san, constraints):
            raise ValueError(f'SAN {value!r} is outside the local CA name constraints '
                             '(private/link-local/loopback IPs, .local, .lan, .home.arpa, localhost)')


def pem(key):
    return key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                             serialization.NoEncryption())


def make_ca(identity):
    key = ec.generate_private_key(ec.SECP256R1())
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, 'Dashcam Connect local CA ' + identity['id'][:8]),
                         x509.NameAttribute(NameOID.ORGANIZATION_NAME, 'Dashcam Connect')])
    now = dt.datetime.now(dt.timezone.utc)
    cert = (x509.CertificateBuilder().subject_name(subject).issuer_name(subject)
            .public_key(key.public_key()).serial_number(x509.random_serial_number())
            .not_valid_before(now - dt.timedelta(days=1)).not_valid_after(now + dt.timedelta(days=3650))
            .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
            .add_extension(x509.KeyUsage(digital_signature=False, content_commitment=False, key_encipherment=False,
                                         data_encipherment=False, key_agreement=False, key_cert_sign=True,
                                         crl_sign=True, encipher_only=False, decipher_only=False), critical=True)
            .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
            .add_extension(x509.NameConstraints(
                permitted_subtrees=[x509.IPAddress(n) for n in CA_NETWORKS] + [x509.DNSName(d) for d in CA_DOMAINS],
                excluded_subtrees=None), critical=True)
            .sign(key, hashes.SHA256()))
    return key, cert


def make_leaf(ca_key, ca, sans):
    key = ec.generate_private_key(ec.SECP256R1())
    # An IP in the CN fails OpenSSL's CN-vs-DNS name-constraint check when there is no DNS SAN.
    # X.520 limits the CN to 64 characters; verifiers use the SAN anyway.
    names = [s for s in sans if isinstance(s, str) and len(s) <= 64]
    subject = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, names[0] if names else 'Dashcam Monitor')])
    start = dt.datetime.now(dt.timezone.utc) - dt.timedelta(days=1)
    # Apple caps TLS server certificates at 825 days (NotAfter - NotBefore); never outlive the CA.
    builder = (x509.CertificateBuilder().subject_name(subject).issuer_name(ca.subject)
               .public_key(key.public_key()).serial_number(x509.random_serial_number())
               .not_valid_before(start)
               .not_valid_after(min(start + dt.timedelta(days=825), ca.not_valid_after_utc))
               .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
               .add_extension(x509.KeyUsage(digital_signature=True, content_commitment=False, key_encipherment=False,
                                            data_encipherment=False, key_agreement=False, key_cert_sign=False,
                                            crl_sign=False, encipher_only=False, decipher_only=False), critical=True)
               .add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False)
               .add_extension(x509.SubjectKeyIdentifier.from_public_key(key.public_key()), critical=False)
               .add_extension(x509.AuthorityKeyIdentifier.from_issuer_public_key(ca_key.public_key()),
                              critical=False))
    if sans:
        builder = builder.add_extension(x509.SubjectAlternativeName(
            [x509.DNSName(s) if isinstance(s, str) else x509.IPAddress(s) for s in sans]), critical=False)
    cert = builder.sign(ca_key, hashes.SHA256())
    return pem(key), cert.public_bytes(serialization.Encoding.PEM) + ca.public_bytes(serialization.Encoding.PEM)


def write_new(path, content, mode):
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
    with os.fdopen(fd, 'wb') as f:
        f.write(content)


def write_atomic(directory, files):
    """Write and fsync every file first, then rename them in order and fsync the directory."""
    temps = []
    try:
        for name, content, mode in files:
            temp = directory / f'.{name}.{secrets.token_hex(4)}.tmp'
            fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
            temps.append((temp, directory / name))
            with os.fdopen(fd, 'wb') as f:
                f.write(content)
                f.flush()
                os.fsync(f.fileno())
        for temp, path in temps:
            os.replace(temp, path)
        if os.name == 'posix':
            fd = os.open(directory, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
    finally:
        for temp, _ in temps:
            temp.unlink(missing_ok=True)


def initialize(directory, name, values=()):
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    if (directory / 'identity.json').exists():
        raise ValueError('Identity already exists; reuse it instead of replacing paired credentials')
    sans = [parse_san(v) for v in values]
    identity = {'id': str(uuid.uuid4()), 'name': name, 'token': secrets.token_urlsafe(32)}
    ca_key, ca = make_ca(identity)
    check_sans(values, sans, ca)
    key, chain = make_leaf(ca_key, ca, sans)
    for filename, content, mode in [('ca-key.pem', pem(ca_key), 0o600),
                                    ('ca.pem', ca.public_bytes(serialization.Encoding.PEM), 0o644),
                                    ('key.pem', key, 0o600), ('cert.pem', chain, 0o644),
                                    ('identity.json', encode(identity).encode(), 0o600)]:
        write_new(directory / filename, content, mode)


def issued_by(cert, key):
    try:
        key.public_key().verify(cert.signature, cert.tbs_certificate_bytes, ec.ECDSA(cert.signature_hash_algorithm))
        return True
    except (InvalidSignature, TypeError, ValueError):
        return False


def load_ca(directory):
    key_path, cert_path = directory / 'ca-key.pem', directory / 'ca.pem'
    if not key_path.exists() and not cert_path.exists():
        return None
    if key_path.exists() and not cert_path.exists():
        # ca-key.pem is renamed before ca.pem: a key that never signed the served leaf
        # is left over from an interrupted upgrade and was never in use.
        key = serialization.load_pem_private_key(key_path.read_bytes(), password=None)
        leaf = x509.load_pem_x509_certificates((directory / 'cert.pem').read_bytes())[0]
        if not issued_by(leaf, key):
            return None
    if not key_path.exists() or not cert_path.exists():
        raise ValueError('Incomplete local CA (ca.pem/ca-key.pem); refusing to replace it')
    key = serialization.load_pem_private_key(key_path.read_bytes(), password=None)
    ca = x509.load_pem_x509_certificate(cert_path.read_bytes())
    if key.public_key().public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo) != \
            ca.public_key().public_bytes(serialization.Encoding.DER, serialization.PublicFormat.SubjectPublicKeyInfo):
        raise ValueError('ca-key.pem does not match ca.pem')
    return key, ca


def issue_certificate(directory, values):
    """Issue a new leaf (new key); the identity id/token never change."""
    if not (directory / 'identity.json').exists():
        raise ValueError('No device identity; run init first')
    identity = load_identity(directory)
    sans = [parse_san(v) for v in values]
    if not sans:
        raise ValueError('At least one --san is required')
    loaded = load_ca(directory)
    ca_key, ca = loaded or make_ca(identity)
    check_sans(values, sans, ca)
    key, chain = make_leaf(ca_key, ca, sans)
    files = [] if loaded else [('ca-key.pem', pem(ca_key), 0o600),
                               ('ca.pem', ca.public_bytes(serialization.Encoding.PEM), 0o644)]
    write_atomic(directory, files + [('key.pem', key, 0o600), ('cert.pem', chain, 0o644)])
    return load_identity(directory)


def export_ca(directory, out):
    if not (directory / 'ca.pem').exists():
        raise ValueError('No local CA; run cert --san <LAN IP> first')
    ca = x509.load_pem_x509_certificate((directory / 'ca.pem').read_bytes())
    try:
        write_new(Path(out), ca.public_bytes(serialization.Encoding.PEM), 0o644)
    except FileExistsError:
        raise ValueError(f'{out} already exists; refusing to overwrite it') from None
    return colon_hex(ca)


def load_identity(directory):
    if os.name == 'posix':
        for name in ('key.pem', 'identity.json', 'ca-key.pem'):
            path = directory / name
            if (name != 'ca-key.pem' or path.exists()) and path.stat().st_mode & 0o077:
                raise ValueError(f'{name} must be accessible only to the service user (chmod 600)')
    identity = json.loads((directory / 'identity.json').read_text())
    if not isinstance(identity.get('token'), str) or len(identity['token']) < 32:
        raise ValueError('Invalid device credential')
    cert = x509.load_pem_x509_certificates((directory / 'cert.pem').read_bytes())[0]
    identity['pin'] = cert.fingerprint(hashes.SHA256()).hex()  # the leaf, as Android pins it
    try:
        names = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName).value
    except x509.ExtensionNotFound:
        names = []
    identity['sans'] = [str(n.value) for n in names if isinstance(n, (x509.DNSName, x509.IPAddress))]
    ca = directory / 'ca.pem'
    identity['ca_fingerprint'] = (x509.load_pem_x509_certificate(ca.read_bytes()).fingerprint(hashes.SHA256()).hex()
                                  if ca.exists() else None)
    return identity


def jpeg_size(data):
    """(width, height) from the first SOF0/1/2 segment, or None."""
    i = 2 if data[:2] == b'\xff\xd8' else len(data)
    while i + 9 <= len(data) and data[i] == 0xFF:
        if data[i + 1] in (0xC0, 0xC1, 0xC2):
            height, width = struct.unpack_from('>HH', data, i + 5)
            return (width, height) if width and height else None
        i += 2 + int.from_bytes(data[i + 2:i + 4], 'big')
    return None


def video_message(frame, age_s):
    age_ms = min(max(0, int(age_s * 1000)), 0xFFFFFFFF)
    return VIDEO_HEADER.pack(VIDEO_MAGIC, VIDEO_HEADER.size, frame['seq'], age_ms,
                             frame['width'], frame['height']) + frame['jpeg']


class State:
    def __init__(self, identity, storage, video_demand=None):
        self.identity = identity
        self.source = None
        self.health = {}
        self.collector = Health(storage)
        self.clients = set()
        self.seq = 0
        self.session = str(uuid.uuid4())
        self.invalid_packets = 0
        self.tasks = []
        self.video = None  # latest frame only
        self.video_seq = 0
        self.invalid_video = 0
        self.video_clients = set()
        self.video_demand = Path(video_demand) if video_demand else None
        self.demand_failed = False

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

    def accept_video(self, data):
        try:
            if not VIDEO_HEADER.size <= len(data) <= VIDEO_HEADER.size + MAX_VIDEO_JPEG:
                raise ValueError('size')
            magic, header, capture_us, sequence, width, height = VIDEO_HEADER.unpack_from(data)
            capture = capture_us / 1e6
            jpeg = bytes(data[header:])
            if (magic != VIDEO_MAGIC or not VIDEO_HEADER.size <= header <= len(data) - 4 or
                    jpeg[:2] != b'\xff\xd8' or jpeg[-2:] != b'\xff\xd9' or
                    not 1 <= width <= 4096 or not 1 <= height <= 4096 or
                    not math.isfinite(capture) or not 0 <= time.monotonic() - capture <= STALE_SECONDS):
                raise ValueError('invalid frame')
        except (ValueError, TypeError, struct.error):
            self.invalid_video += 1
            return
        current = self.video
        if (current and sequence <= current['source_seq'] and
                capture >= current['capture_mono_s'] - VIDEO_NEW_PRODUCER_SECONDS and
                time.monotonic() - current['capture_mono_s'] <= STALE_SECONDS):
            return  # queued or reordered datagram of the same producer; an expired frame binds nothing
        self.video_seq += 1
        self.video = {'jpeg': jpeg, 'seq': self.video_seq, 'source_seq': sequence,
                      'capture_mono_s': capture, 'width': width, 'height': height}

    def touch_demand(self):
        """Tell v0.4 someone is watching; it stops converting once the mtime is 3 s old."""
        if self.video_demand is None:
            return
        try:
            fd = os.open(self.video_demand, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK |
                         os.O_CLOEXEC, 0o600)
            try:
                if not stat.S_ISREG(os.fstat(fd).st_mode):
                    raise OSError('not a regular file')
                os.utime(fd)
            finally:
                os.close(fd)
        except OSError as error:
            if not self.demand_failed:
                print(f'WARNING: cannot update the video demand file {self.video_demand}: {error}', flush=True)
            self.demand_failed = True
        else:
            self.demand_failed = False

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


class VideoReceiver(asyncio.DatagramProtocol):
    def __init__(self, state):
        self.state = state

    def datagram_received(self, data, _addr):
        self.state.accept_video(data)


async def refuse_messages(ws):
    async for msg in ws:
        if msg.type in (WSMsgType.TEXT, WSMsgType.BINARY):
            await ws.close(code=1008, message=b'Read-only channel')
            break


async def touch_demand_while_watched(state):
    while True:
        if state.video_clients:
            state.touch_demand()
        await asyncio.sleep(DEMAND_INTERVAL)


def browser_hosts(sans, port):
    hosts = set()
    for san in sans:
        host = f'[{san}]' if ':' in san else san
        hosts.add(f'{host}:{port}'.lower())
        if port == 443:
            hosts.add(host.lower())
    return hosts


def create_app(state, port=8443, web_dir=None, session_days=30):
    web_dir = Path(web_dir) if web_dir else Path(__file__).parent / 'web'
    allowed = browser_hosts(state.identity.get('sans', []), port)
    key = hmac.new(state.identity['token'].encode(), b'dashcam-browser-session-v1', hashlib.sha256).digest()

    def sign(body):
        return b64(hmac.new(key, body.encode(), hashlib.sha256).digest())

    def valid_cookie(value):
        match = COOKIE_VALUE.fullmatch(value or '')
        return bool(match and hmac.compare_digest(match[3], sign(value.rsplit('.', 1)[0]))
                    and time.time() < int(match[1]))

    def bearer_ok(request):
        expected = 'Bearer ' + state.identity['token']
        supplied = request.headers.get('Authorization', '')  # may hold surrogate-escaped bytes
        return supplied.isascii() and hmac.compare_digest(supplied.encode(), expected.encode())

    def browser_host(request):
        return request.headers.get('Host', '').lower() in allowed

    def same_origin(request):
        host = request.headers.get('Host', '').lower()
        return host in allowed and request.headers.get('Origin', '').lower() == 'https://' + host

    def cookie(value, max_age):
        return f'{COOKIE}={value}; Path=/; Secure; HttpOnly; SameSite=Strict; Max-Age={max_age}'

    async def static(request):
        filename, content_type = STATIC[request.match_info.route.resource.canonical]
        try:
            body = (web_dir / filename).read_bytes()
        except OSError:
            raise web.HTTPNotFound(text='Not found') from None
        return web.Response(body=body, headers={'Content-Type': content_type})

    async def create_session(request):
        if not bearer_ok(request):
            raise web.HTTPUnauthorized(text='Device credential required')
        if not same_origin(request):
            raise web.HTTPForbidden(text='Same-origin browser request required')
        expiry = int(time.time()) + session_days * 86400
        body = f'v1.{expiry}.{b64(secrets.token_bytes(16))}'
        return web.Response(status=204, headers={'Set-Cookie': cookie(body + '.' + sign(body), session_days * 86400)})

    async def delete_session(request):
        if not same_origin(request):
            raise web.HTTPForbidden(text='Same-origin browser request required')
        return web.Response(status=204, headers={'Set-Cookie': cookie('', 0)})

    @web.middleware
    async def authenticate(request, handler):
        if request.match_info.handler in (static, create_session, delete_session) or bearer_ok(request):
            return await handler(request)  # those check for themselves; bearer = Android, unchanged
        if valid_cookie(request.cookies.get(COOKIE)):
            # Browser session: only for our own origin, and never a cross-site WebSocket.
            needs_origin = 'Origin' in request.headers or request.match_info.handler in (stream, video)
            if not browser_host(request) or (needs_origin and not same_origin(request)):
                raise web.HTTPForbidden(text='Same-origin browser request required')
            return await handler(request)
        raise web.HTTPUnauthorized(text='Device credential required')

    async def secure_headers(_request, response):
        response.headers.update(SECURITY_HEADERS)

    app = web.Application(middlewares=[authenticate], client_max_size=4096)
    app.on_response_prepare.append(secure_headers)

    async def identity(_request):
        return web.json_response({'schema': 1, 'id': state.identity['id'], 'name': state.identity['name'],
                                  'capabilities': {'vehicle': True, 'system': True, 'video': True, 'lane': False,
                                                   'driver': False, 'controls': False}}, dumps=encode)

    async def snapshot(_request):
        return web.json_response(state.snapshot(), dumps=encode)

    async def stream(request):
        if len(state.clients) >= MAX_VIEWERS:
            raise web.HTTPServiceUnavailable(text='Maximum viewers reached')
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=4096, compress=False)
        # Reserve a slot before the upgrade awaits, so concurrent handshakes
        # cannot exceed the limit.
        state.clients.add(ws)
        try:
            await ws.prepare(request)
            reader = asyncio.create_task(refuse_messages(ws))
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

    async def video(request):
        if len(state.video_clients) >= MAX_VIEWERS:
            raise web.HTTPServiceUnavailable(text='Maximum video viewers reached')
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=4096, compress=False)
        state.video_clients.add(ws)  # reserved before the upgrade awaits, as for /v1/stream
        try:
            await ws.prepare(request)
            state.touch_demand()  # start the conversion now, not at the next demand tick
            reader = asyncio.create_task(refuse_messages(ws))
            try:
                shown, sent_at = 0, -math.inf
                while not ws.closed and not reader.done():
                    frame, now = state.video, time.monotonic()
                    # Only the newest frame, never queued: a slow viewer skips frames.
                    if frame and frame['seq'] != shown and now - sent_at >= VIDEO_MIN_INTERVAL:
                        shown = frame['seq']
                        age = now - frame['capture_mono_s']
                        if age <= VIDEO_SEND_MAX_AGE:
                            sent_at = now
                            await asyncio.wait_for(ws.send_bytes(video_message(frame, age)), timeout=2)
                    await asyncio.sleep(.05)
            finally:
                reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        except (ConnectionError, asyncio.TimeoutError):
            pass
        finally:
            state.video_clients.discard(ws)
            if ws.prepared:
                await ws.close()
        return ws

    async def shutdown(_app):
        await asyncio.gather(*(ws.close(code=1001) for ws in tuple(state.clients | state.video_clients)),
                             return_exceptions=True)

    async def demand(_app):
        task = asyncio.create_task(touch_demand_while_watched(state))
        yield
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)

    for path in STATIC:
        app.router.add_get(path, static)
    app.router.add_post('/v1/session', create_session)
    app.router.add_delete('/v1/session', delete_session)
    app.router.add_get('/v1/identity', identity)
    app.router.add_get('/v1/snapshot', snapshot)
    app.router.add_get('/v1/stream', stream)
    app.router.add_get('/v1/video', video)
    app.on_shutdown.append(shutdown)
    if state.video_demand:
        app.cleanup_ctx.append(demand)
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


async def simulate_video(state, card):
    size = jpeg_size(card)
    if size is None:
        print('Simulator: testcard.jpg has no readable frame size; no video', flush=True)
        return
    seq = 0
    while True:
        seq = (seq + 1) & 0xFFFFFFFF
        state.accept_video(VIDEO_HEADER.pack(VIDEO_MAGIC, VIDEO_HEADER.size, int(time.monotonic() * 1e6), seq,
                                             *size) + card)
        await asyncio.sleep(.1)


async def bind_datagram(path, protocol):
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    # A stale socket can be left by SIGKILL. Never unlink another type
    # of file or an active receiver.
    if path.exists():
        if not stat.S_ISSOCK(path.lstat().st_mode):
            raise ValueError(f'IPC path {path} exists and is not a socket')
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as probe:
            try:
                probe.connect(str(path))
            except ConnectionRefusedError:
                path.unlink()
            else:
                raise ValueError(f'A monitoring agent already owns {path}')
    transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
        lambda: protocol, local_addr=str(path), family=socket.AF_UNIX)
    path.chmod(0o600)
    return transport


async def run(args, identity):
    state = State(identity, args.storage, video_demand=None if args.simulate else args.video_demand)
    endpoints, advertisement = [], None
    try:
        if args.simulate:
            identity['name'] += ' [SIMULATOR]'
            state.tasks.append(asyncio.create_task(simulate(state)))
            try:
                card = (Path(__file__).parent / 'testcard.jpg').read_bytes()  # read once
            except OSError:
                print('Simulator: no testcard.jpg; no video', flush=True)
            else:
                state.tasks.append(asyncio.create_task(simulate_video(state, card)))
        else:
            for path, protocol in [(Path(args.socket), Receiver(state)),
                                   (Path(args.video_socket), VideoReceiver(state))]:
                endpoints.append((await bind_datagram(path, protocol), path))
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
        app = create_app(state, port=args.port, web_dir=args.web_dir, session_days=args.browser_session_days)
        runner = web.AppRunner(app, access_log=None, keepalive_timeout=10,
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
        for transport, path in endpoints:
            transport.close()
            path.unlink(missing_ok=True)
        if advertisement:
            zc, info = advertisement
            await zc.async_unregister_service(info)
            await zc.async_close()


def pair(args, identity):
    print('Private pairing link (contains the device credential):')
    print('dashcam://pair?' + urlencode({'host': args.host, 'port': args.port, 'pin': identity['pin'],
                                        'token': identity['token'], 'id': identity['id'], 'name': identity['name']}))
    try:
        host = str(ipaddress.ip_address(args.host))
    except ValueError:
        host = args.host.lower()
    print('Browser link (open it on a device that trusts the CA):')
    print(f"https://{f'[{host}]' if ':' in host else host}:{args.port}/#pair={quote(identity['token'], safe='')}")
    if identity['ca_fingerprint']:
        ca = x509.load_pem_x509_certificate((args.state_dir / 'ca.pem').read_bytes())
        print('Dashcam Connect CA SHA-256 fingerprint: ' + colon_hex(ca))
    if host not in identity['sans']:
        print(f'WARNING: {args.host} is not in the certificate SANs {identity["sans"]}; browsers will reject it. '
              f"Run cert {' '.join('--san ' + s for s in identity['sans'] + [host])} "
              '(Android devices must then be paired again).')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=['init', 'pair', 'serve', 'cert', 'ca'])
    p.add_argument('--state-dir', type=Path, default=Path(__file__).parent / '.state')
    p.add_argument('--name', default='Jetson dashcam')
    p.add_argument('--host', help='LAN address/hostname to put in the pairing link')
    p.add_argument('--san', action='append', default=[],
                   help='IP address or DNS name for the TLS certificate (repeatable; init and cert)')
    p.add_argument('--out', help='ca: file to write the CA certificate to')
    p.add_argument('--bind', default='0.0.0.0')
    p.add_argument('--port', type=int, default=8443)
    p.add_argument('--socket', default='/run/dashcam-monitor/telemetry.sock')
    p.add_argument('--video-socket', type=Path,
                   help='camera preview datagram socket (default: video.sock next to --socket)')
    p.add_argument('--video-demand', type=Path,
                   help='file touched while video viewers watch (default: video-demand next to --socket)')
    p.add_argument('--storage', default='/user/output/footage')
    p.add_argument('--web-dir', type=Path, default=Path(__file__).parent / 'web')
    p.add_argument('--browser-session-days', type=int, default=30)
    p.add_argument('--advertise-address', help='Host LAN IPv4 for mDNS; omit for manual pairing only')
    p.add_argument('--simulate', action='store_true')
    args = p.parse_args()
    args.video_socket = args.video_socket or Path(args.socket).parent / 'video.sock'
    args.video_demand = args.video_demand or Path(args.socket).parent / 'video-demand'
    if not 1 <= args.port <= 65535:
        p.error('port must be 1..65535')
    if not 1 <= args.browser_session_days <= 365:
        p.error('browser session days must be 1..365')
    try:
        if args.command == 'init':
            initialize(args.state_dir, args.name, args.san)
            print('Created private device identity. Run pair --host <Jetson LAN IP> to display the pairing link.')
            if not args.san:
                print('Browser access needs a certificate for the LAN address: run cert --san <LAN IP>.')
            return
        if args.command == 'cert':
            if not args.san:
                p.error('cert needs at least one --san')
            identity = issue_certificate(args.state_dir, args.san)
            print(f"Issued a new TLS certificate for {', '.join(identity['sans'])}; pin {identity['pin']}")
            print('WARNING: the certificate pin changed; pair every Android device again (pair --host ...).')
            return
        identity = load_identity(args.state_dir)
        if args.command == 'ca':
            if not args.out:
                p.error('ca needs --out PATH')
            fingerprint = export_ca(args.state_dir, args.out)
            print(f'Wrote the Dashcam Connect CA certificate to {args.out}; SHA-256 fingerprint {fingerprint}')
            print('Install it as a trusted CA on each browser device and check this fingerprint first.')
        elif args.command == 'pair':
            if not args.host:
                p.error('--host is required for pairing')
            pair(args, identity)
        else:
            try:
                asyncio.run(run(args, identity))
            except KeyboardInterrupt:
                pass
    except ValueError as error:
        p.exit(1, f'error: {error}\n')


if __name__ == '__main__':
    main()
