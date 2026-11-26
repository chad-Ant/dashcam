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

from aiohttp import web, WSCloseCode, WSMsgType
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
# HD (direct LAN, `/v1/video?quality=hd`): v0.4 sends the camera's original JPEGs cut into chunks, each one
# datagram on the same socket: magic, header length, capture us, source sequence, width, height, total JPEG
# bytes, offset of this chunk; then the chunk. The client message is the same DCV1 message as the preview.
HD_MAGIC = b'DCH1'
HD_HEADER = struct.Struct('<4sIQIHHII')
MAX_HD_JPEG = 1048576
MAX_HD_VIEWERS = 2  # one 1080p30 viewer is 16-56 Mbit/s
VIDEO_HD_MIN_INTERVAL = .025  # never limits a 30 fps camera
HD_FIRST_FRAME_SECONDS = 5.0  # no HD frame by then: v0.4 does not offer HD
HD_UNAVAILABLE = 4001  # close code: this Jetson sends no HD frames; the client falls back to Low
# Recorded segments (direct LAN): v0.4's dashcam_<seq>_<YYYYmmdd>_<HHMMSS>.mkv plus its .ass captions, read only.
# A segment modified this recently is still being written (no Cues yet) and is neither listed nor served.
RECORDING_FILE = re.compile(r'(dashcam_(\d{6})_(\d{8})_(\d{6}))\.(mkv|ass)')
RECORDING_SETTLE_SECONDS = 10
RECORDING_TYPES = {'mkv': 'video/x-matroska', 'ass': 'text/plain; charset=utf-8'}
CA_NETWORKS = [ipaddress.ip_network(n) for n in (
    '10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16', '127.0.0.0/8', '169.254.0.0/16',
    '::1/128', 'fd00::/8', 'fe80::/10')]
# Fixed, never widened by SANs: a stolen CA key cannot sign for any public name or address.
CA_DOMAINS = ['local', 'lan', 'home.arpa', 'localhost']
LABEL = re.compile(r'[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?')
COOKIE = '__Host-dcsession'
SESSION_EXPIRY = web.RequestKey('session_expiry', int)  # browser socket deadline (Unix seconds)
COOKIE_VALUE = re.compile(r'v1\.([0-9]{1,12})\.([A-Za-z0-9_-]{22})\.([A-Za-z0-9_-]{43})')
SECURITY_HEADERS = {
    'Content-Security-Policy': "default-src 'none'; script-src 'self'; style-src 'self'; "
                               "img-src 'self' data: blob: https://tile.openstreetmap.org; "   # Vehicle map tiles
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


class Recordings:
    """The footage directory as a read-only list of finished segments. Names, never paths, come from clients."""

    def __init__(self, directory):
        self.dir = Path(directory) if directory else None

    def available(self):
        return self.dir is not None and self.dir.is_dir()

    def segments(self, now=None):
        now = time.time() if now is None else now
        found, captions = {}, {}
        with os.scandir(self.dir) as entries:
            for entry in entries:
                match = RECORDING_FILE.fullmatch(entry.name)
                if not match or not entry.is_file(follow_symlinks=False):
                    continue
                info = entry.stat(follow_symlinks=False)
                if match[5] == 'ass':
                    captions[match[1]] = info.st_size
                elif now - info.st_mtime >= RECORDING_SETTLE_SECONDS:
                    found[match[1]] = (match, info)
        out = []
        for base, (match, info) in found.items():
            try:
                start = dt.datetime.strptime(match[3] + match[4], '%Y%m%d%H%M%S')
            except ValueError:
                continue
            # The name is the Jetson's local start time; the last write is the end.
            duration = info.st_mtime - time.mktime(start.timetuple())
            out.append({'id': base, 'seq': int(match[2]), 'start': start.strftime('%Y-%m-%d %H:%M:%S'),
                        'duration_s': round(duration, 1) if 0 <= duration <= 3600 else None,
                        'bytes': info.st_size, 'caption_bytes': captions.get(base)})
        out.sort(key=lambda item: (item['start'], item['seq']), reverse=True)
        return out

    def resolve(self, name, now=None):
        """(path, None) for a servable file, else (None, HTTP status)."""
        match = RECORDING_FILE.fullmatch(name or '')
        if not match or not self.available():
            return None, 404
        path = self.dir / name
        try:
            info = os.lstat(path)
        except OSError:
            return None, 404
        if not stat.S_ISREG(info.st_mode):
            return None, 404
        if match[5] == 'mkv' and (time.time() if now is None else now) - info.st_mtime < RECORDING_SETTLE_SECONDS:
            return None, 409
        return path, None


def replaces(current, sequence, capture):
    """Whether a frame (sequence, capture time) may replace @p current, the latest frame of a channel."""
    return not (current and sequence <= current['source_seq'] and
                capture >= current['capture_mono_s'] - VIDEO_NEW_PRODUCER_SECONDS and
                time.monotonic() - current['capture_mono_s'] <= STALE_SECONDS)


class State:
    def __init__(self, identity, storage, video_demand=None, video_hd_demand=None, recordings_dir=None):
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
        self.video_hd = None  # latest complete original frame only
        self.video_hd_seq = 0
        self.hd_partial = None  # the frame being reassembled
        self.hd_incomplete = 0  # frames abandoned with a chunk missing
        self.video_hd_clients = set()
        self.video_hd_demand = Path(video_hd_demand) if video_hd_demand else None
        self.hd_demand_failed = False
        self.recordings = Recordings(recordings_dir)

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
        if not replaces(self.video, sequence, capture):
            return  # queued or reordered datagram of the same producer; an expired frame binds nothing
        self.video_seq += 1
        self.video = {'jpeg': jpeg, 'seq': self.video_seq, 'source_seq': sequence,
                      'capture_mono_s': capture, 'width': width, 'height': height}

    def accept_hd_chunk(self, data):
        """One DCH1 chunk; a frame becomes current only when its chunks arrived in order and complete."""
        try:
            if len(data) <= HD_HEADER.size:
                raise ValueError('size')
            magic, header, capture_us, sequence, width, height, total, offset = HD_HEADER.unpack_from(data)
            capture = capture_us / 1e6
            if (magic != HD_MAGIC or not HD_HEADER.size <= header < len(data) or not 4 <= total <= MAX_HD_JPEG or
                    offset + len(data) - header > total or not 1 <= width <= 4096 or not 1 <= height <= 4096 or
                    not math.isfinite(capture) or not 0 <= time.monotonic() - capture <= STALE_SECONDS):
                raise ValueError('invalid chunk')
        except (ValueError, TypeError, struct.error):
            self.invalid_video += 1
            self.hd_partial = None
            return
        key = (capture_us, sequence, width, height, total)
        partial = self.hd_partial
        if offset == 0:
            if partial is not None:
                self.hd_incomplete += 1
            partial = self.hd_partial = {'key': key, 'parts': [], 'size': 0}
        elif partial is None or partial['key'] != key or partial['size'] != offset:
            if partial is not None:
                self.hd_incomplete += 1
            self.hd_partial = None
            return  # a chunk went missing (the sender gave up on a busy queue): wait for the next frame
        partial['parts'].append(bytes(data[header:]))
        partial['size'] += len(data) - header
        if partial['size'] < total:
            return
        self.hd_partial = None
        jpeg = b''.join(partial['parts'])
        if jpeg[:2] != b'\xff\xd8' or jpeg[-2:] != b'\xff\xd9':
            self.invalid_video += 1
            return
        if not replaces(self.video_hd, sequence, capture):
            return
        self.video_hd_seq += 1
        self.video_hd = {'jpeg': jpeg, 'seq': self.video_hd_seq, 'source_seq': sequence,
                         'capture_mono_s': capture, 'width': width, 'height': height}

    def touch_demand(self, hd=False):
        """Tell v0.4 someone is watching; it stops converting (or sending HD) once the mtime is 3 s old."""
        path = self.video_hd_demand if hd else self.video_demand
        if path is None:
            return
        failed = self.hd_demand_failed if hd else self.demand_failed
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC, 0o600)
            try:
                if not stat.S_ISREG(os.fstat(fd).st_mode):
                    raise OSError('not a regular file')
                os.utime(fd)
            finally:
                os.close(fd)
        except OSError as error:
            if not failed:
                print(f'WARNING: cannot update the video demand file {path}: {error}', flush=True)
            failed = True
        else:
            failed = False
        if hd:
            self.hd_demand_failed = failed
        else:
            self.demand_failed = failed

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
        if data[:4] == HD_MAGIC:
            self.state.accept_hd_chunk(data)
        else:
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
        if state.video_hd_clients:
            state.touch_demand(hd=True)
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

    def cookie_expiry(value):
        """The session's expiry (Unix seconds) for a valid, unexpired cookie, else None."""
        match = COOKIE_VALUE.fullmatch(value or '')
        if match and hmac.compare_digest(match[3], sign(value.rsplit('.', 1)[0])) and time.time() < int(match[1]):
            return int(match[1])
        return None

    def session_expired(request):
        # Browser sockets carry their cookie's expiry; Android (bearer) sockets have none.
        expiry = request.get(SESSION_EXPIRY)
        return expiry is not None and time.time() >= expiry

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
        expiry = cookie_expiry(request.cookies.get(COOKIE))
        if expiry is not None:
            # Browser session: only for our own origin, and never a cross-site WebSocket.
            needs_origin = 'Origin' in request.headers or request.match_info.handler in (stream, video)
            if not browser_host(request) or (needs_origin and not same_origin(request)):
                raise web.HTTPForbidden(text='Same-origin browser request required')
            request[SESSION_EXPIRY] = expiry  # the stream handlers close the socket when it passes
            return await handler(request)
        raise web.HTTPUnauthorized(text='Device credential required')

    async def secure_headers(_request, response):
        response.headers.update(SECURITY_HEADERS)

    app = web.Application(middlewares=[authenticate], client_max_size=4096)
    app.on_response_prepare.append(secure_headers)

    async def identity(_request):
        return web.json_response({'schema': 1, 'id': state.identity['id'], 'name': state.identity['name'],
                                  'capabilities': {'vehicle': True, 'system': True, 'video': True,
                                                   'video_hd': True, 'recordings': state.recordings.available(),
                                                   'lane': False,
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
                    if session_expired(request):
                        await ws.close(code=WSCloseCode.POLICY_VIOLATION, message=b'Session expired')
                        break
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
        quality = request.query.get('quality', 'low')
        if quality not in ('low', 'hd'):
            raise web.HTTPBadRequest(text='quality must be low or hd')
        hd = quality == 'hd'
        clients = state.video_hd_clients if hd else state.video_clients
        if len(clients) >= (MAX_HD_VIEWERS if hd else MAX_VIEWERS):
            raise web.HTTPServiceUnavailable(text='Maximum HD video viewers reached' if hd else
                                             'Maximum video viewers reached')
        interval, tick = (VIDEO_HD_MIN_INTERVAL, .01) if hd else (VIDEO_MIN_INTERVAL, .05)
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=4096, compress=False)
        clients.add(ws)  # reserved before the upgrade awaits, as for /v1/stream
        try:
            await ws.prepare(request)
            state.touch_demand(hd)  # start the conversion now, not at the next demand tick
            opened = time.monotonic()
            reader = asyncio.create_task(refuse_messages(ws))
            try:
                shown, sent_at = 0, -math.inf
                while not ws.closed and not reader.done():
                    if session_expired(request):
                        await ws.close(code=WSCloseCode.POLICY_VIOLATION, message=b'Session expired')
                        break
                    frame, now = (state.video_hd if hd else state.video), time.monotonic()
                    # Only the newest frame, never queued: a slow viewer skips frames.
                    if frame and frame['seq'] != shown and now - sent_at >= interval:
                        shown = frame['seq']
                        age = now - frame['capture_mono_s']
                        if age <= VIDEO_SEND_MAX_AGE:
                            sent_at = now
                            await asyncio.wait_for(ws.send_bytes(video_message(frame, age)), timeout=2)
                    if hd and sent_at < 0 and now - opened >= HD_FIRST_FRAME_SECONDS:
                        await ws.close(code=HD_UNAVAILABLE, message=b'HD not offered')
                        break
                    await asyncio.sleep(tick)
            finally:
                reader.cancel()
                await asyncio.gather(reader, return_exceptions=True)
        except (ConnectionError, asyncio.TimeoutError):
            pass
        finally:
            clients.discard(ws)
            if ws.prepared:
                await ws.close()
        return ws

    async def recordings(_request):
        if not state.recordings.available():
            raise web.HTTPNotFound(text='No recordings on this Jetson')
        return web.json_response({'schema': 1, 'segments': await asyncio.to_thread(state.recordings.segments)},
                                 dumps=encode)

    async def recording_file(request):
        name = request.match_info['name']
        path, status = await asyncio.to_thread(state.recordings.resolve, name)
        if status == 409:
            raise web.HTTPConflict(text='Segment still recording')
        if path is None:
            raise web.HTTPNotFound(text='No such segment')
        headers = {'Content-Type': RECORDING_TYPES[name.rsplit('.', 1)[1]]}
        if request.query.get('download') == '1':
            headers['Content-Disposition'] = f'attachment; filename="{name}"'
        return web.FileResponse(path, headers=headers)   # Range, If-Range and HEAD included

    async def shutdown(_app):
        everyone = state.clients | state.video_clients | state.video_hd_clients
        await asyncio.gather(*(ws.close(code=1001) for ws in tuple(everyone)), return_exceptions=True)

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
    app.router.add_get('/v1/recordings', recordings)
    app.router.add_get(r'/v1/recordings/{name:[A-Za-z0-9_]+\.(?:mkv|ass)}', recording_file)
    app.on_shutdown.append(shutdown)
    if state.video_demand or state.video_hd_demand:
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
    while True:  # 30 fps, the preview every third frame, HD (the same card, chunked) while watched
        seq = (seq + 1) & 0xFFFFFFFF
        capture_us = int(time.monotonic() * 1e6)
        if seq % 3 == 1:
            state.accept_video(VIDEO_HEADER.pack(VIDEO_MAGIC, VIDEO_HEADER.size, capture_us, seq, *size) + card)
        if state.video_hd_clients:
            for chunk in hd_chunks(card, capture_us, seq, *size):
                state.accept_hd_chunk(chunk)
        await asyncio.sleep(1 / 30)


def hd_chunks(jpeg, capture_us, seq, width, height, chunk_bytes=61440):
    """The DCH1 datagrams v0.4 sends for one original frame (MonitorHdSender)."""
    return [HD_HEADER.pack(HD_MAGIC, HD_HEADER.size, capture_us, seq, width, height, len(jpeg), offset) +
            jpeg[offset:offset + chunk_bytes] for offset in range(0, len(jpeg), chunk_bytes)]


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


async def advertise(address, port, identity):
    """Best-effort mDNS: returns (zeroconf, info), or None when it cannot start.

    mDNS only helps clients find the address; pairing and serving never depend on
    it. A missing address (the Jetson moved to another network, or DHCP has not
    assigned it yet) used to make zeroconf raise ENODEV and stop the whole agent.
    """
    from zeroconf import IPVersion, ServiceInfo
    from zeroconf.asyncio import AsyncZeroconf
    zc = None
    try:
        ip = ipaddress.IPv4Address(address)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind((str(ip), 0))   # EADDRNOTAVAIL before zeroconf opens (and leaks) its sockets
        info = ServiceInfo('_dashcam._tcp.local.', identity['id'] + '._dashcam._tcp.local.',
                           addresses=[ip.packed], port=port, properties={'id': identity['id'], 'schema': '1'},
                           server='dashcam-' + identity['id'][:8] + '.local.')
        zc = AsyncZeroconf(interfaces=[str(ip)], ip_version=IPVersion.V4Only)
        await zc.async_register_service(info)
        return zc, info
    except asyncio.CancelledError:
        if zc is not None:
            await zc.async_close()
        raise
    except Exception as error:  # noqa: BLE001 - any mDNS failure leaves the agent serving
        print(f'mDNS advertisement off ({type(error).__name__}: {error}); '
              f'is {address} assigned to this host? Manual pairing still works.', flush=True)
        if zc is not None:
            await asyncio.gather(zc.async_close(), return_exceptions=True)
        return None


async def maintain_advertisement(address, port, identity):
    delay = 2
    while True:
        advertisement = await advertise(address, port, identity)
        if advertisement:
            break
        await asyncio.sleep(delay)
        delay = min(delay * 2, 60)
    zc, info = advertisement
    try:
        await asyncio.Event().wait()
    finally:
        try:
            await zc.async_unregister_service(info)
        finally:
            await zc.async_close()


async def run(args, identity):
    state = State(identity, args.storage, video_demand=None if args.simulate else args.video_demand,
                  video_hd_demand=None if args.simulate else args.video_hd_demand,
                  recordings_dir=None if args.no_recordings else args.storage)
    endpoints = []
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
        app = create_app(state, port=args.port, web_dir=args.web_dir, session_days=args.browser_session_days)
        runner = web.AppRunner(app, access_log=None, keepalive_timeout=10,
                               shutdown_timeout=3)
        await runner.setup()
        try:
            await web.TCPSite(runner, args.bind, args.port, ssl_context=context).start()
            if args.advertise_address:
                state.tasks.append(asyncio.create_task(maintain_advertisement(args.advertise_address, args.port, identity)))
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
    p.add_argument('--video-hd-demand', type=Path,
                   help='file touched while HD video viewers watch (default: video-demand-hd next to --socket)')
    p.add_argument('--storage', default='/user/output/footage')
    p.add_argument('--no-recordings', action='store_true',
                   help='do not offer the recorded segments in --storage to paired clients')
    p.add_argument('--web-dir', type=Path, default=Path(__file__).parent / 'web')
    p.add_argument('--browser-session-days', type=int, default=30)
    p.add_argument('--advertise-address', help='Host LAN IPv4 for mDNS; omit for manual pairing only')
    p.add_argument('--simulate', action='store_true')
    args = p.parse_args()
    args.video_socket = args.video_socket or Path(args.socket).parent / 'video.sock'
    args.video_demand = args.video_demand or Path(args.socket).parent / 'video-demand'
    args.video_hd_demand = args.video_hd_demand or Path(args.socket).parent / 'video-demand-hd'
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
