import argparse
import asyncio
import base64
import contextlib
import copy
import datetime as dt
import hashlib
import hmac
import io
import ipaddress
import json
import logging
import os
from pathlib import Path
import socket
import ssl
import tempfile
import time
import unittest
from unittest import mock

from aiohttp import ClientSession, DummyCookieJar, Fingerprint, WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer
from cryptography import x509
from cryptography.hazmat.primitives import hashes
from cryptography.x509.oid import ExtendedKeyUsageOID
import service
from service import (SECURITY_HEADERS, State, create_app, encode, export_ca, initialize, issue_certificate,
                     load_identity, pair)
from health import Health


def source():
    return {'schema': 1, 'runtime': 'dashcam_v0_4', 'seq': 1,
            'source_mono_s': time.monotonic(), 'recording': False,
            'bridge_fresh': False, 'vehicle': {'speed_kmh': None},
            'adas': {'lane_available': False, 'driver_available': False}}


class StateTests(unittest.TestCase):
    def setUp(self):
        self.state = State({'id': 'test', 'name': 'test', 'token': 'x' * 32}, '.')

    def test_missing_and_expired_sources_clear_runtime(self):
        self.assertIsNone(self.state.snapshot()['runtime'])
        self.state.accept(encode(source()))
        self.assertTrue(self.state.snapshot()['runtime_fresh'])
        self.state.source['source_mono_s'] -= 3
        snapshot = self.state.snapshot()
        self.assertFalse(snapshot['runtime_fresh'])
        self.assertIsNone(snapshot['runtime'])

    def test_reject_bad_versions_nonfinite_future_and_oversize(self):
        bad = [b'garbage', b'{}', b'[]', b'x' * 16385]
        for key, value in [('schema', 2), ('source_mono_s', float('inf')),
                           ('source_mono_s', time.monotonic() + 3), ('recording', 'yes')]:
            item = source(); item[key] = value
            bad.append(json.dumps(item).encode())
        for data in bad:
            self.state.accept(data)
        self.assertEqual(self.state.invalid_packets, len(bad))
        self.assertIsNone(self.state.source)

    def test_queued_packet_never_refreshes_newer_sample(self):
        item = source(); self.state.accept(encode(item))
        previous = copy.deepcopy(self.state.source)
        item['source_mono_s'] -= .1
        item['vehicle']['speed_kmh'] = 900
        self.state.accept(encode(item))
        self.assertEqual(previous, self.state.source)

    def test_expired_health_is_not_republished_as_current(self):
        self.state.health = {'sample_mono_s': time.monotonic(), 'cpu_percent': 50}
        self.assertEqual(self.state.snapshot()['system']['cpu_percent'], 50)
        self.state.health['sample_mono_s'] -= 4
        self.assertEqual(self.state.snapshot()['system'], {})

    def test_cpu_delta_and_missing_sensors(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'stat').write_text('cpu 10 0 10 80 0 0 0 0\n')
            (root / 'meminfo').write_text('MemTotal: 100 kB\nMemAvailable: 25 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB\n')
            (root / 'uptime').write_text('1000 900')
            health = Health(root, proc=root, sys=root)
            self.assertIsNone(health.sample()['cpu_percent'])
            (root / 'stat').write_text('cpu 20 0 20 100 0 0 0 0\n')
            sample = health.sample()
            self.assertEqual(sample['cpu_percent'], 50)
            self.assertEqual(sample['ram_used_bytes'], 75 * 1024)
            self.assertIsNone(sample['gpu_percent'])
            self.assertEqual(sample['temperatures_c'], {})


class ApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.state = State({'id': 'test', 'name': 'test', 'token': 'x' * 32}, '.')
        self.client = TestClient(TestServer(create_app(self.state)))
        await self.client.start_server()
        self.headers = {'Authorization': 'Bearer ' + 'x' * 32}

    async def asyncTearDown(self):
        await self.client.close()

    async def test_all_routes_require_authentication(self):
        for path in ['/v1/identity', '/v1/snapshot', '/v1/stream']:
            response = await self.client.get(path)
            self.assertEqual(response.status, 401)
        with self.assertRaises(WSServerHandshakeError):
            await self.client.ws_connect('/v1/stream', headers={'Authorization': 'Bearer wrong'})

    async def test_non_utf8_authorization_is_401_not_500(self):
        host, port = self.client.server.host, self.client.server.port
        for path in [b'/v1/identity', b'/v1/snapshot']:
            for method in [b'GET', b'POST']:
                url = b'/v1/session' if method == b'POST' else path
                reader, writer = await asyncio.open_connection(host, port)
                writer.write(method + b' ' + url + b' HTTP/1.1\r\nHost: x\r\nAuthorization: Bearer \xff\xfe\r\n'
                             b'Content-Length: 0\r\nConnection: close\r\n\r\n')
                await writer.drain()
                status = await reader.readline()
                writer.close()
                self.assertEqual(status.split()[1], b'401', (method, url))

    async def test_identity_and_snapshot_do_not_expose_secrets(self):
        response = await self.client.get('/v1/identity', headers=self.headers)
        identity = await response.json()
        self.assertNotIn('token', identity)
        self.assertFalse(identity['capabilities']['controls'])
        response = await self.client.get('/v1/snapshot', headers=self.headers)
        self.assertIsNone((await response.json())['runtime'])

    async def test_two_clients_receive_live_then_expired_runtime(self):
        self.state.accept(encode(source()))
        first = await self.client.ws_connect('/v1/stream', headers=self.headers)
        second = await self.client.ws_connect('/v1/stream', headers=self.headers)
        for ws in [first, second]:
            self.assertTrue((await ws.receive_json(timeout=2))['runtime_fresh'])
        self.state.source['source_mono_s'] -= 3
        for ws in [first, second]:
            self.assertFalse((await ws.receive_json(timeout=2))['runtime_fresh'])
            await ws.close()

    async def test_commands_are_refused_and_client_limit_is_bounded(self):
        viewers = [await self.client.ws_connect('/v1/stream', headers=self.headers) for _ in range(4)]
        with self.assertRaises(WSServerHandshakeError) as failure:
            await self.client.ws_connect('/v1/stream', headers=self.headers)
        self.assertEqual(failure.exception.status, 503)
        await viewers[0].receive_json(timeout=2)
        await viewers[0].send_str('STOP RECORDING')
        for _ in range(4):
            message = await viewers[0].receive(timeout=2)
            if message.type.name in ('CLOSE', 'CLOSED'):
                break
        self.assertTrue(viewers[0].closed)
        for ws in viewers:
            await ws.close()

    async def test_tls_full_certificate_pin_and_wrong_pin(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            initialize(root, 'Test rig')
            identity = load_identity(root)
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(root / 'cert.pem', root / 'key.pem')
            server = TestServer(create_app(State(identity, '.')))
            await server.start_server(ssl=context)
            try:
                async with ClientSession() as client:
                    url = str(server.make_url('/v1/snapshot')).replace('http:', 'https:')
                    async with client.get(url, ssl=Fingerprint(bytes.fromhex(identity['pin'])),
                                          headers={'Authorization': 'Bearer ' + identity['token']}) as response:
                        self.assertEqual(response.status, 200)
                    with self.assertRaises(Exception):
                        await client.get(url, ssl=Fingerprint(b'0' * 32), headers=self.headers)
            finally:
                await server.close()


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def snapshot_files(root):
    return {p.name: p.read_bytes() for p in root.iterdir()}


class PkiTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / 'state'

    def tearDown(self):
        self.temp.cleanup()

    def certs(self):
        chain = x509.load_pem_x509_certificates((self.root / 'cert.pem').read_bytes())
        return chain, x509.load_pem_x509_certificate((self.root / 'ca.pem').read_bytes())

    def test_ca_and_leaf_profile(self):
        initialize(self.root, 'Rig', ['192.168.0.15', 'Jetson.LAN', 'FD00::0005'])
        identity = load_identity(self.root)
        (leaf, chained_ca), ca = self.certs()
        self.assertEqual(chained_ca, ca)
        self.assertEqual(identity['pin'], leaf.fingerprint(hashes.SHA256()).hex())
        self.assertEqual(identity['ca_fingerprint'], ca.fingerprint(hashes.SHA256()).hex())
        self.assertEqual(identity['sans'], ['192.168.0.15', 'jetson.lan', 'fd00::5'])
        for name, mode in [('ca-key.pem', 0o600), ('key.pem', 0o600), ('identity.json', 0o600)]:
            self.assertEqual((self.root / name).stat().st_mode & 0o777, mode)
        self.assertIn(b'PRIVATE KEY', (self.root / 'ca-key.pem').read_bytes())
        self.assertNotIn(b'PRIVATE', (self.root / 'ca.pem').read_bytes() + (self.root / 'cert.pem').read_bytes())

        ext = ca.extensions
        self.assertEqual(ca.subject, ca.issuer)
        self.assertEqual(ca.subject.rfc4514_string(),
                         f"O=Dashcam Connect,CN=Dashcam Connect local CA {identity['id'][:8]}")
        bc = ext.get_extension_for_class(x509.BasicConstraints)
        self.assertTrue(bc.critical and bc.value.ca)
        self.assertEqual(bc.value.path_length, 0)
        ku = ext.get_extension_for_class(x509.KeyUsage)
        self.assertTrue(ku.critical and ku.value.key_cert_sign and ku.value.crl_sign)
        self.assertFalse(ku.value.digital_signature or ku.value.key_encipherment or ku.value.key_agreement)
        ext.get_extension_for_class(x509.SubjectKeyIdentifier)
        nc = ext.get_extension_for_class(x509.NameConstraints)
        self.assertTrue(nc.critical)
        self.assertIsNone(nc.value.excluded_subtrees)
        self.assertEqual({str(n.value) for n in nc.value.permitted_subtrees if isinstance(n, x509.IPAddress)},
                         {'10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16', '127.0.0.0/8', '169.254.0.0/16',
                          '::1/128', 'fd00::/8', 'fe80::/10'})
        self.assertEqual([n.value for n in nc.value.permitted_subtrees if isinstance(n, x509.DNSName)],
                         ['local', 'lan', 'home.arpa', 'localhost'])
        self.assertEqual((ca.not_valid_after_utc - ca.not_valid_before_utc).days, 3651)

        ext = leaf.extensions
        self.assertEqual(leaf.issuer, ca.subject)
        self.assertEqual(leaf.subject.rfc4514_string(), 'CN=jetson.lan')
        self.assertEqual(leaf.not_valid_after_utc - leaf.not_valid_before_utc, dt.timedelta(days=825))
        self.assertLessEqual(leaf.not_valid_after_utc, dt.datetime.now(dt.timezone.utc) + dt.timedelta(days=825))
        bc = ext.get_extension_for_class(x509.BasicConstraints)
        self.assertTrue(bc.critical)
        self.assertFalse(bc.value.ca)
        ku = ext.get_extension_for_class(x509.KeyUsage)
        self.assertTrue(ku.critical and ku.value.digital_signature)
        self.assertFalse(ku.value.key_cert_sign or ku.value.crl_sign or ku.value.key_encipherment)
        self.assertEqual(list(ext.get_extension_for_class(x509.ExtendedKeyUsage).value),
                         [ExtendedKeyUsageOID.SERVER_AUTH])
        san = ext.get_extension_for_class(x509.SubjectAlternativeName).value
        self.assertEqual(san.get_values_for_type(x509.IPAddress),
                         [ipaddress.ip_address('192.168.0.15'), ipaddress.ip_address('fd00::5')])
        self.assertEqual(san.get_values_for_type(x509.DNSName), ['jetson.lan'])
        self.assertEqual(ext.get_extension_for_class(x509.AuthorityKeyIdentifier).value.key_identifier,
                         ca.extensions.get_extension_for_class(x509.SubjectKeyIdentifier).value.digest)
        ext.get_extension_for_class(x509.SubjectKeyIdentifier)
        leaf.verify_directly_issued_by(ca)

    def test_init_without_san_has_no_san_and_ip_only_cn(self):
        initialize(self.root, 'Rig')
        identity = load_identity(self.root)
        (leaf, _), _ca = self.certs()
        self.assertEqual(identity['sans'], [])
        self.assertEqual(leaf.subject.rfc4514_string(), 'CN=Dashcam Monitor')
        with self.assertRaises(x509.ExtensionNotFound):
            leaf.extensions.get_extension_for_class(x509.SubjectAlternativeName)

    def test_long_dns_san_falls_back_to_default_cn(self):
        name = 'dashcam-front-garage-jetson-orin-nano-recording-unit-01.home.arpa'
        self.assertEqual(len(name), 65)
        initialize(self.root, 'Rig', ['192.168.0.15', name, 'jetson.lan'])
        (leaf, _), _ca = self.certs()
        self.assertEqual(leaf.subject.rfc4514_string(), 'CN=jetson.lan')
        issue_certificate(self.root, [name])
        (leaf, _), _ca = self.certs()
        self.assertEqual(leaf.subject.rfc4514_string(), 'CN=Dashcam Monitor')
        self.assertEqual(load_identity(self.root)['sans'], [name])

    def test_san_validation_rejects_and_modifies_nothing(self):
        bad = ['*.lan', 'https://jetson.lan', 'jetson.lan:8443', 'jetson.lan/x', '-bad.lan', 'bad-.lan',
               'a..lan', 'jetson.lan.', '', 'x' * 64 + '.lan', ('a' * 63 + '.') * 4 + 'lan', 'under_score.lan',
               '8.8.8.8', '2001:db8::1', 'fe80::1%eth0', '::ffff:192.168.0.1']
        for value in bad:
            with self.assertRaises(ValueError, msg=value):
                initialize(self.root, 'Rig', [value])
            self.assertFalse((self.root / 'identity.json').exists())
        initialize(self.root, 'Rig', ['10.1.2.3'])
        before = snapshot_files(self.root)
        # The CA permits only private names/addresses, whatever SANs it was created with.
        for value in bad + ['jetson.example.com', 'example.com']:
            with self.assertRaises(ValueError) as failure:
                issue_certificate(self.root, ['192.168.0.15', value])
            if value:
                self.assertIn(repr(value), str(failure.exception))
            self.assertEqual(snapshot_files(self.root), before)
        with self.assertRaises(ValueError):
            issue_certificate(self.root, [])

    def test_cert_keeps_identity_changes_pin_and_upgrades_old_state(self):
        initialize(self.root, 'Rig', ['192.168.0.15'])
        before = load_identity(self.root)
        ca_before = (self.root / 'ca.pem').read_bytes()
        after = issue_certificate(self.root, ['192.168.0.16', 'dashcam.home.arpa'])
        self.assertEqual((after['id'], after['token'], after['name']), (before['id'], before['token'], before['name']))
        self.assertNotEqual(after['pin'], before['pin'])
        self.assertEqual(after['sans'], ['192.168.0.16', 'dashcam.home.arpa'])
        self.assertEqual((self.root / 'ca.pem').read_bytes(), ca_before)
        self.assertEqual((self.root / 'key.pem').stat().st_mode & 0o777, 0o600)
        self.assertEqual((self.root / 'cert.pem').stat().st_mode & 0o777, 0o644)
        self.assertEqual(sorted(p.name for p in self.root.iterdir()),
                         ['ca-key.pem', 'ca.pem', 'cert.pem', 'identity.json', 'key.pem'])
        # An old (pre-CA) state dir: self-signed leaf, no CA files.
        (self.root / 'ca.pem').unlink(); (self.root / 'ca-key.pem').unlink()
        old = load_identity(self.root)
        self.assertIsNone(old['ca_fingerprint'])
        with self.assertRaises(ValueError):
            issue_certificate(self.root, ['192.168.0.16', 'jetson.example.org'])
        self.assertIsNone(load_identity(self.root)['ca_fingerprint'])
        upgraded = issue_certificate(self.root, ['192.168.0.16', 'jetson.home.arpa'])
        self.assertEqual(upgraded['token'], before['token'])
        self.assertIsNotNone(upgraded['ca_fingerprint'])
        (leaf, _), ca = self.certs()
        leaf.verify_directly_issued_by(ca)
        # The new CA never permits a public name, not even one given at its creation.
        issue_certificate(self.root, ['www.jetson.home.arpa'])
        for public in ['example.org', '192.168.000.015', 'com']:
            with self.assertRaises(ValueError):
                issue_certificate(self.root, [public])
        # An incomplete CA is never silently replaced.
        (self.root / 'ca.pem').unlink()
        with self.assertRaises(ValueError):
            issue_certificate(self.root, ['192.168.0.16'])

    def test_interrupted_upgrade_is_repairable_and_failed_write_changes_nothing(self):
        initialize(self.root, 'Rig', ['192.168.0.15'])
        token = load_identity(self.root)['token']
        # A failed write before the renames leaves every file and no temp file behind.
        before = snapshot_files(self.root)
        calls = []
        real = os.fsync
        def failing(fd):
            calls.append(fd)
            if len(calls) == 2:
                raise OSError('disk full')
            real(fd)
        with mock.patch.object(service.os, 'fsync', failing), self.assertRaises(OSError):
            issue_certificate(self.root, ['192.168.0.16'])
        self.assertEqual(snapshot_files(self.root), before)
        # Upgrade interrupted after ca-key.pem: that key never signed the served (old) leaf.
        ca_key = (self.root / 'ca-key.pem').read_bytes()
        (self.root / 'ca.pem').unlink(); (self.root / 'ca-key.pem').unlink()
        issue_certificate(self.root, ['192.168.0.17'])   # the old CA is now gone
        leftover = (self.root / 'ca-key.pem').read_bytes()
        (self.root / 'ca.pem').unlink()
        (self.root / 'ca-key.pem').write_bytes(ca_key)   # does not sign the current leaf
        os.chmod(self.root / 'ca-key.pem', 0o600)
        upgraded = issue_certificate(self.root, ['192.168.0.18'])
        self.assertEqual(upgraded['token'], token)
        self.assertNotEqual((self.root / 'ca-key.pem').read_bytes(), ca_key)
        self.assertNotEqual((self.root / 'ca-key.pem').read_bytes(), leftover)
        (leaf, _), ca = self.certs()
        leaf.verify_directly_issued_by(ca)
        self.assertEqual(sorted(p.name for p in self.root.iterdir()),
                         ['ca-key.pem', 'ca.pem', 'cert.pem', 'identity.json', 'key.pem'])

    def test_pair_warning_keeps_existing_sans(self):
        initialize(self.root, 'Rig', ['192.168.0.15', 'jetson.lan'])
        out = io.StringIO()
        args = argparse.Namespace(host='Jetson.Local', port=8443, state_dir=self.root)
        with contextlib.redirect_stdout(out):
            pair(args, load_identity(self.root))
        self.assertIn('Run cert --san 192.168.0.15 --san jetson.lan --san jetson.local ', out.getvalue())
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            pair(argparse.Namespace(host='192.168.0.15', port=8443, state_dir=self.root), load_identity(self.root))
        self.assertNotIn('WARNING', out.getvalue())

    def test_ca_export_refuses_overwrite_and_has_no_key(self):
        initialize(self.root, 'Rig', ['192.168.0.15'])
        out = Path(self.temp.name) / 'dashcam-ca.crt'
        fingerprint = export_ca(self.root, out)
        data = out.read_bytes()
        self.assertNotIn(b'PRIVATE', data)
        self.assertEqual(data.count(b'BEGIN CERTIFICATE'), 1)
        ca = x509.load_pem_x509_certificate(data)
        self.assertEqual(fingerprint, ':'.join(f'{b:02X}' for b in ca.fingerprint(hashes.SHA256())))
        self.assertEqual(fingerprint.replace(':', '').lower(), load_identity(self.root)['ca_fingerprint'])
        out.write_bytes(b'keep')
        with self.assertRaises(ValueError):
            export_ca(self.root, out)
        self.assertEqual(out.read_bytes(), b'keep')

    def test_load_identity_rejects_readable_ca_key(self):
        initialize(self.root, 'Rig', ['192.168.0.15'])
        os.chmod(self.root / 'ca-key.pem', 0o640)
        with self.assertRaisesRegex(ValueError, 'ca-key.pem'):
            load_identity(self.root)
        os.chmod(self.root / 'ca-key.pem', 0o600)
        load_identity(self.root)


class TlsChainTests(unittest.IsolatedAsyncioTestCase):
    async def test_chain_verifies_against_ca_by_ip_san(self):
        # Rejected handshakes are logged by asyncio on the server side; that is expected here.
        logger = logging.getLogger('asyncio')
        level, logger.level = logger.level, logging.CRITICAL
        self.addCleanup(setattr, logger, 'level', level)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            initialize(root, 'Rig', ['127.0.0.1'])
            identity = load_identity(root)
            server_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            server_context.load_cert_chain(root / 'cert.pem', root / 'key.pem')
            port = free_port()
            server = TestServer(create_app(State(identity, '.'), port=port), host='127.0.0.1', port=port)
            await server.start_server(ssl=server_context)
            try:
                client_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
                client_context.load_verify_locations(root / 'ca.pem')
                self.assertTrue(client_context.check_hostname)
                async with ClientSession() as client:
                    async with client.get(f'https://127.0.0.1:{port}/v1/identity', ssl=client_context,
                                          headers={'Authorization': 'Bearer ' + identity['token']}) as response:
                        self.assertEqual(response.status, 200)
                    # The Android pin (leaf) still matches the served leaf.
                    async with client.get(f'https://127.0.0.1:{port}/v1/snapshot',
                                          ssl=Fingerprint(bytes.fromhex(identity['pin'])),
                                          headers={'Authorization': 'Bearer ' + identity['token']}) as response:
                        self.assertEqual(response.status, 200)
                reader, writer = await asyncio.open_connection('127.0.0.1', port, ssl=client_context,
                                                               server_hostname='127.0.0.1')
                writer.close()
                for wrong in ['127.0.0.2', 'jetson.lan']:
                    with self.assertRaises(ssl.SSLCertVerificationError):
                        await asyncio.open_connection('127.0.0.1', port, ssl=client_context, server_hostname=wrong)
                other = root / 'other'
                initialize(other, 'Other', ['127.0.0.1'])
                foreign = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
                foreign.load_verify_locations(other / 'ca.pem')
                with self.assertRaises(ssl.SSLCertVerificationError):
                    await asyncio.open_connection('127.0.0.1', port, ssl=foreign, server_hostname='127.0.0.1')
            finally:
                await server.close()


TOKEN = 'x' * 32
COOKIE = '__Host-dcsession'


def forge(token, expiry, nonce=b'n' * 16):
    b64 = lambda data: base64.urlsafe_b64encode(data).rstrip(b'=').decode()
    key = hmac.new(token.encode(), b'dashcam-browser-session-v1', hashlib.sha256).digest()
    body = f'v1.{int(expiry)}.{b64(nonce)}'
    return body + '.' + b64(hmac.new(key, body.encode(), hashlib.sha256).digest())


class BrowserTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.web = Path(self.temp.name) / 'web'
        self.web.mkdir()
        self.files = {'index.html': b'<!doctype html><title>Dashcam Connect</title>',
                      'app.js': b'"use strict";', 'app.css': b'body{}', 'icon.svg': b'<svg/>'}
        for name, content in self.files.items():
            (self.web / name).write_bytes(content)
        (self.web / 'secret.txt').write_bytes(b'secret')
        (Path(self.temp.name) / 'outside.txt').write_bytes(b'outside')
        self.state = State({'id': 'test', 'name': 'test', 'token': TOKEN, 'sans': ['127.0.0.1']}, '.')
        port = free_port()
        self.host = f'127.0.0.1:{port}'
        self.origin = 'https://' + self.host
        self.client = TestClient(TestServer(create_app(self.state, port=port, web_dir=self.web, session_days=7),
                                            host='127.0.0.1', port=port), cookie_jar=DummyCookieJar())
        await self.client.start_server()
        self.bearer = {'Authorization': 'Bearer ' + TOKEN}

    async def asyncTearDown(self):
        await self.client.close()
        self.temp.cleanup()

    def assertSecure(self, response):
        for name, value in SECURITY_HEADERS.items():
            self.assertEqual(response.headers.get(name), value, name)
        self.assertFalse([h for h in response.headers if h.lower().startswith('access-control-')])

    async def login(self):
        response = await self.client.post('/v1/session', headers={**self.bearer, 'Origin': self.origin})
        self.assertEqual(response.status, 204)
        return response.headers['Set-Cookie'].split(';')[0].split('=', 1)[1]

    def cookie(self, value, **extra):
        return {'Cookie': f'{COOKIE}={value}', **extra}

    async def test_static_whitelist_without_auth(self):
        for path, name, content_type in [('/', 'index.html', 'text/html; charset=utf-8'),
                                         ('/index.html', 'index.html', 'text/html; charset=utf-8'),
                                         ('/app.js', 'app.js', 'text/javascript; charset=utf-8'),
                                         ('/app.css', 'app.css', 'text/css; charset=utf-8'),
                                         ('/icon.svg', 'icon.svg', 'image/svg+xml')]:
            response = await self.client.get(path)
            self.assertEqual(response.status, 200, path)
            self.assertEqual(response.headers['Content-Type'], content_type)
            self.assertEqual(await response.read(), self.files[name])
            self.assertSecure(response)
            self.assertNotIn('Set-Cookie', response.headers)
        for path in ['/secret.txt', '/web/', '/../outside.txt', '/%2e%2e/outside.txt', '/app.js/',
                     '/%2E%2E%2Foutside.txt', '/..%2Foutside.txt', '/index.htm', '/v1/']:
            response = await self.client.get(path)
            self.assertIn(response.status, (401, 404), path)
            self.assertNotIn(b'secret', await response.read())
            self.assertSecure(response)
        response = await self.client.get('/secret.txt', headers=self.bearer)
        self.assertEqual(response.status, 404)
        self.assertSecure(response)
        (self.web / 'app.css').unlink()
        response = await self.client.get('/app.css')
        self.assertEqual(response.status, 404)
        self.assertSecure(response)

    async def test_security_headers_on_errors_and_api(self):
        response = await self.client.get('/v1/identity')
        self.assertEqual(response.status, 401)
        self.assertSecure(response)
        response = await self.client.get('/v1/identity', headers=self.bearer)
        self.assertEqual(response.status, 200)
        self.assertSecure(response)
        response = await self.client.post('/v1/session', headers={**self.bearer, 'Origin': 'https://evil.test'})
        self.assertEqual(response.status, 403)
        self.assertSecure(response)
        response = await self.client.get('/nope', headers=self.bearer)
        self.assertEqual(response.status, 404)
        self.assertSecure(response)
        response = await self.client.options('/v1/identity', headers={'Origin': self.origin,
                                                                      'Access-Control-Request-Method': 'GET'})
        self.assertSecure(response)

    async def test_create_session_needs_bearer_and_same_origin(self):
        for headers in [{'Origin': self.origin}, {'Origin': self.origin, 'Authorization': 'Bearer wrong'}]:
            response = await self.client.post('/v1/session', headers=headers)
            self.assertEqual(response.status, 401)
            self.assertNotIn('Set-Cookie', response.headers)
        for headers in [{}, {'Origin': 'https://evil.test'}, {'Origin': 'http://' + self.host},
                        {'Origin': 'null'}, {'Origin': self.origin, 'Host': 'evil.test:' + self.host.split(':')[1]},
                        {'Origin': 'https://localhost:' + self.host.split(':')[1],
                         'Host': 'localhost:' + self.host.split(':')[1]}]:
            response = await self.client.post('/v1/session', headers={**self.bearer, **headers})
            self.assertEqual(response.status, 403, headers)
            self.assertNotIn('Set-Cookie', response.headers)
        response = await self.client.post('/v1/session', data=b'ignored',
                                          headers={**self.bearer, 'Origin': self.origin.upper().replace('HTTPS', 'https')})
        self.assertEqual(response.status, 204)
        value, *attributes = response.headers['Set-Cookie'].split('; ')
        self.assertEqual(attributes, ['Path=/', 'Secure', 'HttpOnly', 'SameSite=Strict', f'Max-Age={7 * 86400}'])
        name, cookie = value.split('=', 1)
        self.assertEqual(name, COOKIE)
        version, expiry, nonce, mac = cookie.split('.')
        self.assertEqual(version, 'v1')
        self.assertAlmostEqual(int(expiry), time.time() + 7 * 86400, delta=5)
        self.assertEqual(len(base64.urlsafe_b64decode(nonce + '==')), 16)
        self.assertEqual(forge(TOKEN, int(expiry), base64.urlsafe_b64decode(nonce + '==')), cookie)
        self.assertNotIn(TOKEN, cookie)
        self.assertNotEqual(cookie, await self.login())

    async def test_cookie_authorises_routes_with_same_origin(self):
        value = await self.login()
        for headers in [self.cookie(value), self.cookie(value, Origin=self.origin)]:
            response = await self.client.get('/v1/identity', headers=headers)
            self.assertEqual(response.status, 200)
            self.assertEqual((await response.json())['id'], 'test')
            response = await self.client.get('/v1/snapshot', headers=headers)
            self.assertEqual(response.status, 200)
            self.assertEqual((await response.json())['device_id'], 'test')
        response = await self.client.get('/v1/unknown', headers=self.cookie(value))
        self.assertEqual(response.status, 404)
        ws = await self.client.ws_connect('/v1/stream', headers=self.cookie(value), origin=self.origin)
        self.assertEqual((await ws.receive_json(timeout=2))['type'], 'snapshot')
        await ws.close()

    async def test_cookie_rejected_on_foreign_origin_or_host(self):
        value = await self.login()
        port = self.host.split(':')[1]
        for path in ['/v1/identity', '/v1/snapshot']:
            for headers in [{'Origin': 'https://evil.test'}, {'Origin': 'null'},
                            {'Host': 'evil.test:' + port}, {'Host': '127.0.0.1:1'},
                            {'Host': 'localhost:' + port, 'Origin': 'https://localhost:' + port}]:
                response = await self.client.get(path, headers=self.cookie(value, **headers))
                self.assertEqual(response.status, 403, (path, headers))
                self.assertSecure(response)
        for kwargs in [{}, {'origin': 'https://evil.test'}, {'origin': 'http://' + self.host}]:
            with self.assertRaises(WSServerHandshakeError) as failure:
                await self.client.ws_connect('/v1/stream', headers=self.cookie(value), **kwargs)
            self.assertEqual(failure.exception.status, 403, kwargs)
        with self.assertRaises(WSServerHandshakeError) as failure:
            await self.client.ws_connect('/v1/stream', headers=self.cookie(value, Host='evil.test:' + port),
                                         origin='https://evil.test:' + port)
        self.assertEqual(failure.exception.status, 403)
        self.assertEqual(self.state.clients, set())

    async def test_bad_cookies_rejected(self):
        value = await self.login()
        version, expiry, nonce, mac = value.split('.')
        flipped = mac[:-2] + ('A' if mac[-2] != 'A' else 'B') + mac[-1]
        bad = [version + '.' + expiry + '.' + nonce + '.' + flipped,
               f'v1.{int(expiry) + 1}.{nonce}.{mac}',
               f'v2.{expiry}.{nonce}.{mac}', value + '.x', value[:-1], '', 'garbage', 'v1...',
               forge(TOKEN, time.time() - 1), forge('y' * 32, time.time() + 3600),
               forge(TOKEN, time.time() + 3600).replace('v1.', 'v1.+', 1)]
        for cookie in bad:
            for path in ['/v1/identity', '/v1/snapshot']:
                response = await self.client.get(path, headers=self.cookie(cookie, Origin=self.origin))
                self.assertEqual(response.status, 401, cookie)
            with self.assertRaises(WSServerHandshakeError) as failure:
                await self.client.ws_connect('/v1/stream', headers=self.cookie(cookie), origin=self.origin)
            self.assertEqual(failure.exception.status, 401)
        response = await self.client.get('/v1/identity', headers=self.cookie(forge(TOKEN, time.time() + 60)))
        self.assertEqual(response.status, 200)

    async def test_delete_session_clears_cookie_and_needs_same_origin(self):
        for headers in [{}, {'Origin': 'https://evil.test'}, {'Origin': self.origin, 'Host': 'evil.test'}]:
            response = await self.client.delete('/v1/session', headers={**self.bearer, **headers})
            self.assertEqual(response.status, 403)
            self.assertNotIn('Set-Cookie', response.headers)
            self.assertSecure(response)
        response = await self.client.delete('/v1/session', headers={'Origin': self.origin})
        self.assertEqual(response.status, 204)
        self.assertEqual(response.headers['Set-Cookie'],
                         f'{COOKIE}=; Path=/; Secure; HttpOnly; SameSite=Strict; Max-Age=0')
        self.assertSecure(response)

    async def test_android_bearer_without_origin_still_works(self):
        for headers in [self.bearer, {**self.bearer, 'Host': '192.168.0.15:8443'}]:
            for path in ['/v1/identity', '/v1/snapshot']:
                response = await self.client.get(path, headers=headers)
                self.assertEqual(response.status, 200)
            ws = await self.client.ws_connect('/v1/stream', headers=headers)
            self.assertEqual((await ws.receive_json(timeout=2))['device_id'], 'test')
            await ws.close()

    async def test_viewer_limit_is_shared(self):
        value = await self.login()
        viewers = [await self.client.ws_connect('/v1/stream', headers=self.bearer) for _ in range(2)]
        viewers += [await self.client.ws_connect('/v1/stream', headers=self.cookie(value), origin=self.origin)
                    for _ in range(2)]
        for kwargs in [{'headers': self.bearer}, {'headers': self.cookie(value), 'origin': self.origin}]:
            with self.assertRaises(WSServerHandshakeError) as failure:
                await self.client.ws_connect('/v1/stream', **kwargs)
            self.assertEqual(failure.exception.status, 503)
        await viewers[0].close()
        for _ in range(50):
            if len(self.state.clients) < 4:
                break
            await asyncio.sleep(.05)
        replacement = await self.client.ws_connect('/v1/stream', headers=self.cookie(value), origin=self.origin)
        for ws in viewers + [replacement]:
            await ws.close()


if __name__ == '__main__':
    unittest.main()
