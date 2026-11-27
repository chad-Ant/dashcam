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
import shutil
import socket
import ssl
import stat
import struct
import subprocess
import tempfile
import time
import unittest
from unittest import mock

from aiohttp import ClientSession, DummyCookieJar, Fingerprint, WSMsgType, WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer
from cryptography import x509
from cryptography.hazmat.primitives import hashes
from cryptography.x509.oid import ExtendedKeyUsageOID
import service
from service import (SECURITY_HEADERS, State, VideoReceiver, bind_datagram, create_app, encode, export_ca,
                     initialize, issue_certificate, jpeg_size, load_identity, pair, simulate_video, video_message)
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
        for path in ['/v1/identity', '/v1/snapshot', '/v1/stream', '/v1/video']:
            response = await self.client.get(path)
            self.assertEqual(response.status, 401)
        for path in ['/v1/stream', '/v1/video']:
            for headers in [{}, {'Authorization': 'Bearer wrong'}]:
                with self.assertRaises(WSServerHandshakeError) as failure:
                    await self.client.ws_connect(path, headers=headers)
                self.assertEqual(failure.exception.status, 401)

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
        self.state.accept_video(video_frame())
        ws = await self.client.ws_connect('/v1/video', headers=self.cookie(value), origin=self.origin)
        self.assertEqual((await ws.receive_bytes(timeout=2))[24:], JPEG)
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
        for path in ['/v1/stream', '/v1/video']:
            for kwargs in [{}, {'origin': 'https://evil.test'}, {'origin': 'http://' + self.host}]:
                with self.assertRaises(WSServerHandshakeError) as failure:
                    await self.client.ws_connect(path, headers=self.cookie(value), **kwargs)
                self.assertEqual(failure.exception.status, 403, (path, kwargs))
            with self.assertRaises(WSServerHandshakeError) as failure:
                await self.client.ws_connect(path, headers=self.cookie(value, Host='evil.test:' + port),
                                             origin='https://evil.test:' + port)
            self.assertEqual(failure.exception.status, 403, path)
        self.assertEqual(self.state.clients, set())
        self.assertEqual(self.state.video_clients, set())

    async def test_open_browser_sockets_close_when_the_session_expires(self):
        # The cookie is checked at the handshake; an open socket must not outlive it.
        cookie = forge(TOKEN, int(time.time()) + 2)
        self.state.accept_video(video_frame())
        stream = await self.client.ws_connect('/v1/stream', headers=self.cookie(cookie), origin=self.origin)
        video = await self.client.ws_connect('/v1/video', headers=self.cookie(cookie), origin=self.origin)
        android = await self.client.ws_connect('/v1/stream', headers=self.bearer)
        self.assertEqual((await stream.receive_json(timeout=2))['type'], 'snapshot')

        async def drain_until_closed(ws):   # read like a browser: continuously, both sockets at once
            deadline = time.monotonic() + 5
            while True:
                message = await ws.receive(timeout=max(.1, deadline - time.monotonic()))
                if message.type in (WSMsgType.CLOSE, WSMsgType.CLOSED, WSMsgType.CLOSING):
                    return ws.close_code
                self.assertLess(time.monotonic(), deadline, 'socket outlived its session')

        self.assertEqual(await asyncio.gather(drain_until_closed(stream), drain_until_closed(video)), [1008, 1008])
        # A bearer (Android) socket has no session expiry and keeps streaming.
        while (await android.receive(timeout=2)).type == WSMsgType.TEXT and not android.closed:
            if time.time() > int(cookie.split('.')[1]) + .5:
                break
        self.assertFalse(android.closed)
        await android.close()

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
            for path in ['/v1/stream', '/v1/video']:
                with self.assertRaises(WSServerHandshakeError) as failure:
                    await self.client.ws_connect(path, headers=self.cookie(cookie), origin=self.origin)
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
            self.state.accept_video(video_frame(seq=self.state.video_seq + 1))
            ws = await self.client.ws_connect('/v1/video', headers=headers)
            self.assertEqual((await ws.receive_bytes(timeout=2))[24:], JPEG)
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


JPEG = b'\xff\xd8' + bytes(range(256)) * 3 + b'\xff\xd9'


def video_frame(seq=1, capture=None, width=640, height=360, jpeg=JPEG, header=24, magic=b'DCV1'):
    """A v0.4 datagram: header (+ padding when the header grows) + JPEG."""
    capture = time.monotonic() if capture is None else capture
    return (struct.pack('<4sIQIHH', magic, header, int(capture * 1e6), seq, width, height) +
            b'\0' * (header - 24) + jpeg)


def sof_jpeg(width, height):
    return (b'\xff\xd8\xff\xe0\x00\x04JF\xff\xc0\x00\x11\x08' + struct.pack('>HH', height, width) +
            b'\x03' + b'\x01\x22\x00' * 3 + b'\xff\xd9')


async def wait_for(condition, seconds=2.0):
    deadline = time.monotonic() + seconds
    while not condition():
        if time.monotonic() > deadline:
            raise AssertionError('condition not reached')
        await asyncio.sleep(.02)


class VideoStateTests(unittest.TestCase):
    def setUp(self):
        self.state = State({'id': 'test', 'name': 'test', 'token': TOKEN}, '.')

    def test_header_layouts_are_byte_exact(self):
        datagram = service.VIDEO_HEADER.pack(b'DCV1', 24, 0x0102030405060708, 0x0A0B0C0D, 0x0280, 0x0168) + JPEG
        self.assertEqual(datagram[:24], b'DCV1' + b'\x18\0\0\0' + bytes.fromhex('0807060504030201') +
                         bytes.fromhex('0D0C0B0A') + b'\x80\x02' + b'\x68\x01')
        self.assertEqual(service.VIDEO_HEADER.size, 24)
        frame = {'jpeg': JPEG, 'seq': 0x1122334455667788, 'width': 640, 'height': 360}
        message = video_message(frame, .1234)
        self.assertEqual(message[:24], b'DCV1' + b'\x18\0\0\0' + bytes.fromhex('8877665544332211') +
                         (123).to_bytes(4, 'little') + b'\x80\x02\x68\x01')
        self.assertEqual(message[24:], JPEG)
        self.assertEqual(video_message(frame, -1)[16:20], b'\0\0\0\0')

    def test_valid_frames_keep_only_the_latest(self):
        self.state.accept_video(video_frame(seq=1))
        second = b'\xff\xd8second\xff\xd9'
        self.state.accept_video(video_frame(seq=2, width=320, height=180, jpeg=second))
        self.assertEqual(self.state.invalid_video, 0)
        frame = self.state.video
        self.assertEqual((frame['jpeg'], frame['seq'], frame['source_seq'], frame['width'], frame['height']),
                         (second, 2, 2, 320, 180))
        self.assertAlmostEqual(frame['capture_mono_s'], time.monotonic(), delta=.5)
        self.assertEqual(set(frame), {'jpeg', 'seq', 'source_seq', 'capture_mono_s', 'width', 'height'})
        # A longer header is allowed: the JPEG starts at the stated offset.
        self.state.accept_video(video_frame(seq=3, header=32))
        self.assertEqual((self.state.video['jpeg'], self.state.video['seq']), (JPEG, 3))
        # Largest allowed datagram, and 1..4096 dimensions.
        biggest = b'\xff\xd8' + b'\0' * (service.MAX_VIDEO_JPEG - 4) + b'\xff\xd9'
        for seq, data in enumerate([video_frame(seq=4, jpeg=biggest), video_frame(seq=5, width=1, height=1),
                                    video_frame(seq=6, width=4096, height=4096)], 4):
            self.state.accept_video(data)
            self.assertEqual(self.state.video['source_seq'], seq)
        self.assertEqual(self.state.invalid_video, 0)

    def test_each_validation_rule(self):
        now = time.monotonic()
        too_big = b'\xff\xd8' + b'\0' * (service.MAX_VIDEO_JPEG - 3) + b'\xff\xd9'
        bad = {'short': video_frame()[:23], 'empty': b'', 'header only': video_frame()[:24],
               'magic': video_frame(magic=b'DCV2'), 'header 23': video_frame(header=23),
               'header past end': video_frame()[:4] + struct.pack('<I', 24 + len(JPEG) - 3) + video_frame()[8:],
               'header huge': video_frame()[:4] + b'\xff\xff\xff\xff' + video_frame()[8:],
               'oversize': video_frame(jpeg=too_big),
               'no SOI': video_frame(jpeg=b'\0\xd8' + JPEG[2:]), 'no EOI': video_frame(jpeg=JPEG[:-1] + b'\0'),
               'not jpeg': video_frame(jpeg=b'hello world'),
               'width 0': video_frame(width=0), 'width 4097': video_frame(width=4097),
               'height 0': video_frame(height=0), 'height 4097': video_frame(height=4097),
               'old': video_frame(capture=now - 2.5), 'future': video_frame(capture=now + .5),
               'far future': video_frame()[:8] + b'\xff' * 8 + video_frame()[16:]}
        for name, data in bad.items():
            before = self.state.invalid_video
            self.state.accept_video(data)
            self.assertEqual(self.state.invalid_video, before + 1, name)
        self.state.accept_video(memoryview(video_frame()))
        self.assertIsNotNone(self.state.video)
        self.assertEqual(self.state.invalid_video, len(bad))

    def test_sequence_and_new_producer(self):
        self.state.accept_video(video_frame(seq=10))
        for seq in (9, 10):  # reordered or repeated datagram of the same run: kept, not counted invalid
            self.state.accept_video(video_frame(seq=seq, jpeg=b'\xff\xd8old\xff\xd9'))
        self.assertEqual((self.state.video['source_seq'], self.state.video['seq'], self.state.video['jpeg']),
                         (10, 1, JPEG))
        self.assertEqual(self.state.invalid_video, 0)
        self.state.accept_video(video_frame(seq=11))
        self.assertEqual((self.state.video['source_seq'], self.state.video['seq']), (11, 2))
        # Capture time more than 5 s behind the current frame: another producer, accepted.
        self.state.video['capture_mono_s'] += 6
        self.state.accept_video(video_frame(seq=1))
        self.assertEqual((self.state.video['source_seq'], self.state.video['seq']), (1, 3))
        # Not 5 s behind: still the same producer.
        self.state.video['capture_mono_s'] += 4
        self.state.accept_video(video_frame(seq=1))
        self.assertEqual(self.state.video['seq'], 3)
        # A restarted v0.4 (sequence from 0, later capture time) once the old frame has expired.
        self.state.accept_video(video_frame(seq=500, capture=time.monotonic() - .1))
        self.state.accept_video(video_frame(seq=2))
        self.assertEqual(self.state.video['source_seq'], 500)
        self.state.video['capture_mono_s'] -= 2.5
        self.state.accept_video(video_frame(seq=0))
        self.assertEqual((self.state.video['source_seq'], self.state.video['seq']), (0, 5))
        self.assertEqual(self.state.invalid_video, 0)

    def test_demand_file_is_private_and_errors_are_logged_once(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'video-demand'
            self.assertIsNone(State({'id': 't'}, '.').video_demand)
            State({'id': 't'}, '.').touch_demand()  # no path: nothing to do
            state = State({'id': 't'}, '.', video_demand=path)
            state.touch_demand()
            info = path.stat()
            self.assertTrue(stat.S_ISREG(info.st_mode))
            self.assertEqual(info.st_mode & 0o777, 0o600)
            self.assertAlmostEqual(info.st_mtime, time.time(), delta=2)
            os.utime(path, (1, 1))
            state.touch_demand()
            self.assertAlmostEqual(path.stat().st_mtime, time.time(), delta=2)
            self.assertEqual(path.read_bytes(), b'')
            for bad in [Path(directory) / 'missing' / 'video-demand', Path(directory) / 'dir',
                        Path(directory) / 'link', Path(directory) / 'fifo']:
                (Path(directory) / 'dir').mkdir(exist_ok=True)
                if not os.path.lexists(Path(directory) / 'link'):
                    os.symlink(path, Path(directory) / 'link')
                    os.mkfifo(Path(directory) / 'fifo')
                state = State({'id': 't'}, '.', video_demand=bad)
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    state.touch_demand()  # never raises, never blocks (FIFO)
                    state.touch_demand()
                self.assertEqual(out.getvalue().count('WARNING'), 1, bad)
            self.assertTrue(stat.S_ISFIFO(os.lstat(Path(directory) / 'fifo').st_mode))

    def test_jpeg_size(self):
        self.assertEqual(jpeg_size(sof_jpeg(640, 360)), (640, 360))
        for data in [b'', b'\xff\xd8', JPEG, b'not a jpeg', sof_jpeg(0, 360)]:
            self.assertIsNone(jpeg_size(data))
        card = Path(service.__file__).parent / 'testcard.jpg'
        if card.exists():
            self.assertEqual(jpeg_size(card.read_bytes()), (640, 360))


HD_JPEG = b'\xff\xd8' + bytes(range(256)) * 700 + b'\xff\xd9'   # 179,204 bytes: three chunks


def hd_frame(seq=1, capture=None, width=1920, height=1080, jpeg=HD_JPEG):
    capture_us = int((time.monotonic() if capture is None else capture) * 1e6)
    return service.hd_chunks(jpeg, capture_us, seq, width, height)


class HdStateTests(unittest.TestCase):
    def setUp(self):
        self.state = State({'id': 'test', 'name': 'test', 'token': TOKEN}, '.')

    def test_chunk_header_is_byte_exact(self):
        chunk = service.HD_HEADER.pack(b'DCH1', 32, 0x0102030405060708, 0xA1B2C3D4, 1920, 1080, 0x030201, 0xF000)
        self.assertEqual(service.HD_HEADER.size, 32)
        self.assertEqual(chunk, b'DCH1' + b'\x20\0\0\0' + bytes.fromhex('0807060504030201') +
                         bytes.fromhex('D4C3B2A1') + b'\x80\x07\x38\x04' + bytes.fromhex('01020300') +
                         bytes.fromhex('00F00000'))
        chunks = hd_frame()
        self.assertEqual([len(c) - 32 for c in chunks], [61440, 61440, len(HD_JPEG) - 122880])

    def test_complete_frame_becomes_current_and_preview_is_untouched(self):
        chunks = hd_frame(seq=4)
        for chunk in chunks[:-1]:
            self.state.accept_hd_chunk(chunk)
            self.assertIsNone(self.state.video_hd)
        self.state.accept_hd_chunk(memoryview(chunks[-1]))
        frame = self.state.video_hd
        self.assertEqual((frame['jpeg'], frame['seq'], frame['source_seq'], frame['width'], frame['height']),
                         (HD_JPEG, 1, 4, 1920, 1080))
        self.assertIsNone(self.state.video)
        self.assertEqual((self.state.invalid_video, self.state.hd_incomplete), (0, 0))
        # A single-chunk frame, through the receiver as v0.4 would send it.
        small = b'\xff\xd8small\xff\xd9'
        for chunk in hd_frame(seq=5, width=1280, height=720, jpeg=small):
            VideoReceiver(self.state).datagram_received(chunk, None)
        self.assertEqual((self.state.video_hd['jpeg'], self.state.video_hd['seq']), (small, 2))
        VideoReceiver(self.state).datagram_received(video_frame(seq=1), None)  # DCV1 still goes to the preview
        self.assertEqual(self.state.video['jpeg'], JPEG)
        self.assertEqual(self.state.video_hd['seq'], 2)

    def test_missing_or_reordered_chunks_drop_only_that_frame(self):
        first, second = hd_frame(seq=1), hd_frame(seq=2)
        for chunk in (first[0], first[2]):  # a gap
            self.state.accept_hd_chunk(chunk)
        self.assertIsNone(self.state.video_hd)
        self.state.accept_hd_chunk(second[0])  # a new frame starts while one is incomplete
        self.state.accept_hd_chunk(first[1])   # a chunk of another frame
        self.assertIsNone(self.state.video_hd)
        for chunk in hd_frame(seq=3):
            self.state.accept_hd_chunk(chunk)
        self.assertEqual(self.state.video_hd['source_seq'], 3)
        self.assertEqual(self.state.hd_incomplete, 2)
        self.assertEqual(self.state.invalid_video, 0)
        for chunk in hd_frame(seq=2):  # older than the current frame of the same producer
            self.state.accept_hd_chunk(chunk)
        self.assertEqual(self.state.video_hd['source_seq'], 3)

    def test_each_validation_rule(self):
        now = time.monotonic()
        good = hd_frame()[0]
        pack = service.HD_HEADER.pack

        def chunk(**kw):
            fields = dict(magic=b'DCH1', header=32, capture=int(now * 1e6), seq=1, width=1920, height=1080,
                          total=len(HD_JPEG), offset=0)
            fields.update(kw)
            return pack(*fields.values()) + HD_JPEG[:100]
        bad = {'header only': good[:32], 'short': good[:31], 'magic': chunk(magic=b'DCHX'),
               'header 31': chunk(header=31), 'header past end': chunk(header=132),
               'total 3': chunk(total=3), 'total over 1 MiB': chunk(total=service.MAX_HD_JPEG + 1),
               'past total': chunk(total=50), 'width 0': chunk(width=0), 'height 4097': chunk(height=4097),
               'old': chunk(capture=int((now - 2.5) * 1e6)), 'future': chunk(capture=int((now + .5) * 1e6))}
        for name, data in bad.items():
            before = self.state.invalid_video
            self.state.accept_hd_chunk(data)
            self.assertEqual(self.state.invalid_video, before + 1, name)
        for data in hd_frame(jpeg=b'\0\xd8' + HD_JPEG[2:]):  # not a JPEG once reassembled
            self.state.accept_hd_chunk(data)
        self.assertEqual(self.state.invalid_video, len(bad) + 1)
        self.assertIsNone(self.state.video_hd)

    def test_hd_demand_file_is_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            low, hd = Path(directory) / 'video-demand', Path(directory) / 'video-demand-hd'
            state = State({'id': 't'}, '.', video_demand=low, video_hd_demand=hd)
            state.touch_demand(hd=True)
            self.assertTrue(hd.exists())
            self.assertFalse(low.exists())
            self.assertEqual(hd.stat().st_mode & 0o777, 0o600)
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                State({'id': 't'}, '.', video_hd_demand=Path(directory) / 'missing' / 'x').touch_demand(hd=True)
            self.assertIn('WARNING', out.getvalue())


class AdvertiseTests(unittest.IsolatedAsyncioTestCase):
    async def test_unassigned_address_disables_mdns_instead_of_stopping(self):
        try:
            import zeroconf  # noqa: F401
        except ImportError:
            self.skipTest('zeroconf not installed')
        from service import advertise
        # 192.0.2.1 (TEST-NET-1) is never assigned here: the multicast join fails (ENODEV).
        with contextlib.redirect_stdout(io.StringIO()) as out:
            result = await advertise('192.0.2.1', 8443, {'id': '12345678-aaaa-bbbb-cccc-1234567890ab'})
        self.assertIsNone(result)
        self.assertIn('mDNS advertisement off', out.getvalue())
        self.assertIn('Manual pairing still works', out.getvalue())

    def test_lan_candidate_accepts_private_ipv4_only(self):
        for good in ['192.168.0.138', '192.168.1.78', '10.1.2.3', '172.16.5.4']:
            self.assertEqual(service.lan_candidate(good), good)
        for bad in ['8.8.8.8', '127.0.0.1', '169.254.3.4', '::1', 'fe80::1', '', 'not an address', '192.0.2.1x']:
            self.assertIsNone(service.lan_candidate(bad), bad)

    def test_detect_gives_a_private_address_or_none(self):
        found = service.detect_lan_address()
        self.assertTrue(found is None or service.lan_candidate(found) == found, found)

    async def test_auto_follows_the_default_route_address(self):
        # Bench, unchanged, car network, no LAN for a while, car network again.
        sequence = ['192.168.0.138', '192.168.0.138', '192.168.1.78', None, None, '192.168.1.78']
        registered, withdrawn = [], []

        class FakeZeroconf:
            def __init__(self, address): self.address = address
            async def async_unregister_service(self, info): withdrawn.append(self.address)
            async def async_close(self): pass

        async def fake_advertise(address, port, identity):
            registered.append(address)
            return FakeZeroconf(address), object()

        def detect():
            return sequence.pop(0) if len(sequence) > 1 else sequence[0]

        with mock.patch.object(service, 'advertise', fake_advertise), contextlib.redirect_stdout(io.StringIO()) as out:
            task = asyncio.create_task(service.maintain_advertisement('auto', 8443, {'id': 'x'}, detect=detect, recheck=.02))
            await wait_for(lambda: len(sequence) == 1 and len(registered) == 3)
            await asyncio.sleep(.1)   # stays put on an unchanged address
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self.assertEqual(registered, ['192.168.0.138', '192.168.1.78', '192.168.1.78'])
        # Withdrawn on the change, while there was no LAN, and at shutdown.
        self.assertEqual(withdrawn, ['192.168.0.138', '192.168.1.78', '192.168.1.78'])
        self.assertIn('mDNS: advertising on 192.168.1.78 (detected)', out.getvalue())

    async def test_fixed_address_never_detects(self):
        registered = []

        async def fake_advertise(address, port, identity):
            registered.append(address)
            return mock.AsyncMock(), object()

        def detect():
            raise AssertionError('a fixed address must not be detected')

        with mock.patch.object(service, 'advertise', fake_advertise):
            task = asyncio.create_task(service.maintain_advertisement('192.168.0.138', 8443, {'id': 'x'}, detect=detect))
            await wait_for(lambda: registered)
            await asyncio.sleep(.05)
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self.assertEqual(registered, ['192.168.0.138'])


class VideoIpcTests(unittest.IsolatedAsyncioTestCase):
    async def test_receiver_binds_privately_and_replaces_only_stale_sockets(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'run' / 'video.sock'
            state = State({'id': 't'}, '.')
            transport = await bind_datagram(path, VideoReceiver(state))
            try:
                self.assertEqual(path.stat().st_mode & 0o777, 0o600)
                with self.assertRaisesRegex(ValueError, 'already owns'):
                    await bind_datagram(path, VideoReceiver(state))
                with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as sender:
                    sender.sendto(video_frame(seq=7), str(path))
                await wait_for(lambda: state.video is not None)
                self.assertEqual((state.video['source_seq'], state.video['jpeg']), (7, JPEG))
            finally:
                transport.close()
                await asyncio.sleep(0)  # the transport closes its socket on the next loop iteration
            # Left behind by SIGKILL: replaced.
            self.assertTrue(path.exists())
            transport = await bind_datagram(path, VideoReceiver(state))
            transport.close()
            await asyncio.sleep(0)
            path.unlink()
            path.write_bytes(b'not a socket')
            with self.assertRaisesRegex(ValueError, 'not a socket'):
                await bind_datagram(path, VideoReceiver(state))
            self.assertEqual(path.read_bytes(), b'not a socket')

    async def test_simulator_sends_the_test_card(self):
        state = State({'id': 't'}, '.')
        card = sof_jpeg(640, 360)
        task = asyncio.create_task(simulate_video(state, card))
        try:
            await wait_for(lambda: state.video_seq >= 3)
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        self.assertEqual((state.video['jpeg'], state.video['width'], state.video['height']), (card, 640, 360))
        self.assertEqual(state.video['source_seq'], 3 * state.video_seq - 2)  # every third frame of 30 fps
        self.assertIsNone(state.video_hd)  # no HD viewer, no HD frames
        self.assertLess(time.monotonic() - state.video['capture_mono_s'], .5)
        self.assertEqual(state.invalid_video, 0)
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            await asyncio.wait_for(simulate_video(state, b'\xff\xd8\xff\xd9'), 1)  # no size: no video
        self.assertIn('no video', out.getvalue())


class VideoApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.demand = Path(self.temp.name) / 'video-demand'
        self.state = State({'id': 'test', 'name': 'test', 'token': TOKEN}, '.', video_demand=self.demand)
        self.client = TestClient(TestServer(create_app(self.state)))
        await self.client.start_server()
        self.headers = {'Authorization': 'Bearer ' + TOKEN}

    async def asyncTearDown(self):
        await self.client.close()
        self.temp.cleanup()

    def feed(self, **kwargs):
        self.state.accept_video(video_frame(seq=self.state.video_seq + 1, **kwargs))

    async def test_binary_message_is_byte_exact(self):
        self.feed(width=1280, height=720)
        ws = await self.client.ws_connect('/v1/video', headers=self.headers)
        message = await ws.receive(timeout=2)
        self.assertEqual(message.type.name, 'BINARY')
        data = message.data
        magic, header, seq, age_ms, width, height = struct.unpack('<4sIQIHH', data[:24])
        self.assertEqual((magic, header, seq, width, height), (b'DCV1', 24, 1, 1280, 720))
        self.assertLess(age_ms, 1000)
        self.assertEqual(data[24:], JPEG)
        # The same frame is never sent twice; the next one carries the next agent sequence.
        with self.assertRaises(asyncio.TimeoutError):
            await ws.receive(timeout=.3)
        self.state.accept_video(video_frame(seq=99))
        self.assertEqual(struct.unpack_from('<Q', await ws.receive_bytes(timeout=2), 8)[0], 2)
        await ws.close()

    async def test_rate_is_capped_at_10_fps_and_only_the_newest_frame_is_sent(self):
        async def producer():
            while True:
                self.feed()
                await asyncio.sleep(.005)
        task = asyncio.create_task(producer())
        try:
            ws = await self.client.ws_connect('/v1/video', headers=self.headers)
            arrivals, sequences = [], []
            start = time.monotonic()
            while time.monotonic() - start < 1.0:
                with contextlib.suppress(asyncio.TimeoutError):
                    data = await ws.receive_bytes(timeout=.2)
                    arrivals.append(time.monotonic())
                    sequences.append(struct.unpack_from('<Q', data, 8)[0])
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await ws.close()
        self.assertLessEqual(len(arrivals), 12)
        self.assertGreaterEqual(len(arrivals), 5)
        self.assertGreaterEqual((arrivals[-1] - arrivals[0]) / (len(arrivals) - 1), .085)
        self.assertEqual(sequences, sorted(set(sequences)))
        self.assertGreater(min(b - a for a, b in zip(sequences, sequences[1:])), 1)  # skipped, never queued

    async def test_frames_older_than_one_second_are_not_sent(self):
        self.feed()
        self.state.video['capture_mono_s'] -= 1.2
        ws = await self.client.ws_connect('/v1/video', headers=self.headers)
        with self.assertRaises(asyncio.TimeoutError):
            await ws.receive(timeout=.4)
        self.feed()
        self.assertEqual(struct.unpack_from('<Q', await ws.receive_bytes(timeout=2), 8)[0], 2)
        await ws.close()

    async def test_frames_go_only_to_video_viewers(self):
        self.feed()
        ws = await self.client.ws_connect('/v1/stream', headers=self.headers)
        for _ in range(3):
            self.feed()
            message = await ws.receive(timeout=2)
            self.assertEqual(message.type.name, 'TEXT')
            self.assertNotIn('jpeg', message.data)
        await ws.close()

    async def test_video_viewer_limit_is_separate_from_stream(self):
        streams = [await self.client.ws_connect('/v1/stream', headers=self.headers) for _ in range(4)]
        videos = [await self.client.ws_connect('/v1/video', headers=self.headers) for _ in range(4)]
        for path in ['/v1/stream', '/v1/video']:
            with self.assertRaises(WSServerHandshakeError) as failure:
                await self.client.ws_connect(path, headers=self.headers)
            self.assertEqual(failure.exception.status, 503, path)
        await videos[0].close()
        await wait_for(lambda: len(self.state.video_clients) < 4)
        self.assertEqual(len(self.state.clients), 4)
        videos[0] = await self.client.ws_connect('/v1/video', headers=self.headers)
        for ws in streams + videos:
            await ws.close()

    async def test_client_message_closes_with_1008(self):
        for payload in ['STOP RECORDING', b'\0']:
            ws = await self.client.ws_connect('/v1/video', headers=self.headers)
            await (ws.send_str(payload) if isinstance(payload, str) else ws.send_bytes(payload))
            message = await ws.receive(timeout=2)
            self.assertEqual(message.type.name, 'CLOSE')
            self.assertEqual(ws.close_code, 1008)
            await ws.close()
        await wait_for(lambda: not self.state.video_clients)

    async def test_demand_file_touched_only_while_watched(self):
        await asyncio.sleep(.6)
        self.assertFalse(self.demand.exists())  # nobody watching yet
        ws = await self.client.ws_connect('/v1/video', headers=self.headers)
        await wait_for(self.demand.exists, .5)  # immediately on connect
        self.assertEqual(self.demand.stat().st_mode & 0o777, 0o600)
        first = self.demand.stat().st_mtime_ns
        await asyncio.sleep(1.1)
        touched = self.demand.stat().st_mtime_ns
        self.assertGreater(touched, first)
        self.assertLess(time.time() - touched / 1e9, 1.0)  # at least once per second
        await ws.close()
        await wait_for(lambda: not self.state.video_clients)
        last = self.demand.stat().st_mtime_ns
        await asyncio.sleep(1.2)
        self.assertEqual(self.demand.stat().st_mtime_ns, last)

    async def test_csp_allows_blob_images_and_identity_has_video(self):
        response = await self.client.get('/v1/identity', headers=self.headers)
        csp = response.headers['Content-Security-Policy']
        # Vehicle map: OSM tiles are images only; connections stay same-origin.
        self.assertIn("img-src 'self' data: blob: https://tile.openstreetmap.org;", csp)
        self.assertIn("connect-src 'self';", csp)
        self.assertEqual(csp, "default-src 'none'; script-src 'self'; style-src 'self'; "
                              "img-src 'self' data: blob: https://tile.openstreetmap.org; "
                              "connect-src 'self'; base-uri 'none'; form-action 'none'; frame-ancestors 'none'")
        capabilities = (await response.json())['capabilities']
        self.assertIs(capabilities['video'], True)
        self.assertIs(capabilities['video_hd'], True)
        self.assertFalse(capabilities['controls'])


class HdApiTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.low = Path(self.temp.name) / 'video-demand'
        self.hd = Path(self.temp.name) / 'video-demand-hd'
        self.state = State({'id': 'test', 'name': 'test', 'token': TOKEN}, '.', video_demand=self.low,
                           video_hd_demand=self.hd)
        self.client = TestClient(TestServer(create_app(self.state)))
        await self.client.start_server()
        self.headers = {'Authorization': 'Bearer ' + TOKEN}

    async def asyncTearDown(self):
        await self.client.close()
        self.temp.cleanup()

    def feed_hd(self, seq=None):
        for chunk in hd_frame(seq=self.state.video_hd_seq + 1 if seq is None else seq):
            self.state.accept_hd_chunk(chunk)

    async def test_hd_viewer_gets_original_frames_and_only_the_hd_demand(self):
        self.feed_hd()
        self.state.accept_video(video_frame(seq=1))
        ws = await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers)
        data = await ws.receive_bytes(timeout=2)
        magic, header, seq, age_ms, width, height = struct.unpack('<4sIQIHH', data[:24])
        self.assertEqual((magic, header, seq, width, height), (b'DCV1', 24, 1, 1920, 1080))
        self.assertEqual(data[24:], HD_JPEG)
        await wait_for(self.hd.exists, .5)
        await asyncio.sleep(.6)
        self.assertFalse(self.low.exists())  # an HD viewer never starts the 640x360 conversion
        self.assertEqual(len(self.state.video_hd_clients), 1)
        self.assertFalse(self.state.video_clients)
        await ws.close()

    async def test_hd_rate_follows_the_camera(self):
        async def producer():
            while True:
                self.feed_hd()
                await asyncio.sleep(1 / 30)
        ws = await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers)
        task = asyncio.create_task(producer())
        received = 0
        try:
            start = time.monotonic()
            while time.monotonic() - start < 1.0:
                with contextlib.suppress(asyncio.TimeoutError):
                    await ws.receive_bytes(timeout=.2)
                    received += 1
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await ws.close()
        self.assertGreaterEqual(received, 20)  # far above the preview's 10 fps cap
        self.assertLessEqual(received, 41)

    async def test_hd_limit_quality_validation_and_low_route_unchanged(self):
        hd = [await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers) for _ in range(2)]
        with self.assertRaises(WSServerHandshakeError) as failure:
            await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers)
        self.assertEqual(failure.exception.status, 503)
        low = await self.client.ws_connect('/v1/video?quality=low', headers=self.headers)  # separate limit
        with self.assertRaises(WSServerHandshakeError) as failure:
            await self.client.ws_connect('/v1/video?quality=4k', headers=self.headers)
        self.assertEqual(failure.exception.status, 400)
        self.state.accept_video(video_frame(seq=1))
        self.assertEqual((await low.receive_bytes(timeout=2))[24:], JPEG)
        for ws in hd + [low]:
            await ws.close()

    async def test_no_hd_frames_closes_with_4001(self):
        with mock.patch.object(service, 'HD_SILENCE_SECONDS', .3):
            ws = await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers)
            message = await ws.receive(timeout=2)
        self.assertEqual(message.type, WSMsgType.CLOSE)
        self.assertEqual((ws.close_code, message.extra), (service.HD_UNAVAILABLE, 'HD not offered'))
        await wait_for(lambda: not self.state.video_hd_clients)

    async def test_hd_that_stops_closes_with_4001_and_sd_takes_over(self):
        # Review finding: one HD frame used to disable the check for the rest of the socket's life.
        with mock.patch.object(service, 'HD_SILENCE_SECONDS', .4):
            sd = await self.client.ws_connect('/v1/video', headers=self.headers)   # a simultaneous SD viewer
            hd = await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers)
            self.feed_hd()
            self.assertEqual((await hd.receive_bytes(timeout=2))[24:], HD_JPEG)
            start = time.monotonic()
            closed = None
            while time.monotonic() - start < 2 and closed is None:   # HD stops; SD keeps coming
                self.state.accept_video(video_frame(seq=self.state.video_seq + 1))
                with contextlib.suppress(asyncio.TimeoutError):
                    message = await hd.receive(timeout=.1)
                    if message.type == WSMsgType.CLOSE:
                        closed = (time.monotonic() - start, hd.close_code, message.extra)
            self.assertIsNotNone(closed)
            self.assertEqual(closed[1:], (service.HD_UNAVAILABLE, 'HD stopped'))
            self.assertGreaterEqual(closed[0], .3)
            self.assertEqual((await sd.receive_bytes(timeout=2))[24:], JPEG)   # fresh SD for the fallback
            await sd.close()

    async def test_short_hd_gap_keeps_hd(self):
        with mock.patch.object(service, 'HD_SILENCE_SECONDS', .6):
            hd = await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers)
            for _ in range(3):   # gaps of 0.3 s, each shorter than the timeout
                self.feed_hd()
                self.assertEqual(await hd.receive_bytes(timeout=2) is not None, True)
                await asyncio.sleep(.3)
            self.feed_hd()
            message = await hd.receive(timeout=2)
            self.assertEqual(message.type, WSMsgType.BINARY)
            self.assertFalse(hd.closed)
            await hd.close()

    async def test_simulator_serves_hd_while_watched(self):
        card = sof_jpeg(640, 360)
        task = asyncio.create_task(simulate_video(self.state, card))
        try:
            ws = await self.client.ws_connect('/v1/video?quality=hd', headers=self.headers)
            self.assertEqual((await ws.receive_bytes(timeout=2))[24:], card)
            await ws.close()
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)



class RecordingsTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.dir = Path(self.temp.name)
        old = time.time() - 600
        self.video = bytes(range(256)) * 4096   # 1 MiB
        for name, data in [('dashcam_000861_20261126_134121.mkv', self.video),
                           ('dashcam_000861_20261126_134121.ass', b'[Script Info]\n'),
                           ('dashcam_000860_20261126_133821.mkv', b'older'),
                           ('notes.txt', b'x'), ('dashcam_1_2_3.mkv', b'x')]:
            (self.dir / name).write_bytes(data)
            os.utime(self.dir / name, (old, old))
        start = time.mktime(dt.datetime(2026, 11, 26, 13, 41, 21).timetuple())
        os.utime(self.dir / 'dashcam_000861_20261126_134121.mkv', (start + 180, start + 180))
        os.utime(self.dir / 'dashcam_000860_20261126_133821.mkv', (start - 900, start - 900))   # before its own start
        (self.dir / 'dashcam_000862_20261126_134421.mkv').write_bytes(b'growing')   # still recording
        (self.dir / 'dashcam_000859_20261126_133521.mkv').mkdir()                    # not a file
        os.symlink('/etc/passwd', self.dir / 'dashcam_000858_20261126_133221.mkv')    # never followed
        self.state = State({'id': 'test', 'name': 'test', 'token': TOKEN}, '.', recordings_dir=self.dir)
        self.client = TestClient(TestServer(create_app(self.state)))
        await self.client.start_server()
        self.headers = {'Authorization': 'Bearer ' + TOKEN}

    async def asyncTearDown(self):
        await self.client.close()
        self.temp.cleanup()

    async def test_list_has_finished_segments_only_newest_first(self):
        response = await self.client.get('/v1/recordings', headers=self.headers)
        self.assertEqual(response.status, 200)
        body = await response.json()
        self.assertEqual(body['schema'], 1)
        self.assertEqual([s['id'] for s in body['segments']],
                         ['dashcam_000861_20261126_134121', 'dashcam_000860_20261126_133821'])
        first = body['segments'][0]
        self.assertEqual((first['seq'], first['start'], first['bytes'], first['caption_bytes'], first['duration_s']),
                         (861, '2026-11-26 13:41:21', len(self.video), 14, 180.0))
        self.assertIsNone(body['segments'][1]['caption_bytes'])
        self.assertIsNone(body['segments'][1]['duration_s'])   # last write before its named start: unknown

    async def test_ranges_downloads_and_types(self):
        name = 'dashcam_000861_20261126_134121.mkv'
        response = await self.client.get('/v1/recordings/' + name, headers={**self.headers, 'Range': 'bytes=1000-1999'})
        self.assertEqual(response.status, 206)
        self.assertEqual(await response.read(), self.video[1000:2000])
        self.assertEqual(response.headers['Content-Type'], 'video/x-matroska')
        self.assertEqual(response.headers['Content-Range'], f'bytes 1000-1999/{len(self.video)}')
        self.assertEqual(response.headers['Cache-Control'], 'no-store')
        tail = await self.client.get('/v1/recordings/' + name, headers={**self.headers, 'Range': 'bytes=-16'})
        self.assertEqual((tail.status, await tail.read()), (206, self.video[-16:]))
        whole = await self.client.get('/v1/recordings/' + name + '?download=1', headers=self.headers)
        self.assertEqual((whole.status, len(await whole.read())), (200, len(self.video)))
        self.assertEqual(whole.headers['Content-Disposition'], f'attachment; filename="{name}"')
        ass = await self.client.get('/v1/recordings/dashcam_000861_20261126_134121.ass', headers=self.headers)
        self.assertEqual((ass.status, ass.headers['Content-Type']), (200, 'text/plain; charset=utf-8'))

    async def test_refusals(self):
        for name, status in [('dashcam_000862_20261126_134421.mkv', 409),   # still recording
                             ('dashcam_000859_20261126_133521.mkv', 404),   # a directory
                             ('dashcam_000858_20261126_133221.mkv', 404),   # a symlink
                             ('dashcam_000857_20261126_132921.mkv', 404),   # missing
                             ('dashcam_000862_20261126_134421.txt', 404),
                             ('notes.mkv', 404), ('..%2Fservice.mkv', 404), ('%2Fetc%2Fpasswd.ass', 404)]:
            response = await self.client.get('/v1/recordings/' + name, headers=self.headers)
            self.assertEqual(response.status, status, name)
        for path in ['/v1/recordings', '/v1/recordings/dashcam_000861_20261126_134121.mkv']:
            self.assertEqual((await self.client.get(path)).status, 401, path)

    async def test_capability_and_disabled(self):
        ident = await (await self.client.get('/v1/identity', headers=self.headers)).json()
        self.assertIs(ident['capabilities']['recordings'], True)
        off = State({'id': 'test', 'name': 'test', 'token': TOKEN}, '.')
        client = TestClient(TestServer(create_app(off)))
        await client.start_server()
        try:
            ident = await (await client.get('/v1/identity', headers=self.headers)).json()
            self.assertIs(ident['capabilities']['recordings'], False)
            self.assertEqual((await client.get('/v1/recordings', headers=self.headers)).status, 404)
            self.assertEqual((await client.get('/v1/recordings/dashcam_000861_20261126_134121.mkv',
                                               headers=self.headers)).status, 404)
        finally:
            await client.close()



FIXTURE = Path(__file__).resolve().parents[1] / 'android/app/src/test/resources/recorder_fixture.mkv'


class ClipTests(unittest.IsolatedAsyncioTestCase):
    def test_command_offsets_burned_captions_to_segment_time(self):
        cmd = service.clip_command('ffmpeg', Path('/f/v.mkv'), 12.5, 30, 'small', Path('/f/v.ass'), Path('/t/o.mp4'))
        self.assertEqual(cmd[cmd.index('-ss') + 1], '12.500')
        self.assertEqual(cmd[cmd.index('-t') + 1], '30.000')
        self.assertIn('-lowres', cmd)
        self.assertLess(cmd.index('-lowres'), cmd.index('-i'))   # a decoder option: before the input
        vf = cmd[cmd.index('-vf') + 1]
        self.assertEqual(vf, "fps=10,scale=trunc(iw/2)*2:trunc(ih/2)*2,setpts=PTS+12.500/TB,ass='/f/v.ass',"
                             "setpts=PTS-STARTPTS,format=yuv420p")   # even sizes: x264 needs them
        self.assertEqual(cmd[-3:], ['-movflags', '+faststart', '/t/o.mp4'])
        hd = service.clip_command('ffmpeg', Path('v.mkv'), 0, 5, '720p', None, Path('o.mp4'))
        self.assertNotIn('-lowres', hd)
        self.assertEqual(hd[hd.index('-vf') + 1], 'scale=1280:720,format=yuv420p')
        tricky = service.clip_command('ffmpeg', Path('v.mkv'), 0, 5, '720p', "C:/a'b.ass", Path('o.mp4'))
        self.assertIn("ass='C\\:/a\\'b.ass'", tricky[tricky.index('-vf') + 1])

    async def asyncSetUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.dir = Path(self.temp.name)
        old = time.time() - 600
        name = 'dashcam_000001_20261126_120000'
        (self.dir / (name + '.mkv')).write_bytes(FIXTURE.read_bytes())
        (self.dir / (name + '.ass')).write_text(
            '[Script Info]\nPlayResX: 96\nPlayResY: 54\n\n[V4+ Styles]\n'
            'Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, '
            'Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, '
            'MarginR, MarginV, Encoding\n'
            'Style: TL,Monospace,8,&H00FFFFFF,&H00FFFFFF,&H26000000,&H26000000,-1,0,0,0,100,100,0,0,3,1,0,7,2,2,2,1\n\n'
            '[Events]\nFormat: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n'
            'Dialogue: 0,0:00:00.00,0:00:01.00,TL,,0,0,0,,SPD 50\n')
        for f in self.dir.iterdir():
            os.utime(f, (old, old))
        (self.dir / 'dashcam_000002_20261126_120300.mkv').write_bytes(b'growing')   # still recording
        self.name = name
        self.state = State({'id': 'test', 'name': 'test', 'token': TOKEN}, '.', recordings_dir=self.dir)
        self.client = TestClient(TestServer(create_app(self.state)))
        await self.client.start_server()
        self.headers = {'Authorization': 'Bearer ' + TOKEN}

    async def asyncTearDown(self):
        await self.client.close()
        self.temp.cleanup()

    def clip(self, name, **body):
        return self.client.post(f'/v1/recordings/{name}.mkv/clip', headers=self.headers, json=body)

    async def test_capability_and_validation(self):
        ident = await (await self.client.get('/v1/identity', headers=self.headers)).json()
        self.assertIs(ident['capabilities']['clips'], bool(shutil.which('ffmpeg')))
        if not shutil.which('ffmpeg'):
            self.skipTest('ffmpeg not installed')
        good = dict(start_s=0, duration_s=1, quality='small', captions=False)
        for bad in [dict(good, quality='4k'), dict(good, duration_s=0), dict(good, duration_s=181),
                    dict(good, start_s=-1), dict(good, captions='yes'), dict(good, start_s='nan'), {}]:
            self.assertEqual((await self.clip(self.name, **bad)).status, 400, bad)
        self.assertEqual((await self.clip('dashcam_000002_20261126_120300', **good)).status, 409)
        self.assertEqual((await self.clip('dashcam_000009_20261126_120900', **good)).status, 404)
        unauth = await self.client.post(f'/v1/recordings/{self.name}.mkv/clip', json=good)
        self.assertEqual(unauth.status, 401)

    async def test_real_clip_with_burned_captions(self):
        if not shutil.which('ffmpeg') or not shutil.which('ffprobe'):
            self.skipTest('ffmpeg not installed')
        for quality in ('small', '720p'):
            r = await self.clip(self.name, start_s=0.2, duration_s=1, quality=quality, captions=True)
            data = await r.read()
            self.assertEqual(r.status, 200, data[:300])
            self.assertEqual(r.headers['Content-Type'], 'video/mp4')
            self.assertEqual(r.headers['X-Clip-Captions'], 'burned')
            self.assertEqual(int(r.headers['Content-Length']), len(data))
            with tempfile.NamedTemporaryFile(suffix='.mp4') as f:
                f.write(data)
                f.flush()
                probe = json.loads(subprocess.run(
                    ['ffprobe', '-v', 'error', '-show_entries', 'stream=codec_name,width,height,pix_fmt:format=duration',
                     '-of', 'json', f.name], capture_output=True, text=True, check=True).stdout)
            stream = probe['streams'][0]
            self.assertEqual((stream['codec_name'], stream['pix_fmt']), ('h264', 'yuv420p'))
            self.assertEqual((stream['width'], stream['height']), (48, 26) if quality == 'small' else (1280, 720))   # 96x54 halved, made even
            self.assertAlmostEqual(float(probe['format']['duration']), 0.8, delta=.25)   # 1 s asked, 0.8 s left in the fixture
            self.assertEqual(data[4:8], b'ftyp')
            self.assertLess(data.find(b'moov'), data.find(b'mdat'))   # fast start: the index first

    async def test_clip_past_the_end_is_refused_not_empty(self):
        """Review 2026-11-27 #5: a start beyond the last frame gave exit 0 and a 261-byte MP4 with no video."""
        if not shutil.which('ffmpeg') or not shutil.which('ffprobe'):
            self.skipTest('ffmpeg not installed')
        self.assertIn('empty_output', service.clip_command('ffmpeg', Path('v.mkv'), 0, 5, '720p', None, Path('o.mp4')))
        # Known length (the fixture's Duration, 1 s): refused before ffmpeg runs.
        r = await self.clip(self.name, start_s=2, duration_s=1, quality='720p')
        self.assertEqual(r.status, 416, await r.text())
        # Unknown length: ffmpeg itself must fail, not hand back an empty file.
        self.state.recordings.index = lambda path, info: (True, None)
        r = await self.clip(self.name, start_s=2, duration_s=1, quality='720p')
        self.assertEqual(r.status, 422, await r.text())
        # Overlapping the end still gives the frames that exist.
        r = await self.clip(self.name, start_s=0.5, duration_s=1, quality='720p')
        data = await r.read()
        self.assertEqual(r.status, 200)
        with tempfile.NamedTemporaryFile(suffix='.mp4') as f:
            f.write(data)
            f.flush()
            probe = json.loads(subprocess.run(['ffprobe', '-v', 'error', '-show_entries', 'stream=codec_type',
                                               '-of', 'json', f.name], capture_output=True, text=True, check=True).stdout)
        self.assertEqual([s['codec_type'] for s in probe['streams']], ['video'])

    async def test_cut_short_segment_is_listed_but_not_playable(self):
        """Review 2026-11-27 #6: a segment without a usable Cues index (power cut) was offered for playback."""
        full = FIXTURE.read_bytes()
        old = time.time() - 600
        cut = 'dashcam_000003_20261126_120600.mkv'
        (self.dir / cut).write_bytes(full[:len(full) * 7 // 10])   # aged, truncated before the Cues
        cues = full.find(bytes.fromhex('1C53BB6B'))
        self.assertGreater(cues, 0)
        broken = 'dashcam_000004_20261126_120900.mkv'
        (self.dir / broken).write_bytes(full[:cues] + b'\0\0\0\0' + full[cues + 4:])   # index points at no Cues
        for name in (cut, broken):
            os.utime(self.dir / name, (old, old))
        listed = {s['id']: s for s in (await (await self.client.get('/v1/recordings', headers=self.headers)).json())['segments']}
        self.assertIs(listed[self.name]['playable'], True)               # finished segment
        self.assertIs(listed[cut[:-4]]['playable'], False)
        self.assertIs(listed[broken[:-4]]['playable'], False)
        self.assertNotIn('dashcam_000002_20261126_120300', listed)       # growing: not listed at all
        # Still downloadable for recovery, byte for byte.
        r = await self.client.get('/v1/recordings/' + cut, headers=self.headers)
        self.assertEqual((r.status, await r.read()), (200, full[:len(full) * 7 // 10]))
        playable, length = service.mkv_index(self.dir / (self.name + '.mkv'), len(full))
        self.assertIs(playable, True)
        self.assertAlmostEqual(length, 1.0, delta=.1)

    async def test_one_export_at_a_time(self):
        if not shutil.which('ffmpeg'):
            self.skipTest('ffmpeg not installed')
        await self.state.clip_lock.acquire()
        try:
            r = await self.clip(self.name, start_s=0, duration_s=1, quality='small')
            self.assertEqual(r.status, 503)
        finally:
            self.state.clip_lock.release()


if __name__ == '__main__':
    unittest.main()
