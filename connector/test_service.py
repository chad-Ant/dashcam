import asyncio
import copy
import json
from pathlib import Path
import ssl
import tempfile
import time
import unittest

from aiohttp import ClientSession, Fingerprint, WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer
from service import State, create_app, encode, initialize, load_identity
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


if __name__ == '__main__':
    unittest.main()
