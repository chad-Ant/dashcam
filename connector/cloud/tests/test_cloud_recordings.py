"""Cloud recordings without Firebase, Firestore or Cloud Storage: the real relay and uplink request paths.

The relay runs in-process with a fake store, fake sign-in and a fake signer; a device socket and viewer
sockets talk to it. The uplink handlers run against a fake local connector and a fake storage server.
From connector/cloud: python -m unittest tests.test_cloud_recordings
"""
import asyncio
import json
import sys
import time
import types
import unittest
from pathlib import Path

import aiohttp
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Stand-ins for the Google libraries the relay and store import (not installed where these tests run).
firestore = types.SimpleNamespace(async_transactional=lambda f: f, DELETE_FIELD=object(), AsyncClient=object)
sys.modules.setdefault('google', types.ModuleType('google'))
sys.modules.setdefault('google.cloud', types.ModuleType('google.cloud'))
sys.modules['google.cloud'].firestore = firestore
sys.modules.setdefault('google.cloud.firestore', firestore)
firebase = types.ModuleType('firebase_admin')
firebase.auth = types.SimpleNamespace(verify_id_token=None, verify_session_cookie=None)
firebase.exceptions = types.SimpleNamespace(FirebaseError=Exception)
sys.modules.setdefault('firebase_admin', firebase)
sys.modules.setdefault('firebase_admin.auth', firebase.auth)

import relay  # noqa: E402
import test_uplink_resources as stubs  # noqa: E402,F401  (installs the uplink's native/listener stubs)
import uplink  # noqa: E402

SEGMENT = 'dashcam_000311_20261003_223550'


class Doc:
    def __init__(self, data):
        self.data = data
        self.exists = data is not None

    def to_dict(self):
        return self.data


class Ref:
    def __init__(self, data):
        self.data = data

    async def get(self):
        return Doc(self.data)


class FakeStore:
    def __init__(self):
        self.granted = {('owner-1', 'A'): {'role': 'owner'}, ('viewer-1', 'A'): {'role': 'viewer'}}

    def grant(self, uid, key):
        return Ref(self.granted.get((uid, key)))

    async def device_control(self, key, uid):
        if uid != 'device-A':
            raise PermissionError
        return dict(deviceUid=uid, demandGeneration=1, configVersion=1, expiresAt=None, desiredMode='on_demand',
                    channels=[])

    async def update(self, key, uid, **kwargs):
        return dict(desiredMode='on_demand', configVersion=1, demandGeneration=1, channels=kwargs.get('channels') or [],
                    expiresAt=None, requestId=kwargs.get('session'))


class FakeClips:
    LINK_SECONDS = 3600

    def __init__(self):
        self.signed = []

    @staticmethod
    def object_name(vehicle, segment, start, quality):
        return f'clips/{vehicle}/{segment}_{int(start)}s_{quality}.mp4'

    def upload_url(self, name):
        self.signed.append(('PUT', name))
        return f'https://storage.googleapis.com/bucket/{name}?X-Goog-Signature=put'

    def download_urls(self, name, filename):
        self.signed.append(('GET', name, filename))
        return (f'https://storage.googleapis.com/bucket/{name}?X-Goog-Signature=get',
                f'https://storage.googleapis.com/bucket/{name}?X-Goog-Signature=dl')


async def fake_identity(request, device=False):
    who = request.headers.get('Authorization', '')[7:]
    if device:
        if who != 'device-A':
            raise PermissionError
        return dict(uid=who, firebase=dict(sign_in_provider='custom'))
    if who not in ('owner-1', 'viewer-1', 'stranger'):
        raise PermissionError
    return dict(uid=who, exp=time.time() + 3600, auth_time=time.time(), firebase=dict(sign_in_provider='google.com'))


class RelayRecordingsTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.original = relay.identity
        relay.identity = fake_identity
        self.clips = FakeClips()
        self.client = TestClient(TestServer(relay.application(FakeStore(), {}, self.clips)))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        relay.identity = self.original

    async def device(self):
        return await self.client.ws_connect('/v1/uplink?vehicle=A', headers={'Authorization': 'Bearer device-A'})

    async def viewer(self, who='viewer-1'):
        ws = await self.client.ws_connect('/v1/view/A', headers={'Authorization': 'Bearer ' + who})
        self.assertEqual((await ws.receive_json(timeout=2))['type'], 'connecting')
        return ws

    async def next_of(self, ws, kind):
        while True:
            message = await ws.receive_json(timeout=3)
            if message.get('type') == kind:
                return message

    async def test_viewer_lists_recordings_through_the_vehicle(self):
        device = await self.device()
        viewer = await self.viewer()   # a viewer (not only an owner) may use recordings
        await viewer.send_json(dict(type='recordings', id='list-1'))
        asked = await self.next_of(device, 'list_recordings')
        self.assertNotEqual(asked['id'], 'list-1')   # the device never sees viewer ids
        segments = [dict(id=SEGMENT, start='2026-10-03 22:35:50', bytes=935532527, caption_bytes=838539)]
        await device.send_json(dict(type='recordings', id=asked['id'], segments=segments))
        answer = await self.next_of(viewer, 'recordings')
        self.assertEqual((answer['id'], answer['segments']), ('list-1', segments))
        await viewer.close()
        await device.close()

    async def test_clip_upload_then_signed_links(self):
        device = await self.device()
        viewer = await self.viewer('owner-1')
        await viewer.send_json(dict(type='clip', id='c1', segment=SEGMENT, start_s=60, duration_s=30, quality='720p',
                                    captions=True))
        self.assertEqual((await self.next_of(viewer, 'clip_status'))['status'], 'exporting')
        asked = await self.next_of(device, 'export_clip')
        self.assertEqual({k: asked[k] for k in ('segment', 'start_s', 'duration_s', 'quality', 'captions')},
                         dict(segment=SEGMENT, start_s=60.0, duration_s=30.0, quality='720p', captions=True))
        self.assertTrue(asked['upload_url'].startswith('https://storage.googleapis.com/bucket/clips/A/'))
        # A second clip from the same viewer waits for the first.
        await viewer.send_json(dict(type='clip', id='c2', segment=SEGMENT, start_s=0, duration_s=10, quality='small'))
        self.assertIn('Wait for the clip', (await self.next_of(viewer, 'error'))['error'])
        await device.send_json(dict(type='clip_result', id=asked['id'], ok=True, bytes=5500000, captions=True))
        clip = await self.next_of(viewer, 'clip')
        self.assertEqual(clip['id'], 'c1')
        self.assertTrue(clip['url'].endswith('X-Goog-Signature=get') and clip['download'].endswith('Signature=dl'))
        self.assertEqual((clip['bytes'], clip['captions'], clip['filename']),
                         (5500000, True, f'{SEGMENT}_60s_30s_720p.mp4'))
        self.assertEqual([s[0] for s in self.clips.signed], ['PUT', 'GET'])
        await viewer.close()
        await device.close()

    async def test_refusals_and_failures(self):
        viewer = await self.viewer()
        await viewer.send_json(dict(type='recordings', id='r0'))
        self.assertIn('not connected', (await self.next_of(viewer, 'error'))['error'])
        device = await self.device()
        for bad in [dict(segment='../etc/passwd'), dict(duration_s=0), dict(duration_s=181), dict(quality='4k'),
                    dict(start_s='x'), dict(captions='yes')]:
            body = dict(type='clip', id='b', segment=SEGMENT, start_s=0, duration_s=10, quality='small', captions=False)
            body.update(bad)
            await viewer.send_json(body)
            self.assertIn('not valid', (await self.next_of(viewer, 'error'))['error'], bad)
        # The vehicle reports an error: the viewer gets it.
        await viewer.send_json(dict(type='clip', id='e1', segment=SEGMENT, start_s=0, duration_s=10, quality='small'))
        asked = await self.next_of(device, 'export_clip')
        await device.send_json(dict(type='clip_result', id=asked['id'], error='That segment is still recording'))
        self.assertEqual((await self.next_of(viewer, 'error'))['error'], 'That segment is still recording')
        # An answer for a viewer that has left goes nowhere.
        await viewer.send_json(dict(type='recordings', id='gone'))
        asked = await self.next_of(device, 'list_recordings')
        await viewer.close()
        await asyncio.sleep(.1)
        await device.send_json(dict(type='recordings', id=asked['id'], segments=[]))
        await device.send_json(dict(type='heartbeat'))
        self.assertEqual((await self.next_of(device, 'heartbeat'))['type'], 'heartbeat')   # still healthy
        await device.close()

    async def test_bad_request_id_closes_and_ungranted_viewer_is_refused(self):
        viewer = await self.viewer()
        await viewer.send_json(dict(type='recordings', id='no spaces allowed'))
        self.assertEqual((await viewer.receive(timeout=2)).type, aiohttp.WSMsgType.CLOSE)
        with self.assertRaises(aiohttp.WSServerHandshakeError):
            await self.viewer('stranger')

    async def test_without_a_bucket_clips_are_refused_but_lists_work(self):
        await self.client.close()
        self.client = TestClient(TestServer(relay.application(FakeStore(), {}, None)))
        await self.client.start_server()
        device = await self.device()
        viewer = await self.viewer()
        await viewer.send_json(dict(type='clip', id='c', segment=SEGMENT, start_s=0, duration_s=10, quality='small'))
        self.assertIn('not set up', (await self.next_of(viewer, 'error'))['error'])
        await viewer.send_json(dict(type='recordings', id='r'))
        self.assertEqual((await self.next_of(device, 'list_recordings'))['type'], 'list_recordings')
        await viewer.close()
        await device.close()


class ClipStorageSigningTests(unittest.TestCase):
    """What the relay asks google-cloud-storage to sign (the signing itself needs the real IAM API)."""

    def test_signed_url_parameters(self):
        calls = []

        class Blob:
            def __init__(self, name):
                self.name = name

            def generate_signed_url(self, **options):
                calls.append((self.name, options))
                return 'https://storage.googleapis.com/signed'

        clips = relay.ClipStorage.__new__(relay.ClipStorage)
        clips.credentials = types.SimpleNamespace(valid=True, service_account_email='relay@p.iam', token='tok')
        clips.bucket = types.SimpleNamespace(blob=Blob)
        name = relay.ClipStorage.object_name('A', SEGMENT, 60.7, '720p')
        self.assertRegex(name, r'^clips/A/' + SEGMENT + r'_60s_720p_[0-9a-f]{12}\.mp4$')
        clips.upload_url(name)
        clips.download_urls(name, 'clip.mp4')
        upload, play, download = [c[1] for c in calls]
        common = dict(version='v4', service_account_email='relay@p.iam', access_token='tok')
        self.assertEqual(upload, dict(common, method='PUT', content_type='video/mp4',
                                      expiration=relay.timedelta(seconds=15 * 60)))
        self.assertEqual(play, dict(common, method='GET', expiration=relay.timedelta(seconds=3600)))
        self.assertEqual(download, dict(common, method='GET', expiration=relay.timedelta(seconds=3600),
                                        response_disposition='attachment; filename="clip.mp4"'))


class UplinkRecordingsTests(unittest.IsolatedAsyncioTestCase):
    """The uplink's handlers against a fake local connector and a fake storage server."""

    async def asyncSetUp(self):
        self.uploads, self.clip = [], b'\x00\x00\x00\x18ftypisom' + b'x' * 5000
        self.local_status = 200

        async def recordings(_request):
            return web.json_response(dict(schema=1, segments=[dict(id=SEGMENT)] * 3))

        async def clip(request):
            body = await request.json()
            self.asked = (request.match_info['name'], body)
            if self.local_status != 200:
                return web.Response(status=self.local_status, text='no')
            return web.Response(body=self.clip, headers={'Content-Type': 'video/mp4', 'X-Clip-Captions': 'burned'})

        async def put(request):
            self.uploads.append((request.headers.get('Content-Type'), await request.read()))
            return web.Response(status=200)

        app = web.Application()
        app.router.add_get('/v1/recordings', recordings)
        app.router.add_post('/v1/recordings/{name}/clip', clip)
        app.router.add_put('/bucket/{tail:.*}', put)
        self.server = TestServer(app)
        await self.server.start_server()
        self.http = aiohttp.ClientSession()
        self.resources = uplink.Resources(self.http, dict(vehicle='A', local=dict(host='127.0.0.1', port=self.server.port,
                                                                               pin='00' * 32, token='t')), 'token')
        self.resources.local_url = lambda path: self.server.make_url(path)
        self.resources.local_request = lambda: {}
        self.patch = (uplink.CLIP_UPLOAD_SCHEME, uplink.CLIP_UPLOAD_HOST)
        uplink.CLIP_UPLOAD_SCHEME, uplink.CLIP_UPLOAD_HOST = 'http', '127.0.0.1'
        self.ws = stubs.FakeSocket()

    async def asyncTearDown(self):
        uplink.CLIP_UPLOAD_SCHEME, uplink.CLIP_UPLOAD_HOST = self.patch
        await self.http.close()
        await self.server.close()

    def request(self, **changes):
        body = dict(type='export_clip', id='tok', segment=SEGMENT, start_s=60.0, duration_s=30.0, quality='720p',
                    captions=True, upload_url=str(self.server.make_url('/bucket/clips/A/x.mp4')))
        body.update(changes)
        return body

    async def test_list(self):
        await self.resources.list_recordings(self.ws, dict(id='tok'))
        self.assertEqual(self.ws.sent, [dict(type='recordings', id='tok', segments=[dict(id=SEGMENT)] * 3)])

    async def test_export_uploads_to_the_signed_url_only(self):
        await self.resources.export_clip(self.ws, self.request())
        self.assertEqual(self.asked, (SEGMENT + '.mkv', dict(start_s=60.0, duration_s=30.0, quality='720p', captions=True)))
        self.assertEqual(self.uploads, [('video/mp4', self.clip)])
        self.assertEqual(self.ws.sent[-1], dict(type='clip_result', id='tok', captions=True, ok=True, bytes=len(self.clip)))
        # Anywhere but the storage host is refused before anything is read.
        # (Value checks such as the quality are the local connector's: it answers 400.)
        for bad in [dict(upload_url='http://evil.example/x'), dict(segment='../../etc/passwd'), dict(upload_url=None)]:
            self.ws.sent.clear()
            await self.resources.export_clip(self.ws, self.request(**bad))
            self.assertTrue(self.ws.sent[-1].get('error'), bad)
        self.assertEqual(len(self.uploads), 1)

    async def test_local_errors_are_reported_and_busy_is_refused(self):
        self.local_status = 409
        await self.resources.export_clip(self.ws, self.request())
        self.assertEqual(self.ws.sent[-1]['error'], 'That segment is still recording')
        self.resources.exporting = True
        await self.resources.export_clip(self.ws, self.request())
        self.assertEqual(self.ws.sent[-1]['error'], 'The Jetson is busy exporting another clip')
        self.assertEqual(self.uploads, [])


if __name__ == '__main__':
    unittest.main()
