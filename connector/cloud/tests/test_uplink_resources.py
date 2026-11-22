"""Uplink task lifetimes: the normal end of an on-demand session must not restart the uplink.

Runs without the native controller or Firestore: both are replaced with fakes, and the
relay and LAN sockets are in-memory. From connector/cloud: python -m unittest tests.test_uplink_resources
"""
import asyncio
import sys
import types
import unittest
from pathlib import Path

import aiohttp

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.modules.setdefault('listener', types.SimpleNamespace(Snapshot=object, listen=None))


class State:
    def __init__(self, attempt=0, generation=0, requested=0, active=0):
        self.attempt, self.generation, self.requested, self.active, self.remaining = (
            attempt, generation, requested, active, -1)


class FakeController:
    def __init__(self, vehicle, reconcile):
        self.reconcile, self.current = reconcile, State()

    def state(self):
        return self.current

    def set(self, **fields):
        self.current = State(**fields)
        self.reconcile(self.current)

    def confirm(self, attempt, generation, channels):
        pass

    def close(self):
        pass


sys.modules['native'] = types.SimpleNamespace(Controller=FakeController)
import uplink  # noqa: E402


class Message:
    def __init__(self, kind, data=''):
        self.type, self.data = kind, data


class FakeSocket:
    def __init__(self):
        self.inbox = asyncio.Queue()
        self.sent = []

    async def receive(self, timeout=None):
        return await self.inbox.get()

    async def send_json(self, value):
        self.sent.append(value)

    async def send_str(self, value):
        self.sent.append(value)

    async def send_bytes(self, value):
        self.sent.append(value)

    def __aiter__(self):
        return self

    async def __anext__(self):
        msg = await self.inbox.get()
        if msg.type == aiohttp.WSMsgType.CLOSED:
            raise StopAsyncIteration
        return msg

    def close_from_peer(self):
        self.inbox.put_nowait(Message(aiohttp.WSMsgType.CLOSED))


class FakeHttp:
    def __init__(self):
        self.opened = []   # (path, socket)

    def ws_connect(self, url, **_):
        socket = FakeSocket()
        self.opened.append((str(url.path), socket))

        class Context:
            async def __aenter__(self):
                return socket

            async def __aexit__(self, *exc):
                return False
        return Context()

    def sockets(self, path):
        return [s for p, s in self.opened if p == path]


CONFIG = {'relay': 'https://relay.example/', 'vehicle': 'A', 'project': 'p',
          'local': {'host': '127.0.0.1', 'port': '8443', 'pin': '00' * 32, 'token': 't', 'id': 'dev'}}


async def settle():
    for _ in range(10):
        await asyncio.sleep(0)


class UplinkTaskLifetimeTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.http = FakeHttp()
        self.resources = uplink.Resources(self.http, CONFIG, 'token')
        self.core = self.resources.core

    async def asyncTearDown(self):
        for task in list(self.resources.tasks):
            task.cancel()
        await asyncio.gather(*self.resources.tasks, return_exceptions=True)

    async def start_session(self, attempt):
        self.core.set(attempt=attempt, generation=attempt, requested=1, active=1)
        await settle()

    async def end_session(self):
        self.core.set()   # idle: attempt 0, nothing active
        await settle()

    async def test_back_to_back_short_sessions_keep_the_uplink(self):
        for attempt in range(1, 7):
            await self.start_session(attempt)
            self.assertEqual(len(self.http.sockets('/v1/uplink')), attempt, 'one relay socket per session')
            self.assertIn(1, self.resources.readers)
            await self.end_session()
            self.assertFalse(self.resources.failure.done(), f'session {attempt} ended as a failure')
            self.assertEqual(self.resources.readers, {})

    async def test_reader_that_notices_its_channel_ended_is_not_a_failure(self):
        await self.start_session(1)
        reader = self.resources.readers[1]
        # The controller drops telemetry before reconcile runs; the reader sees it on its next message.
        self.core.current = State(attempt=1, generation=1, requested=1, active=0)
        self.http.sockets('/v1/stream')[0].inbox.put_nowait(Message(aiohttp.WSMsgType.TEXT, '{}'))
        await settle()
        self.assertTrue(reader.done())
        self.assertFalse(self.resources.failure.done())
        self.assertNotIn(1, self.resources.readers)
        self.core.set(attempt=1, generation=2, requested=1, active=1)
        await settle()
        self.assertIsNot(self.resources.readers[1], reader, 'a renewed channel gets a fresh reader')

    async def test_relay_closing_a_current_session_still_restarts(self):
        await self.start_session(1)
        self.http.sockets('/v1/uplink')[0].close_from_peer()
        await settle()
        self.assertTrue(self.resources.failure.done())
        self.assertIsInstance(self.resources.failure.result(), ConnectionError)

    async def test_lan_stream_ending_during_a_session_still_restarts(self):
        await self.start_session(1)
        self.http.sockets('/v1/stream')[0].close_from_peer()
        await settle()
        self.assertTrue(self.resources.failure.done())


if __name__ == '__main__':
    unittest.main()
