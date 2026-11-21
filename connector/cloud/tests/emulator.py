"""Run under firebase emulators:exec. Never initialize a production client."""
import asyncio
import json
import os
import sys
import time
from pathlib import Path

import aiohttp
import firebase_admin
from firebase_admin import auth, credentials
from google.auth.credentials import AnonymousCredentials
from google.cloud import firestore

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from listener import listen
from native import Controller
from store import INITIAL, Store

PROJECT = 'demo-dashcam'
AUTH = 'http://127.0.0.1:9099/identitytoolkit.googleapis.com/v1/accounts:signInWithCustomToken'
REST = 'http://127.0.0.1:8080/v1/projects/demo-dashcam/databases/(default)/documents'


def guard():
    assert os.environ.get('GCLOUD_PROJECT') == PROJECT
    assert os.environ.get('FIRESTORE_EMULATOR_HOST') == '127.0.0.1:8080'
    assert os.environ.get('FIREBASE_AUTH_EMULATOR_HOST') == '127.0.0.1:9099'


class LocalCredential(credentials.Base):
    def get_credential(self):
        return AnonymousCredentials()


async def client(key):
    token = sys.stdin.readline().strip()
    def report(s):
        print(json.dumps(dict(attempt=s.attempt, generation=s.generation, active=s.active,
                              requested=s.requested)), flush=True)
    core = Controller(key, report)
    try:
        first = True
        async for snap in listen(PROJECT, key, token, emulator=True):
            if first:
                assert snap.path == f'vehicles/{key}/control/current' and snap.current
                if snap.fields['demandGeneration'] == 0:
                    assert all(snap.fields[k] == v for k, v in INITIAL.items())
                first = False
            core.snapshot(snap)
            s = core.state()
            if s.requested and not s.active:
                core.confirm(s.attempt, s.generation, ['telemetry'])
    except PermissionError:
        print('{"denied":true}', flush=True)
    finally:
        core.close()


async def device(key, uid, http):
    custom = auth.create_custom_token(uid).decode()
    async with http.post(AUTH, params={'key':'demo'}, json=dict(token=custom, returnSecureToken=True)) as r:
        r.raise_for_status()
        token = (await r.json())['idToken']
    proc = await asyncio.create_subprocess_exec(sys.executable, __file__, key, stdin=asyncio.subprocess.PIPE,
                                                stdout=asyncio.subprocess.PIPE)
    proc.stdin.write((token + '\n').encode())
    await proc.stdin.drain()
    proc.stdin.close()
    return proc, {'Authorization': 'Bearer ' + token}


async def state(proc):
    line = await asyncio.wait_for(proc.stdout.readline(), 36)
    assert line, 'device process ended before expected listener event'
    data = json.loads(line)
    print(data, flush=True)
    return data


async def test():
    firebase_admin.initialize_app(LocalCredential(), options={'projectId':PROJECT})
    db = firestore.AsyncClient(project=PROJECT, credentials=AnonymousCredentials())
    store = Store(db)
    for key in ('A','B'):
        v = store.vehicle(key)
        await v.set(dict(leases={}))
        await v.collection('control').document('current').set(dict(INITIAL, deviceUid='sim-'+key))
    await store.grant('viewer', 'A').set(dict(role='viewer', label='A'))
    assert [v['id'] for v in await store.vehicles('viewer')] == ['A']
    assert not await store.vehicles('unassigned')
    ref = store.vehicle('A').collection('control').document('current')
    processes = []
    async with aiohttp.ClientSession() as http:
        try:
            a, _ = await device('A', 'sim-A', http)
            processes.append(a)
            b, headers = await device('B', 'sim-B', http)
            processes.append(b)
            for proc in (a,b):
                assert (await state(proc)) == dict(attempt=0, generation=0, active=0, requested=0)
            for method, key, status in [('GET','B',200), ('GET','A',403), ('PATCH','B',403)]:
                async with http.request(method, REST + '/vehicles/'+key+'/control/current',
                                        headers=headers, json={} if method=='PATCH' else None) as r:
                    print('B', method, key, r.status)
                    assert r.status == status
            start = time.monotonic()
            await store.update('A','viewer', session='first', channels=['telemetry'])
            pending, live = await state(a), await state(a)
            assert pending == dict(attempt=1,generation=1,active=0,requested=1)
            assert live == {**pending,'active':1}
            assert (await state(a))['attempt'] == 0
            elapsed = time.monotonic()-start
            assert 28 <= elapsed <= 33
            print('Lease released in %.2fs' % elapsed)
            a.terminate()
            await a.wait()
            a, _ = await device('A','sim-A',http)
            processes.append(a)
            assert (await state(a))['attempt'] == 0
            doc = await store.update('A','viewer',session='second',channels=['telemetry'])
            assert doc['demandGeneration'] == 2
            assert (await state(a))['requested'] == 1
            assert (await state(a))['active'] == 1
            start = time.monotonic()
            await store.update('A','viewer',session='second')
            assert (await state(a))['attempt'] == 0
            print('Explicit leave released in %.2fs' % (time.monotonic()-start))
            await store.grant('owner','A').set(dict(role='owner',label='A'))
            code = await store.issue('A','owner')
            assert (await store.exchange(code))[0] == 'A'
            try:
                await store.exchange(code)
            except PermissionError:
                pass
            else:
                raise AssertionError('Enrollment reused')
            assert (await state(a))['denied']
            assert (await ref.get()).get('channels') == []
            await store.update('A','owner',mode='ready_while_powered',version=1)
            print('Revocation, replacement enrollment and owner mode update passed')
        finally:
            for proc in processes:
                if proc.returncode is None:
                    proc.terminate()
                await proc.wait()


if __name__ == '__main__':
    guard()
    asyncio.run(client(sys.argv[1]) if len(sys.argv) > 1 else test())
