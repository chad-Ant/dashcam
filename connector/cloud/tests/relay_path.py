"""TLS relay → native adapter → existing LAN connector with synthetic recorder data."""
import asyncio
import ssl
import tempfile
from contextlib import AsyncExitStack
from functools import partial
from pathlib import Path
from unittest.mock import patch
from urllib.parse import urlencode

import aiohttp
from aiohttp.test_utils import TestServer
import jwt
from emulator import firebase_admin, firestore, LocalCredential, AnonymousCredentials
from emulator import guard, PROJECT, AUTH, INITIAL, Store
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import service
from relay import application
from uplink import Resources
from listener import listen


async def eventual(check):
    for _ in range(100):
        if check():
            return
        await asyncio.sleep(.05)
    raise AssertionError('Expected resource transition')


async def google_user(http, name):
    # Auth emulator's documented mock Google credential.
    assertion = jwt.encode(dict(sub=name, email=name+'@example.test', email_verified=True), '', algorithm='none')
    body = dict(requestUri='http://localhost', returnSecureToken=True,
                postBody=urlencode(dict(providerId='google.com', id_token=assertion)))
    async with http.post(AUTH.replace('signInWithCustomToken','signInWithIdp'), params={'key':'demo'}, json=body) as r:
        r.raise_for_status()
        result = await r.json()
    return result['localId'], {'Authorization': 'Bearer ' + result['idToken']}


async def run():
    guard()
    firebase_admin.initialize_app(LocalCredential(), options={'projectId':PROJECT})
    store = Store(firestore.AsyncClient(project=PROJECT, credentials=AnonymousCredentials()))
    with tempfile.TemporaryDirectory() as tmp, patch('uplink.listen', partial(listen, emulator=True)):
        path = Path(tmp)
        service.initialize(path, 'TLS test', ['localhost','127.0.0.1'])
        identity = service.load_identity(path)
        tls = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        tls.load_cert_chain(path/'cert.pem', path/'key.pem')
        trust = ssl.create_default_context(cafile=str(path/'ca.pem'))
        async with AsyncExitStack() as stack:
            http = await stack.enter_async_context(aiohttp.ClientSession(connector=aiohttp.TCPConnector(ssl=trust)))
            user, headers = await google_user(http,'pilot')
            _, other = await google_user(http,'uninvited')
            config = dict(projectId=PROJECT,apiKey='demo')
            relay = TestServer(application(store, config), scheme='https')
            await relay.start_server(ssl=tls)
            stack.push_async_callback(relay.close)
            url = str(relay.make_url('/')).rstrip('/')
            states, resources, tasks = {}, {}, []
            async def cleanup():
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
            for key in ('tlsA','tlsB'):
                await store.vehicle(key).set(dict(leases={}))
                await store.vehicle(key).collection('control').document('current').set(INITIAL)
                await store.grant(user,key).set(dict(role='owner',label=key))
                async with http.post(url+'/v1/vehicles/'+key+'/enrollment', headers=headers) as r:
                    r.raise_for_status()
                    ticket = await r.json()
                async with http.post(url+'/v1/enroll',json=ticket) as r:
                    r.raise_for_status()
                    exchange = await r.json()
                async with http.post(AUTH,params={'key':'demo'},json=dict(token=exchange['token'],returnSecureToken=True)) as r:
                    token = (await r.json())['idToken']
                local = dict(identity, id=key+'-local')
                state = service.State(local,tmp)
                states[key] = state
                lan = TestServer(service.create_app(state), scheme='https')
                await lan.start_server(ssl=tls)
                stack.push_async_callback(lan.close)
                local.update(host='127.0.0.1',port=lan.port)
                device = Resources(http,dict(vehicle=key,local=local,project=PROJECT,relay=url),token)
                resources[key] = device
                tasks.extend([asyncio.create_task(device.run(300)),asyncio.create_task(service.simulate(state)),
                              asyncio.create_task(service.simulate_video(state,(Path(service.__file__).parent/'testcard.jpg').read_bytes()))])
            stack.push_async_callback(cleanup)
            assert await (await http.get(url+'/v1/vehicles',headers=other)).json() == []
            try:
                await http.ws_connect(url+'/v1/view/tlsA',headers=other)
            except aiohttp.WSServerHandshakeError as e:
                assert e.status == 403
            else:
                raise AssertionError('Unauthorized viewer admitted')
            async with http.post(url+'/v1/session',headers=dict(headers, Origin=url)) as r:
                r.raise_for_status()
                cookie = r.cookies['__Host-dashcam']
                assert cookie['secure'] and cookie['httponly']
            browser = {'Cookie':'__Host-dashcam='+cookie.value, 'Origin':url}
            # A cold request for an offline vehicle is cancellable before any uplink registers.
            await store.vehicle('offline').set(dict(leases={}))
            offline = store.vehicle('offline').collection('control').document('current')
            await offline.set(INITIAL)
            await store.grant(user,'offline').set(dict(role='viewer',label='Offline'))
            denied = await http.put(url+'/v1/vehicles/offline/policy',headers=headers,
                                    json=dict(mode='ready_while_powered',version=1))
            assert denied.status == 403
            async with http.ws_connect(url+'/v1/view/offline',headers=headers) as waiting:
                await waiting.send_json(dict(channels=['telemetry']))
                for _ in range(50):
                    if (await offline.get()).get('channels') == ['telemetry']:
                        break
                    await asyncio.sleep(.05)
                else:
                    raise AssertionError('Offline request failed to commit')
            for _ in range(50):
                if (await offline.get()).get('channels') == []:
                    break
                await asyncio.sleep(.05)
            else:
                raise AssertionError('Offline cancellation failed')
            viewers = []
            for index,key in enumerate(('tlsA','tlsA','tlsA','tlsB','tlsB')):
                ws = await stack.enter_async_context(http.ws_connect(url+'/v1/view/'+key,headers=browser if index==0 else headers))
                viewers.append(ws)
                await ws.send_json(dict(channels=['telemetry']))
                ack = await ws.receive_json(timeout=3)
                assert ack['type'] == 'connecting' and ack['requestId']
                item = await ws.receive_json(timeout=8)
                assert item['device_id']==key+'-local' and item['runtime_fresh']
                assert (await ws.receive_json(timeout=3))['seq']>item['seq']
            assert all(len(s.clients)==1 and not s.video_clients for s in states.values())
            await viewers[0].send_json(dict(channels=['telemetry','video']))
            while True:
                packet = await viewers[0].receive(timeout=5)
                if packet.type==aiohttp.WSMsgType.BINARY:
                    assert packet.data.startswith(b'DCV1')
                    break
                assert packet.type==aiohttp.WSMsgType.TEXT
            assert len(states['tlsA'].video_clients)==1 and not states['tlsB'].video_clients
            await viewers[0].close()
            await eventual(lambda: not states['tlsA'].video_clients and len(states['tlsA'].clients)==1)
            async def mode(desired,version):
                async with http.put(url+'/v1/vehicles/tlsA/policy',headers=headers,json=dict(mode=desired,version=version)) as r:
                    r.raise_for_status()
            await mode('ready_while_powered',1)
            await eventual(lambda: resources['tlsA'].watcher is None)
            for ws in viewers[1:3]:
                await ws.close()
            await eventual(lambda: not states['tlsA'].clients and resources['tlsA'].socket is not None)
            assert states['tlsB'].clients
            await mode('on_demand',2)
            await eventual(lambda: not resources['tlsA'].attempt and resources['tlsA'].watcher is not None)
            # A lost grant must close an already subscribed viewer on its next renewal.
            await store.grant(user,'tlsB').delete()
            await viewers[3].send_json(dict(channels=['telemetry']))
            for _ in range(100):
                if (await viewers[3].receive(timeout=3)).type == aiohttp.WSMsgType.CLOSE:
                    break
            assert viewers[3].close_code == 1008
            await viewers[4].close()
            await eventual(lambda: not resources['tlsB'].attempt and not states['tlsB'].clients)
            print('Five TLS viewers, two vehicles, one reader per requested channel, camera release, '
                  'mode handoff and open-session revocation passed')


if __name__=='__main__':
    asyncio.run(run())
