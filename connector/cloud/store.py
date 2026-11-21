"""Relay-only Firestore writes. Each vehicle's bounded lease set is one transaction."""
import asyncio
import hashlib
import re
import secrets
from datetime import datetime, timedelta, timezone
from weakref import WeakValueDictionary

from google.cloud import firestore

MODES = ('on_demand', 'ready_while_powered')
INITIAL = dict(desiredMode=MODES[0], configVersion=1, demandGeneration=0, channels=[], expiresAt=None)
CAPACITY = 5


class Conflict(Exception):
    pass


def vehicle_id(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', value):
        raise ValueError('Invalid vehicle ID')
    return value


def digest(code):
    return hashlib.sha256(code.encode()).hexdigest()


class Store:
    def __init__(self, db):
        self.db = db
        self.writers = WeakValueDictionary()

    def vehicle(self, key):
        return self.db.collection('vehicles').document(vehicle_id(key))

    def grant(self, uid, key):
        return self.db.collection('users').document(uid).collection('vehicles').document(vehicle_id(key))

    async def vehicles(self, uid):
        return [dict(id=item.id, **item.to_dict()) async for item in
                self.db.collection('users').document(uid).collection('vehicles').stream()]

    async def device_control(self, key, uid):
        doc = (await self.vehicle(key).collection('control').document('current').get()).to_dict()
        if not doc or doc.get('deviceUid') != uid:
            raise PermissionError
        return doc

    async def update(self, key, uid, *, session=None, channels=None, mode=None, version=None):
        vehicle = self.vehicle(key)
        control = vehicle.collection('control').document('current')

        @firestore.async_transactional
        async def write(tx):
            grant = (await self.grant(uid, key).get(transaction=tx)).to_dict() or {}
            data = (await vehicle.get(transaction=tx)).to_dict()
            doc = (await control.get(transaction=tx)).to_dict()
            if not data or not doc:
                raise PermissionError
            now = datetime.now(timezone.utc)
            leases = {k: v for k, v in data.get('leases', {}).items() if v['until'] > now}
            if mode is not None:
                if grant.get('role') != 'owner':
                    raise PermissionError
                if mode not in MODES:
                    raise ValueError('Invalid mode')
                if mode != doc['desiredMode']:
                    if version != doc['configVersion']:
                        raise Conflict
                    doc.update(desiredMode=mode, configVersion=version + 1)
            if session is not None:
                if channels is None:
                    leases.pop(session, None)
                else:
                    if grant.get('role') not in ('owner', 'viewer'):
                        raise PermissionError
                    if session not in leases and len(leases) >= CAPACITY:
                        raise Conflict
                    leases[session] = dict(channels=channels, until=now + timedelta(seconds=30))
            wanted = sorted({c for lease in leases.values() for c in lease['channels']})
            if wanted != doc['channels'] or (wanted and doc['expiresAt'] <= now):
                doc['demandGeneration'] += 1
            doc.update(channels=wanted, expiresAt=max((x['until'] for x in leases.values()), default=None))
            if channels is not None:
                doc['requestId'] = session
            tx.update(vehicle, {'leases': leases})
            tx.set(control, doc)
            return doc
        # Simultaneous leaves on this instance should not contend for read/write
        # locks in Firestore. The transaction still protects cross-revision writes.
        async with self.writers.setdefault(key, asyncio.Lock()):
            return await write(self.db.transaction())

    async def issue(self, key, uid):
        code = secrets.token_urlsafe(24)
        hashed = digest(code)
        vehicle = self.vehicle(key)
        @firestore.async_transactional
        async def write(tx):
            grant = (await self.grant(uid, key).get(transaction=tx)).to_dict() or {}
            if grant.get('role') != 'owner':
                raise PermissionError
            tx.update(vehicle, {'pendingCode': hashed})
            tx.create(self.db.collection('enrollment').document(hashed),
                      dict(vehicle=key, until=datetime.now(timezone.utc) + timedelta(minutes=5)))
        await write(self.db.transaction())
        return code

    async def exchange(self, code):
        hashed = digest(code)
        ref = self.db.collection('enrollment').document(hashed)
        uid = 'device-' + secrets.token_hex(20)
        @firestore.async_transactional
        async def write(tx):
            ticket = (await ref.get(transaction=tx)).to_dict()
            if not ticket or ticket['until'] <= datetime.now(timezone.utc):
                raise PermissionError
            vehicle = self.vehicle(ticket['vehicle'])
            state = (await vehicle.get(transaction=tx)).to_dict()
            if not state or state.get('pendingCode') != hashed:
                raise PermissionError
            tx.update(vehicle.collection('control').document('current'), {'deviceUid': uid})
            tx.delete(ref)
            tx.update(vehicle, {'pendingCode': firestore.DELETE_FIELD})
            return ticket['vehicle']
        return await write(self.db.transaction()), uid
