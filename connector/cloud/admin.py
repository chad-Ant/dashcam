"""Administrator bootstrap with application default credentials."""
import argparse
import asyncio
from google.cloud import firestore
from store import INITIAL, Store


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--project', required=True)
    parser.add_argument('command', choices=['grant', 'remove-grant', 'revoke-device'])
    parser.add_argument('vehicle')
    parser.add_argument('--uid')
    parser.add_argument('--role', choices=['owner', 'viewer'], default='viewer')
    parser.add_argument('--label')
    args = parser.parse_args()
    db = firestore.AsyncClient(project=args.project)
    store = Store(db)
    v = store.vehicle(args.vehicle)
    if args.command == 'revoke-device':
        batch = db.batch()
        batch.update(v, dict(pendingCode=firestore.DELETE_FIELD))
        batch.update(v.collection('control').document('current'), {'deviceUid': None})
        await batch.commit()
    elif not args.uid:
        parser.error('--uid is required for grants')
    elif args.command == 'remove-grant':
        await store.grant(args.uid, args.vehicle).delete()
    else:
        @firestore.async_transactional
        async def grant(tx):
            if not (await v.get(transaction=tx)).exists:
                tx.create(v, dict(leases={}))
                tx.create(v.collection('control').document('current'), INITIAL)
            tx.set(store.grant(args.uid, args.vehicle), dict(label=args.label or args.vehicle, role=args.role))
        await grant(db.transaction())


if __name__ == '__main__':
    asyncio.run(main())
