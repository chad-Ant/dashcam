"""Device-side Firestore Listen RPC. Only CURRENT on a new target proves freshness."""
from collections.abc import AsyncIterator
from dataclasses import dataclass

import grpc
from google.cloud.firestore_v1 import _helpers, types


@dataclass(frozen=True)
class Snapshot:
    path: str
    fields: dict | None
    current: bool


async def listen(project: str, vehicle: str, token: str, emulator=False) -> AsyncIterator[Snapshot]:
    database = f'projects/{project}/databases/(default)'
    prefix = database + '/documents/'
    path = f'vehicles/{vehicle}/control/current'
    if emulator:
        if project != 'demo-dashcam':
            raise ValueError('Emulator requires demo-dashcam')
        channel = grpc.aio.insecure_channel('127.0.0.1:8080')
    else:
        channel = grpc.aio.secure_channel('firestore.googleapis.com:443', grpc.ssl_channel_credentials())
    async with channel:
        rpc = channel.stream_stream('/google.firestore.v1.Firestore/Listen',
                                    request_serializer=types.ListenRequest.serialize,
                                    response_deserializer=types.ListenResponse.deserialize)
        call = rpc(metadata=(('authorization', 'Bearer ' + token), ('google-cloud-resource-prefix', database)))
        await call.write(types.ListenRequest(database=database, add_target=types.Target(
            target_id=1, documents=types.Target.DocumentsTarget(documents=[prefix + path]))))
        pending = Snapshot(path, None, False)
        current = False
        dirty = False
        async for event in call:
            kind = types.ListenResponse.pb(event).WhichOneof('response_type')
            if kind == 'document_change':
                doc = event.document_change.document
                if doc.name != prefix + path:
                    raise ValueError('Unexpected control document')
                pending = Snapshot(doc.name[len(prefix):], _helpers.decode_dict(doc.fields, None), False)
                dirty = True
            elif kind in ('document_delete', 'document_remove'):
                pending = Snapshot(path, None, False)
                dirty = True
            elif kind == 'filter':
                raise ConnectionError('Listen target requires reconciliation')
            elif kind == 'target_change':
                change = event.target_change
                action = change.target_change_type.name
                if action == 'REMOVE':
                    if change.cause.code == 7:
                        raise PermissionError('Control access denied')
                    raise ConnectionError('Listen target removed')
                if action == 'RESET':
                    raise ConnectionError('Listen target reset')
                if action == 'CURRENT':
                    current, dirty = True, True
                # Firestore groups changes at a consistent snapshot boundary.
                if current and dirty and change.read_time:
                    yield Snapshot(pending.path, pending.fields, True)
                    dirty = False
        raise ConnectionError('Control listener closed')
