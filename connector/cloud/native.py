"""The native controller and its asyncio deadline share one owning event loop."""
import asyncio
import ctypes as c
from datetime import datetime
from pathlib import Path


class State(c.Structure):
    _fields_ = [('attempt', c.c_uint64), ('generation', c.c_int64), ('requested', c.c_int),
                ('active', c.c_int), ('remaining', c.c_double)]


def library():
    lib = c.CDLL(str(Path(__file__).with_name('libdashcam_uplink.so')))
    for name, result, args in (
        ('create', c.c_void_p, [c.c_char_p]), ('destroy', None, [c.c_void_p]),
        ('apply', c.c_int, [c.c_void_p, c.c_char_p, c.c_bool, c.c_bool, c.c_int64,
                            c.c_int64, c.c_int, c.c_double]),
        ('confirm', None, [c.c_void_p, c.c_uint64, c.c_int64, c.c_int]),
        ('state', State, [c.c_void_p]),
    ):
        fn = getattr(lib, 'dc_' + name)
        fn.argtypes, fn.restype = args, result
    return lib


def mask(channels):
    if not isinstance(channels, list) or any(c not in ('telemetry', 'video') for c in channels):
        raise ValueError('Invalid control channels')
    return int('telemetry' in channels) | 2 * int('video' in channels)


class Controller:
    def __init__(self, vehicle, reconcile):
        self.lib = library()
        self.handle = self.lib.dc_create(vehicle.encode())
        if not self.handle:
            raise ValueError('Invalid vehicle')
        self.reconcile = reconcile
        self.timer = None

    def state(self):
        return self.lib.dc_state(self.handle)

    def update(self):
        if self.timer:
            self.timer.cancel()
        state = self.state()
        if state.remaining >= 0:
            self.timer = asyncio.get_running_loop().call_later(state.remaining, self.update)
        self.reconcile(state)

    def snapshot(self, event):
        f = event.fields
        if not f:
            raise PermissionError('Control document removed')
        mode = f['desiredMode']
        versions = (f['configVersion'], f['demandGeneration'])
        if mode not in ('on_demand', 'ready_while_powered') or any(
                type(v) is not int or not 0 <= v < 2**63 for v in versions):
            raise ValueError('Invalid control policy')
        expiry = f['expiresAt']
        if expiry is not None and not isinstance(expiry, datetime):
            raise ValueError('Expected Firestore timestamp')
        result = self.lib.dc_apply(self.handle, event.path.encode(), event.current,
                                   mode == 'ready_while_powered', *versions, mask(f['channels']),
                                   expiry.timestamp() if expiry else float('nan'))
        if result == 2:
            raise ValueError('Inconsistent control document')
        self.update()

    def confirm(self, attempt, generation, channels):
        self.lib.dc_confirm(self.handle, attempt, generation, mask(channels))
        self.update()

    def close(self):
        if self.timer:
            self.timer.cancel()
        self.lib.dc_destroy(self.handle)
        self.handle = None
