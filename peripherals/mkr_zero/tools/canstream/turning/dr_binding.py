"""Optional ctypes bridge to the real Orin C++ library (not a Python port)."""
import ctypes
import math


class Result(ctypes.Structure):
    _fields_ = [(k, ctypes.c_double) for k in
                ("x_m", "y_m", "heading_rad", "distance_m", "speed_mps", "yaw_radps")] + [
                (k, ctypes.c_int) for k in ("status", "continuous", "limited")]


class DeadReckoning:
    def __init__(self, path, profile):
        self.lib = ctypes.CDLL(str(path))
        self.lib.dr_create.argtypes = [ctypes.c_double] * 6
        self.lib.dr_create.restype = ctypes.c_void_p
        self.lib.dr_destroy.argtypes = [ctypes.c_void_p]
        self.lib.dr_destroy.restype = None
        self.lib.dr_reset.argtypes = [ctypes.c_void_p]
        self.lib.dr_reset.restype = ctypes.c_int
        self.lib.dr_update.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_uint64,
                                      ctypes.c_uint16, ctypes.c_uint16, ctypes.c_int, ctypes.c_int,
                                      ctypes.POINTER(Result)]
        self.lib.dr_update.restype = ctypes.c_int
        self.lib.dr_status_name.argtypes = [ctypes.c_int]
        self.lib.dr_status_name.restype = ctypes.c_char_p
        self.handle = self.lib.dr_create(profile.wheelbase_m, profile.wheel_kmh_per_count,
                                         profile.yaw_dps_per_count, profile.mismatch_ratio,
                                         profile.filter_tau_s, profile.max_speed_kmh / 3.6)
        if not self.handle:
            raise ValueError("dead-reckoning library refused configuration")
        self.segment = None

    def close(self):
        if self.handle:
            self.lib.dr_destroy(self.handle)
            self.handle = None

    def update(self, row):
        if self.segment != row['segment']:
            if not self.lib.dr_reset(self.handle):
                raise ValueError("dead-reckoning reset failed")
            self.segment = row['segment']
        stamp = int(row['time'] * 1e9)
        if not 0 <= stamp < 2 ** 64:
            raise ValueError("timestamp outside C++ nanosecond range")
        gear = 1 if row['gear_raw'] in (4, 7, 10) else 0
        result = Result()
        # Replayed acquisition clock; not delivery clock. The demo separately
        # gates live source latency. Wall-clock jumps are NOT repaired here.
        if not self.lib.dr_update(self.handle, stamp, stamp, row['wheel_rl'], row['wheel_rr'],
                                  gear, int(row['status'] == 'ok'), ctypes.byref(result)):
            raise ValueError("dead-reckoning update failed")
        return {"dr_x_m": result.x_m, "dr_y_m": result.y_m,
                "dr_heading_deg": math.degrees(result.heading_rad), "dr_distance_m": result.distance_m,
                "dr_status": self.lib.dr_status_name(result.status).decode('ascii'),
                "dr_continuous": bool(result.continuous), "dr_limited": bool(result.limited)}
