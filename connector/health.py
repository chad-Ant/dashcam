"""Host-only, best-effort Linux health sampling. No subprocesses or root required."""
import time
from pathlib import Path


def read(path):
    try:
        return Path(path).read_text().strip()
    except (OSError, UnicodeError):
        return None


def numeric(path, scale=1):
    try:
        return float(read(path)) / scale
    except (TypeError, ValueError):
        return None


class Health:
    def __init__(self, storage, proc=Path('/proc'), sys=Path('/sys')):
        self.storage, self.proc, self.sys = Path(storage), Path(proc), Path(sys)
        self.previous = None

    def sample(self):
        out = {'scope': 'host', 'sample_mono_s': time.monotonic(), 'cpu_percent': None,
               'gpu_percent': None, 'ram_used_bytes': None, 'ram_total_bytes': None,
               'swap_used_bytes': None, 'swap_total_bytes': None,
               'storage_free_bytes': None, 'storage_total_bytes': None,
               'uptime_s': None, 'temperatures_c': {}}
        try:
            values = list(map(int, (read(self.proc / 'stat') or '').splitlines()[0].split()[1:9]))
            total, idle = sum(values), values[3] + values[4]
            if self.previous and total > self.previous[0]:
                out['cpu_percent'] = max(0., min(100., 100 * (1 - (idle - self.previous[1]) / (total - self.previous[0]))))
            self.previous = total, idle
        except (IndexError, ValueError):
            pass
        mem = {}
        for line in (read(self.proc / 'meminfo') or '').splitlines():
            try:
                key, value = line.split(':', 1)
                mem[key] = int(value.split()[0]) * 1024
            except (ValueError, IndexError):
                continue
        for name, total, free in [('ram', 'MemTotal', 'MemAvailable'), ('swap', 'SwapTotal', 'SwapFree')]:
            if total in mem and free in mem:
                out[name + '_total_bytes'] = mem[total]
                out[name + '_used_bytes'] = max(0, mem[total] - mem[free])
        try:
            out['uptime_s'] = float((read(self.proc / 'uptime') or '').split()[0])
        except (ValueError, IndexError):
            pass
        # Resolve names, never assume thermal_zone0 is the CPU.
        for zone in sorted((self.sys / 'class/thermal').glob('thermal_zone*'))[:64]:
            name, value = read(zone / 'type'), numeric(zone / 'temp', 1000)
            if name and value is not None and -50 <= value <= 150:
                out['temperatures_c'][name] = value
        candidates = list((self.sys / 'class/devfreq').glob('*gpu*/load'))
        candidates += list((self.sys / 'devices/platform').glob('*gpu/load'))
        for path in candidates[:16]:
            value = numeric(path, 10)  # Tegra GPU load is in tenths of a percent.
            if value is not None and 0 <= value <= 100:
                out['gpu_percent'] = value
                break
        try:
            import shutil
            usage = shutil.disk_usage(self.storage)
            out['storage_free_bytes'], out['storage_total_bytes'] = usage.free, usage.total
        except OSError:
            pass
        return out
