"""Opt-in integration test: real v0.4 without host camera/serial devices.

Run from the isolated validation checkout on the Jetson host, with the Python
requirements installed. Does not modify the deployed checkout or services.
"""
import asyncio
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile

from aiohttp import ClientSession, Fingerprint
from service import initialize, load_identity


async def smoke():
    root = Path(__file__).resolve().parent.parent
    # Newest timestamped build (bin/build_<YYYYMMDD>_<HHMMSS>); ad-hoc names such as build_kz3_* sort after them.
    build = sorted((root / 'bin').glob('build_[0-9]*_[0-9]*/dashcam_v0_4'))[-1]
    with tempfile.TemporaryDirectory(prefix='connector-smoke-', dir=root) as directory:
        test = Path(directory)
        state = test / 'identity'
        initialize(state, 'Isolated v0.4 validation')
        identity = load_identity(state)
        with socket.socket() as port_socket:
            port_socket.bind(('127.0.0.1', 0))
            port = port_socket.getsockname()[1]
        ipc = test / 'telemetry.sock'
        config = test / 'configs'
        config.mkdir()
        inside = '/work/' + test.name
        (config / 'dashcam.xml').write_text(f'''<DashcamConfig>
<Network><MonitoringEnabled>true</MonitoringEnabled><MonitoringSocket>{inside}/telemetry.sock</MonitoringSocket>
<TimeSyncEnabled>false</TimeSyncEnabled><ClockSetEnabled>false</ClockSetEnabled></Network>
<DisplayPreview><Enabled>false</Enabled></DisplayPreview><Navigation><Enabled>false</Enabled></Navigation>
</DashcamConfig>''')
        for name in ('footage', 'logs'):
            (test / name).mkdir()
        name = 'dashcam-connector-smoke-' + str(os.getpid())
        with (test / 'agent.log').open('w') as agent_log, (test / 'runtime.log').open('w') as runtime_log:
            agent = subprocess.Popen([sys.executable, str(root / 'connector/service.py'), 'serve',
                                      '--state-dir', str(state), '--bind', '127.0.0.1', '--port', str(port),
                                      '--socket', str(ipc), '--storage', str(test / 'footage')],
                                     stdout=agent_log, stderr=agent_log)
            runtime = None
            try:
                for _ in range(100):
                    if ipc.exists():
                        break
                    if agent.poll() is not None:
                        raise RuntimeError('Agent exited before creating IPC socket')
                    await asyncio.sleep(.1)
                runtime = subprocess.Popen(['docker', 'run', '--rm', '--name', name,
                    '--network', 'none', '--cap-drop', 'ALL', '--user', str(os.getuid()) + ':' + str(os.getgid()),
                    '--volume', str(root) + ':/work',
                    '--volume', str(config) + ':/user/output/configs',
                    '--volume', str(test / 'footage') + ':/user/output/footage',
                    '--volume', str(test / 'logs') + ':/user/output/logs',
                    '--workdir', '/work', 'l4t-ml-gpio', '/work/' + str(build.relative_to(root))],
                    stdout=runtime_log, stderr=runtime_log)
                async with ClientSession() as client:
                    fingerprint = Fingerprint(bytes.fromhex(identity['pin']))
                    headers = {'Authorization': 'Bearer ' + identity['token']}
                    async with client.ws_connect(f'https://127.0.0.1:{port}/v1/stream', ssl=fingerprint,
                                                 headers=headers) as ws:
                        found = None
                        for _ in range(100):
                            data = await ws.receive_json(timeout=3)
                            if data['runtime_fresh']:
                                found = data
                                break
                        if found is None:
                            runtime_log.flush()
                            print((test / 'runtime.log').read_text()[-6000:])
                            print((test / 'agent.log').read_text()[-2000:])
                            raise AssertionError('v0.4 did not publish via Unix IPC')
                        assert found['runtime']['runtime'] == 'dashcam_v0_4'
                        assert found['runtime']['recording'] is False
                        assert found['runtime']['bridge_fresh'] is False
                        assert found['runtime']['vehicle']['speed_kmh'] is None
                        assert found['system']['ram_total_bytes'] > 0
                        subprocess.run(['docker', 'stop', '--time', '5', name], check=True, stdout=subprocess.DEVNULL)
                        for _ in range(30):
                            expired = await ws.receive_json(timeout=3)
                            if not expired['runtime_fresh']:
                                break
                        assert expired['runtime'] is None, 'Stopped v0.4 data did not expire'
                        assert expired['system']['ram_total_bytes'] > 0
                        print('PASS: real v0.4 -> Unix datagram -> host health -> pinned TLS WebSocket; stopped runtime expired')
            finally:
                if runtime and runtime.poll() is None:
                    subprocess.run(['docker', 'stop', '--time', '5', name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    runtime.wait(timeout=10)
                agent.terminate()
                agent.wait(timeout=10)


if __name__ == '__main__':
    asyncio.run(smoke())
