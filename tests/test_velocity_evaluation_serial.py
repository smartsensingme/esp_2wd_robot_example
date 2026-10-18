"""Full Octave capture/replay via PTY, plus rejection of an incomplete stream."""
import math
import os
import pty
import select
import subprocess
import tempfile
import time
from pathlib import Path

root = Path(__file__).resolve().parents[1]
os.chdir(root)
n = 2400
source = ''.join(f'{(i+1)*.005:.12g},{i*17*2*math.pi/16384:.17g}\n' for i in range(n))
velocities = subprocess.run(
    [str(root / 'build/native/velocity-replay'), str(.05*math.pi/180), '20', '.025'],
    input=source, capture_output=True, text=True, check=True,
).stdout.splitlines()
header = ('ESTIMATOR_CAPTURE/1 wheel=left samples=2400 period_us=5000 stages=3 '
          'sigma_deg=0.05 accel_std=20 tau_s=0.025 encoder_sign=1 ratio=1 '
          'lut_crc32=0000007B releases=2400\n')
rows = []
for i in range(n):
    counts = i * 17 % 16384
    duty = 90 if 800 <= i < 1600 else 70
    rows.append(f'{i},{(i+1)*5000},{counts},{counts},{duty},{int(i>0)},'
                f'{velocities[i]},180,40\n')

for truncated in (False, True):
    with tempfile.TemporaryDirectory(dir=root / 'build/native') as folder:
        master, slave = pty.openpty()
        port = os.ttyname(slave)
        call = f'robot_evaluate_velocity("{port}","left","{folder}");'
        if truncated:
            call = ('try; ' + call + ' error("UNEXPECTED_SUCCESS"); catch err; '
                    'assert(!isempty(strfind(err.message,"Incomplete evaluation transfer"))); end;')
        code = 'addpath("components/esp_2wd_robot/tools/octave"); ' + call
        process = subprocess.Popen(['octave', '--quiet', '--eval', code],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        process.stdin.write('TESTAR LEFT\n')
        process.stdin.flush()
        command = b''
        sent = False
        try:
            deadline = time.monotonic() + 15
            while not sent and time.monotonic() < deadline:
                if not select.select([master], [], [], .1)[0]:
                    if process.poll() is not None:
                        break
                    continue
                command += os.read(master, 4096)
                while b'\n' in command:
                    line, command = command.split(b'\n', 1)
                    if line.strip() == b'MODE':
                        os.write(master, b'> OK command=MODE mode=calibration\n')
                    elif line.strip() == b'CAL STATUS left':
                        os.write(master, b'OK command=CAL_STATUS wheel=left loaded=1 enabled=1 '
                                 b'bins=256 full_scale_counts=16384 crc32=0000007B\n')
                    elif line.startswith(b'ESTIMATE START left'):
                        payload = header + 'sample,timestamp_us,raw_counts\n'
                        payload += ''.join(rows[:-1] if truncated else rows)
                        payload += 'END-ESTIMATOR\nOK command=ESTIMATE_START wheel=left\n'
                        payload = payload.encode()
                        for offset in range(0, len(payload), 127):
                            os.write(master, payload[offset:offset+127])
                        sent = True
            assert sent, process.communicate(timeout=5)[0]
            output, _ = process.communicate(timeout=15)
            assert process.returncode == 0, output
            saved = list(Path(folder).glob('eval_left_*.mat'))
            assert len(saved) == (0 if truncated else 1), output
            if not truncated:
                assert 'Diferenca maxima placa/replay' in output, output
                assert 'analise offline pendente' not in output, output
        finally:
            if process.poll() is None:
                process.kill()
                process.wait()
            os.close(master)
            os.close(slave)
print('Octave 2400-sample evaluation/replay and truncated-transfer tests passed')
