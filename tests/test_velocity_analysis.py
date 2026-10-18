"""Check synthetic time-reference/gain measurements against discrete theory."""
import csv
import cmath
import math
import subprocess
from pathlib import Path

base = Path(__file__).resolve().parents[1] / 'build/velocity-evaluation'
rows = [[float(x) for x in row] for row in csv.reader((base/'synthetic_sine.csv').open())]
for row in rows:
    omega = 2*math.pi*row[1]*.005
    derivative = (1-cmath.exp(-1j*omega))/(1j*omega)
    lpf = derivative * (1/6)/(1-(5/6)*cmath.exp(-1j*omega))
    for observed_gain, observed_delay, expected in ((row[2],row[5],derivative), (row[3],row[6],lpf)):
        assert abs(observed_gain-abs(expected)) < .0001
        delay = -cmath.phase(expected)/(2*math.pi*row[1])*1000
        assert abs(observed_delay-delay) < .02, (row, delay)
    assert all(math.isfinite(x) for x in row)
ramps = [[float(x) for x in row] for row in csv.reader((base/'synthetic_ramp.csv').open())]
candidate4 = ramps[3]
assert candidate4[3] < candidate4[2]  # This fixed, documented simulation only.
assert candidate4[6] < candidate4[5]
exe = base.parent/'native/velocity-replay'
for payload in ('0,0\n0,1\n', '0,0\n1,1\ninvalid\n', '0,nan\n1,1\n'):
    result = subprocess.run([str(exe), '.001', '5', '.025'], input=payload,
                            capture_output=True, text=True)
    assert result.returncode != 0
print('Synthetic gain/delay time-reference and replay input validation passed')
