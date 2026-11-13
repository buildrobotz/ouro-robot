"""Measure YOUR robot so the simulator can be told what it is like. Robot flat on its belly, open.

    python sysid.py /dev/cu.usbserial-0001
For each joint it commands a 30-degree step and watches the head/mid gyro. It reports
  delay  : command sent -> first motion seen (radio + serial + servo dead time)
  slew   : how fast the servo actually moves (rad/s)
  amps   : current while moving, and holding
Put the printed ranges into _set_params() in sim/ouro_env.py (self.delay, self.rate) and retrain,
or keep the defaults if your numbers already fall inside them.
"""
import sys, time, json
import numpy as np
from link import OuroLink

link = OuroLink(sys.argv[1]); time.sleep(1.0)
STEP = np.deg2rad(30); out = []
for j in [7, 6, 5, 0, 1, 2]:                 # outer joints: they move a free end the gyro can see
    q = np.zeros(8)
    for _ in range(40):
        link.send(q); time.sleep(0.02)
    q[j] = STEP; log = []; t0 = time.perf_counter()
    while (t := time.perf_counter() - t0) < 0.8:
        link.send(q); tel = link.latest()
        g = tel['gyro_head'][1] if j >= 4 else tel['gyro_mid'][1]
        log.append((t, abs(g), tel['amps'])); time.sleep(0.01)
    log = np.array(log); moving = log[:, 1] > 0.5
    if moving.any():
        delay = log[moving, 0][0]; slew = np.percentile(log[moving, 1], 90)
        out.append(dict(joint=j + 1, delay_ms=round(delay * 1e3), slew_rad_s=round(float(slew), 1),
                        amps_move=round(float(log[moving, 2].mean()), 2), amps_hold=round(float(log[-10:, 2].mean()), 2)))
        print(out[-1])
link.limp()
d = [o['delay_ms'] for o in out]; s = [o['slew_rad_s'] for o in out]
print(f'\ndelay {min(d)}-{max(d)} ms  ->  self.delay = {max(1, round(min(d)/20))}..{round(max(d)/20)+1} control steps')
print(f'slew  {min(s)}-{max(s)} rad/s  ->  self.rate range')
json.dump(out, open('sysid.json', 'w'), indent=1)
