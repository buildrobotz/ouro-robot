"""Step 1 on the real robot: find each servo's straight position and direction. Writes calibration.json.

    python calibrate.py /dev/cu.usbserial-0001
Lay Ouro flat on its belly. For each joint: nudge with a/d (10 us) or A/D (50 us) until the two links
are in one straight line, press Enter. Then it curls the joint a little: answer whether it went UP.
"""
import sys, json, time
import numpy as np
from link import OuroLink, CAL_FILE

link = OuroLink(sys.argv[1])
centre, sign = [1500] * 8, [1] * 8
for j in range(8):
    while True:
        link.send_us(centre)
        k = input(f'joint {j+1}: {centre[j]} us  [a/d/A/D, Enter = straight] ')
        if k == '':
            break
        centre[j] += {'a': -10, 'd': 10, 'A': -50, 'D': 50}.get(k, 0)
    us = list(centre); us[j] += 150
    for _ in range(25):
        link.send_us(us); time.sleep(0.02)
    sign[j] = 1 if input('  did the head-side link tip UP (toward the back)? [y/n] ').lower().startswith('y') else -1
link.limp()
CAL_FILE.write_text(json.dumps(dict(centre_us=centre, sign=sign), indent=1))
print('wrote', CAL_FILE)
