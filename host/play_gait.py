"""Run a hand-written motion from sim/gaits.py on the real robot. Same function, same numbers as in MuJoCo.

    python play_gait.py /dev/cu.usbserial-0001 wave 8      # gait name, seconds
"""
import sys, time
sys.path.insert(0, '../sim')
import gaits
from link import OuroLink

link = OuroLink(sys.argv[1]); fn = getattr(gaits, sys.argv[2]); T = float(sys.argv[3]) if len(sys.argv) > 3 else 5
t0 = time.perf_counter(); k = 0
try:
    while (t := time.perf_counter() - t0) < T:
        link.send(fn(t))
        tel = link.latest()
        if tel and k % 25 == 0:
            print(f't={t:4.1f}s  {tel["amps"]:.2f} A  {tel["batt_v"]:.2f} V  latch={int(tel["latch"])}  round trip {link.rtt_ms:.0f} ms')
        k += 1
        time.sleep(max(0, t0 + k * 0.02 - time.perf_counter()))
finally:
    link.limp()
