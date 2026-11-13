"""Run a policy trained in MuJoCo on the real robot. Inference happens here, on the computer, at 50 Hz.

    python run_policy.py /dev/cu.usbserial-0001 roll [--log run1.npz]
The observation is built exactly as in sim/ouro_env.py:_frame. If you change one, change the other.
"""
import sys, time, argparse
import numpy as np
sys.path.insert(0, '../sim')
from stable_baselines3 import PPO
from ouro_env import TASK_CFG, STACK
from ouro_model import SERVO
from link import OuroLink

ap = argparse.ArgumentParser(); ap.add_argument('port'); ap.add_argument('task')
ap.add_argument('--policy'); ap.add_argument('--seconds', type=float, default=20); ap.add_argument('--log')
ap.add_argument('--gain', type=float, default=0.6, help='scale the action range; start low on a new robot')
a = ap.parse_args()
cfg = TASK_CFG[a.task]
policy = PPO.load(a.policy or f'../sim/policies/{a.task}', device='cpu')
link = OuroLink(a.port)
while link.latest() is None:
    print('waiting for telemetry...'); time.sleep(0.5)

def frame(tel, last):
    acc = np.concatenate([tel['acc_head'], tel['acc_mid']]) / 9.81
    gyr = np.concatenate([tel['gyro_head'], tel['gyro_mid']]) / 5.0
    return np.concatenate([np.clip(acc, -4, 4), np.clip(gyr, -4, 4), [min(tel['amps'], 3.2) / 3.0, float(tel['latch'])], last]).astype(np.float32)

last = np.zeros(8, np.float32); frames = [frame(link.latest(), last)] * STACK; rows = []
t0 = time.perf_counter(); k = 0; hot = 0
try:
    while time.perf_counter() - t0 < a.seconds:
        tel = link.latest()
        if time.perf_counter() - tel['rx_time'] > 0.2:
            print('lost the robot: going limp'); break
        hot = hot + 1 if tel['amps'] > 3.1 else 0
        if hot > 100:
            print('2 s pinned at the 3.2 A sensor limit: servos are fighting or stalled. Stopping.'); break
        frames = frames[1:] + [frame(tel, last)]
        act, _ = policy.predict(np.concatenate(frames), deterministic=True)
        last = np.clip(act, -1, 1).astype(np.float32)
        link.send(np.clip(cfg['centre'] + a.gain * cfg['scale'] * last, SERVO['lo'], SERVO['hi']))
        rows.append(np.concatenate([[time.perf_counter() - t0, link.rtt_ms], frames[-1]]))
        k += 1
        time.sleep(max(0, t0 + k * 0.02 - time.perf_counter()))
finally:
    link.limp()
    if a.log:
        np.savez(a.log, rows=np.array(rows)); print('saved', a.log)
