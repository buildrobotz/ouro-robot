"""How far past its command does each joint really swing? Electronics on s4..s6 touch at +92 deg.

    python limits.py        # max true joint angle (servo + backlash) per joint, over 30 random builds per behaviour
"""
import numpy as np
from stable_baselines3 import PPO
from ouro_env import OuroEnv, TASK_CFG, CTRL_DT
import gaits

def true_angles(env):
    return np.rad2deg(env.data.qpos[env.servo_qadr] + env.data.qpos[env.slop_qadr])

def run(task, seed, policy=None, script=None, start='', T=None):
    env = OuroEnv(start or task, randomize=True, seed=seed); obs, _ = env.reset()
    env.task, env.cfg = task, TASK_CFG[task]; c = env.cfg; mx = np.full(8, -999.0)
    for k in range(T or c['horizon']):
        a = policy.predict(obs, deterministic=True)[0] if policy else (script(k * CTRL_DT) - c['centre']) / c['scale']
        obs, *_ = env.step(a); mx = np.maximum(mx, true_angles(env))
    return mx

if __name__ == '__main__':
    N = 30
    cases = [('roll (policy)', dict(task='roll', policy=PPO.load('policies/roll', device='cpu'))),
             ('bite (policy)', dict(task='bite', policy=PPO.load('policies/bite', device='cpu'))),
             ('bite (script)', dict(task='bite', script=gaits.bite)), ('wave', dict(task='crawl', script=gaits.wave, T=200)),
             ('cobra', dict(task='crawl', script=gaits.cobra, T=150)), ('startle from wheel', dict(task='bite', script=gaits.startle, start='roll', T=100))]
    print('max true joint angle, deg (worst of %d builds)      j1   j2   j3   j4   j5   j6   j7   j8   | limit: j3 98, j4-j6 92' % N)
    for name, kw in cases:
        m = np.max([run(seed=4000 + s, **kw) for s in range(N)], 0)
        flag = 'OVER' if (m[3:6] > 92).any() or m[2] > 98 else 'ok'
        print(f'{name:48s}' + ''.join(f'{v:5.0f}' for v in m) + f'   {flag}')
