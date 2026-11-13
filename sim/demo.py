"""Play the whole Ouro act in MuJoCo and save what happened.

    python demo.py                  # writes ../visual/traj.json (side-view shapes) and ../visual/film_*.png
Act: caterpillar crawl -> bite the tail -> roll as a wheel (learned policy) -> startle open -> lie flat.
Phases with a trained policy in policies/ use it; the others fall back to the hand-written gait.
"""
import json, os, sys
import numpy as np
from PIL import Image
from ouro_env import OuroEnv, TASK_CFG, CTRL_DT
import gaits

def load(task):
    p = f'policies/{task}.zip'
    if os.path.exists(p):
        from stable_baselines3 import PPO
        return PPO.load(p, device='cpu')

_since = {}
def wheel_for(e):
    """seconds the latch has been shut"""
    if not e.latched:
        _since.pop(id(e), None); return 0.0
    return e.data.time - _since.setdefault(id(e), e.data.time)

def run(randomize=False, seed=0, film=True, verbose=True):
    env = OuroEnv('crawl', randomize=randomize, seed=seed); obs, _ = env.reset()
    pol = {t: load(t) for t in ('roll', 'bite')}
    traj, marks, shots = [], [], {}
    def phase(name, task, seconds, fn=None, until=None, snap=()):
        nonlocal obs
        env.task, env.cfg = task, TASK_CFG[task]
        env.t = 0; env.last_action[:] = 0; env.frames = [env._frame()] * 3; obs = env._obs()
        marks.append((len(traj), name)); x0 = env._com()[0]
        for k in range(int(seconds / CTRL_DT)):
            if fn is not None:
                a = (fn(k * CTRL_DT) - env.cfg['centre']) / env.cfg['scale']
            else:
                a, _ = pol[task].predict(obs, deterministic=True)
            obs, _, _, _, info = env.step(a)
            traj.append(dict(p=np.round(env.shape2d() * 1000).astype(int).tolist(), l=int(env.latched), a=round(info['amps'], 2)))
            if film and k * CTRL_DT in snap:
                shots[f'{name}_{k * CTRL_DT:.1f}'] = env.render('hero', width=800, height=500)
            if until and until(env):
                break
        if verbose:
            print(f'{name:8s} {len(traj) - marks[-1][0]:4d} steps  moved {100 * (env._com()[0] - x0):+6.1f} cm  latched={env.latched}')
        return env._com()[0] - x0
    res = {}
    res['crawl'] = phase('crawl', 'crawl', 4.0, gaits.wave, snap=(1.0,))
    phase('rest', 'crawl', 0.6, lambda t: np.zeros(8))
    if pol['bite']:
        phase('bite', 'bite', 5.0, until=lambda e: e.latched and e.t > 25 and e._pull_f < 0.5 and wheel_for(e) > 0.4, snap=(0.0, 0.4, 0.8))
    else:
        phase('bite', 'bite', 3.0, gaits.bite, snap=(0.0, 1.2, 2.2))
    res['bitten'] = env.latched
    if pol['roll'] and env.latched:
        res['roll'] = phase('roll', 'roll', 6.0, snap=(1.0, 3.0, 5.0))
        res['still_wheel'] = env.latched
    phase('startle', 'bite', 2.0, gaits.startle, snap=(0.3, 0.9))
    res['opened'] = not env.latched
    return res, traj, marks, shots

if __name__ == '__main__':
    res, traj, marks, shots = run()
    os.makedirs('../visual', exist_ok=True)
    json.dump(dict(dt=CTRL_DT, marks=marks, frames=traj), open('../visual/traj.json', 'w'), separators=(',', ':'))
    for k, im in shots.items():
        Image.fromarray(im).save(f'../visual/film_{k}.png')
    print(res, 'frames:', len(traj))
