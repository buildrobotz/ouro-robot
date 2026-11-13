"""Score behaviours over many randomised builds (different slop, offsets, friction, magnets, latency).

    python evaluate.py bite 100        # scripted bite vs trained bite policy
    python evaluate.py roll 100        # trained roll policy
"""
import sys, os
import numpy as np
from ouro_env import OuroEnv, CTRL_DT
import gaits

def episode(task, seed, policy=None, script=None):
    env = OuroEnv(task, randomize=True, seed=seed); obs, _ = env.reset(); c = env.cfg
    amps, done, k, info = [], False, 0, {}
    while not done:
        a = policy.predict(obs, deterministic=True)[0] if policy else (script(k * CTRL_DT) - c['centre']) / c['scale']
        obs, r, term, trunc, info = env.step(a); amps.append(info['amps']); k += 1; done = term or trunc
    return info['x'], np.mean(amps), info['latched'], k * CTRL_DT

if __name__ == '__main__':
    task, n = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 50
    runs = {}
    if task == 'bite':
        runs['scripted bite'] = dict(script=gaits.bite)
    if os.path.exists(f'policies/{task}.zip'):
        from stable_baselines3 import PPO
        runs[f'trained {task}'] = dict(policy=PPO.load(f'policies/{task}', device='cpu'))
    for name, kw in runs.items():
        r = np.array([episode(task, 1000 + s, **kw) for s in range(n)], float)
        print(f'{name:14s} n={n}: latched at end {100*r[:,2].mean():3.0f} %   distance {r[:,0].mean():+.2f} m (min {r[:,0].min():+.2f})   '
              f'speed {100*(r[:,0]/r[:,3]).mean():.1f} cm/s   mean current {r[:,1].mean():.2f} A   episode {r[:,3].mean():.1f} s')
