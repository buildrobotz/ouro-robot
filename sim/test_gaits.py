import numpy as np, mujoco, sys
from ouro_env import OuroEnv, CTRL_DT, TASK_CFG
import gaits

def run(task, fn, T, randomize=False, seed=0):
    env = OuroEnv(task, randomize=randomize, seed=seed); env.reset()
    c = env.cfg; log = []
    for k in range(int(T / CTRL_DT)):
        a = (fn(k * CTRL_DT) - c['centre']) / c['scale']
        _, r, term, trunc, info = env.step(a)
        log.append((info['x'], info['amps'], info['latched'], info['gap'], np.abs(env.data.actuator_force).max()))
    return env, np.array(log, float)

env, lg = run('crawl', gaits.wave, 6)
print(f"wave : moved {lg[-1,0]*100:6.1f} cm in 6 s, mean {lg[:,1].mean():.2f} A, peak torque {lg[:,4].max():.3f} Nm")
env, lg = run('crawl', gaits.cobra, 3)
print(f"cobra: head height {env.data.site_xpos[env.sid['latch_head']][2]*100:.1f} cm, mean {lg[:,1].mean():.2f} A, peak torque {lg[:,4].max():.3f}")
for rnd, seed in [(False, 0), (True, 1), (True, 2), (True, 3), (True, 4), (True, 5)]:
    env, lg = run('bite', gaits.bite, 4, rnd, seed)
    t_l = np.argmax(lg[:, 2]) * CTRL_DT if lg[:, 2].any() else None
    print(f"bite rnd={rnd} seed={seed}: latched at {t_l}, final latched={bool(lg[-1,2])}, min gap {lg[:,3].min()*1000:.1f} mm, peak {lg[:,1].max():.2f} A, com z {env._com()[2]*100:.1f} cm")
