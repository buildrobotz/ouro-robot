"""Push the trained policies outside the conditions they were trained in, to find where the design breaks.

    python stress.py            # prints one table; about 2 minutes
Every row is 30 randomised builds with ONE thing made worse than the training range.
"""
import numpy as np, mujoco
from stable_baselines3 import PPO
from ouro_env import OuroEnv, TASK_CFG
import ouro_model
from ouro_model import SERVO

def run(task, pol, seed, mod=None, push=0.0, steps=None):
    ouro_model.PAYLOAD_Y = getattr(mod, 'battery_y', 0.0)        # baked into the model at build time
    env = OuroEnv(task, randomize=True, seed=seed); obs, _ = env.reset()
    m, d = env.model, env.data
    if mod: mod(env)
    s4 = m.body('s4').id; rng = np.random.default_rng(seed); load = []
    tipped = False
    for k in range(steps or env.cfg['horizon']):
        d.xfrc_applied[:] = 0
        if push and k % 50 < 8:                       # a sideways shove for 0.16 s every second
            d.xfrc_applied[s4, 1] = push * (1 if (k // 50) % 2 else -1)
        a, _ = pol.predict(obs, deterministic=True)
        obs, r, term, trunc, info = env.step(a)
        load.append(np.abs(d.actuator_force) / env.stall)
        tilt = abs(d.xmat[s4].reshape(3, 3)[2, 1])
        if tilt > 0.7: tipped = True
        if term or trunc: break
    load = np.array(load)
    return dict(x=info['x'], latched=info['latched'], tipped=tipped, amps=info['amps'],
                load_mean=load.mean(), load_max_joint=load.mean(0).max(), t=(k + 1) * 0.02)

def weak(f):
    def mod(env):
        env.stall = np.full(8, SERVO['stall'] * f)
        env.model.actuator_forcerange[:, 0], env.model.actuator_forcerange[:, 1] = -env.stall, env.stall
    return mod
def heavy(f):
    def mod(env): env.model.body_mass[:] = env.nominal['body_mass'] * f
    return mod
def slick(mu):
    def mod(env): env.model.geom_friction[:, 0] = mu
    return mod
def late(n):
    def mod(env): env.delay = n; env.action_queue = [env.action_queue[0].copy() for _ in range(n)]
    return mod
def offcentre(mm):
    def mod(env): pass
    mod.battery_y = mm / 1000.0
    return mod
def sidetilt(deg):
    def mod(env): env.model.opt.gravity[1] = 9.81 * np.sin(np.deg2rad(deg))
    return mod
def magnet(n):
    def mod(env): env.break_force = n
    return mod

if __name__ == '__main__':
    N = 30
    roll, bite = PPO.load('policies/roll', device='cpu'), PPO.load('policies/bite', device='cpu')
    cases = [('as trained', None, 0), ('battery 10 mm off centre', offcentre(10), 0), ('battery 20 mm off centre', offcentre(20), 0)] if 'quick' in __import__('sys').argv else [('as trained', None, 0), ('servo stall 1.0 kg.cm (67 %)', weak(0.67), 0), ('servo stall 0.8 kg.cm (53 %)', weak(0.53), 0),
             ('mass +25 %', heavy(1.25), 0), ('mass +40 %', heavy(1.40), 0), ('friction 0.3 (dusty desk)', slick(0.3), 0),
             ('latency 100 ms', late(5), 0), ('battery 10 mm off centre', offcentre(10), 0), ('desk tilted 5 deg sideways', sidetilt(5), 0),
             ('sideways shove 0.5 N every second', None, 0.5), ('sideways shove 1.5 N every second', None, 1.5),
             ('magnet 1.0 N (weak)', magnet(1.0), 0), ('magnet 3.0 N (strong)', magnet(3.0), 0)]
    print(f'{"condition":36s} | ROLL wheel kept  dist   tipped  amps  servo load mean/worst joint | BITE closed')
    for name, mod, push in cases:
        R = [run('roll', roll, 2000 + s, mod, push) for s in range(N)]
        B = [run('bite', bite, 3000 + s, mod, push) for s in range(N)]
        print(f'{name:36s} | {sum(r["latched"] for r in R):2d}/{N}  {np.mean([r["x"] for r in R]):5.2f} m  {sum(r["tipped"] for r in R):2d}/{N}  '
              f'{np.mean([r["amps"] for r in R]):.1f} A   {np.mean([r["load_mean"] for r in R])*100:3.0f} % / {np.mean([r["load_max_joint"] for r in R])*100:3.0f} %'
              f'           | {sum(b["latched"] for b in B):2d}/{N}', flush=True)
