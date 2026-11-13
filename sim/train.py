"""Train one Ouro behaviour with PPO on a laptop CPU.

  python train.py roll            # ~10 min on an 8-core laptop
  python train.py bite --steps 3000000
  tensorboard --logdir runs       # watch rollout/ep_rew_mean and ouro/* climb
"""
import argparse, os, time
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from ouro_env import OuroEnv


def make(task, seed):
    def f():
        return OuroEnv(task, randomize=True, seed=seed)
    return f


class Progress(BaseCallback):
    """Logs the numbers a student should watch: distance rolled, amps, latch rate."""
    def __init__(self):
        super().__init__(); self.buf = []; self.t0 = time.time()
    def _on_step(self):
        for info, done in zip(self.locals['infos'], self.locals['dones']):
            if done:
                self.buf.append((info['x'], info['amps'], float(info['latched']), info['episode']['l']))
        return True
    def _on_rollout_end(self):
        if self.buf:
            b = np.array(self.buf[-200:])
            for k, v in zip(('distance_m', 'amps', 'latched_at_end', 'ep_len'), b.mean(0)):
                self.logger.record(f'ouro/{k}', float(v))
            print(f'{self.num_timesteps:>9,d} steps  {time.time()-self.t0:5.0f}s  dist {b[:,0].mean():+.2f} m  '
                  f'amps {b[:,1].mean():.2f}  latched {b[:,2].mean():.2f}  len {b[:,3].mean():.0f}', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('task', choices=['roll', 'stroll', 'bite', 'crawl'])
    ap.add_argument('--steps', type=int, default=2_000_000)
    ap.add_argument('--envs', type=int, default=max(2, os.cpu_count() - 2))
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--init', help='continue from a saved policy, e.g. policies/roll.zip (use after sysid)')
    args = ap.parse_args()
    env = VecMonitor(SubprocVecEnv([make(args.task, args.seed * 100 + i) for i in range(args.envs)]))
    model = PPO('MlpPolicy', env, n_steps=4096 // args.envs * 2, batch_size=512, n_epochs=6, gamma=0.98,
                gae_lambda=0.95, learning_rate=3e-4, ent_coef=0.002, clip_range=0.2,
                policy_kwargs=dict(net_arch=[128, 128], log_std_init=-0.7),
                tensorboard_log='runs', device='cpu', seed=args.seed, verbose=0)
    if args.init:
        model.set_parameters(args.init, exact_match=False, device='cpu')
    ckpt = CheckpointCallback(500_000 // args.envs, 'runs/ckpt', name_prefix=args.task)   # a save every 500k steps
    model.learn(args.steps, callback=[Progress(), ckpt], tb_log_name=args.task)
    os.makedirs('policies', exist_ok=True)
    model.save(f'policies/{args.task}')
    print('saved policies/' + args.task + '.zip')
