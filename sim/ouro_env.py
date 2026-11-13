"""Ouro gymnasium environment (MuJoCo).

Tasks
  roll  : start as a closed wheel (tail bitten); reward = forward speed of the centre of mass.
  bite  : start flat on the desk; reward = bring head latch to tail latch and snap the magnets.
  crawl : start flat; reward = forward speed while staying open (caterpillar).

The policy sees ONLY what the real robot can send over ESP-NOW:
  2 IMUs (accel + gyro, head and mid), total servo current, latch bit, its own last commands.
It never sees joint angles. Three frames are stacked so it can infer motion.
"""
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from ouro_model import build_xml, N_SERVO, LOOP_ANGLE, SERVO, L

CTRL_DT = 0.02          # 50 Hz policy
SUBSTEPS = 10           # 500 Hz physics
FRAME = 12 + 1 + 1 + N_SERVO   # one observation frame
STACK = 3

# action -> servo target mapping, per task: target = centre + scale * a,  a in [-1, 1]
TASK_CFG = {
    'roll':  dict(centre=LOOP_ANGLE, scale=0.60, horizon=400, vmax=0.25, amps_w=0.30),
    'stroll': dict(centre=LOOP_ANGLE, scale=0.60, horizon=400, vmax=0.12, amps_w=0.80),   # slow, cool-running wheel
    'bite':  dict(centre=0.45,       scale=1.10, horizon=300),
    'crawl': dict(centre=0.0,        scale=0.80, horizon=400),
}


class OuroEnv(gym.Env):
    metadata = {'render_modes': ['rgb_array'], 'render_fps': 50}

    def __init__(self, task='roll', randomize=True, render_mode=None, seed=None):
        assert task in TASK_CFG
        self.task, self.randomize, self.render_mode = task, randomize, render_mode
        self.cfg = TASK_CFG[task]
        self.model = mujoco.MjModel.from_xml_string(build_xml())
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.nominal = dict(
            body_mass=m.body_mass.copy(), kp=m.actuator_gainprm[:, 0].copy(),
            force=m.actuator_forcerange.copy(), friction=m.geom_friction.copy(),
            jrange=m.jnt_range.copy(), gravity=m.opt.gravity.copy(), damping=m.dof_damping.copy())
        self.sid = {n: m.site(n).id for n in ('latch_tail', 'latch_head', 'imu_head', 'imu_mid')}
        self.slop_j = [m.joint(f'slop{i}').id for i in range(1, 9)]
        self.servo_qadr = [m.joint(f'j{i}').qposadr[0] for i in range(1, 9)]
        self.slop_qadr = [m.joint(f'slop{i}').qposadr[0] for i in range(1, 9)]
        self.eq = [m.equality('latch_l').id, m.equality('latch_r').id]
        self.action_space = spaces.Box(-1, 1, (N_SERVO,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (FRAME * STACK,), np.float32)
        self.np_random = np.random.default_rng(seed)
        self._renderer = None
        self._row = 0
        self.max_pull = 0.0
        self._set_params(nominal=True)

    # ---------------------------------------------------------------- randomisation
    def _set_params(self, nominal=False):
        m, n, r = self.model, self.nominal, self.np_random
        u = (lambda lo, hi, size=None: (lo + hi) / 2 if nominal and size is None else
             (np.full(size, (lo + hi) / 2) if nominal else r.uniform(lo, hi, size)))
        m.body_mass[:] = n['body_mass'] * u(0.85, 1.15, m.nbody)
        m.actuator_gainprm[:, 0] = n['kp'] * u(0.6, 1.6, m.nu)          # servo + leaf-spring stiffness
        m.actuator_biasprm[:, 1] = -m.actuator_gainprm[:, 0]
        stall = SERVO['stall'] * u(0.7, 1.25, m.nu)                     # 1.05 .. 1.9 kg.cm
        m.actuator_forcerange[:, 0], m.actuator_forcerange[:, 1] = -stall, stall
        self.stall = stall
        m.geom_friction[:, 0] = u(0.5, 1.1)
        back = np.deg2rad(u(0.5, 4.0, N_SERVO))                         # backlash per joint
        for j, b in zip(self.slop_j, back):
            m.jnt_range[j] = (-b, b)
        self.offset = np.deg2rad(u(-4.0, 4.0, N_SERVO))                 # horn spline / trim error
        self.deadband = np.deg2rad(u(0.3, 1.5))                         # servo won't correct inside this
        self.rate = u(5.0, 10.0)                                        # rad/s slew (0.10-0.21 s / 60 deg)
        self.delay = 2 if nominal else int(r.integers(1, 4))            # 20-60 ms loop latency
        self.break_force = u(1.2, 2.0)                                  # magnet pull-off, newtons (set by the air gap)
        self.capture_r = u(0.007, 0.012)                                # magnet capture radius
        tilt = np.deg2rad(u(-2.0, 2.0, 2))
        m.opt.gravity[:] = 9.81 * np.array([np.sin(tilt[0]), np.sin(tilt[1]), -1.0])
        self.acc_bias = u(-0.3, 0.3, 6)
        self.gyro_bias = u(-0.05, 0.05, 6)

    # ---------------------------------------------------------------- latch (the magnets)
    @property
    def latched(self):
        return bool(self.data.eq_active[self.eq[0]])

    def latch_gap(self):
        d = self.data
        return float(np.linalg.norm(d.site_xpos[self.sid['latch_head']] - d.site_xpos[self.sid['latch_tail']]))

    def _latch_force(self):
        """Pull-out force on the latch, newtons. The head pin sits in a V cradle at the tail tip, so
        pushing in and sideways loads are carried by plastic; only pulling straight out of the
        cradle (along the tail's own axis) is resisted by the magnets."""
        d, f = self.data, np.zeros(3)
        for k in range(d.nefc):
            if d.efc_type[k] == mujoco.mjtConstraint.mjCNSTR_EQUALITY and d.efc_id[k] in self.eq:
                f[self._row % 3] += d.efc_force[k]
                self._row += 1
        self._row = 0
        n = -d.xmat[self.model.body('s0').id].reshape(3, 3)[:, 0]
        return float(f @ n)

    def _update_latch(self):
        """Magnets are a short-range well, not a force switch: a 2 ms knock when a knuckle lands does not
        separate them, a pull held for a few tens of ms does. So the pull is low-passed (30 ms) first."""
        if self.latched:
            self._pull_f += (self._latch_force() - self._pull_f) * self.model.opt.timestep / 0.03
            self.max_pull = max(self.max_pull, self._pull_f)
            if self._pull_f > self.break_force:
                self.data.eq_active[self.eq] = 0
                self._cooldown = self.data.time + 0.3
        elif self.data.time > self._cooldown and self.latch_gap() < self.capture_r:
            self.data.eq_active[self.eq] = 1
            self._pull_f = 0.0

    # ---------------------------------------------------------------- servo model
    def _apply_servo(self, cmd):
        """What the cheap servo actually does with a commanded angle."""
        step = self.rate * self.model.opt.timestep
        self.servo_target += np.clip(cmd - self.servo_target, -step, step)
        q = self.data.qpos[self.servo_qadr]
        tgt = self.servo_target + self.offset
        hold = np.abs(tgt - q) < self.deadband           # inside deadband: stop correcting
        self.data.ctrl[:] = np.where(hold, q, tgt)

    def current_amps(self):
        """Total servo-rail current, as the INA219 would read it (torque ~ current)."""
        frac = np.abs(self.data.actuator_force) / self.stall
        return float(0.08 + 0.75 * frac.sum())

    # ---------------------------------------------------------------- observation
    def _frame(self):
        s = self.data.sensordata
        nz = self.np_random.normal if self.randomize else (lambda a, b, c: np.zeros(c))
        acc = (np.concatenate([s[0:3], s[6:9]]) + self.acc_bias * self.randomize + nz(0, 0.4, 6)) / 9.81
        gyr = (np.concatenate([s[3:6], s[9:12]]) + self.gyro_bias * self.randomize + nz(0, 0.05, 6)) / 5.0
        cur = min(self.current_amps() + nz(0, 0.05, 1)[0], 3.2) / 3.0      # INA219 tops out at 3.2 A
        return np.concatenate([np.clip(acc, -4, 4), np.clip(gyr, -4, 4), [cur, float(self.latched)],
                               self.last_action]).astype(np.float32)

    def _obs(self):
        return np.concatenate(self.frames)

    # ---------------------------------------------------------------- reset
    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self.np_random = np.random.default_rng(seed)
        m, d = self.model, self.data
        self._set_params(nominal=not self.randomize)
        mujoco.mj_resetData(m, d)
        d.eq_active[self.eq] = 0
        self._cooldown, self._latch_t, self._pull_f = 0.0, 0.0, 0.0
        r = self.np_random
        if self.task in ('roll', 'stroll'):
            q = np.full(N_SERVO, LOOP_ANGLE)
            d.qpos[self.servo_qadr] = q
            phi = r.uniform(0, 2 * np.pi) if self.randomize else np.deg2rad(160)
            d.qpos[3:7] = [np.cos(phi / 2), 0, np.sin(phi / 2), 0]      # spin the wheel to a random flat
            d.qpos[2] = 0.3
            mujoco.mj_forward(m, d)
            d.qpos[2] -= min(d.geom_xpos[g][2] for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER) - (0.009 + 0.0005)
            d.eq_active[self.eq] = 1
        else:
            q = r.uniform(-0.05, 0.05, N_SERVO) if self.randomize else np.zeros(N_SERVO)
            d.qpos[self.servo_qadr] = q
        self.servo_target = q.copy()
        self.last_action = np.zeros(N_SERVO, np.float32)
        self.action_queue = [q.copy() for _ in range(self.delay)]
        mujoco.mj_forward(m, d)
        for _ in range(100):                                            # settle 0.2 s
            self._apply_servo(q); mujoco.mj_step(m, d)
        self._x0 = self._com()[0]
        self._prev_gap = self.latch_gap()
        self.t = 0
        f = self._frame()
        self.frames = [f.copy() for _ in range(STACK)]
        return self._obs(), {}

    def _com(self):
        return self.data.subtree_com[1].copy()

    # ---------------------------------------------------------------- step
    def step(self, action):
        a = np.clip(np.asarray(action, np.float64), -1, 1)
        cmd = np.clip(self.cfg['centre'] + self.cfg['scale'] * a, SERVO['lo'], SERVO['hi'])
        self.action_queue.append(cmd)
        cmd = self.action_queue.pop(0)                                  # radio + serial latency
        x_before, was_latched = self._com()[0], self.latched
        self.max_pull = 0.0
        amps = 0.0
        for _ in range(SUBSTEPS):
            self._apply_servo(cmd)
            mujoco.mj_step(self.model, self.data)
            self._update_latch()
            amps += self.current_amps() / SUBSTEPS
        vx = (self._com()[0] - x_before) / CTRL_DT
        jerk = float(np.sum((a - self.last_action) ** 2))
        self.last_action = a.astype(np.float32)
        self.t += 1
        side_tilt = abs(self.data.xmat[self.model.body('s4').id].reshape(3, 3)[2, 1])   # fell on its side?
        terminated, info = False, {}

        if self.task in ('roll', 'stroll'):
            strain = np.clip(self.max_pull / self.break_force, 0, 1)     # privileged: how close the magnets are to letting go
            rew = 4.0 * np.clip(vx, -0.4, self.cfg['vmax']) - self.cfg['amps_w'] * amps - 0.02 * jerk - 1.0 * strain ** 2
            if not self.latched:
                rew, terminated = rew - 40.0, True                     # the wheel burst open
        elif self.task == 'bite':
            gap = self.latch_gap()
            rew = 20.0 * (self._prev_gap - gap) - 0.15 * amps - 0.01 * jerk
            self._prev_gap = gap
            if self.latched:
                rew += 1.0                                             # paid every step it stays a wheel
                if not was_latched:
                    rew += 10.0
        else:  # crawl
            rew = 6.0 * np.clip(vx, -0.3, 0.3) - 0.03 * amps - 0.02 * jerk
            if self.latched:
                rew, terminated = rew - 2.0, True
        if side_tilt > 0.7:
            rew, terminated = rew - 3.0, True
        self.frames = self.frames[1:] + [self._frame()]
        truncated = self.t >= self.cfg['horizon']
        info.update(x=self._com()[0] - self._x0, amps=amps, latched=self.latched, gap=self.latch_gap())
        return self._obs(), float(rew), terminated, truncated, info

    # ---------------------------------------------------------------- render / logging
    def render(self, camera='side', track=True, width=960, height=540):
        if self._renderer is None or (self._renderer.width, self._renderer.height) != (width, height):
            self._renderer = mujoco.Renderer(self.model, height, width)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        com = self._com()
        cam.lookat[:] = [com[0] if track else 0.2, 0, 0.07]
        cam.distance, cam.azimuth, cam.elevation = (0.75, 90, -8) if camera == 'side' else (0.8, 125, -22)
        self._renderer.update_scene(self.data, cam)
        return self._renderer.render()

    def shape2d(self):
        """Side-view polyline of the body: 10 points (tail tip, 8 joints, head tip), metres (x, z)."""
        d, m = self.data, self.model
        pts = [d.xpos[m.body(f's{i}').id][[0, 2]] for i in range(9)]
        pts.append(d.site_xpos[self.sid['latch_head']][[0, 2]])
        return np.array(pts)
