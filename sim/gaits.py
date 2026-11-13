"""Hand-written motions: no RL. These are the first things a student runs, in sim and on the real robot.
Each returns 8 servo targets in radians for time t (seconds)."""
import numpy as np
from ouro_model import LOOP_ANGLE

def wave(t, amp=0.55, freq=1.2, wavelength=5.0):
    """Caterpillar: a hump travels tail -> head."""
    k = np.arange(1, 9)
    return amp * np.sin(2 * np.pi * (freq * t - k / wavelength))

def cobra(t):
    """Rear the front three segments up, then nod."""
    q = np.zeros(8)
    up = np.clip(t / 1.0, 0, 1)
    q[5] = 0.9 * up
    q[6] = 0.5 * up
    q[7] = -0.7 * up + 0.35 * np.sin(4 * t) * (t > 1.2)
    return q

def bite(t, T=2.4):
    """Roll up from both ends like a carpet, tips first so no servo lifts more than it can, then close."""
    order = [0, 7, 1, 6, 2, 5, 3, 4]          # joint indices: outermost first
    q = np.zeros(8)
    for n, j in enumerate(order):
        start = (n // 2) * T / 4
        q[j] = LOOP_ANGLE * 1.12 * np.clip((t - start) / (T / 4), 0, 1)
    return q

def startle(t):
    """Burst the wheel open. Three joints at each end curl hard and the middle two bend backward: that
    pulls the head pins straight out of the magnet cradle with 2.1 to 3 N (sim, 12 random builds), which is
    more than the 1.2 to 2.0 N the magnets are set to and more than rolling ever puts on them (about 1 N).
    Then everything snaps flat and shivers to a stop."""
    if t < 0.6:
        return np.array([1.45, 1.45, 1.45, -0.6, -0.6, 1.45, 1.45, 1.45])
    if t < 1.0:
        return np.zeros(8)
    return 0.15 * np.sin(18 * t) * np.exp(-4 * (t - 1.0)) * np.ones(8)
