"""Where every bought part sits on every printed link, and how the links chain together.

Used by check_fit.py (interference checks) and export.py (STL/GLB/PNG). Each link's contents are given in
that link's own frame (origin on its rear joint axis, see make_parts.py), as a list of Item(name, solid, colour,
group). Groups say what may legitimately touch what:
  part    the printed link
  rotor   servo spline + horn + horn screw of the joint at this link's REAR axis (driven by the previous servo,
          moves with this link)
  stator  the servo in this link's front cradle, its zip tie and the M2 pivot screw
  rubber  O-rings
  latch   head pins, tail magnets and the latch-sense wire
  elec    boards, battery, wires
"""
from dataclasses import dataclass
import numpy as np
from manifold3d import Manifold
import make_parts as P
import components as C

SEGS = ['tail', 'link', 'link', 'link_s3_battery', 'link_s4_esp32', 'link_s5_power', 'link', 'link', 'head']
COL = dict(link=(0.95, 0.45, 0.15), link2=(0.98, 0.72, 0.20), servo=(0.18, 0.20, 0.55), horn=(0.92, 0.92, 0.90),
           steel=(0.65, 0.66, 0.70), rubber=(0.10, 0.10, 0.12), pcb=(0.10, 0.45, 0.30), chip=(0.15, 0.15, 0.17),
           battery=(0.25, 0.45, 0.85), tie=(0.95, 0.95, 0.92), magnet=(0.55, 0.55, 0.60), copper=(0.85, 0.5, 0.2),
           cap=(0.15, 0.25, 0.65), plug=(0.12, 0.12, 0.12))

@dataclass
class Item:
    name: str
    solid: Manifold
    colour: tuple
    group: str


def band(pts, w, t, wdir):
    """A zip tie: chain of hulls through points (3D); w = band width along wdir ('x' or 'y'), t = thickness."""
    sz = {'x': (w, t, t), 'y': (t, w, t)}[wdir]
    m = Manifold()
    for a, b in zip(pts, pts[1:] + pts[:1]):
        m += Manifold.hull(C.cbox(*a, *sz) + C.cbox(*b, *sz))
    return m

def servo_tie():
    xc = (P.TIE_X0 + P.TIE_X1) / 2
    top, bot = P.S_THK / 2 + 0.55, -P.HT + P.TIE_RECESS - 0.55
    ye, yw = (P.TIE_SLOT_Y[0] + 0.6), P.Y0 - 2.0 - 0.6
    t = band([(xc, yw, top), (xc, ye, top), (xc, ye, bot), (xc, yw, bot)], 2.5, 1.1, 'x')
    head = C.cbox(xc, ye + 0.55 + 1.6, 0.0, 2.6 + 1.0, 3.2, 3.5)       # ratchet head pulled round to the +y end
    return t + head

def rotor():
    """Spline, horn and horn screw of the joint at this link's rear axis (x = 0)."""
    s = C.MG90S
    spline = C.ycyl(0, 0, s['spline_r'], P.Y0 + s['boss_top'] + 0.02, P.Y0 + s['spline_top'])
    hrn, scr = C.horn(P.HORN_IN + P.HORN_T)
    return [Item('spline', spline, COL['steel'], 'rotor'), Item('horn', hrn, COL['horn'], 'rotor'),
            Item('horn_screw', scr, COL['steel'], 'rotor')]

def front_servo():
    st = C.mg90s_stator().translate([P.L, P.Y0, 0])
    piv = C.m2x8().translate([P.L, P.PIV_OUT - 0.15, 0])            # 0.15 mm axial play on the arm
    return [Item('servo', st, COL['servo'], 'stator'), Item('pivot_screw', piv, COL['steel'], 'stator'),
            Item('servo_tie', servo_tie(), COL['tie'], 'stator')]

def orings(x0):
    yc = (P.BODY_HW + P.GROOVE_Y) / 2
    return [Item(f'oring', C.oring_stadium(P.GROOVE_R, 10.0, x0=x0).translate([0, s * yc, 0]), COL['rubber'], 'rubber')
            for s in (-1, 1)]

def place(d, x, y, z, rz=0):
    """Place a dict of local-frame solids from components.py."""
    return {k: v.rotate([0, 0, rz]).translate([x, y, z]) for k, v in d.items()}

# ------------------------------------------------------------------------------------------------ payloads
BATTERY = dict(x0=P.FENCE_X0 + P.FENCE_T + C.FOAM, y=(-39.6, 6.4), z0=P.HT + 3.0)
ESP = dict(xc=24.95, yc=8.25, z0=P.PAD_TOP + C.FOAM - C.ESP32['spacer_h'])
IMU_MID = dict(xc=21.9, yc=14.8)     # on the module can, shifted to the rear end: x 11.3..32.5, y 6.6..23.0
POWER = dict(x=(19.0, 39.5), z0=P.HT + C.FOAM)
COL1_Y, PITCH = -31.7, 2.54                      # power board: column 1 at y -31.7, columns across the link
ROW_A_X = 19.0 + (20.5 - 7 * PITCH) / 2          # rows A..H along the body, row A faces s4
def col_y(c): return COL1_Y + (c - 1) * PITCH
def row_x(r): return ROW_A_X + r * PITCH

def battery_items():
    b = BATTERY; t = 12.5
    d = place(C.lipo_2s500(), b['x0'] + t / 2, sum(b['y']) / 2, b['z0'])
    xf, zt = P.FENCE_X0 - 0.55, b['z0'] + 25.0 + 0.55
    ties = Manifold()
    for yc in (-24.0, -4.0):
        ties += band([(xf, yc, P.HT + 1.4), (xf, yc, zt), (b['x0'] + t + 0.55, yc, zt),
                      (b['x0'] + t + 0.55, yc, b['z0'] - 0.55), (P.FENCE_X0 + P.FENCE_T + 0.6, yc, b['z0'] - 0.55),
                      (P.FENCE_X0 + P.FENCE_T + 0.6, yc, P.HT + 1.4)], 2.5, 1.1, 'y')
    return [Item('battery', d['pack'], COL['battery'], 'elec'), Item('battery_lead', d['lead'], COL['chip'], 'elec'),
            Item('battery_ties', ties, COL['tie'], 'elec')]

def esp_items():
    d = place(C.esp32_devkit(), ESP['xc'], ESP['yc'], ESP['z0'])
    ztop = ESP['z0'] + C.ESP32['spacer_h'] + C.ESP32['pcb'] + C.ESP32['mod_h'] + C.FOAM
    imu = place(C.gy521(), IMU_MID['xc'], IMU_MID['yc'], ztop)
    return [Item('esp32_pcb', d['pcb'], COL['pcb'], 'elec'), Item('esp32_module', d['module'], COL['steel'], 'elec'),
            Item('esp32_parts', d['parts'], COL['chip'], 'elec'),
            Item('imu_mid', imu['pcb'], (0.2, 0.3, 0.75), 'elec'), Item('imu_mid_parts', imu['parts'], COL['chip'], 'elec')]

def battery_extras():
    """XT30 pair + F1. docs/electronics.md puts them flat on the s3 front block (x 29-39, y -33..-8, z <= 14);
    that spot is inside the ESP32's y-band and is hit from j4 = 75 deg and by the battery when s3's leaf flexes.
    They lie instead on top of the standing pack, along y, taped to it (moves with the battery)."""
    b = BATTERY
    return [Item('battery_xt30_fuse', C.xt30_fuse().translate([b['x0'] + 6.25, -20.5, b['z0'] + 25.0]), COL['plug'], 'elec')]

def esp_extras():
    """Battery pair (22 AWG red/black, twisted) taped along the -y edge of s4's front block, clear of the ESP32."""
    return [Item('bat_lead_pair', C.box(27.6, 39.4, -31.0, -26.0, P.HT, P.HT + 2.6), (0.7, 0.1, 0.1), 'elec')]

def power_items():
    """Final power board, docs/electronics.md + perfboard.svg: 26 x 8 holes, 66 x 20.5, on 1 mm foam."""
    x0, x1 = POWER['x']; z = POWER['z0']; xc = (x0 + x1) / 2
    zb = z + 1.6                                                  # board top, z 10.6
    it = [Item('perfboard', C.perfboard(x1 - x0, 66.0).translate([xc, 0, z]), (0.75, 0.62, 0.35), 'elec')]
    yb = (col_y(2) + col_y(12)) / 2                               # buck stack, columns 2..12
    for k, zz in enumerate((zb + C.FOAM, zb + C.FOAM + 5.4 + C.FOAM)):
        d = place(C.mini560(), xc, yb, zz)
        it += [Item(f'buck{k}', d['pcb'], (0.1, 0.2, 0.6), 'elec'), Item(f'buck{k}_parts', d['parts'], COL['chip'], 'elec')]
    yi = (col_y(14) + col_y(22)) / 2 + 0.3                        # INA219, columns 14..22
    d = place(C.ina219_generic(), xc, yi, zb + C.FOAM)
    it += [Item('ina219', d['pcb'], (0.45, 0.1, 0.55), 'elec'), Item('ina219_parts', d['parts'], COL['chip'], 'elec')]
    it += [Item('C2_220uF', C.cap_220uF().translate([xc - 3.0, yi, zb + C.FOAM + 1.6 + 1.2 + 0.4]), COL['cap'], 'elec')]
    it += [Item('R1_R2', C.box(row_x(2) - 1.3, row_x(5) + 1.3, col_y(1) - 1.2, col_y(1) + 1.2, zb, 20.5), COL['chip'], 'elec')]
    yh = (col_y(24) + col_y(26)) / 2                              # J1, columns 24..26, rows A..H
    hx0, hx1 = row_x(0) - 1.27, row_x(7) + 1.27
    it += [Item('servo_header', C.box(hx0, hx1, yh - 3.8, yh + 3.8, zb, zb + 2.5), COL['chip'], 'elec'),
           Item('servo_plugs', C.box(hx0, hx1, yh - 3.8, yh + 3.8, zb + 2.5, 27.1), COL['plug'], 'elec'),
           Item('servo_wire_bend', C.box(hx0, hx1, 24.0, 33.0, 27.1, 29.0), (0.85, 0.45, 0.2), 'elec')]
    return it

def head_items():
    it = []
    for s in (-1, 1):
        y = s * (P.BODY_HW - P.PIN_HEAD_DEPTH)
        pin = C.m3x25().rotate([0, 0, 180]) if s > 0 else C.m3x25()
        it.append(Item('latch_pin', pin.translate([P.L, y, 0]), COL['steel'], 'latch'))
    imu = place(C.gy521(), 29.8, 0.0, P.HT + C.FOAM, rz=90)
    it += [Item('imu_head', imu['pcb'], (0.2, 0.3, 0.75), 'elec'), Item('imu_head_parts', imu['parts'], COL['chip'], 'elec')]
    return it

def tail_items():
    it = []
    for s in (-1, 1):
        ym = s * sum(P.NOTCH_BLOCK) / 2
        it.append(Item('magnet', C.magnet_6x3().translate([P.MAG_X[0] + 0.1, ym, -0.15]), COL['magnet'], 'latch'))
    ang = np.radians(P.LATCH_WIRE_ANG); r = P.LATCH_WIRE_R
    wx, wz = r * np.cos(ang), r * np.sin(ang)
    for s in (-1, 1):                     # one bare wire per notch: across the seat, up the outer face groove
        a, b = sorted((s * P.NOTCH_BLOCK[0], s * P.NOTCH_BLOCK[1]))
        yo = (b if s > 0 else a) - s * 0.25
        w = C.ycyl(wx, wz, 0.25, a, b, 12) + C.zcyl(wx, yo, 0.25, wz, P.HT + 0.5, 12)
        it.append(Item('latch_wire', w, COL['copper'], 'latch'))
    return it

def link_items(k, payload=True):
    """Everything that moves with segment k, in segment k's frame."""
    seg = SEGS[k]
    col = COL['link'] if k % 2 == 0 else COL['link2']
    it = [Item(seg, P.PARTS[seg], col, 'part')]
    if k > 0:
        it += rotor()
    if k < 8:
        it += front_servo() + orings(38.0)
    if k == 0:
        it += orings(0.0) + tail_items()
    if k == 8:
        it += head_items() if payload else [i for i in head_items() if i.group != 'elec']
    if payload:
        it += {3: lambda: battery_items() + battery_extras(), 4: lambda: esp_items() + esp_extras(),
               5: power_items}.get(k, list)()
    return it

# ------------------------------------------------------------------------------------------------ kinematics
def Ry(theta_deg):
    """Joint rotation: positive curls the next link's far end toward +z."""
    t = np.radians(theta_deg); c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, -s, 0], [0, 1, 0, 0], [s, 0, c, 0], [0, 0, 0, 1]], float)

def Tx(x):
    m = np.eye(4); m[0, 3] = x; return m

def chain(angles):
    """World transform of every segment for 8 joint angles (deg). s0 is the world frame."""
    T = [np.eye(4)]
    for a in angles:
        T.append(T[-1] @ Tx(P.L) @ Ry(a))
    return T

def xf(m, T):
    return m.transform(T[:3, :])

def assembly(angles, payload=True):
    T = chain(angles)
    out = []
    for k in range(9):
        for i in link_items(k, payload):
            out.append((k, Item(i.name, xf(i.solid, T[k]), i.colour, i.group)))
    return out, T
