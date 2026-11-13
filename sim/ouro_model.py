"""Ouro: MJCF generator. Run `python ouro_model.py` to write ouro.xml.

Units: metres, kilograms, radians. x = along the body (tail -> head), y = across
(all eight servo axes), z = up. Positive joint angle = curl upward, toward the back.
Body origin of every segment is its REAR joint axis.
"""
import numpy as np

N_SEG = 9            # rigid segments s0 (tail) .. s8 (head)
N_SERVO = 8          # servo k sits in segment k-1 and drives segment k (k = 1..8)
L = 0.048            # joint-to-joint pitch
HALF_W = 0.040       # half width (80 mm wide tread)
HALF_T = 0.008       # half thickness of the flat link
KNUCKLE_R = 0.009    # O-ring tread at every joint (8 mm plastic + 1 mm of rubber proud): the only thing that touches the desk
LOOP_ANGLE = 2 * np.pi / N_SEG   # 40 deg: every corner of the closed 9-gon

# segment masses (kg), from cad/check_fit (printed PETG) + MG90S 13.4 g + O-rings, zip tie, screws, wire ~4 g
SEG_MASS = [0.036, 0.031, 0.031, 0.033, 0.032, 0.032, 0.031, 0.031, 0.020]   # tail .. head (no servo, 2 steel pins)
PAYLOAD = {   # electronics on the back (inside of the wheel): name, mass, centre (x, y, z), half sizes; from docs/mechanical.md s5
    3: ("battery", 0.034, (0.0224, -0.0155, 0.0235), (0.0063, 0.0230, 0.0125)),   # 2S pack standing on its long edge
    4: ("esp32",   0.012, (0.0250,  0.0080, 0.0175), (0.0141, 0.0257, 0.0045)),   # DevKit, pins cut flush, IMU on the can
    5: ("power",   0.016, (0.0292,  0.0000, 0.0150), (0.0102, 0.0330, 0.0060)),   # perfboard: 2 bucks, INA219, cap, servo header
    8: ("headimu", 0.004, (0.0300,  0.0000, 0.0110), (0.0080, 0.0100, 0.0030)),
}

PAYLOAD_Y = 0.0       # sideways offset of the battery, metres (stress tests move it)
SERVO = dict(kp=1.0, stall=0.15, damping=0.012, armature=1.5e-4, frictionloss=0.006,
             lo=-1.20, hi=1.50)    # +86 deg, so servo + 4 deg of backlash stops at the printed +90 deg end stop; electronics touch at +92
BACKLASH = np.deg2rad(2.0)   # nominal +-2 deg free play; randomised per episode in the env


def build_xml() -> str:
    s = SERVO
    out = []
    a = out.append
    a('<mujoco model="ouro">')
    a('  <compiler angle="radian" autolimits="true"/>')
    a('  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="5"/>')
    a('  <size memory="20M"/>')
    a('  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="2048"/>'
      '<headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/></visual>')
    a('  <asset>')
    a('    <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.91 0.86" rgb2="0.86 0.84 0.78" width="256" height="256"/>')
    a('    <material name="desk" texture="grid" texrepeat="10 10" reflectance="0.05"/>')
    a('    <material name="link" rgba="0.95 0.45 0.15 1"/>')
    a('    <material name="link2" rgba="0.98 0.72 0.2 1"/>')
    a('    <material name="rubber" rgba="0.12 0.12 0.14 1"/>')
    a('    <material name="pcb" rgba="0.1 0.45 0.3 1"/>')
    a('    <material name="eye" rgba="1 1 1 1"/>')
    a('  </asset>')
    a('  <default>')
    a('    <geom condim="3" friction="0.9 0.005 0.0001" solref="0.004 1" solimp="0.95 0.99 0.001"/>')
    a(f'    <default class="servo"><joint axis="0 -1 0" range="{s["lo"]} {s["hi"]}" damping="{s["damping"]}" '
      f'armature="{s["armature"]}" frictionloss="{s["frictionloss"]}" solreflimit="0.004 1" solimplimit="0.98 0.999 0.0005"/></default>')   # hard end stop: plastic on plastic
    a(f'    <default class="slop"><joint axis="0 -1 0" range="{-BACKLASH:.5f} {BACKLASH:.5f}" damping="0.0004" armature="4e-6" solreflimit="0.004 1" solimplimit="0.98 0.999 0.0005"/></default>')
    a(f'    <position kp="{s["kp"]}" forcerange="{-s["stall"]} {s["stall"]}" ctrlrange="{s["lo"]} {s["hi"]}"/>')
    a('  </default>')
    a('  <worldbody>')
    a('    <light pos="0.2 -0.6 1.2" dir="-0.1 0.4 -1" diffuse="0.5 0.5 0.5"/>')
    a('    <geom name="floor" type="plane" size="3 3 0.1" material="desk"/>')
    a('    <camera name="side" pos="0.2 -0.75 0.16" xyaxes="1 0 0 0 0.12 1" fovy="38"/>')
    a('    <camera name="hero" pos="0.55 -0.6 0.32" xyaxes="0.75 0.66 0 -0.25 0.28 0.93" fovy="40"/>')
    indent = '    '
    for i in range(N_SEG):
        pos = f'0 0 {KNUCKLE_R + 0.0005}' if i == 0 else f'{L} 0 0'
        a(f'{indent}<body name="s{i}" pos="{pos}">')
        ind = indent + '  '
        if i == 0:
            a(f'{ind}<freejoint name="root"/>')
            a(f'{ind}<site name="latch_tail" pos="0 0 0" size="0.004" rgba="0.8 0.1 0.1 1"/>')
            a(f'{ind}<site name="latch_tail_l" pos="0 0.03 0" size="0.002"/><site name="latch_tail_r" pos="0 -0.03 0" size="0.002"/>')
            # magnet cradle at the tail tip
            a(f'{ind}<geom name="k0" type="cylinder" fromto="0 {-HALF_W} 0 0 {HALF_W} 0" size="{KNUCKLE_R}" material="rubber" mass="0.004"/>')
        else:
            a(f'{ind}<joint name="slop{i}" class="slop"/>')
            a(f'{ind}<joint name="j{i}" class="servo"/>')
        mat = 'link' if i % 2 == 0 else 'link2'
        a(f'{ind}<geom name="box{i}" type="box" pos="{L/2} 0 0" size="{L/2 - 0.0055} {HALF_W - 0.002} {HALF_T}" material="{mat}" mass="{SEG_MASS[i] - 0.003:.4f}"/>')
        if i < N_SEG - 1:   # tread cheeks: on the front of the link that carries the servo (so the tail can hit the s7 knuckle)
            a(f'{ind}<geom name="k{i+1}" type="cylinder" fromto="{L} {-HALF_W} 0 {L} {HALF_W} 0" size="{KNUCKLE_R}" material="rubber" mass="0.003"/>')
        if i in PAYLOAD:
            nm, m, (px, py, pz), (hx, hy, hz) = PAYLOAD[i]
            a(f'{ind}<geom name="{nm}" type="box" pos="{px} {py + (PAYLOAD_Y if nm == "battery" else 0)} {pz}" size="{hx} {hy} {hz}" material="pcb" mass="{m}"/>')
        if i == 4:
            a(f'{ind}<site name="imu_mid" pos="{L/2} 0 {HALF_T + 0.008}" size="0.003"/>')
        if i == N_SEG - 1:
            a(f'{ind}<site name="imu_head" pos="{L/2} 0 {HALF_T + 0.006}" size="0.003"/>')
            a(f'{ind}<site name="latch_head" pos="{L} 0 0" size="0.004" rgba="0.1 0.1 0.8 1"/>')
            a(f'{ind}<site name="latch_head_l" pos="{L} 0.03 0" size="0.002"/><site name="latch_head_r" pos="{L} -0.03 0" size="0.002"/>')
            a(f'{ind}<geom name="k9" type="cylinder" fromto="{L} {-0.033} 0 {L} {0.033} 0" size="0.0055" material="rubber" mass="0.004"/>')   # head lugs + steel pins, no tread
            # two googly eyes on the back of the head (visual only)
            for sy in (-0.018, 0.018):
                a(f'{ind}<geom type="sphere" pos="{L*0.7} {sy} {HALF_T + 0.007}" size="0.008" material="eye" contype="0" conaffinity="0" mass="0.0005"/>')
                a(f'{ind}<geom type="sphere" pos="{L*0.7 + 0.005} {sy} {HALF_T + 0.010}" size="0.004" material="rubber" contype="0" conaffinity="0" mass="0.0001"/>')
        indent = ind
    for i in reversed(range(N_SEG)):
        indent = indent[:-2]
        a(f'{indent}</body>')
    a('  </worldbody>')
    a('  <contact>')
    for i in range(N_SEG - 1):
        a(f'    <exclude body1="s{i}" body2="s{i+1}"/>')
    a('    <exclude body1="s0" body2="s8"/>')   # neighbours once the tail is bitten
    a('  </contact>')
    a('  <equality>')   # the magnetic latch: two pin points = a free hinge across the width
    a('    <connect name="latch_l" site1="latch_tail_l" site2="latch_head_l" active="false" solref="0.006 1"/>')
    a('    <connect name="latch_r" site1="latch_tail_r" site2="latch_head_r" active="false" solref="0.006 1"/>')
    a('  </equality>')
    a('  <actuator>')
    for i in range(1, N_SEG):
        a(f'    <position name="servo{i}" joint="j{i}"/>')
    a('  </actuator>')
    a('  <sensor>')
    a('    <accelerometer name="acc_head" site="imu_head"/><gyro name="gyro_head" site="imu_head"/>')
    a('    <accelerometer name="acc_mid" site="imu_mid"/><gyro name="gyro_mid" site="imu_mid"/>')
    a('  </sensor>')
    a('</mujoco>')
    return '\n'.join(out)


if __name__ == '__main__':
    import pathlib, mujoco
    xml = build_xml()
    p = pathlib.Path(__file__).with_name('ouro.xml')
    p.write_text(xml)
    m = mujoco.MjModel.from_xml_string(xml)
    print(f'wrote {p.name}: nq={m.nq} nv={m.nv} nu={m.nu} total mass={m.body_subtreemass[1]*1000:.0f} g')
