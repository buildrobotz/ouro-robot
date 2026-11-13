"""Ouro printed parts -> STL. `pip install manifold3d numpy` then `python make_parts.py` (writes into cad/out/).

All parts print flat on the belly (z = -8 on the bed), no supports, 0.2 mm layers, 3 walls, 15 % gyroid infill:
  link.stl             x4  s1 s2 s6 s7: rear fork, leaf spring, servo cradle, two O-ring tread cheeks, wire channel
  link_s3_battery.stl  x1  s3: same link + a fence on the rear block that the battery stands against
  link_s4_esp32.stl    x1  s4: same link + a raised pad on the front block that the ESP32 sits on
  link_s5_power.stl    x1  s5: same link, solid front crossbar (no lightening pockets) for the power board
  tail.stl             x1  s0: cradle at the front, magnet notches + two extra O-ring treads at the rear
  head.stl             x1  s8: rear fork, no servo, two M3 x 25 steel screws as the latch pins
  coupon.stl               PRINT THIS FIRST: one cradle + one fork, to check servo and horn fit on your printer

Frame (mm): x along the body, origin at the link's REAR joint axis, front joint axis at x = 48.
y across (servo shaft axis), z up (+z is the back = inside of the wheel). Servo shaft sits at (48, *, 0),
servo case bottom at y = Y0. Every number that depends on your batch of MG90S clones is a parameter at the top;
cad/components.py says where each default comes from, cad/check_fit.py checks the result.
"""
import os
import numpy as np
from manifold3d import Manifold, CrossSection

L, HW, HT = 48.0, 40.0, 8.0          # pitch, half width, half thickness  (fixed by the simulation)
# --- MG90S. Re-measure yours with calipers and edit (heights are from the case bottom). ---
S_LEN, S_THK, S_H = 22.8, 12.2, 22.7  # case length (x), thickness (z), height to case top (y)
S_SHAFT = 6.0                         # shaft centre to the front end of the case
S_TAB_Y, S_TAB_T, S_TAB_L = 16.0, 2.5, 4.85  # tab underside height, tab thickness, tab overhang
S_SPLINE_TOP = 32.1                   # top of the output spline
HORN_T, HORN_CEIL = 1.7, 0.8          # horn plate thickness; closed top of the horn hub above the spline bore
HORN_W0, HORN_W1, HORN_LEN = 7.0, 4.0, 15.5   # horn plate width at the hub, at the tip; hub centre to tip
Y0 = -17.0                            # servo case bottom
CLR = 0.25                            # print clearance
# --- the fork that grabs the servo: horn arm on +y, pivot arm on -y ---
HORN_IN = Y0 + S_SPLINE_TOP + 0.3 + HORN_CEIL - HORN_T   # inner face of the horn arm = underside of the horn plate
HORN_OUT = HORN_IN + 5.0
PIV_IN, PIV_OUT = Y0 - 4.5, Y0 - 8.5  # pivot arm faces; the 2 mm pivot screw threads into a boss at Y0-4..Y0-2
HOLE_R = 2.4                          # screwdriver/screw-head access holes (horn screw head is ~4 mm)
# --- compliance and tread ---
FLEX_T, FLEX_X0, FLEX_X1 = 1.0, 17.5, 27.5   # leaf spring: 1.0 mm thick, 10 mm long (+1 mm root fillets), 66 mm wide
FLEX_FILLET = 1.0                            # root fillets, both ends (fatigue)
TREAD_R, GROOVE_R = 8.0, 6.5                 # 18 x 2.5 mm O-ring sits on the R6.5 groove core, stands 1 mm proud of R8
BODY_HW = 33.0                               # body half width inboard of the O-ring grooves
GROOVE_Y = 36.5                              # groove |y| 33..36.5, flange |y| 36.5..40
# --- zip tie that holds the servo down (one per servo, 2.5 mm tie) ---
TIE_X0, TIE_X1 = 37.5, 40.5                  # tie lane (behind the gear humps)
TIE_SLOT_Y = (8.4, 10.6)                     # where the tie passes the floor beyond the servo top
TIE_RECESS = 0.75                            # belly recess for the tie (leaves 0.9 mm of floor)
# --- payload mounts ---
PAD_TOP = HT + 4.1                           # s4 ESP32 pad / wire channel top (ESP32 PCB sits 1 mm of foam above)
FENCE_X0, FENCE_T, FENCE_TOP = 13.5, 1.6, HT + 22.0   # s3 battery fence on the rear crossbar
FENCE_Y = (-BODY_HW, 7.5)
CHAN_Y = (24.5, 31.5)                        # wire channel on the front crossbar (inner faces)
CHAN_X = (FLEX_X1, 31.0)                     # s3/s4: short and rearward, so s4's ESP32 clears it when j4 is at +90
CHAN_X_PLAIN = (30.0, 34.0)                  # other links: forward, so s5's servo-plug wires (z 29) clear s6 at j6 +92


# ----------------------------------------------------------------------------------------------- primitives
def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def ycyl(x, z, r, y0, y1, seg=48):
    return Manifold.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])

def ccw(pts):
    a = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    return pts if a > 0 else pts[::-1]

def prism_xz(pts, y0, y1):
    """Extrude a polygon given in the xz plane across y0..y1."""
    return Manifold.extrude(CrossSection([ccw(pts)]), y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])

def tdy(x, z, r, y0, y1):
    """Horizontal hole along y with a 45 deg teardrop roof, so it prints belly-down without support."""
    roof = prism_xz([(x - r * 0.7071, z + r * 0.7071), (x + r * 0.7071, z + r * 0.7071), (x, z + r * 1.4142)], y0, y1)
    return ycyl(x, z, r, y0, y1) + roof + prism_xz([(x - r * 0.7071, z), (x + r * 0.7071, z),
                                                     (x + r * 0.7071, z + r * 0.7071), (x - r * 0.7071, z + r * 0.7071)], y0, y1)

def chamfer_front(xc, r, y0, y1):
    """Material to remove so the front-bottom quarter of a round knuckle becomes a 45 deg slope."""
    return prism_xz([(xc - 1, -r - 1), (xc + r + 3, -r - 1), (xc + r + 3, 3)], y0, y1)

def chamfer_rear(xc, r, y0, y1):
    return prism_xz([(xc + 1, -r - 1), (xc - r - 3, -r - 1), (xc - r - 3, 3)], y0, y1)

def racetrack(r, y0, y1, xa=38.0, xb=L):
    return box(xa, xb, y0, y1, -r, r) + ycyl(xa, 0, r, y0, y1) + ycyl(xb, 0, r, y0, y1)

def sides(fn):
    """Call fn(a, b) for the +y and -y copies of a feature spanning |y| in [lo, hi]."""
    def both(lo, hi):
        return fn(lo, hi) + fn(-hi, -lo)
    return both


# ----------------------------------------------------------------------------------------------- the link
def horn_outline(c=0.0):
    return [(0.0, HORN_W0 / 2 + c), (HORN_LEN - HORN_W1 / 2, HORN_W1 / 2 + c)]

def horn_recess():
    """Pocket in the inner face of the horn arm that the horn plate presses into. Its roof slopes at 45 deg."""
    d = HORN_T + 0.15
    def plate(y0, y1, dz):
        (x0, r0), (x1, r1) = horn_outline(0.15)
        return Manifold.hull(ycyl(x0, dz, r0, y0, y1) + ycyl(x1, dz, r1, y0, y1))
    return Manifold.hull(plate(HORN_IN - 0.01, HORN_IN + d, 0) + plate(HORN_IN - 0.01, HORN_IN, d))

def rear_arms():
    """Fork that grabs the previous link's servo: horn arm (+y) and pivot arm (-y), round rear knuckles."""
    horn = box(0, FLEX_X0, HORN_IN, HORN_OUT, -HT, HT) + ycyl(0, 0, HT, HORN_IN, HORN_OUT)
    horn -= chamfer_rear(0, HT, HORN_IN - 1, HORN_OUT + 1)
    piv = box(0, FLEX_X0, PIV_OUT, PIV_IN, -HT, HT) + ycyl(0, 0, HT, PIV_OUT, PIV_IN)
    piv -= chamfer_rear(0, HT, PIV_OUT - 1, PIV_IN + 1)
    return horn + piv

# --- +90 deg hard end stop: a toe on every link's rear crossbar (|y| 26..33, both sides) lands flat on the top
#     of the previous link's cheeks (x 40..43 there) at ~89.2 deg, before any electronics can touch. ---
TOE_X, TOE_Z0, STOP_Y = 7.95, 5.0, (26.0, BODY_HW)
CHAMFER_C2 = 2.4

def end_stop_toe():
    pts = [(TOE_X, HT), (TOE_X, TOE_Z0), (13.6, TOE_Z0 - (13.6 - TOE_X)), (13.6, HT)]   # 45 deg underside
    return prism_xz(pts, STOP_Y[0], STOP_Y[1]) + prism_xz(pts, -STOP_Y[1], -STOP_Y[0])

def fork_cuts():
    """Cut after the fork is joined to the rear crossbar (the horn tip reaches into the crossbar)."""
    return (horn_recess() + tdy(0, 0, HOLE_R, HORN_IN, HORN_OUT + 1)                     # horn plate, screw head + driver
            + tdy(0, 0, 1.2, PIV_OUT - 1, PIV_IN + 1))                                  # free hole for the M2 pivot

def leaf():
    """The leaf spring (belly side) with root fillets. It is the ONLY thing joining the rear block to the front block."""
    f = FLEX_FILLET
    m = box(FLEX_X0, FLEX_X1, -BODY_HW, BODY_HW, -HT, -HT + FLEX_T)
    zt = -HT + FLEX_T
    for xc, sgn in ((FLEX_X0, 1), (FLEX_X1, -1)):
        corner = box(min(xc, xc + sgn * f), max(xc, xc + sgn * f), -BODY_HW, BODY_HW, zt - 0.01, zt + f)
        m += corner - ycyl(xc + sgn * f, zt + f, f, -BODY_HW - 1, BODY_HW + 1, 24)
    return m

def spine():
    """Rear crossbar, then the leaf spring that makes every joint series-elastic."""
    return box(13.5, FLEX_X0, -BODY_HW, BODY_HW, -HT, HT) + leaf()

def servo_void():
    xr = L - S_LEN + S_SHAFT
    v = box(xr - CLR, L + S_SHAFT + CLR, Y0 - CLR, 12.0, -S_THK / 2 - CLR, HT + 1)                 # case + gear humps
    v += box(xr - S_TAB_L - CLR, L + S_SHAFT + S_TAB_L + CLR, Y0 + S_TAB_Y - CLR, Y0 + S_TAB_Y + S_TAB_T + CLR,
             -S_THK / 2 - CLR, HT + 1)                                                               # tabs
    v += box(25.5, xr + 0.5, Y0 + 1.0, Y0 + 7.0, -2.6, HT + 1)                                       # lead exit
    return v

def cradle(pockets=True):
    """Front block: servo pocket (drop in from the back, one zip tie), pivot boss, two tread cheeks."""
    c = box(FLEX_X1, 34.0, -BODY_HW, BODY_HW, -HT, HT)                                # crossbar 2
    c += box(29.0, 56.2, Y0 - 2.0, 12.0, -HT, -S_THK / 2 - CLR)                       # floor
    c += box(29.0, 56.2, Y0 - 2.0, Y0 - CLR, -HT, 6.0)                                # wall under the servo
    c += box(L + S_SHAFT + CLR, 56.2, Y0 - 2.0, Y0 + S_TAB_Y - CLR, -HT, 6.0)         # front post under the tab
    c += ycyl(L, 0, 4.5, Y0 - 4.0, Y0 - 2.0) + box(L - 4.5, L + 4.5, Y0 - 4.0, Y0 - 2.0, -HT, 0)   # pivot boss, down to the bed
    for s in (-1, 1):                                                                # cheeks with O-ring groove
        a, b = sorted((s * 26.0, s * BODY_HW))
        c += (box(34.0, L, a, b, -HT, HT) + ycyl(L, 0, HT, a, b)) - chamfer_front(L, HT, a - 1, b + 1)
        a, b = sorted((s * BODY_HW, s * GROOVE_Y)); c += racetrack(GROOVE_R, a, b)
        a, b = sorted((s * GROOVE_Y, s * HW));      c += racetrack(TREAD_R, a, b)
    if pockets:
        for a, b in ((12.0, 22.5), (-25.0, -20.0)):                                  # lightening pockets
            c -= box(29.0, 32.6, a, b, -HT + 1.6, HT + 1)
    c -= servo_void()
    c -= tdy(L, 0, 0.8, Y0 - 4.5, Y0)                                                # M2 pivot screw pilot
    c -= tdy(L, 0, HOLE_R, 15.0, HW + 1)                                             # driver path to the horn screw
    c -= tdy(L, 0, HOLE_R + 0.2, -HW - 1, -26.0)                                     # driver path + head of the pivot screw
    c -= box(TIE_X0, TIE_X1, Y0 - 3.6, TIE_SLOT_Y[1] + 0.2, -HT - 1, -HT + TIE_RECESS)   # tie recess under the belly
    c -= box(TIE_X0, TIE_X1, TIE_SLOT_Y[0], TIE_SLOT_Y[1], -HT - 1, 0)               # tie slot through the floor
    return c

def wire_channel(top=HT + 3.5, cx=CHAN_X):
    """Open-top channel for the servo leads on the front crossbar, with 45 deg snap lips. Front block only."""
    y0, y1 = CHAN_Y
    m = box(cx[0], cx[1], y0 - 1.2, y0, HT - 0.01, top) + box(cx[0], cx[1], y1, y1 + 1.2, HT - 0.01, top)
    lip = 1.1
    for y, s in ((y0, 1), (y1, -1)):                 # lip triangle in the yz plane, 45 deg underside
        tri = CrossSection([ccw([(0, top - lip - 0.8), (s * lip, top - 0.8), (s * lip, top - 0.01), (0, top - 0.01)])])
        m += Manifold.extrude(tri, cx[1] - cx[0]).transform(np.array([[0, 0, 1, cx[0]], [1, 0, 0, y], [0, 1, 0, 0]], float))
    return m

def lightening(rear=True):
    """Pockets open to the back (print without support): rear crossbar between the fork arms, and the cheeks."""
    floor = -HT + 1.6
    m = Manifold()
    if rear:
        for a, b in ((-20.3, 13.3), (20.7, 31.8), (-31.8, -26.7)):
            m += box(14.7, FLEX_X0 - 1.2, a, b, floor, HT + 1)
    for s in (-1, 1):
        a, b = sorted((s * 27.2, s * 31.8))
        m += box(35.2, 39.4, a, b, floor, HT + 1)            # stops short of the end-stop landing at x 40..43.5
    return m

def link(variant='plain'):
    m = rear_arms() + spine() + end_stop_toe() + cradle(pockets=(variant in ('plain', 'battery')))
    m -= fork_cuts() + lightening(rear=(variant != 'battery'))
    if variant == 'plain':      # 45 deg chamfer on the top rear edge of the front crossbar: more room for s5's plug wires at j6
        m -= prism_xz([(FLEX_X1 - 0.1, HT + 0.1), (FLEX_X1 - 0.1, HT - CHAMFER_C2), (FLEX_X1 + CHAMFER_C2, HT + 0.1)], 23.4, BODY_HW + 1)
    if variant != 'power':
        m += wire_channel(PAD_TOP if variant == 'esp32' else HT + 3.5,
                          CHAN_X if variant in ('esp32', 'battery') else CHAN_X_PLAIN)   # on s4 the walls also carry the ESP32
    if variant == 'battery':                     # fence the battery stands against, two tie windows
        f = box(FENCE_X0, FENCE_X0 + FENCE_T, FENCE_Y[0], FENCE_Y[1], HT - 0.01, FENCE_TOP)
        for yc in (-24.0, -4.0):
            f -= box(FENCE_X0 - 1, FENCE_X0 + FENCE_T + 1, yc - 1.6, yc + 1.6, HT, HT + 2.8)
        m += f
    if variant == 'esp32':                       # pad on crossbar 2; the ESP32 header spacers hang either side
        xr = L - S_LEN + S_SHAFT - CLR - 0.3                                         # stop short of the servo pocket
        m += box(FLEX_X1 + 0.5, xr, -9.5, CHAN_Y[0] - 1.6, HT - 0.01, PAD_TOP)       # (lead exit is at y -16..-10)
    return m


# ----------------------------------------------------------------------------------------------- tail and head
NOTCH_R = 1.75           # head pin (M3 thread, 3.0 mm) seats in this
NOTCH_BLOCK = (14.5, 24.5)
MAG_X = (NOTCH_R + 0.8, NOTCH_R + 0.8 + 3.25)    # 6 x 3 magnet pocket behind 0.8 mm of plastic
LATCH_WIRE = 0.6         # groove for a bare 0.5 mm tinned copper wire
LATCH_WIRE_R, LATCH_WIRE_ANG = NOTCH_R + 0.1, -45.0   # wire centre: stands 0.15 proud of the seat, lower front

def notch_cut(y0, y1):
    """Seat for the pin at x = 0 with a 45 deg funnel opening to the rear (no flat roof to print)."""
    r = NOTCH_R
    seat = tdy(0, 0, r, y0, y1)
    funnel = prism_xz([(0, r * 1.414), (-6, r * 1.414 + 6), (-6, -r - 6), (0, -r)], y0, y1)
    return seat + funnel

def tail_rear():
    """Rear of the tail (s0): crossbar, two magnet/notch blocks, cheeks and the extra pair of O-ring treads
    that put rubber under the latch corner of the wheel (the sim has a tread there)."""
    c = box(13.5, FLEX_X0, -BODY_HW, BODY_HW, -HT, HT)
    for s in (-1, 1):
        a, b = sorted((s * NOTCH_BLOCK[0], s * NOTCH_BLOCK[1])); ym = (a + b) / 2
        blk = box(-2.0, 13.5, a, b, -HT, HT)
        blk -= notch_cut(a - 1, b + 1)
        blk -= box(MAG_X[0], MAG_X[1], ym - 3.15, ym + 3.15, -3.15, HT + 1)              # magnet, loaded from the back
        # latch-sense wire: across the seat (lower front, -40 deg) and up the outer face to the back
        ang = np.radians(LATCH_WIRE_ANG); wx, wz = LATCH_WIRE_R * np.cos(ang), LATCH_WIRE_R * np.sin(ang)
        blk -= Manifold.hull(tdy(wx, wz, LATCH_WIRE / 2, a - 1, b + 1)                # groove opens cleanly into the seat
                             + ycyl(wx * 0.7, wz * 0.7, LATCH_WIRE / 2, a - 1, b + 1))
        yo = b if s > 0 else a
        blk -= box(wx - LATCH_WIRE / 2, wx + LATCH_WIRE / 2, *sorted((yo - s * 0.6, yo + s * 0.1)), wz, HT + 1)
        c += blk
        a, b = sorted((s * 26.0, s * BODY_HW))
        c += box(6.5, FLEX_X0, a, b, -HT, HT)                                                  # cheek, clear of the head fork
        a, b = sorted((s * BODY_HW, s * GROOVE_Y)); c += racetrack(GROOVE_R, a, b, xa=0.0, xb=10.0)
        a, b = sorted((s * GROOVE_Y, s * HW));      c += racetrack(TREAD_R, a, b, xa=0.0, xb=10.0)
    return c

HEAD_FORKS = ((9.0, 13.0), (26.0, BODY_HW))
PIN_R, PIN_HEAD_R, PIN_HEAD_DEPTH = 1.3, 3.0, 2.9   # M3 self-tapping pilot; pan head 5.5 x 2.4 sunk in the outer fork

def head_front():
    c = box(FLEX_X1, 38.0, -BODY_HW, BODY_HW, -HT, HT)                               # IMU sits on top of this
    r = 5.5
    for s in (-1, 1):
        for lo, hi in HEAD_FORKS:
            a, b = sorted((s * lo, s * hi))
            fork = box(38.0, L, a, b, -HT, r) + ycyl(L, 0, r, a, b)
            fork += Manifold.hull(ycyl(L, 0, r, a, b) + box(38.0, L - 0.22, a, b, -HT, -HT + 0.01))
            c += fork
        a, b = sorted((s * 8.0, s * (BODY_HW + 1)))
        c -= tdy(L, 0, PIN_R, a, b)                                                   # M3 x 25 steel screw = latch pin
        a, b = sorted((s * (BODY_HW - PIN_HEAD_DEPTH), s * (BODY_HW + 1)))
        c -= tdy(L, 0, PIN_HEAD_R, a, b)                                              # screw head sunk flush
    return c


# ----------------------------------------------------------------------------------------------- output
def to_stl(man, path):
    m = man.to_mesh(); v = np.asarray(m.vert_properties)[:, :3]; t = np.asarray(m.tri_verts)
    tri = v[t]; n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = n, tri
    with open(path, 'wb') as f:
        f.write(b'Ouro'.ljust(80, b' ')); f.write(np.uint32(len(t)).tobytes()); f.write(rec.tobytes())

def build_parts():
    tail = tail_rear() + leaf() + cradle() + wire_channel(cx=CHAN_X_PLAIN) - lightening(rear=False)
    for s in (-1, 1):                                                              # tail rear cheeks
        a, b = sorted((s * 27.2, s * 31.8)); tail -= box(7.7, FLEX_X0 - 1.2, a, b, -HT + 1.6, HT + 1)
    head = rear_arms() + spine() + end_stop_toe() + head_front() + wire_channel(cx=CHAN_X_PLAIN) - fork_cuts()
    fork = rear_arms() + box(13.5, FLEX_X0, PIV_OUT, HORN_OUT, -HT, HT) - fork_cuts()
    coupon = fork.translate([62, 0, 0]) + (cradle() ^ box(26, 140, PIV_OUT - 2, 24, -9, 9))
    return {
        'link': link('plain'), 'link_s3_battery': link('battery'), 'link_s4_esp32': link('esp32'),
        'link_s5_power': link('power'), 'tail': tail, 'head': head, 'coupon': coupon,
    }

PARTS = build_parts()
COUNT = {'link': 4, 'link_s3_battery': 1, 'link_s4_esp32': 1, 'link_s5_power': 1, 'tail': 1, 'head': 1, 'coupon': 0}
DENSITY = 1.27e-3 * 0.55      # g/mm3: PETG (1.27 g/cm3) at 3 walls + 15 % gyroid prints at roughly 55 % of solid

if __name__ == '__main__':
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
    os.makedirs(out, exist_ok=True)
    total = 0.0
    for name, part in PARTS.items():
        to_stl(part, os.path.join(out, f'{name}.stl'))
        bb = part.bounding_box(); g = part.volume() * DENSITY; total += g * COUNT[name]
        print(f'{name:16s} {part.volume()/1000:5.1f} cm3 solid  ~{g:4.1f} g printed  '
              f'bbox {bb[3]-bb[0]:.0f} x {bb[4]-bb[1]:.0f} x {bb[5]-bb[2]:.0f} mm  pieces={len(part.decompose())}')
    print(f'robot set (4 link + 3 variants + tail + head): ~{total:.0f} g printed')
