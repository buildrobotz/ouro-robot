"""Dummy solids of every bought part, for fit checks and renders (not for printing).

All dimensions in mm. Each function returns a manifold3d.Manifold in the part's own local frame,
documented in its docstring; cad/assembly.py places them on the links.

MG90S numbers, and where they come from
---------------------------------------
TowerPro-style dimension table (Hobby Components listing): A 32.5, B 22.8, C 28.4, D 12.4, E 32.1, F 18.5.
Read together with the Adafruit/TowerPro drawing (32.5 across the tabs, 35.5 tall with the horn fitted):
  B 22.8  case length                          D 12.2 to 12.4  case thickness (most listings 12.2)
  32.5    length across the mounting tabs       -> tab overhang (32.5 - 22.8) / 2 = 4.85
  F 18.5  case bottom to the TOP of the tabs   -> tabs 2.5 thick, underside at 16.0
  C 28.4  case bottom to the top of the output bearing boss (case top 22.7, round gear hump on top)
  E 32.1  case bottom to the top of the spline
  35.5    case bottom to the top of the horn screw head
Clone-to-clone variation seen across listings: body 22.2 to 23.0 long, 11.8 to 12.6 thick, overall
height 28 to 31 without horn; tab height and spline height differ by +-0.5 mm; horn shapes differ a lot.
Re-measure: case length, thickness, tab underside height, spline top height, horn plate thickness.
"""
import numpy as np
from manifold3d import Manifold, CrossSection

# ---------------------------------------------------------------- MG90S (TowerPro / clone)
MG90S = dict(
    length=22.8, thick=12.2, case_h=22.7,       # case: x length, z thickness, y height to case top
    shaft_from_front=6.0,                        # shaft centre to the front end of the case
    tab_under=16.0, tab_t=2.5, tab_over=4.85,    # tab underside height, thickness, overhang past the case
    hump_r=6.1, hump_top=26.7,                   # round gear-cover hump around the shaft
    idler_r=3.0, idler_x=-8.5, idler_top=25.2,   # small second hump behind the shaft (idler gear)
    boss_r=3.6, boss_top=28.4,                   # output bearing boss (C)
    spline_r=2.4, spline_top=32.1,               # 4.8 mm spline, 20/21 teeth (E)
    cable_z=0.0, cable_y=4.0,                    # lead leaves the rear end, centred, 4 mm above the bottom
    mass=13.4,
)
# single-arm horn from the servo bag (the one-sided "1-arm" horn)
HORN = dict(hub_r=3.5, hub_h=4.3, plate_t=1.7, plate_len=15.5, w0=7.0, w1=4.0, ceil=0.8,
            screw_head_r=2.0, screw_head_h=1.3)


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cbox(cx, cy, cz, sx, sy, sz):
    return box(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, cz - sz / 2, cz + sz / 2)

def ycyl(x, z, r, y0, y1, seg=40):
    return Manifold.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])

def xcyl(y, z, r, x0, x1, seg=32):
    return Manifold.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])

def zcyl(x, y, r, z0, z1, seg=32):
    return Manifold.cylinder(z1 - z0, r, r, seg).translate([x, y, z0])


def mg90s_stator():
    """Servo case, tabs, humps and bearing boss. Frame: output shaft axis = +y through the origin,
    case bottom at y = 0, case from x = -16.8 (cable end) to +6.0, thickness along z."""
    s = MG90S
    x0, x1 = s['shaft_from_front'] - s['length'], s['shaft_from_front']
    t = s['thick'] / 2
    m = box(x0, x1, 0, s['case_h'], -t, t)
    m += box(x0 - s['tab_over'], x1 + s['tab_over'], s['tab_under'], s['tab_under'] + s['tab_t'], -t, t)
    for xh in (x0 - 2.4, x1 + 2.4):                     # 2 mm mounting holes in the tabs
        m -= ycyl(xh, 0, 1.0, s['tab_under'] - 1, s['tab_under'] + 4)
    m += ycyl(0, 0, s['hump_r'], s['case_h'] - 0.01, s['hump_top'])
    m += ycyl(s['idler_x'], 0, s['idler_r'], s['case_h'] - 0.01, s['idler_top'])
    m += ycyl(0, 0, s['boss_r'], s['hump_top'] - 0.01, s['boss_top'])
    # lead strain relief and the first 4 mm of the three-wire ribbon
    m += box(x0 - 3.0, x0 + 0.01, s['cable_y'] - 1.6, s['cable_y'] + 1.6, -2.0, 2.0)
    return m

def mg90s_spline():
    s = MG90S
    return ycyl(0, 0, s['spline_r'], s['boss_top'] - 0.01, s['spline_top'])

def horn(plate_top_y):
    """Single-arm horn + its screw. Frame: hub on the +y axis, arm pointing +x, plate top face at y = plate_top_y.
    The hub hangs below the plate toward the servo (-y)."""
    h = HORN
    pt, pb = plate_top_y, plate_top_y - h['plate_t']
    plate = Manifold.hull(ycyl(0, 0, h['w0'] / 2, pb, pt) + ycyl(h['plate_len'] - h['w1'] / 2, 0, h['w1'] / 2, pb, pt))
    hub = ycyl(0, 0, h['hub_r'], pt - h['hub_h'], pt)
    screw = ycyl(0, 0, h['screw_head_r'], pt, pt + h['screw_head_h'])
    return plate + hub, screw

def horn_outline_r():
    return HORN['w0'] / 2

# ---------------------------------------------------------------- fasteners, magnet, rubber
def screw(d, length, head_d, head_h):
    """Pan-head screw along +y: head underside at y = 0, head at y in [-head_h, 0], shank y in [0, length]."""
    return ycyl(0, 0, d / 2, 0, length, 24) + ycyl(0, 0, head_d / 2, -head_h, 0, 24)

def m3x25():
    return screw(3.0, 25.0, 5.5, 2.4)

def m2x8():
    """The servo's own 2 mm self-tapping mounting screw, used as the pivot."""
    return screw(2.0, 8.0, 3.8, 1.3)

def magnet_6x3():
    """6 x 3 mm N35 disc, axis along +x, x in [0, 3], centred on y = z = 0."""
    return xcyl(0, 0, 3.0, 0, 3.0, 32)

def oring_stadium(core_r, straight, cord=2.5, x0=0.0, seg=36):
    """O-ring stretched over a racetrack core in the xz plane: core radius core_r, two round ends at
    x = x0 and x0 + straight. Cord centred in y = 0. Returned as a solid tube."""
    rc = core_r + cord / 2
    ring = CrossSection.circle(cord / 2, 16).translate([rc, 0])
    tor = Manifold.revolve(ring, seg, 360.0).rotate([90, 0, 0])            # full torus, axis along y
    big = rc + cord
    a = (tor ^ box(0, big, -big, big, -big, big)).translate([x0 + straight, 0, 0])   # front half
    b = (tor ^ box(-big, 0, -big, big, -big, big)).translate([x0, 0, 0])             # rear half
    bars = xcyl(0, rc, cord / 2, x0, x0 + straight, 16) + xcyl(0, -rc, cord / 2, x0, x0 + straight, 16)
    return a + b + bars

def zip_tie_band(path_pts, w=2.5, t=1.1):
    """Zip tie as a chain of thin boxes along a closed polyline in a plane of constant x (points are (y, z))."""
    m = Manifold()
    for (y0, z0), (y1, z1) in zip(path_pts, path_pts[1:] + path_pts[:1]):
        seg = Manifold.hull(cbox(0, y0, z0, w, t, t) + cbox(0, y1, z1, w, t, t))
        m += seg
    return m

# ---------------------------------------------------------------- electronics (local frame: origin at the
# bottom centre of the mounting face, +z up from the mounting surface)
ESP32 = dict(l=51.5, w=28.3, pcb=1.6, row_pitch=25.4, spacer_h=2.5, mod_l=25.5, mod_w=18.0, mod_h=3.1,
             can_l=17.6, mass=9.0)

def esp32_devkit():
    """ESP32 DevKit V1, 30 pin, header spacers DOWN with the pins cut flush under the spacers, wires soldered
    to the pin stubs on top. Frame: long axis along y, short along x, z = 0 at the underside of the spacers,
    centred in x and y. Module (antenna end) at +y. Returns dict of named solids."""
    e = ESP32
    sp = Manifold()
    for sx in (-1, 1):
        sp += cbox(sx * e['row_pitch'] / 2, 0, e['spacer_h'] / 2, 2.54, 15 * 2.54, e['spacer_h'])
    z0 = e['spacer_h']
    pcb = cbox(0, 0, z0 + e['pcb'] / 2, e['w'], e['l'], e['pcb'])
    ztop = z0 + e['pcb']
    mod = cbox(0, e['l'] / 2 - e['mod_l'] / 2, ztop + e['mod_h'] / 2, e['mod_w'], e['mod_l'], e['mod_h'])
    parts = cbox(0, -e['l'] / 2 + 3.5, ztop + 1.5, 8.0, 7.0, 3.0)            # USB socket
    parts += cbox(0, -8.0, ztop + 0.9, 7.0, 7.0, 1.8)                         # AMS1117 + CP2102
    for sx in (-1, 1):                                                        # wire stubs soldered on the pin tops
        parts += cbox(sx * e['row_pitch'] / 2, 0, ztop + 1.0, 2.0, 15 * 2.54, 2.0)
    return dict(pcb=pcb + sp, module=mod, parts=parts)

def gy521():
    """GY-521 (MPU-6050), 21.2 x 16.4, no header (wires soldered). Frame: long side along x, origin at the
    bottom centre of the PCB."""
    pcb = cbox(0, 0, 0.8, 21.2, 16.4, 1.6)
    parts = cbox(0, 0, 1.6 + 0.5, 4, 4, 1.0) + cbox(-6, 3, 1.6 + 0.6, 3, 3, 1.2) + cbox(9.0, 0, 1.6 + 0.8, 1.5, 15, 1.6)
    return dict(pcb=pcb, parts=parts)

def ina219():
    """INA219 breakout 25.6 x 20.5, screw terminal NOT fitted (wires soldered to the pads)."""
    pcb = cbox(0, 0, 0.8, 20.5, 25.6, 1.6)
    parts = cbox(0, 0, 1.6 + 0.5, 3, 3, 1.0) + cbox(4, 0, 1.6 + 0.4, 6.4, 3.2, 0.8) + cbox(-8.5, 0, 1.6 + 1.0, 1.5, 15, 2.0)
    return dict(pcb=pcb, parts=parts)

def mini560():
    """Mini560 fixed 5 V buck, 29 x 18 x 5.4 (castellated module; the inductor is the tall part).
    18 mm side along x (along the body), 29 mm along y."""
    pcb = cbox(0, 0, 0.6, 18.0, 29.0, 1.2)
    parts = cbox(0, -4, 1.2 + 2.1, 8.0, 8.0, 4.2) + cbox(0, 7, 1.2 + 1.2, 10, 8, 2.4)
    return dict(pcb=pcb, parts=parts)

def cap_220uF():
    """C2: 220 uF 10/16 V low-ESR electrolytic, 6.3 x 11 mm, lying along y, bottom at z = 0."""
    return ycyl(0, 3.15, 3.15, -5.5, 5.5, 24)

def ina219_generic():
    """INA219 breakout as fitted on the final board: 20 (x) x 23 (y), terminal block left out, R5 lying flat."""
    pcb = cbox(0, 0, 0.8, 20.0, 23.0, 1.6)
    parts = cbox(0, 0, 1.6 + 0.5, 3, 3, 1.0) + cbox(5, 0, 1.6 + 0.6, 4.0, 12.0, 1.2)
    return dict(pcb=pcb, parts=parts)

def xt30_fuse():
    """XT30 male + female mated (lying) and the radial PTC fuse in heat-shrink beside it: one block 10 x 25 x 6."""
    return cbox(0, 0, 3.0, 10.0, 25.0, 6.0)

def lipo_2s500(l=46.0, w=25.0, t=12.5):
    """2S 500 mAh pouch pack (two ~6 x 25 x 45 cells stacked). Typical SCX24-size listings: 47 x 25 x 10
    to 56 x 28 x 13. Frame: standing on its long thin edge: x = thickness, y = length, z = width (height),
    origin at the bottom centre."""
    pack = cbox(0, 0, w / 2, t, l, w)
    lead = cbox(0, -l / 2 - 4.0, w - 5.0, 4.0, 8.0, 3.0)
    return dict(pack=pack, lead=lead)

def perfboard(lx, ly):
    return cbox(0, 0, 0.8, lx, ly, 1.6)

def servo_header_3x8():
    """3 x 8 straight male header on the perfboard with eight servo plugs fitted. Long side along x."""
    pins = cbox(0, 0, 1.25, 20.3, 7.6, 2.5)
    plugs = cbox(0, 0, 2.5 + 7.0, 20.3, 7.6, 14.0)
    return dict(header=pins, plugs=plugs)

FOAM = 1.0      # double-sided foam tape thickness
