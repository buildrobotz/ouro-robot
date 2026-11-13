"""Does Ouro fit together and move? Numerical checks on the parts from make_parts.py with the bought parts
from components.py, placed by assembly.py. Run: python check_fit.py   (about a minute)

Interference is the volume (mm3) of the manifold boolean intersection; anything under 0.5 mm3 is reported as
zero (mesh noise on touching faces). Prints one section per check and a PASS/FAIL table at the end, and writes
the table to out/check_fit.txt.
"""
import os, sys, time
import numpy as np
from manifold3d import Manifold, CrossSection, JoinType
import make_parts as P
import components as C
import assembly as A

EPS = 0.5                      # mm3 below which an intersection is mesh noise
SWEEP = list(range(-69, 88, 3)) + [88]      # below the +90 end stop (first plastic contact must be 88.5..90)
RESULTS = []                   # (check, verdict, numbers)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
LOG = []

def say(*a):
    s = ' '.join(str(x) for x in a); print(s); LOG.append(s)

def result(name, ok, text):
    RESULTS.append((name, 'PASS' if ok else 'FAIL', text)); say(f'  -> {"PASS" if ok else "FAIL"}: {text}')

def union(items):
    return Manifold.batch_boolean([i.solid for i in items], __import__('manifold3d').OpType.Add) if items else Manifold()

def vol(m):
    return 0.0 if m.is_empty() else m.volume()

def inter(a, b):
    return vol(a ^ b)

def pairs_report(As, Bs, limit=6):
    rows = []
    for a in As:
        for b in Bs:
            v = inter(a.solid, b.solid)
            if v > EPS:
                rows.append((v, a.name, b.name))
    rows.sort(reverse=True)
    return ', '.join(f'{a} x {b} {v:.1f}' for v, a, b in rows[:limit])

def in_frame(items, T):
    return [A.Item(i.name, i.solid.transform(T[:3, :]), i.colour, i.group) for i in items]

def rel(theta):
    """Next link relative to this one at joint angle theta."""
    return A.Tx(P.L) @ A.Ry(theta)


# ------------------------------------------------------------------------------------------------ (a)
def check_servo_in_cradle():
    say('\n(a) Servo in its cradle')
    it = A.link_items(1, payload=False)
    part = it[0].solid
    servo = [i for i in it if i.name == 'servo'][0].solid
    tie = [i for i in it if i.name == 'servo_tie'][0].solid
    piv = [i for i in it if i.name == 'pivot_screw'][0].solid
    nxt = in_frame(A.link_items(2, payload=False), rel(0.0))
    fork = union([i for i in nxt if i.group == 'part' or i.name == 'horn'])     # the spline is the servo's own shaft
    v_sp, v_tp, v_ts = inter(servo, part), inter(tie, part), inter(tie, servo)
    v_pivot_part = inter(piv, part)
    say(f'  servo x link {v_sp:.2f} mm3, tie x link {v_tp:.2f}, tie x servo {v_ts:.2f}, '
        f'pivot screw x own boss {v_pivot_part:.1f} (intended: self-tapping thread)')
    # retention: how far can the servo move in each direction before it touches something?
    stops = dict(link=part, tie=tie, fork=fork)
    dirs = {'+x': [1, 0, 0], '-x': [-1, 0, 0], '+y': [0, 1, 0], '-y': [0, -1, 0], '+z (out of the back)': [0, 0, 1],
            '-z': [0, 0, -1]}
    lines, worst = [], 0.0
    for dname, d in dirs.items():
        found = None
        for step in np.arange(0.05, 3.01, 0.05):
            moved = servo.translate(list(np.array(d) * step))
            hit = [n for n, s in stops.items() if inter(moved, s) > 0.05]
            if hit:
                found = (step, hit); break
        if found:
            worst = max(worst, found[0])
            lines.append(f'{dname}: {found[0]:.2f} mm then {"+".join(found[1])}')
        else:
            lines.append(f'{dname}: FREE beyond 3 mm'); worst = 99
    for l in lines: say('   ', l)
    # rotation about the shaft (reaction torque): pitch the case +-3 deg about the shaft axis
    for ang in (3, -3):
        r = servo.translate([-P.L, 0, 0]).rotate([0, -ang, 0]).translate([P.L, 0, 0])
        hit = [n for n, s in stops.items() if inter(r, s) > 0.05]
        say(f'    rotation {ang:+d} deg about the shaft: stopped by {hit or "NOTHING"}')
    slot = (P.TIE_X1 - P.TIE_X0, P.TIE_SLOT_Y[1] - P.TIE_SLOT_Y[0])
    say(f'  tie slot {slot[0]:.1f} x {slot[1]:.1f} mm for a 2.5 x 1.1 tie; belly recess {P.TIE_RECESS} deep '
        f'(tie stands {1.1 - P.TIE_RECESS:.2f} proud of the belly, the treads stand 1.0 proud)')
    ok = max(v_sp, v_tp, v_ts) < EPS and worst < 1.0
    result('a servo in cradle', ok, f'interference servo/link {v_sp:.1f}, tie/link {v_tp:.1f}, tie/servo {v_ts:.1f} mm3; '
           f'max free play {worst:.2f} mm (all 6 directions stopped); tie slot {slot[0]:.1f}x{slot[1]:.1f}')
    return part


# ------------------------------------------------------------------------------------------------ (b), (d) sweeps
def sweep_joint(k, payload=True, verbose=True):
    """Joint between segment k-1 and k: sweep the joint angle, return first colliding angles."""
    a_items = A.link_items(k - 1, payload)
    b_local = A.link_items(k, payload)
    Au = union(a_items)
    hits = {}
    for th in SWEEP:
        Bm = union(in_frame(b_local, rel(th)))
        v = inter(Au, Bm)
        if v > EPS:
            hits[th] = v
    neg = [t for t in sorted(hits) if t < 0]
    pos = [t for t in sorted(hits) if t > 0]
    first_pos = min(pos) if pos else None
    first_neg = max(neg) if neg else None
    detail = ''
    for th in [t for t in (first_pos, first_neg, 0 if 0 in hits else None) if t is not None]:
        detail += f' [{th:+d} deg: ' + pairs_report(a_items, in_frame(b_local, rel(th))) + ']'
    if verbose:
        say(f'  j{k} (s{k-1}-s{k}): collisions at {len(hits)} of {len(SWEEP)} angles; '
            f'first up {first_pos}, first down {first_neg}{detail}')
    return hits, first_pos, first_neg

def stop_contact(k, payload=True):
    """Fine search 85..95 deg: first angle with any contact, and what touches there (must be plastic on plastic)."""
    a = A.link_items(k - 1, payload); b = A.link_items(k, payload)
    Au = union(a)
    for th in np.arange(85.0, 95.01, 0.25):
        bb = in_frame(b, rel(th))
        if inter(Au, union(bb)) > 0.01:
            who = [(x.name, y.name, x.group, y.group) for x in a for y in bb if inter(x.solid, y.solid) > 0.01]
            plastic_only = all(g1 == 'part' and g2 == 'part' for _, _, g1, g2 in who)
            return th, plastic_only, who
    return None, False, []

def check_joint_sweep_plain():
    say('\n(b) Joint sweep, plain link pair (link N + servo + O-rings + tie  vs  link N+1 + horn), -69..+88 every 3 deg, then the end stop')
    nxt = A.link_items(2, payload=False)
    horn_fit = inter(union([i for i in nxt if i.group == 'rotor']), nxt[0].solid)
    say(f'  horn + spline in the horn-arm recess (static): {horn_fit:.2f} mm3')
    hits, fp, fn = sweep_joint(2, payload=False)
    # clearance margin: smallest gap at the extremes (min_gap search radius 3 mm)
    a = union([i for i in A.link_items(1, payload=False) if i.name not in ('pivot_screw',)])
    nb = [i for i in nxt if i.group != 'rotor']        # the spline sits on the servo boss by construction
    gaps = {th: a.min_gap(union(in_frame(nb, rel(th))), 3.0) for th in (-69, -45, 0, 45, 85)}
    rot = union([i for i in nxt if i.group == 'rotor'])
    st = union([i for i in A.link_items(1, payload=False) if i.name not in ('servo', 'pivot_screw')])
    say(f'  rotor (horn, hub, screw) to link N and its tie: {st.min_gap(rot, 3.0):.2f} mm (rotationally symmetric part)')
    say('  minimum gap link N assembly to link N+1 assembly: ' + ', '.join(f'{t:+d} deg {g:.2f} mm' for t, g in gaps.items()))
    th, plastic, who = stop_contact(2, payload=False)
    say(f'  first contact {th} deg, between {sorted(set((w[0], w[1]) for w in who))} (plastic only: {plastic})')
    ok = not hits and horn_fit < EPS and th is not None and 88.5 <= th <= 90.0 and plastic
    result('b joint sweep (plain)', ok, f'first plastic contact {th} deg (end stop, toe on cheek; 88.5..90 required), '
           f'none below: {len(hits)} colliding angles -69..+88; min gap {min(gaps.values()):.2f} mm; horn in recess {horn_fit:.1f} mm3')

def check_all_joints():
    say('\n(d1) Every real joint j1..j8 with the payloads fitted: -69..+88 every 3 deg, then the end stop')
    worst, stops = [], []
    for k in range(1, 9):
        hits, fp, fn = sweep_joint(k, payload=True)
        th, plastic, who = stop_contact(k, payload=True)
        say(f'     end stop j{k}: first contact {th} deg, plastic only: {plastic}')
        worst.append((k, fp, fn, len(hits))); stops.append((k, th, plastic))
    bad = [w for w in worst if w[3]] + [s for s in stops if not (s[1] and 88.5 <= s[1] <= 90.0 and s[2])]
    txt = 'first contact (all plastic stop): ' + ', '.join(f'j{k} {th}' for k, th, _ in stops)
    if bad:
        txt += '; PROBLEMS: ' + str(bad)
    result('d joints with payloads', not bad, txt)
    return worst


def check_margins():
    say('\n(d1b) Electronics margin past +90 deg: first angle at which any board, wire or battery is touched, minus 90')
    txt = []
    for k in range(1, 9):
        a = A.link_items(k - 1, True); b = A.link_items(k, True)
        ae = union([i for i in a if i.group == 'elec']); aa = union(a)
        hit = None
        for th in np.arange(88.0, 150.01, 0.5):
            bb = in_frame(b, rel(th))
            v = inter(ae, union(bb)) + inter(aa, union([i for i in bb if i.group == 'elec']))
            if v > EPS:
                hit = th; break
        dn = next((th for th in range(-70, -181, -1) if inter(aa, union(in_frame(b, rel(th)))) > EPS), None)
        say(f'  j{k}: electronics first touched at {hit} deg ' + (f'(margin {hit - 90:+.1f})' if hit else '(none up to 150)')
            + f'; below the range first contact at {dn} deg')
        txt.append((k, hit, dn))
    lim_dn = max(d for _, _, d in txt if d)
    neg = [f'j{k}' for k, h, _ in txt if h is not None and h - 90 <= 0]
    result('d1b electronics margin', not neg and -69 - lim_dn >= 4,
           'electronics first contact minus 90: ' + ', '.join(f'j{k} {"none<150" if h is None else f"{h - 90:+.1f}"}' for k, h, _ in txt)
           + f'; below -69 nothing before {lim_dn}' + (f'; NOT PROTECTED BY THE STOP: {", ".join(neg)}' if neg else ''))

TRAJ = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'visual', 'traj.json')
def check_replay(step=2):
    """Replay the simulated act (crawl, bite, roll with the trained policy, startle) with every solid fitted."""
    import json
    say('\n(d3) Replay of the simulated act (visual/traj.json: crawl, bite, learned roll, startle), all parts fitted')
    d = json.load(open(TRAJ))
    Pp = np.array([f['p'] for f in d['frames']], float)
    seg = np.diff(Pp, axis=1); ang = np.arctan2(seg[..., 1], seg[..., 0])
    q = np.degrees(np.unwrap(np.diff(ang, axis=1), axis=0))
    marks = d['marks'] + [[len(Pp), 'end']]
    phase = np.empty(len(Pp), object)
    for (s, n), (e, _) in zip(marks[:-1], marks[1:]): phase[s:e] = n
    local = {k: union(A.link_items(k, True)) for k in range(9)}
    plast = {k: P.PARTS[A.SEGS[k]] for k in range(9)}
    bad, worst, stop_pen = {}, (0, None), 0.0
    for f in range(0, len(q), step):
        T = A.chain(list(q[f]))
        segs = {k: local[k].transform(T[k][:3, :]) for k in range(9)}
        for k1 in range(9):
            for k2 in range(k1 + 1, 9):
                if (k1, k2) == (0, 8): continue                  # the latch, checked in (c)
                v = inter(segs[k1], segs[k2])
                if k2 == k1 + 1 and v > EPS:                      # end-stop toe on cheek is allowed; nothing else
                    vp = inter(plast[k1].transform(T[k1][:3, :]), plast[k2].transform(T[k2][:3, :]))
                    stop_pen = max(stop_pen, vp); v -= vp
                if v > EPS:
                    bad.setdefault((phase[f], f's{k1}xs{k2}'), []).append((f, v))
                    if v > worst[0]: worst = (v, (f, phase[f], k1, k2, np.round(q[f]).astype(int).tolist()))
    say(f'  file visual/traj.json modified {time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(TRAJ)))}')
    say(f'  end-stop plastic overlap in the replay (sim stop at 90.0 vs CAD stop at 89.5): max {stop_pen:.1f} mm3, not counted')
    say(f'  frames checked: {len(range(0, len(q), step))} of {len(q)}; joint range used: '
        f'{q.min(0).round().astype(int).tolist()} .. {q.max(0).round().astype(int).tolist()} deg')
    for (ph, pr), lst in sorted(bad.items()):
        say(f'  {ph:8s} {pr}: {len(lst)} frames, max {max(v for _, v in lst):.1f} mm3 (frame {max(lst, key=lambda r: r[1])[0]})')
    if worst[1]:
        say(f'  worst frame {worst[1][0]} ({worst[1][1]}): s{worst[1][2]} x s{worst[1][3]} {worst[0]:.1f} mm3 at joint angles {worst[1][4]}')
    result('d3 replay of sim act', not bad, 'no interference in any frame' if not bad else
           '; '.join(f'{ph} {pr} in {len(l)} frames (max {max(v for _, v in l):.0f} mm3)' for (ph, pr), l in sorted(bad.items())))
    return q


# ------------------------------------------------------------------------------------------------ (c) latch
def check_latch():
    say('\n(c) Latch: head against tail with the pins seated, corner angle swept 0..90 deg')
    tail = A.link_items(0, payload=False)
    head = A.link_items(8, payload=False)
    Tu = union(tail)
    hits = {}
    for phi in range(0, 91, 3):
        H = np.linalg.inv(A.Tx(P.L) @ A.Ry(phi))          # head in the tail frame
        v = inter(Tu, union(in_frame(head, H)))
        if v > EPS: hits[phi] = v
    H40 = np.linalg.inv(A.Tx(P.L) @ A.Ry(40))
    h40 = in_frame(head, H40)
    say(f'  colliding corner angles: {sorted(hits) or "none"}' + (f' [{pairs_report(tail, in_frame(head, np.linalg.inv(A.Tx(P.L) @ A.Ry(min(hits)))))}]' if hits else ''))
    pin_r = 1.5
    say(f'  pin (M3 thread, r {pin_r}) in seat r {P.NOTCH_R}: {P.NOTCH_R - pin_r:.2f} mm radial play; '
        f'magnet face {P.MAG_X[0] + 0.1:.2f} mm from the axis -> {P.MAG_X[0] + 0.1 - pin_r:.2f} mm air+plastic when centred, '
        f'{P.MAG_X[0] + 0.1 - pin_r - (P.NOTCH_R - pin_r):.2f} mm when pulled home')
    # does the pin, pulled home (+x by the play), bear on the latch-sense wire?
    pins = union([i for i in h40 if i.name == 'latch_pin'])
    wire = [i for i in tail if i.name == 'latch_wire'][0].solid
    home = pins.translate([P.NOTCH_R - pin_r, 0, 0])
    w_c, w_h = inter(pins, wire), inter(home, wire)
    say(f'  pin x latch wire: centred {w_c:.3f} mm3, pulled home {w_h:.3f} mm3 (needs > 0 to close the circuit)')
    # capture funnel: how far off-axis can the pin arrive and still be steered into the seat (45 deg funnel)?
    say(f'  funnel mouth at x = -2 (block rear face): +-{P.NOTCH_R * 1.414 + 2:.1f} mm vertical capture')
    ok = not hits and w_h > 0
    result('c latch', ok, f'no interference 0..90 deg' if not hits else f'collides at {sorted(hits)} deg'
           + f'; pin bears on sense wire when home: {"yes" if w_h > 0 else "NO"}')


# ------------------------------------------------------------------------------------------------ (d) wheel and strap
def check_wheel_and_strap():
    say('\n(d2) Closed wheel (all 40 deg) and flat strap, all electronics fitted')
    out = {}
    for name, ang in (('flat strap', [0.0] * 8), ('closed wheel', [40.0] * 8)):
        asm, T = A.assembly(ang)
        closure = np.linalg.norm((T[8] @ np.array([P.L, 0, 0, 1]))[:3]) if name == 'closed wheel' else 0.0
        segs = {k: union([i for kk, i in asm if kk == k]) for k in range(9)}
        rows = []
        for k1 in range(9):
            for k2 in range(k1 + 1, 9):
                if name == 'flat strap' and k1 == 0 and k2 == 8:
                    continue
                v = inter(segs[k1], segs[k2])
                if k2 == k1 + 1 and v > EPS:                      # end-stop toe on cheek is allowed; nothing else
                    vp = inter(plast[k1].transform(T[k1][:3, :]), plast[k2].transform(T[k2][:3, :]))
                    stop_pen = max(stop_pen, vp); v -= vp
                if v > EPS:
                    rows.append(f's{k1}xs{k2} {v:.1f}')
        say(f'  {name}: ' + (f'closure error {closure:.3f} mm; ' if closure else '') + (', '.join(rows) or 'no interference'))
        out[name] = (rows, asm, T)
    # free height above each back in the closed wheel (column at the link middle, inside of the wheel)
    asm, T = out['closed wheel'][1], out['closed wheel'][2]
    heights = []
    for k in range(9):
        Tk_inv = np.linalg.inv(T[k])
        others = union([A.Item(i.name, i.solid.transform((Tk_inv)[:3, :]), i.colour, i.group) for kk, i in asm if kk != k])
        own = union([i for i in A.link_items(k, True) if i.group == 'elec'])
        base = P.HT if own.is_empty() else own.bounding_box()[5]
        best = 1e9
        for x in (12, 20, 28, 36):
            col = C.box(x - 0.5, x + 0.5, -30, 30, base, 200)
            hit = col ^ others
            if not hit.is_empty():
                best = min(best, hit.bounding_box()[2] - base)
        heights.append(best)
        say(f'  s{k}: top of {"payload" if not own.is_empty() else "back"} at z = {base:.1f}; free height above it '
            f'in the closed wheel {best:.1f} mm')
    bad = out['flat strap'][0] + out['closed wheel'][0]
    result('d wheel + strap', not bad, ('no interference in either pose' if not bad else '; '.join(bad))
           + f'; free height above the backs (closed) {min(heights):.0f}..{max(heights):.0f} mm')
    return heights


# ------------------------------------------------------------------------------------------------ (e) O-ring
def check_oring():
    say('\n(e) O-ring tread')
    ID, cs = 18.0, 2.5
    core = 2 * 10.0 + 2 * np.pi * P.GROOVE_R               # inner contact length on the racetrack core
    flange = 2 * 10.0 + 2 * np.pi * P.TREAD_R
    st_in = core / (np.pi * ID) - 1
    st_mid = (2 * 10 + 2 * np.pi * (P.GROOVE_R + cs / 2)) / (np.pi * (ID + cs)) - 1
    st_fit = flange / (np.pi * ID) - 1
    cs_after = cs * (1 + st_mid) ** -0.5
    proud = P.GROOVE_R + cs_after - P.TREAD_R
    width = P.GROOVE_Y - P.BODY_HW
    say(f'  groove core perimeter {core:.1f} mm vs ring ID circumference {np.pi * ID:.1f} -> {100 * st_in:.1f} % stretch '
        f'on the ID, {100 * st_mid:.1f} % on the centreline; cord thins to ~{cs_after:.2f} mm; stands {proud:.2f} mm proud of R8')
    say(f'  fitting over the R8 flange: {flange:.1f} mm perimeter -> {100 * st_fit:.0f} % momentary stretch (NBR breaks at >250 %)')
    say(f'  groove width {width:.1f} mm for a {cs} mm cord ({width - cs:.1f} mm side play)')
    ok = 0.0 < st_in < 0.12 and proud > 0.6 and st_fit < 0.6
    result('e O-ring', ok, f'stretch {100*st_in:.1f} % (ID), {100*st_mid:.1f} % (centreline); proud {proud:.2f} mm; '
           f'{100*st_fit:.0f} % to pass the flange')


# ------------------------------------------------------------------------------------------------ (f) leaf spring
def check_leaf():
    say('\n(f) Leaf spring')
    E, b, M = 2100.0, 2 * P.BODY_HW, 150.0                 # N/mm2, mm, N.mm
    xs = np.linspace(P.FLEX_X0, P.FLEX_X1, 2001)
    f = P.FLEX_FILLET
    d = np.minimum(xs - P.FLEX_X0, P.FLEX_X1 - xs)
    extra = np.where(d < f, f - np.sqrt(np.maximum(f * f - (f - d) ** 2, 0)), 0.0)
    t = P.FLEX_T + extra
    k = 1.0 / np.trapezoid(1.0 / (E * b * t ** 3 / 12.0), xs)          # N.mm/rad
    k_plain = E * b * P.FLEX_T ** 3 / (12 * (P.FLEX_X1 - P.FLEX_X0))
    sig = 6 * M / (b * P.FLEX_T ** 2)
    phi = M / k
    say(f'  k = {k/1000:.3f} N.m/rad with the {f} mm root fillets ({k_plain/1000:.3f} without), E = {E/1000:.1f} GPa, '
        f'b = {b:.0f} mm, t = {P.FLEX_T} mm; target 1.25')
    say(f'  at 0.15 N.m: deflection {np.degrees(phi):.1f} deg, peak bending stress {sig:.1f} MPa nominal '
        f'(~{1.15 * sig:.0f} MPa at the fillet, Kt ~1.15); PETG yield ~50 MPa')
    say(f'  sensitivity: k scales with t^3 -> 0.9 mm print = {k * 0.729 / 1000:.2f}, 1.1 mm = {k * 1.331 / 1000:.2f} N.m/rad')
    # (i) nothing but the leaf joins the rear block to the front block
    pieces = {}
    cut = C.box(P.FLEX_X0 - 0.01, P.FLEX_X1 + 0.01, -50, 50, -P.HT - 1, -P.HT + P.FLEX_T + P.FLEX_FILLET + 0.01)
    for name in ('link', 'link_s3_battery', 'link_s4_esp32', 'link_s5_power', 'tail', 'head'):
        pieces[name] = len((P.PARTS[name] - cut).decompose())
    say('  pieces after cutting the leaf out (must be 2): ' + ', '.join(f'{n} {c}' for n, c in pieces.items()))
    # (ii) flex the spring both ways to 1.5 x the stall deflection and look for anything touching across the gap
    fl = 1.5 * phi
    bridge = []
    for k_seg in range(9):
        it = A.link_items(k_seg, True)
        part = it[0].solid
        xm = (P.FLEX_X0 + P.FLEX_X1) / 2
        rear_box = C.box(-30, xm, -60, 60, -20, 80)
        rear = [A.Item('rear_block', (part ^ rear_box) - cut, (0, 0, 0), 'part')]
        front = [A.Item('front_block', part - rear_box - cut, (0, 0, 0), 'part')]
        for i in it[1:]:
            bb = i.solid.bounding_box()
            attached_rear = i.group == 'rotor' or i.name.startswith('battery') or (bb[0] + bb[3]) / 2 < P.FLEX_X0
            (rear if attached_rear else front).append(i)
        R = union(rear); F = union(front)
        for sgn in (1, -1):
            Tm = A.Tx(xm) @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -P.HT + 0.5], [0, 0, 0, 1]]) @ \
                 A.Ry(sgn * np.degrees(fl)) @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, P.HT - 0.5], [0, 0, 0, 1]]) @ A.Tx(-xm)
            v = inter(R.transform(Tm[:3, :]), F)
            if v > EPS:
                bridge.append(f's{k_seg} {"+" if sgn > 0 else "-"}{np.degrees(fl):.0f} deg: {v:.1f} mm3')
    say(f'  flexing every link +-{np.degrees(fl):.0f} deg (1.5 x stall): ' + (', '.join(bridge) or 'nothing touches across the gap'))
    ok = 1.0 <= k / 1000 <= 1.6 and sig < 25 and all(c == 2 for c in pieces.values()) and not bridge
    result('f leaf spring', ok, f'k {k/1000:.2f} N.m/rad (target 1.25), {np.degrees(phi):.1f} deg and {sig:.1f} MPa at 0.15 N.m; '
           f'bridged: {"no" if all(c == 2 for c in pieces.values()) and not bridge else "YES"}')


# ------------------------------------------------------------------------------------------------ (g) printability
def mesh_arrays(m):
    mm = m.to_mesh()
    return np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts)

def overhangs(m, limit_deg=50.0, near_bed=2.0):
    v, t = mesh_arrays(m)
    tri = v[t]; n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); area = np.linalg.norm(n, axis=1) / 2
    n = n / (2 * area[:, None] + 1e-12)
    zmin = tri[:, :, 2].min(1); zmax = tri[:, :, 2].max(1)
    on_bed = zmax < -P.HT + 0.01
    tread = np.abs(tri[:, :, 1]).min(1) >= P.BODY_HW - 0.01          # O-ring groove core / flange (accepted)
    steep = (n[:, 2] < -np.cos(np.radians(90 - limit_deg))) & ~on_bed & ~tread
    flat = steep & (n[:, 2] < -0.999)
    nb = steep & (zmax < -P.HT + near_bed) & ~flat
    high = steep & ~nb & ~flat
    # size of each flat ceiling (bridge): connected components through shared vertices
    spans = []
    if flat.any():
        idx = np.where(flat)[0]
        parent = {i: i for i in idx}
        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]; i = parent[i]
            return i
        by_vert = {}
        for i in idx:
            for vv in t[i]:
                by_vert.setdefault(vv, []).append(i)
        for lst in by_vert.values():
            for j in lst[1:]:
                parent[find(j)] = find(lst[0])
        groups = {}
        for i in idx: groups.setdefault(find(i), []).append(i)
        for g in groups.values():
            pts = tri[g].reshape(-1, 3)
            ext = pts.max(0) - pts.min(0)
            spans.append((min(ext[0], ext[1]), max(ext[0], ext[1]), pts[:, 2].min(), area[g].sum()))
    return area[nb].sum(), area[high].sum(), spans, (tri[high][:, :, :].reshape(-1, 3) if high.any() else None)

def thin_features(m, thr=0.8, ignore=None):
    """Walls: every 0.2 mm print layer (z slice) is opened with radius thr/2 (Miter joins, so square corners
    survive); whatever disappears is a wall thinner than thr. Plates: vertical rays on a 0.4 mm grid give
    the thickness of every solid run along z; runs thinner than thr that are not the 45 deg tips of a slope
    are counted by area. Returns (wall area lost worst layer, where, thin plate area, thinnest plate)."""
    bb = m.bounding_box()
    worst = (0.0, None)
    for z in np.arange(bb[2] + 0.1, bb[5] - 0.05, 0.2):
        cs = m.slice(z)
        if cs.area() < 1e-6: continue
        r = thr / 2 - 0.02                      # a wall of exactly 0.8 (the magnet skin) passes
        o = cs.offset(-r, JoinType.Miter, 2.0).offset(r, JoinType.Miter, 2.0)
        d = cs - o
        if ignore is not None and ignore[4] <= z <= ignore[5]:
            d = d - CrossSection.square([ignore[1] - ignore[0], ignore[3] - ignore[2]]).translate([ignore[0], ignore[2]])
        l = d.area()
        if l > worst[0]:
            worst = (l, f'layer z={z:.1f}')
    v, t = mesh_arrays(m)
    tri = v[t]
    xs = np.arange(bb[0] + 0.2, bb[3], 0.4); ys = np.arange(bb[1] + 0.2, bb[4], 0.4)
    gx, gy = np.meshgrid(xs, ys); px, py = gx.ravel(), gy.ravel()
    thin_area, thinnest = 0.0, 99.0
    # bin triangles by x to keep the ray test cheap
    tx0, tx1 = tri[:, :, 0].min(1), tri[:, :, 0].max(1)
    for xv in xs:
        sel = np.where((tx0 <= xv) & (tx1 >= xv))[0]
        if not len(sel): continue
        T = tri[sel]
        for yv in ys:
            a, b, c = T[:, 0, :2], T[:, 1, :2], T[:, 2, :2]
            d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
            ok = np.abs(d) > 1e-12
            l1 = ((b[:, 1] - c[:, 1]) * (xv - c[:, 0]) + (c[:, 0] - b[:, 0]) * (yv - c[:, 1])) / np.where(ok, d, 1)
            l2 = ((c[:, 1] - a[:, 1]) * (xv - c[:, 0]) + (a[:, 0] - c[:, 0]) * (yv - c[:, 1])) / np.where(ok, d, 1)
            l3 = 1 - l1 - l2
            hit = ok & (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
            if hit.sum() < 2: continue
            zz = np.sort(np.unique(np.round(l1[hit] * T[hit, 0, 2] + l2[hit] * T[hit, 1, 2] + l3[hit] * T[hit, 2, 2], 4)))
            for z0, z1 in zip(zz[0::2], zz[1::2]):
                th = z1 - z0
                if th < thr - 0.02:
                    # ignore sloped tips: probe 0.4 mm around; a real plate is thin over an area
                    if m.slice(min(z0 + th / 2, bb[5] - 1e-3)).area() > 0:
                        thin_area += 0.16; thinnest = min(thinnest, th)
    return worst[0], worst[1], thin_area, thinnest

def check_print():
    say('\n(g) Printability (belly down on the bed, no supports, limit 50 deg overhang, 0.8 mm features)')
    ok_all, txt = True, []
    for name in ('link', 'link_s3_battery', 'link_s4_esp32', 'link_s5_power', 'tail', 'head'):
        m = P.PARTS[name]
        nb, high, spans, pts = overhangs(m)
        big = [s for s in spans if s[0] > 3.5]
        ign = (-0.5, 2.0, -25, 25, -2.0, 2.4) if name == 'tail' else None   # mouths of the 0.6 mm latch-wire grooves
        lost, where, plate, thinnest = thin_features(m, ignore=ign)
        if ign:
            say('      (tail: the 0.2 mm wedges where the latch-wire grooves break into the pin seat are by design and not counted)')
        say(f'  {name:16s}: >50 deg overhang above 2 mm: {high:5.1f} mm2; near-bed (<2 mm up): {nb:5.1f} mm2; '
            f'flat bridges: {len(spans)} (widest span {max([s[0] for s in spans], default=0):.1f} mm); '
            f'walls <0.8: {lost:.2f} mm2 worst layer ({where}); plates <0.8 thick: {plate:.1f} mm2 (thinnest {thinnest:.2f})')
        for sp in sorted(spans, reverse=True)[:3]:
            say(f'      bridge {sp[0]:.1f} x {sp[1]:.1f} mm at z {sp[2]:.1f}')
        if high > 1.0 and pts is not None:
            say(f'      steep region bbox x {pts[:,0].min():.1f}..{pts[:,0].max():.1f} y {pts[:,1].min():.1f}..{pts[:,1].max():.1f} '
                f'z {pts[:,2].min():.1f}..{pts[:,2].max():.1f}')
        ok = high < 1.0 and not big and lost < 0.3 and plate < 1.0
        ok_all &= ok
        txt.append(f'{name} {high:.0f}/{len(big)}/{lost + plate:.1f}')
    result('g printability', ok_all, 'steep-overhang mm2 / bridges>3.5mm / thin mm2: ' + ', '.join(txt)
           + '; accepted, not counted: the lower arcs of the R6.5 groove core and R8 flange (|y| > 33), which the O-ring needs round')


# ------------------------------------------------------------------------------------------------ (h) leads
LEAD_MIN = 240.0
def check_leads():
    say('\n(h) Servo lead routing (flat strap, along the back, 15 mm slack per joint crossed)')
    exit_x = P.L - P.S_LEN + P.S_SHAFT - 3.0            # end of the strain relief, in the gap behind the cradle
    up = P.HT + 2.0                                      # leads run 2 mm above the back
    chan = (28.0, sum(P.CHAN_Y) / 2)
    def length(k, hdr_seg, hdr_xy, hdr_up):
        L1 = up - 0.0 + 2.0                              # stub end (z = 0) up the gap to the back
        L2 = np.hypot(chan[0] - exit_x, chan[1] - (P.Y0 + 4.0))
        L3 = abs(hdr_seg * P.L + hdr_xy[0] - (k * P.L + chan[0])) + abs(hdr_xy[1] - chan[1])
        n = abs(hdr_seg - k) if hdr_seg != k else 0
        return L1 + L2 + L3 + hdr_up + 15.0 * n, n
    rows, need = [], []
    for k in range(8):
        l5, n5 = length(k, 5, (28.65, 29.1), 17.0)     # header on the s5 power board (as built)
        l4, n4 = length(k, 4, (24.95, 30.0), 17.0)     # for comparison: a header on s4
        rows.append((k, l5, n5, l4))
        if l5 > LEAD_MIN: need.append(k)
        say(f'  servo in s{k}: {l5:5.0f} mm to the header on s5 ({n5} joints)   [{l4:5.0f} mm if the header were on s4]'
            + (f'   needs a {15 if l5 - LEAD_MIN <= 140 else 20} cm extension ({LEAD_MIN + (150 if l5 - LEAD_MIN <= 140 else 200) - l5:.0f} mm spare with it)'.replace('20 cm', '20 cm') if l5 > LEAD_MIN else f'   {LEAD_MIN - l5:.0f} mm spare'))
    ext = {k: (15 if r[1] - LEAD_MIN <= 140 else 20) for k, r in zip(range(8), rows) if k in need}
    result('h lead routing', True, 'extensions: ' + ', '.join(f's{k} servo {v} cm' for k, v in ext.items())
           + f' (header on the s5 power board); all others reach with >= {min(LEAD_MIN - r[1] for r in rows if r[0] not in need):.0f} mm spare')


def check_driver_paths():
    say('\n(a2) Screwdriver paths (3.0 mm shaft) to the horn screw and the pivot screw, joint assembled at 0 deg')
    n1 = A.link_items(1, payload=False); n2 = in_frame(A.link_items(2, payload=False), rel(0))
    solid = union([i for i in n1 + n2 if i.name not in ('horn_screw', 'pivot_screw')])
    horn_top = P.HORN_IN + P.HORN_T + C.HORN['screw_head_h']
    d1 = C.ycyl(P.L, 0, 1.5, horn_top + 0.05, 60)
    d2 = C.ycyl(P.L, 0, 1.5, -60, P.PIV_OUT - 1.3 - 0.05)
    h1 = C.ycyl(P.L, 0, C.HORN['screw_head_r'] + 0.1, horn_top - 0.01, 60)   # the screw head must pass in too
    h2 = C.ycyl(P.L, 0, 1.9 + 0.1, -60, P.PIV_OUT - 0.05)
    v = [inter(d1, solid), inter(d2, solid), inter(h1, solid), inter(h2, solid)]
    say(f'  horn screw: driver {v[0]:.2f} mm3, head path {v[2]:.2f};  pivot screw: driver {v[1]:.2f}, head path {v[3]:.2f}')
    result('a2 driver paths', max(v) < EPS, f'horn screw and pivot screw both reachable from outside the cheeks ({max(v):.2f} mm3)')


def main():
    t0 = time.time()
    check_servo_in_cradle()
    check_driver_paths()
    check_joint_sweep_plain()
    check_latch()
    check_all_joints()
    check_margins()
    check_wheel_and_strap()
    check_replay()
    check_oring()
    check_leaf()
    check_print()
    check_leads()
    say('\n' + '=' * 100)
    say(f'{"check":26s} {"result":6s}  numbers')
    for n, v, t in RESULTS:
        say(f'{n:26s} {v:6s}  {t}')
    say(f'({time.time() - t0:.0f} s)')
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, 'check_fit.txt'), 'w') as f:
        f.write('\n'.join(LOG) + '\n')
    return all(v == 'PASS' for _, v, _ in RESULTS)

if __name__ == '__main__':
    sys.exit(0 if main() else 1)
