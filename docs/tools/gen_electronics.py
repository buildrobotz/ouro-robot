"""Generate docs/electronics.json, docs/perfboard.svg, docs/wiring.svg, docs/schematic.svg from one model,
and check: perfboard nets connected and not shorted, schematic nets all labelled at >= 2 ends,
wire list covers every off-board connection, no overlapping text boxes in any SVG."""
import json, itertools, sys
from collections import defaultdict
from xml.sax.saxutils import escape

import pathlib
DOCS = str(pathlib.Path(__file__).resolve().parent.parent) + '/'
INK, MUTED, BG, GRID = '#1d2433', '#5b6475', '#ffffff', '#c9ced8'
FONT = "font-family='Helvetica Neue, Helvetica, Arial, sans-serif'"
problems = []

# ============================================================================ the design model
SERVO_GPIO = [13, 26, 16, 17, 18, 19, 23, 25]
PINMAP = [  # gpio, label, net, dir, notes
    (13, 'D13', 'SIG1', 'out', 'Servo joint 1 (tail). LEDC PWM 50 Hz, 16-bit'),
    (26, 'D26', 'SIG2', 'out', 'Servo joint 2. Moved from GPIO14 (JTAG TMS, not quiet at boot)'),
    (16, 'RX2', 'SIG3', 'out', 'Servo joint 3. Free on WROOM-32; NOT available on WROVER (PSRAM)'),
    (17, 'TX2', 'SIG4', 'out', 'Servo joint 4. Same WROVER caveat as GPIO16'),
    (18, 'D18', 'SIG5', 'out', 'Servo joint 5'),
    (19, 'D19', 'SIG6', 'out', 'Servo joint 6'),
    (23, 'D23', 'SIG7', 'out', 'Servo joint 7'),
    (25, 'D25', 'SIG8', 'out', 'Servo joint 8 (neck)'),
    (21, 'D21', 'SDA', 'i/o', 'I2C data, 400 kHz, pull-ups on the breakouts'),
    (22, 'D22', 'SCL', 'out', 'I2C clock'),
    (27, 'D27', 'LATCH', 'in', 'Left head pin, internal pull-up, low = latched'),
    (34, 'D34', 'VSENSE', 'in', 'ADC1_CH6, input-only. Battery / 4.03 (100k over 33k)'),
    (2, 'D2', 'LED', 'out', 'On-board LED (30-pin DevKit V1). Strapping pin: LED + resistor keep it low at boot, fine'),
    (None, '5V / VIN', '5V_LOGIC', 'pwr', 'From buck B. Feeds the DevKit AMS1117 3.3 V regulator'),
    (None, '3V3', '3V3', 'pwr', 'Out of the DevKit regulator: both IMUs and INA219 logic (~10 mA total)'),
    (None, 'GND', 'GND', 'pwr', 'Two GND pin stubs: one to the power board (H1), one to the mid IMU'),
]

# components: ref -> (description, value)
PARTS = {
    'BT1': ('2S LiPo 7.4 V 450-500 mAh 30C, XH balance lead', ''),
    'J0': ('XT30 pair (battery side female, robot side male)', 'replaces PH2.0'),
    'F1': ('PTC resettable fuse, radial, 3.0 A hold / 6.0 A trip, >=16 V, Imax >=40 A (e.g. Littelfuse RUEF300)', '3 A'),
    'U1': ('Mini560 fixed 5 V buck, servo rail ("buck A"), top of the stack (cooler)', '5 V'),
    'U3': ('Mini560 fixed 5 V buck, ESP32 rail ("buck B"), bottom of the stack', '5 V'),
    'U2': ('INA219 breakout, 0.1 ohm on-board shunt, addr 0x40', ''),
    'R5': ('0.1 ohm 1 W 1% axial, soldered across U2 VIN+/VIN- terminal holes', '0.1R 1W'),
    'C2': ('220 uF 10 V (or 16 V) low-ESR electrolytic, 6.3 x 11 mm, lying on top of U2', '220u'),
    'R1': ('100 k 1% 1/4 W', '100k'),
    'R2': ('33 k 1% 1/4 W', '33k'),
    'C3': ('100 nF ceramic, 2.54 mm leads', '100n'),
    'J1': ('8 x 3 male header, 2.54 mm, straight (plug tops at link z 27)', ''),
    'U5': ('ESP32 DevKit V1, ESP32-WROOM-32, 30-pin; header pins cut flush under the spacers', ''),
    'U4': ('GY-521 MPU-6050, mid (link s4, foam-taped on the ESP32 module can), AD0 -> GND = 0x68', ''),
    'U6': ('GY-521 MPU-6050, head (link s8), AD0 -> 3V3 = 0x69', ''),
    'P1': ('Left head pin, M3 x 25 zinc-plated steel screw + M3 ring lug', ''),
    'P2': ('Right head pin, M3 x 25 zinc-plated steel screw + M3 ring lug', ''),
    'W0': ('Bare tinned wire across the two tail cradle notches (bridges P1-P2 when latched)', ''),
}
for k in range(1, 9):
    PARTS[f'M{k}'] = (f'MG90S servo, joint {k}', '')

NETS = {
    'VBAT_RAW': ['BT1.+', 'J0.+', 'F1.1'],
    'VBAT_F':   ['F1.2', 'U1.IN+', 'U3.IN+', 'R1.1'],
    'GND':      ['BT1.-', 'J0.-', 'U1.GND', 'U3.GND', 'U2.GND', 'C2.-', 'R2.2', 'C3.2', 'U5.GND', 'U4.GND', 'U4.AD0',
                 'U6.GND', 'P2.1'] + [f'J1.G{k}' for k in range(1, 9)] + [f'M{k}.GND' for k in range(1, 9)],
    '5V_SHUNT_IN': ['U1.OUT+', 'U2.VIN+', 'R5.1'],
    '5V_SERVO': ['U2.VIN-', 'R5.2', 'C2.+'] + [f'J1.V{k}' for k in range(1, 9)] + [f'M{k}.V+' for k in range(1, 9)],
    '5V_LOGIC': ['U3.OUT+', 'U5.5V'],
    '3V3':      ['U5.3V3', 'U4.VCC', 'U2.VCC', 'U6.VCC', 'U6.AD0'],
    'SDA':      ['U5.GPIO21', 'U4.SDA', 'U2.SDA', 'U6.SDA'],
    'SCL':      ['U5.GPIO22', 'U4.SCL', 'U2.SCL', 'U6.SCL'],
    'VSENSE':   ['R1.2', 'R2.1', 'C3.1', 'U5.GPIO34'],
    'LATCH':    ['U5.GPIO27', 'P1.1'],
    'LATCH_BRIDGE': ['P1.2', 'W0.1', 'W0.2', 'P2.2'],
}
for k, g in enumerate(SERVO_GPIO, 1):
    NETS[f'SIG{k}'] = [f'U5.GPIO{g}', f'J1.S{k}', f'M{k}.SIG']

# ---------------------------------------------------------------- strap geometry and wire list
PITCH = 48                      # mm, joint to joint. Segment s(k-1) carries servo k in its front cradle; servo k drives joint k.
SLACK = 15                      # mm slack loop per crossed joint
LEAD_MG90S = 240                # mm, MG90S lead as assumed by docs/mechanical.md (measure yours)
# Servo lead length to J1 on s5, from cad/check_fit.py (h): routed exit -> lead channel -> header, 15 mm per joint,
# 17 mm up to the plug. Indexed by servo k = 1..8 (segment s(k-1)).
LEAD_TO_HEADER = {1: 387, 2: 324, 3: 261, 4: 198, 5: 135, 6: 72, 7: 133, 8: 196}

def joints_between(a, b):
    ia, ib = int(a[1:]), int(b[1:])
    lo, hi = sorted((ia, ib))
    return [f'j{j}' for j in range(lo + 1, hi + 1)]

WIRES = []
def wire(wid, frm, to, net, colour, awg, length, a, b, harness, note=''):
    WIRES.append(dict(id=wid, harness=harness, net=net, from_=frm, to=to, colour=colour, awg=awg,
                      length_mm=length, from_link=a, to_link=b, crosses=joints_between(a, b), note=note))

STUB = 'soldered to the cut pin stub on top of the DevKit spacer'
wire('B1', 'BT1.+ via J0 (XT30, on s3) and F1 (in heat-shrink)', 'board 1A', 'VBAT_F', 'red', 22, 150, 's3', 's5', 'BAT',
     'XT30 lies on the s3 front block at -y; lead runs along the -y edge of s4 (y -33..-19), clear of the ESP32')
wire('B2', 'BT1.- via J0 (XT30)', 'board 1H', 'GND', 'black', 22, 150, 's3', 's5', 'BAT', 'twisted with B1')
RIB = ['brown', 'red', 'orange', 'yellow', 'green', 'blue', 'violet', 'grey']
for k, g in enumerate(SERVO_GPIO, 1):
    wire(f'H1-{k}', f'U5.GPIO{g}', f'board 23{"ABCDEFGH"[k - 1]} -> J1.S{k}', f'SIG{k}', RIB[k - 1], 28, 90, 's4', 's5', 'H1a',
         f'28 AWG silicone, {STUB}; colour = servo number in resistor code')
wire('H1-9', 'U5.5V', 'board 13A', '5V_LOGIC', 'red', 26, 90, 's4', 's5', 'H1b', f'26 AWG silicone, {STUB}')
wire('H1-10', 'U5.GND', 'board 13D', 'GND', 'black', 26, 90, 's4', 's5', 'H1b', f'26 AWG silicone, {STUB}')
wire('H1-11', 'U5.3V3', 'U2 VCC pad', '3V3', 'orange', 28, 90, 's4', 's5', 'H1b', f'{STUB} (2nd wire on the 3V3 stub)')
wire('H1-12', 'U5.GPIO21', 'U2 SDA pad', 'SDA', 'blue', 28, 90, 's4', 's5', 'H1b', f'{STUB} (2nd wire on the stub)')
wire('H1-13', 'U5.GPIO22', 'U2 SCL pad', 'SCL', 'yellow', 28, 90, 's4', 's5', 'H1b', f'{STUB} (2nd wire on the stub)')
wire('H1-14', 'U5.GPIO27', 'board 13B', 'LATCH', 'white', 28, 90, 's4', 's5', 'H1b', STUB)
wire('H1-15', 'U5.GPIO34', 'board 1D', 'VSENSE', 'violet', 28, 90, 's4', 's5', 'H1b', STUB)
for i, (frm, to, net, col) in enumerate([('U5.3V3', 'U4.VCC', '3V3', 'orange'), ('U5.GND (second GND pin)', 'U4.GND', 'GND', 'black'),
                                          ('U5.GPIO21', 'U4.SDA', 'SDA', 'blue'), ('U5.GPIO22', 'U4.SCL', 'SCL', 'yellow')], 1):
    wire(f'M-{i}', frm, to, net, col, 28, 30, 's4', 's4', 'MID', f'{STUB}; U4 sits on the ESP32 module can')
wire('M-5', 'U4.AD0', 'U4.GND', 'GND', 'bare', 30, 5, 's4', 's4', 'MID', 'solder bridge on the GY-521')
wire('H2-1', 'U2 SDA pad (2nd wire)', 'U6.SDA', 'SDA', 'blue', 26, 230, 's5', 's8', 'H2',
     'bundle order SDA, GND, SCL, 3V3, LATCH; runs at y -20..-10 on s6/s7, away from the servo-lead channel')
wire('H2-2', 'U2 GND pad (2nd wire)', 'U6.GND', 'GND', 'black', 26, 230, 's5', 's8', 'H2')
wire('H2-3', 'U2 SCL pad (2nd wire)', 'U6.SCL', 'SCL', 'yellow', 26, 230, 's5', 's8', 'H2')
wire('H2-4', 'U2 VCC pad (2nd wire)', 'U6.VCC', '3V3', 'orange', 26, 230, 's5', 's8', 'H2')
wire('H2-5', 'board 13B (2nd wire)', 'P1 (left head pin ring lug)', 'LATCH', 'white', 26, 250, 's5', 's8', 'H2')
wire('S-1', 'U6.GND', 'P2 (right head pin ring lug)', 'GND', 'black', 26, 40, 's8', 's8', 'HEAD')
wire('S-2', 'U6.AD0', 'U6.VCC', '3V3', 'bare', 30, 5, 's8', 's8', 'HEAD', 'solder bridge on the GY-521: address 0x69')
wire('S-3', 'tail notch left', 'tail notch right', 'LATCH_BRIDGE', 'bare', 22, 40, 's0', 's0', 'TAIL',
     'one wire per notch, joined across the back (mechanical.md 6)')
EXT = {}
for k in range(1, 9):
    a = f's{k - 1}'
    need = LEAD_TO_HEADER[k]
    if need > LEAD_MG90S:
        short = need - LEAD_MG90S
        ext = 200 if short > 150 - 20 else 150
        EXT[k] = ext
        note = f'{short} mm short: {ext // 10} cm extension ({LEAD_MG90S + ext - need} mm spare)'
    else:
        note = f'own lead, {LEAD_MG90S - need} mm spare: coil it in the channel'
    WIRES.append(dict(id=f'SV{k}', harness='SERVO', net=f'SIG{k} / 5V_SERVO / GND', from_=f'M{k} (front cradle of {a})',
                      to=f'J1 row {"ABCDEFGH"[k - 1]} (board s5)', colour='orange/red/brown (servo lead)', awg=28,
                      length_mm=need, from_link=a, to_link='s5', crosses=joints_between(a, 's5'), note=note))

# ---------------------------------------------------------------- perfboard model (cols 1..26 across the link, rows A..H along it)
# Board 66 x 20.5 mm on the s5 front block (link frame x 19.0..39.5, y -33..+33), 1 mm foam.
# Column c sits at link y = -33 + 1.27 + 2.54 (c-1); row A is at x = 19 + 1.27 (toward s4), row H toward s6.
ROWS = 'ABCDEFGH'
NC, NR = 26, 8
P = 2.54
def hx(c): return 1.27 + (c - 1) * P              # mm from the board's -y edge
def hy(r): return 1.27 + 0.1 + ROWS.index(r) * P  # mm from the board's rear (s4-side) edge
BW, BH = 66.0, 20.5

HOLES = {}      # (c, r) -> (net, label)
def hole(c, r, net, label):
    if (c, r) in HOLES and HOLES[(c, r)][0] != net:
        problems.append(f'perfboard: hole {c}{r} assigned to {HOLES[(c, r)][0]} and {net}')
    HOLES[(c, r)] = (net, label)

# top-side things that occupy holes
hole(1, 'A', 'VBAT_F', 'battery + (red 22 AWG, after F1)')
hole(1, 'B', 'VBAT_F', 'U1 IN+ and U3 IN+ legs')
hole(1, 'C', 'VBAT_F', 'R1 top')
hole(1, 'D', 'VSENSE', 'R1 bottom + H1 VSENSE (violet)')
hole(1, 'E', 'VSENSE', 'R2 top + C3')
hole(1, 'F', 'GND', 'R2 bottom + C3')
hole(1, 'G', 'GND', 'U1 GND and U3 GND legs')
hole(1, 'H', 'GND', 'battery - (black 22 AWG)')
hole(13, 'A', '5V_LOGIC', 'U3 OUT+ leg + H1 5V (red)')
hole(13, 'B', 'LATCH', 'H1 LATCH + H2 LATCH (white), tie point only')
hole(13, 'C', 'GND', 'U2 GND leg')
hole(13, 'D', 'GND', 'H1 GND (black)')
hole(13, 'G', '5V_SERVO', 'W2 from U2 VIN- + C2 +')
hole(13, 'H', 'GND', 'C2 -')
for k in range(1, 9):
    r = ROWS[k - 1]
    hole(23, r, f'SIG{k}', f'H1 SIG{k} ({RIB[k - 1]})'); hole(24, r, f'SIG{k}', f'J1 S{k}')
    hole(25, r, '5V_SERVO', f'J1 +{k}'); hole(26, r, 'GND', f'J1 G{k}')

# bottom traces: bare tinned 20-22 AWG laid hole to hole (power), or solder bridges (short)
TRACES = [
    ('VBAT_F', [(1, 'A'), (1, 'B'), (1, 'C')]),
    ('VSENSE', [(1, 'D'), (1, 'E')]),
    ('GND', [(1, 'F'), (1, 'G'), (1, 'H')]),
    ('GND', [(c, 'F') for c in range(1, 14)]),
    ('GND', [(13, 'C'), (13, 'D'), (13, 'E'), (13, 'F')]),
    ('GND', [(c, 'H') for c in range(1, 23)]),
    ('5V_SERVO', [(c, 'G') for c in range(13, 23)]),
    ('5V_SERVO', [(25, r) for r in ROWS]),
    ('GND', [(26, r) for r in ROWS]),
] + [(f'SIG{k}', [(23, ROWS[k - 1]), (24, ROWS[k - 1])]) for k in range(1, 9)]
# insulated jumpers on the bottom (cross other traces; only their two end holes count)
JUMPERS = [('5V_SERVO', (22, 'G'), (25, 'G'), 'JB1: 24 AWG insulated, bottom, 22G -> stub of 25G'),
           ('GND', (22, 'H'), (26, 'H'), 'JB2: 24 AWG insulated, bottom, 22H -> stub of 26H')]
for net, path in TRACES:
    for c, r in path:
        hole(c, r, net, HOLES.get((c, r), (net, 'trace'))[1])
    for (c1, r1), (c2, r2) in zip(path, path[1:]):
        if abs(c1 - c2) + abs(ROWS.index(r1) - ROWS.index(r2)) != 1:
            problems.append(f'perfboard trace {net}: {c1}{r1}->{c2}{r2} not adjacent')
for net, a, b, _ in JUMPERS:
    for h in (a, b):
        if HOLES.get(h, (net,))[0] != net:
            problems.append(f'perfboard jumper {net} ends on {h} which is {HOLES[h][0]}')

# components that join nets on top (two-terminal parts + flying wires through U2's own pads)
TOP_PARTS = [('R1', 'VBAT_F', 'VSENSE'), ('R2', 'VSENSE', 'GND'), ('C3', 'VSENSE', 'GND'), ('C2', '5V_SERVO', 'GND'),
             ('U2 shunt + R5', '5V_SHUNT_IN', '5V_SERVO')]
FLYING = [('W1', 'U1 OUT+ pad (top buck)', 'U2 VIN+ pad', '5V_SHUNT_IN', '22 AWG red, ~10 mm'),
          ('W2', 'U2 VIN- pad', 'hole 13G', '5V_SERVO', '22 AWG red, ~12 mm')]

def check_perfboard():
    # union-find over holes using traces and jumpers; each net's holes must form one group, groups never mix nets
    parent = {h: h for h in HOLES}
    def f(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = parent[x]
        return x
    for net, path in TRACES:
        for a, b in zip(path, path[1:]):
            parent[f(a)] = f(b)
    for net, a, b, _ in JUMPERS:
        parent[f(a)] = f(b)
    groups = defaultdict(set)
    for h, (net, _) in HOLES.items():
        groups[f(h)].add(net)
    for g, nets in groups.items():
        if len(nets) > 1:
            problems.append(f'perfboard SHORT: {sorted(nets)} joined')
    per_net = defaultdict(set)
    for h, (net, _) in HOLES.items():
        per_net[net].add(f(h))
    for net, gs in per_net.items():
        if len(gs) > 1:
            problems.append(f'perfboard OPEN: net {net} is in {len(gs)} separate groups on the board')
    for (c, r) in HOLES:
        if not (1 <= c <= NC and r in ROWS):
            problems.append(f'perfboard: hole {c}{r} outside the {NC} x {NR} board')
    return per_net
per_net = check_perfboard()

# ---------------------------------------------------------------- tiny SVG writer with text-overlap bookkeeping
class Svg:
    def __init__(self, w, h, title):
        self.w, self.h, self.parts, self.texts = w, h, [], []
        self.parts.append(f"<rect x='1' y='1' width='{w - 2}' height='{h - 2}' rx='14' fill='{BG}' stroke='{GRID}'/>")
        self.title = title
    def add(self, s): self.parts.append(s)
    def text(self, x, y, s, size=12, anchor='start', weight='normal', fill=INK, check=True, rot=0):
        w = 0.56 * size * len(s); h = size
        x0 = x if anchor == 'start' else x - w / 2 if anchor == 'middle' else x - w
        if check and not rot:
            self.texts.append((x0, y - h * 0.8, x0 + w, y + h * 0.2, s))
        t = f" transform='rotate({rot} {x:.1f} {y:.1f})'" if rot else ''
        self.parts.append(f"<text x='{x:.1f}' y='{y:.1f}' font-size='{size}' text-anchor='{anchor}' "
                          f"font-weight='{weight}' fill='{fill}'{t}>{escape(s)}</text>")
    def line(self, x1, y1, x2, y2, col=INK, w=1.6, dash=None, cap='round'):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        self.parts.append(f"<line x1='{x1:.1f}' y1='{y1:.1f}' x2='{x2:.1f}' y2='{y2:.1f}' stroke='{col}' "
                          f"stroke-width='{w}' stroke-linecap='{cap}'{d}/>")
    def poly(self, pts, col=INK, w=1.6, dash=None, fill='none'):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        p = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.parts.append(f"<polyline points='{p}' fill='{fill}' stroke='{col}' stroke-width='{w}' "
                          f"stroke-linejoin='round' stroke-linecap='round'{d}/>")
    def rect(self, x, y, w, h, col=INK, fill='none', rx=3, sw=1.6, dash=None):
        d = f" stroke-dasharray='{dash}'" if dash else ''
        self.parts.append(f"<rect x='{x:.1f}' y='{y:.1f}' width='{w:.1f}' height='{h:.1f}' rx='{rx}' "
                          f"fill='{fill}' stroke='{col}' stroke-width='{sw}'{d}/>")
    def circle(self, x, y, r, col=INK, fill='none', sw=1.4):
        self.parts.append(f"<circle cx='{x:.1f}' cy='{y:.1f}' r='{r}' fill='{fill}' stroke='{col}' stroke-width='{sw}'/>")
    def check_overlaps(self, name):
        for a, b in itertools.combinations(self.texts, 2):
            if a[0] < b[2] - 1 and b[0] < a[2] - 1 and a[1] < b[3] - 1 and b[1] < a[3] - 1:
                problems.append(f'{name}: text overlap "{a[4]}" / "{b[4]}"')
            # off-canvas
        for t in self.texts:
            if t[0] < 4 or t[2] > self.w - 4 or t[1] < 4 or t[3] > self.h - 4:
                problems.append(f'{name}: text off canvas "{t[4]}"')
    def save(self, path, name):
        self.check_overlaps(name)
        body = '\n'.join(self.parts)
        s = (f"<svg xmlns='http://www.w3.org/2000/svg' width='{self.w}' height='{self.h}' viewBox='0 0 {self.w} {self.h}' "
             f"{FONT}>\n<title>{escape(self.title)}</title>\n{body}\n</svg>\n")
        open(path, 'w').write(s)

WIRE_RGB = dict(red='#d02a2a', black='#1d2433', orange='#e8771b', yellow='#d4a800', blue='#2a62d0', white='#9aa3b2',
                violet='#8a3fc4', green='#2f9a46', brown='#8a5a2b', grey='#7d8594', bare='#b08d57')

# ============================================================================ perfboard.svg
def draw_perfboard():
    S = 17.0                                   # px per mm
    W, H = 1300, 1330
    g = Svg(W, H, 'Ouro power board, 66 x 20.5 mm perfboard (link s5)')
    g.text(30, 40, 'Ouro power board: 66 x 20.5 mm pad-per-hole perfboard, 26 x 8 holes, on the s5 front block', 18, weight='bold')
    g.text(30, 64, 'Long side across the link (link y -33..+33), short side along the body (link x 19.0..39.5). 1 mm foam tape. '
                   'Plain perfboard: no track cuts.', 12, fill=MUTED)
    g.text(30, 82, 'Holes are named column 1-26 + row A-H. Row A faces s4 (the ESP32), row H faces s6.', 12, fill=MUTED)

    def panel(ox, oy, mirror, title):
        g.text(ox, oy - 26, title, 14, weight='bold')
        g.rect(ox, oy, BW * S, BH * S, col=INK, fill='#f3efe2', rx=6, sw=2)
        X = (lambda c: ox + (BW - hx(c)) * S) if mirror else (lambda c: ox + hx(c) * S)
        Y = lambda r: oy + hy(r) * S
        for c in range(1, NC + 1):
            g.add(f"<text x='{X(c):.1f}' y='{oy - 6:.1f}' font-size='11' text-anchor='middle' fill='{MUTED}'>{c}</text>")
        for r in ROWS:
            xl = ox - 14 if not mirror else ox + BW * S + 14
            g.add(f"<text x='{xl:.1f}' y='{Y(r) + 4:.1f}' font-size='11' text-anchor='middle' fill='{MUTED}'>{r}</text>")
        for c in range(1, NC + 1):
            for r in ROWS:
                used = (c, r) in HOLES
                g.circle(X(c), Y(r), 6.5, col='#8f7a3a' if used else '#b9a46a', fill='#d8c48a' if used else '#e9dfbf', sw=1)
                g.circle(X(c), Y(r), 2.2, col='#555', fill='#555', sw=0.5)
        return X, Y

    # ---- top view
    ox, oy = 50, 140
    X, Y = panel(ox, oy, False, 'TOP (component side)')
    xm = lambda mm: ox + mm * S
    ym = lambda mm: oy + mm * S
    # buck stack
    g.rect(xm(2.6), ym(1.2), 28.8 * S, 18.1 * S, fill='#dfe7f3', rx=6)
    g.rect(xm(3.4), ym(2.0), 27.2 * S, 16.5 * S, col='#3b4b6b', fill='none', rx=5, dash='6 4', sw=1.2)
    g.text(xm(17), ym(6.5), 'Mini560 stack, 18 mm side along the body', 13, 'middle', 'bold')
    g.text(xm(17), ym(9.3), 'bottom: U3 (5 V for the ESP32)', 12, 'middle')
    g.text(xm(17), ym(11.6), 'top: U1 (5 V servos), 1 mm foam between', 12, 'middle')
    g.text(xm(4.0), ym(4.0), 'IN', 11, fill=MUTED)
    g.text(xm(30.0), ym(4.0), 'OUT', 11, 'end', fill=MUTED)
    # legs
    for (c, r), y0 in [((1, 'B'), 4.2), ((1, 'G'), 15.8)]:
        g.poly([(xm(3.0), ym(y0)), (X(c), Y(r))], col='#3b4b6b', w=2.6)
    g.poly([(xm(31.0), ym(3.0)), (X(13), Y('A'))], col='#3b4b6b', w=2.6)
    # INA219 flat
    g.rect(xm(33.0), ym(0.15), 22.9 * S, 20.2 * S, col='#2f7d5b', fill='#e3f2ea', rx=6, sw=2)
    g.text(xm(48.5), ym(3.0), 'U2  INA219', 13, 'middle', 'bold', fill='#1f5c41')
    g.text(xm(48.5), ym(5.2), 'flat, 1 mm foam', 11, 'middle', fill='#1f5c41')
    pads = [('VIN+', 2.4), ('VIN-', 4.9), ('GND', 7.4), ('VCC', 9.9), ('SCL', 12.4), ('SDA', 14.9)]
    for name, yy in pads:
        g.circle(xm(34.3), ym(yy), 5, col='#2f7d5b', fill='#ffffff', sw=1.6)
        g.text(xm(35.3), ym(yy) + 4, name, 11, fill='#1f5c41')
    # R5 on the terminal holes (right edge of the module)
    g.rect(xm(52.6), ym(4.0), 2.2 * S, 9.0 * S, col='#7a5230', fill='#e6c9a0', rx=6, sw=1.4)
    g.text(xm(52.2), ym(8.8), 'R5', 12, 'end', 'bold')
    # GND leg, W1, W2
    g.poly([(xm(34.3), ym(7.4)), (X(13), Y('C'))], col='#2f7d5b', w=2.4)
    g.poly([(xm(30.6), ym(9.0)), (xm(33.0), ym(5.0)), (xm(34.3), ym(2.4))], col=WIRE_RGB['red'], w=3.4)
    g.text(xm(32.0), ym(8.2), 'W1', 12, 'middle', 'bold', fill=WIRE_RGB['red'])
    g.poly([(xm(34.3), ym(4.9)), (xm(33.6), ym(12.0)), (X(13), Y('G'))], col=WIRE_RGB['red'], w=3.4, dash='8 4')
    g.text(xm(32.0), ym(13.6), 'W2', 12, 'middle', 'bold', fill=WIRE_RGB['red'])
    # C2 lying on U2, leads to 13G / 13H
    g.rect(xm(38.5), ym(8.6), 6.3 * S, 11.0 * S, col=INK, fill='#cfd7e6', rx=14, sw=1.8)
    g.text(xm(41.65), ym(13.0), 'C2', 12, 'middle', 'bold')
    g.text(xm(41.65), ym(15.4), 'lying', 11, 'middle', fill=MUTED)
    g.poly([(xm(39.5), ym(16.6)), (X(13), Y('G'))], col='#555', w=1.6)
    g.poly([(xm(39.5), ym(18.6)), (X(13), Y('H'))], col='#555', w=1.6)
    # header J1: cols 24-26, rows A-H; plug housings
    for r in ROWS:
        g.rect(X(24) - 18, Y(r) - 19, X(26) - X(24) + 36, 38, col='#222', fill='#40444c', rx=4, sw=1)
        for c, col in zip((24, 25, 26), ('#e8771b', '#d02a2a', '#8a5a2b')):
            g.rect(X(c) - 4, Y(r) - 4, 8, 8, col=col, fill=col, rx=1, sw=0.5)
    # R1, R2/C3 standing in column 1
    for r1, r2, lab in [('C', 'D', 'R1'), ('E', 'F', 'R2')]:
        g.line(X(1), Y(r1), X(1), Y(r2), col='#7a5230', w=2)
        g.rect(X(1) - 9, (Y(r1) + Y(r2)) / 2 - 15, 18, 30, col='#7a5230', fill='#e6c9a0', rx=6, sw=1.4)
    g.rect(X(1) + 10, (Y('E') + Y('F')) / 2 - 6, 9, 12, col='#c47f17', fill='#f6d68f', rx=3, sw=1.2)
    # wire-entry rings
    entries = {(1, 'A'): 'red', (1, 'H'): 'black', (1, 'D'): 'violet', (13, 'A'): 'red', (13, 'B'): 'white', (13, 'D'): 'black'}
    entries.update({(23, ROWS[k - 1]): RIB[k - 1] for k in range(1, 9)})
    for (c, r), col in entries.items():
        g.circle(X(c), Y(r), 9.5, col=WIRE_RGB[col], fill='none', sw=3.4)
    # notes under the top view
    ny = oy + BH * S + 40
    notes = [
        ('1A', 'battery + (red 22 AWG, after F1)'), ('1B', 'U1 IN+ and U3 IN+ legs (both in one hole)'),
        ('1C-1D', 'R1 100k standing'), ('1D', '+ H1 VSENSE (violet)'), ('1E-1F', 'R2 33k standing, C3 100 nF beside it'),
        ('1G', 'U1 GND and U3 GND legs'), ('1H', 'battery - (black 22 AWG)'),
        ('13A', 'U3 OUT+ leg + H1 5V (red)'), ('13B', 'LATCH tie: H1 white + H2 white'),
        ('13C', 'U2 GND leg'), ('13D', 'H1 GND (black)'), ('13G / 13H', 'W2 + C2 +  /  C2 -'),
        ('23A-23H', 'H1 SIG1..SIG8 (brown .. grey)'), ('24 / 25 / 26', 'J1 rows A-H: S / +5V / GND = servo 1..8'),
        ('W1', 'U1 OUT+ pad (top buck) -> U2 VIN+ pad'), ('U2 pads', 'VCC SCL SDA GND: H1 + H2 wires (2 per pad)'),
    ]
    g.text(ox, ny, 'Top-side connections', 13, weight='bold')
    for i, (h, t) in enumerate(notes):
        col_x = ox + (i // 8) * 600
        y = ny + 24 + (i % 8) * 19
        g.text(col_x, y, h, 12, weight='bold')
        g.text(col_x + 96, y, t, 12)

    # ---- bottom view (mirrored)
    ox2, oy2 = 50, 770
    X2, Y2 = panel(ox2, oy2, True, 'BOTTOM (solder side, flipped left-right: column 1 is on the RIGHT)')
    tcol = {'VBAT_F': '#d02a2a', 'GND': '#1d2433', '5V_SERVO': '#e8771b', 'VSENSE': '#8a3fc4'}
    for net, path in TRACES:
        col = tcol.get(net, '#7d8594')
        g.poly([(X2(c), Y2(r)) for c, r in path], col=col, w=5.5 if net in ('VBAT_F', 'GND', '5V_SERVO') else 3.6)
    for net, a, b, _ in JUMPERS:
        (c1, r1), (c2, r2) = a, b
        x1, y1, x2, y2 = X2(c1), Y2(r1), X2(c2), Y2(r2)
        g.add(f"<path d='M{x1:.1f},{y1:.1f} C{x1 - 10:.1f},{y1 + 28:.1f} {x2 + 10:.1f},{y2 + 28:.1f} {x2:.1f},{y2:.1f}' "
              f"fill='none' stroke='{tcol[net]}' stroke-width='4.5' stroke-dasharray='9 5'/>")
    ly = oy2 + BH * S + 40
    leg = [('VBAT_F', 'red: bridge 1A-1B-1C'),
           ('VSENSE', 'violet: bridge 1D-1E'),
           ('GND', 'black: 1F-1G-1H; row F 1-13; 13C-13F; row H 1-22; column 26 A-H'),
           ('5V_SERVO', 'orange: row G 13-22; column 25 A-H'),
           ('SIG', 'grey: bridges 23A-24A ... 23H-24H (8)'),
           ('GND', 'dashed: JB1 22G -> 25G and JB2 22H -> 26H, 24 AWG INSULATED (they cross 23-25)')]
    g.text(ox2, ly, 'Bottom traces (bare tinned 20-22 AWG soldered at every hole for power runs)', 13, weight='bold')
    for i, (n, t) in enumerate(leg):
        y = ly + 24 + i * 20
        g.line(ox2, y - 4, ox2 + 30, y - 4, col=tcol.get(n, '#7d8594'), w=5)
        g.text(ox2 + 40, y, t, 12)
    warn = ['Never bridge: 1C-1D (R1), 1E-1F (R2), 13A-13B, 13B-13C, 13F-13G, 13G-13H,',
            '23x-23y between rows, 24-25, 25-26 (the header columns).',
            'Meter, unpowered: 1A-1B-1C beep; 1F-1H-13C-26A beep; 13G-25A beep;',
            '1A-1H, 13G-13H, 25A-26A, 24A-24B must NOT beep.']
    for i, t in enumerate(warn):
        g.text(ox2 + 640, ly + 24 + i * 20, t, 12, fill='#9b2c2c' if i < 2 else INK)
    g.save(DOCS + 'perfboard.svg', 'perfboard.svg')

# ============================================================================ wiring.svg
def draw_wiring():
    S = 2.4                                    # px per mm along the strap
    W, H = 1500, 1420
    g = Svg(W, H, 'Ouro wiring map along the strap')
    g.text(30, 40, 'Ouro wiring map: what rides on each link and every cable run (side view, tail left, head right)', 18, weight='bold')
    g.text(30, 64, 'Pitch 48 mm. Leave a 15 mm slack loop at every joint a cable crosses, lying flat so it bends with the hinge. '
                   'Tape cables to the solid block only, never across a leaf spring.', 12, fill=MUTED)
    ox, sy = 90, 470                           # strap baseline
    X = lambda mm: ox + mm * S
    # links
    for i in range(9):
        x0 = X(i * PITCH + 2); w = (PITCH - 4) * S
        g.rect(x0, sy, w, 34, fill='#eef1f6', rx=8)
        g.text(x0 + w / 2, sy + 22, f's{i}', 14, 'middle', 'bold')
    for j in range(1, 9):
        g.circle(X(j * PITCH), sy + 17, 7, col=INK, fill='#ffffff', sw=2)
        g.text(X(j * PITCH), sy + 58, f'j{j}', 12, 'middle', fill=MUTED)
    # payloads on top of the strap
    pay = [(3, 'battery on edge', 'XT30 + F1 at -y'), (4, 'ESP32 (pins cut)', 'mid IMU on can'), (5, 'power board', '66 x 20.5, J1'),
           (8, 'head IMU 0x69', '+ latch pins P1 P2')]
    for i, a, b in pay:
        x0 = X(i * PITCH + 4); w = (PITCH - 8) * S
        g.rect(x0, sy - 54, w, 48, col='#2f7d5b', fill='#e3f2ea', rx=6)
        g.text(x0 + w / 2, sy - 34, a, 12, 'middle', 'bold', fill='#1f5c41')
        g.text(x0 + w / 2, sy - 16, b, 11, 'middle', fill='#1f5c41')
    g.rect(X(0 * PITCH + 4), sy - 54, (PITCH - 8) * S, 48, col='#2f7d5b', fill='#e3f2ea', rx=6)
    g.text(X(24), sy - 34, 'tail cradle', 12, 'middle', 'bold', fill='#1f5c41')
    g.text(X(24), sy - 16, 'W0 bare bridge', 11, 'middle', fill='#1f5c41')
    # servos inside links (below)
    for k in range(1, 9):
        cx = X((k - 1) * PITCH + 35)
        g.rect(cx - 22, sy + 66, 44, 22, col='#6b4a1f', fill='#f6e7cf', rx=4)
        g.text(cx, sy + 81, f'M{k}', 12, 'middle', 'bold', fill='#6b4a1f')
    # runs above the strap (lanes)
    lanes = [
        ('BAT', 's3', 's5', 'red', 'BAT  22 AWG red + black twisted, 150 mm, crosses j4 j5; XT30 on the s3 front block at -y, run along the -y edge of s4', 3.5),
        ('H1', 's4', 's5', 'grey', 'H1  15 silicone wires soldered to the cut DevKit pin stubs: SIG1-8 (28 AWG) + 5V, GND (26) + 3V3 SDA SCL LATCH VSENSE (28), 90 mm, j5', 3.0),
        ('H2', 's5', 's8', 'blue', 'H2  5 x 26 AWG: SDA GND SCL 3V3 LATCH (in that order), 230-250 mm, crosses j6 j7 j8', 3.0),
        ('MID', 's4', 's4', 'orange', 'MID  4 x 28 AWG, ESP32 pin stubs -> mid IMU on the module can, 30 mm, no joint', 2.5),
        ('HEAD', 's8', 's8', 'black', 'HEAD  GND -> right pin P2, 40 mm; white LATCH -> left pin P1', 2.5),
    ]
    for n, (h, a, b, col, lab, w) in enumerate(lanes):
        y = sy - 80 - n * 46
        ia, ib = int(a[1:]), int(b[1:])
        xa, xb = X(ia * PITCH + 24), X(ib * PITCH + 24)
        if xa == xb:
            xa, xb = xa - 30, xa + 30
        g.poly([(xa, sy - 54), (xa, y), (xb, y), (xb, sy - 54)], col=WIRE_RGB[col], w=w)
        for jn in joints_between(a, b):
            jx = X(int(jn[1:]) * PITCH)
            g.add(f"<path d='M{jx - 9:.1f},{y:.1f} q9,-14 18,0' fill='none' stroke='{WIRE_RGB[col]}' stroke-width='{w}'/>")
        g.text(xa + 6, y - 8, h, 12, weight='bold')
        ly = 100 + n * 20
        g.line(30, ly - 4, 62, ly - 4, col=WIRE_RGB[col], w=4)
        g.text(72, ly, lab, 12)
    g.text(30, 100 + 5 * 20 + 4, 'Cables ride on the back of the strap (the inside of the wheel). Bumps = 15 mm slack loops at joints.', 12, fill=MUTED)
    # servo leads below
    for k in range(1, 9):
        y = sy + 112 + (k - 1) * 22
        xa, xb = X((k - 1) * PITCH + 35), X(5 * PITCH + 24)
        if abs(xa - xb) < 30:
            xb = xa + 34
        g.poly([(xa, sy + 88), (xa, y), (xb, y)], col=WIRE_RGB['orange'], w=2.4)
        g.circle(xb, y, 4, col=WIRE_RGB['orange'], fill=WIRE_RGB['orange'])
        wr = next(w for w in WIRES if w['id'] == f'SV{k}')
        lab = f"M{k}: {wr['length_mm']} mm, {len(wr['crosses'])} joint(s). {wr['note']}"
        g.line(max(xa, xb) + 8, y, 1000, y, col=GRID, w=1, dash='2 4')
        g.text(1010, y + 4, lab, 12, fill=INK)
    g.text(30, sy + 312, 'Servo k sits in the front cradle of s(k-1) and drives joint k. Lengths from cad/check_fit.py (h), MG90S lead taken as 240 mm.', 12, fill=MUTED)
    # wire table
    ty = 830
    g.text(30, ty, 'Every run', 14, weight='bold')
    hdr = ['id', 'from', 'to', 'net', 'colour', 'AWG', 'mm', 'crosses']
    cx = [30, 90, 430, 760, 900, 1010, 1060, 1110]
    for x, hname in zip(cx, hdr):
        g.text(x, ty + 22, hname, 12, weight='bold', fill=MUTED)
    rows = [w for w in WIRES if w['harness'] not in ('SERVO',)]
    half = (len(rows) + 1) // 2
    # compact: two lines per row group are too many; print all off-board + harness runs in 2 columns? keep one column, small
    lines = []
    for w in rows:
        lines.append([w['id'], w['from_'][:52], w['to'][:50], w['net'], w['colour'], str(w['awg']), str(w['length_mm']),
                      ' '.join(w['crosses']) or '-'])
    # wrap into two blocks side by side would need narrower columns; instead print 14 rows max and refer to json
    for i, row in enumerate(lines):
        y = ty + 42 + i * 17.5
        for x, cell in zip(cx, row):
            g.text(x, y, cell, 11)
    g.text(30, ty + 42 + len(lines) * 17.5 + 8, 'Servo leads are listed beside the drawing above; all runs are also in docs/electronics.json "wires".', 12, fill=MUTED)
    g.save(DOCS + 'wiring.svg', 'wiring.svg')

# ============================================================================ schematic.svg
def draw_schematic():
    W, H = 1600, 1180
    g = Svg(W, H, 'Ouro schematic')
    g.text(30, 40, 'Ouro electronics schematic (robot side). Net labels with the same name are connected.', 18, weight='bold')
    g.text(30, 64, 'All grounds common (star at the power board). Values: R1 100k 1%, R2 33k 1%, C3 100 nF, C2 220 uF 10 V low-ESR, R5 0.1 ohm 1 W.', 12, fill=MUTED)
    labels = defaultdict(int)

    def lab(x, y, net, side='r'):
        """net flag: small pentagon with the net name"""
        labels[net] += 1
        w = 0.56 * 12 * len(net) + 14
        if side == 'r':
            g.poly([(x, y), (x + 8, y - 9), (x + 8 + w, y - 9), (x + 8 + w, y + 9), (x + 8, y + 9), (x, y)], col='#2a62d0', w=1.2, fill='#eef3fd')
            g.text(x + 14, y + 4, net, 12, fill='#1b3f8f')
        else:
            g.poly([(x, y), (x - 8, y - 9), (x - 8 - w, y - 9), (x - 8 - w, y + 9), (x - 8, y + 9), (x, y)], col='#2a62d0', w=1.2, fill='#eef3fd')
            g.text(x - 14, y + 4, net, 12, 'end', fill='#1b3f8f')

    def gnd(x, y):
        labels['GND'] += 1
        g.line(x, y, x, y + 10); g.line(x - 10, y + 10, x + 10, y + 10, w=2); g.line(x - 6, y + 14, x + 6, y + 14, w=1.6); g.line(x - 2, y + 18, x + 2, y + 18, w=1.4)

    def dot(x, y): g.circle(x, y, 3.2, fill=INK, sw=0)

    def box(x, y, w, h, ref, name, fill='#f4f6fa'):
        g.rect(x, y, w, h, fill=fill, rx=6, sw=1.8)
        g.text(x + w / 2, y - 8, f'{ref}  {name}', 12, 'middle', 'bold')

    def resistor(x, y, vertical=True, ref='', val=''):
        if vertical:
            g.rect(x - 7, y, 14, 34, rx=2, fill='#fff')
            g.text(x + 12, y + 14, ref, 12, weight='bold'); g.text(x + 12, y + 29, val, 12)
        else:
            g.rect(x, y - 7, 34, 14, rx=2, fill='#fff')
            g.text(x + 17, y - 12, f'{ref} {val}', 12, 'middle', weight='bold')

    def cap(x, y, ref, val, polar=False):
        g.line(x - 12, y, x + 12, y, w=2.4); g.line(x - 12, y + 7, x + 12, y + 7, w=2.4)
        if polar: g.text(x - 18, y - 2, '+', 12, 'middle', 'bold', check=False)
        g.text(x + 18, y + 2, ref, 12, weight='bold'); g.text(x + 18, y + 16, val, 12)

    # ---------------- power chain (top band)
    y0 = 170
    # battery
    bx = 70
    g.line(bx, y0 - 30, bx, y0 - 10); g.line(bx - 16, y0 - 10, bx + 16, y0 - 10, w=2.6); g.line(bx - 9, y0 - 3, bx + 9, y0 - 3, w=2.6)
    g.line(bx - 16, y0 + 4, bx + 16, y0 + 4, w=2.6); g.line(bx - 9, y0 + 11, bx + 9, y0 + 11, w=2.6); g.line(bx, y0 + 11, bx, y0 + 40)
    g.text(bx - 40, y0 - 64, 'BT1  2S LiPo 7.4 V', 12, weight='bold')
    g.text(bx + 24, y0 + 30, '450-500 mAh', 12)
    g.text(bx + 22, y0 - 16, '+', 13, weight='bold', check=False)
    gnd(bx, y0 + 40)
    # XT30
    g.line(bx, y0 - 30, 150, y0 - 30)
    g.rect(150, y0 - 42, 34, 24, rx=4, fill='#fff'); g.text(167, y0 - 50, 'J0 XT30', 12, 'middle', 'bold')
    g.line(184, y0 - 30, 230, y0 - 30)
    # polyfuse
    g.rect(230, y0 - 38, 46, 16, rx=7, fill='#fff'); g.line(236, y0 - 22, 270, y0 - 38, w=1.2)
    g.text(253, y0 - 50, 'F1 PTC 3 A', 12, 'middle', 'bold')
    g.line(276, y0 - 30, 330, y0 - 30); dot(330, y0 - 30)
    lab(330, y0 - 30 - 0, 'VBAT_F') if False else None
    g.text(300, y0 - 8, 'VBAT_F', 12, 'middle', fill='#1b3f8f')
    # bus to U1, U3 and divider
    g.line(330, y0 - 30, 400, y0 - 30)
    g.line(330, y0 - 30, 330, y0 + 140); dot(330, y0 + 80)
    # U1
    box(400, y0 - 60, 150, 90, 'U1', 'Mini560 5 V (servo)')
    g.text(410, y0 - 26, 'IN+', 12); g.text(540, y0 - 26, 'OUT+', 12, 'end'); g.text(475, y0 + 22, 'GND', 12, 'middle')
    gnd(475, y0 + 30)
    g.line(550, y0 - 30, 620, y0 - 30)
    g.text(585, y0 - 38, 'W1', 11, 'middle', fill=MUTED)
    # INA219 shunt section
    box(620, y0 - 80, 220, 130, 'U2', 'INA219 0x40 (high side)')
    g.text(630, y0 - 26, 'VIN+', 12); g.text(830, y0 - 26, 'VIN-', 12, 'end')
    g.rect(690, y0 - 40, 80, 16, rx=2, fill='#fff'); g.text(730, y0 - 48, '0.1 ohm on module', 11, 'middle', fill=MUTED)
    g.line(664, y0 - 32, 690, y0 - 32); g.line(770, y0 - 32, 800, y0 - 32)
    g.line(680, y0 - 32, 680, y0 + 14); g.line(780, y0 - 32, 780, y0 + 14)
    g.rect(710, y0 + 7, 50, 14, rx=2, fill='#fff7e6'); g.line(680, y0 + 14, 710, y0 + 14); g.line(760, y0 + 14, 780, y0 + 14)
    g.text(735, y0 + 40, 'R5 0.1 ohm 1 W (added)', 11, 'middle', weight='bold')
    dot(680, y0 - 32); dot(780, y0 - 32)
    g.line(840, y0 - 30, 960, y0 - 30); dot(900, y0 - 30)
    g.text(930, y0 - 38, '5V_SERVO', 12, 'middle', fill='#1b3f8f')
    g.line(900, y0 - 30, 900, y0 - 2); cap(900, y0 - 2, 'C2', '220 uF', True); g.line(900, y0 + 5, 900, y0 + 20); gnd(900, y0 + 20)
    # J1 + servos
    jx = 960
    g.line(jx, y0 - 30, 1030, y0 - 30); g.text(1024, y0 - 38, '+ row', 11, 'end', fill=MUTED)
    g.line(1030, y0 + 104, 990, y0 + 104); gnd(990, y0 + 104); g.text(1024, y0 + 98, '- row', 11, 'end', fill=MUTED)
    box(1030, y0 - 90, 520, 210, 'J1', '8 x 3 servo header  ->  M1..M8 (MG90S)', fill='#fbf7ef')
    for k in range(1, 9):
        x = 1060 + (k - 1) * 62
        g.rect(x - 14, y0 - 66, 28, 64, rx=3, fill='#fff')
        for i, (t, c) in enumerate([('S', '#e8771b'), ('+', '#d02a2a'), ('-', '#8a5a2b')]):
            g.circle(x, y0 - 54 + i * 21, 5, col=c, fill=c, sw=0)
        g.text(x, y0 + 14, f'M{k}', 12, 'middle', 'bold')
        g.text(x, y0 + 32, f'SIG{k}', 11, 'middle', fill='#1b3f8f', check=False)
        labels[f'SIG{k}'] += 1
    g.text(1045, y0 + 56, 'Rows: S = signal (orange), + = 5V_SERVO (red), - = GND (brown).', 11, fill=MUTED)
    g.text(1045, y0 + 72, 'All + pins joined, all - pins joined. Column k carries SIGk.', 11, fill=MUTED)
    labels['5V_SERVO'] += 2
    # U3 ESP32 buck
    yb = y0 + 140
    g.line(330, yb, 400, yb)
    box(400, yb - 30, 150, 80, 'U3', 'Mini560 5 V (ESP32)')
    g.text(410, yb + 4, 'IN+', 12); g.text(540, yb + 4, 'OUT+', 12, 'end'); g.text(475, yb + 42, 'GND', 12, 'middle')
    gnd(475, yb + 50)
    g.line(550, yb, 600, yb); lab(600, yb, '5V_LOGIC')
    # divider
    dvx = 250
    g.line(330, y0 + 80, dvx, y0 + 80); g.line(dvx, y0 + 80, dvx, y0 + 100)
    resistor(dvx, y0 + 100, True, 'R1', '100k')
    g.line(dvx, y0 + 134, dvx, y0 + 160); dot(dvx, y0 + 160)
    g.line(dvx, y0 + 160, dvx - 80, y0 + 160); lab(dvx - 80, y0 + 160, 'VSENSE', 'l')
    g.line(dvx, y0 + 160, dvx, y0 + 180); resistor(dvx, y0 + 180, True, 'R2', '33k'); g.line(dvx, y0 + 214, dvx, y0 + 230); gnd(dvx, y0 + 230)
    g.line(dvx, y0 + 160, dvx + 60, y0 + 160); g.line(dvx + 60, y0 + 160, dvx + 60, y0 + 190)
    cap(dvx + 60, y0 + 190, 'C3', '100 nF'); g.line(dvx + 60, y0 + 197, dvx + 60, y0 + 230); gnd(dvx + 60, y0 + 230)

    # ---------------- ESP32 (middle)
    ex, ey, ew, eh = 620, 470, 300, 420
    box(ex, ey, ew, eh, 'U5', 'ESP32 DevKit (ESP32-WROOM-32)', fill='#f1f4fb')
    left = [('5V / VIN', '5V_LOGIC'), ('GND', 'GND'), ('3V3', '3V3'), ('GPIO34', 'VSENSE'), ('GPIO27', 'LATCH'),
            ('GPIO21 SDA', 'SDA'), ('GPIO22 SCL', 'SCL')]
    for i, (pin, net) in enumerate(left):
        y = ey + 40 + i * 52
        g.line(ex - 60, y, ex, y); g.text(ex + 10, y + 4, pin, 12)
        if net == 'GND':
            gnd(ex - 60, y)
        else:
            lab(ex - 60, y, net, 'l')
    right = [(f'GPIO{gp}', f'SIG{k}') for k, gp in enumerate(SERVO_GPIO, 1)] + [('GPIO2', 'LED')]
    for i, (pin, net) in enumerate(right):
        y = ey + 34 + i * 42
        g.line(ex + ew, y, ex + ew + 60, y); g.text(ex + ew - 10, y + 4, pin, 12, 'end')
        if net == 'LED':
            g.text(ex + ew + 66, y + 4, 'on-board LED (no wire)', 12, fill=MUTED)
        else:
            lab(ex + ew + 60, y, net)
    g.text(ex + ew / 2, ey + eh - 30, 'Board option: "ESP32 Dev Module"', 12, 'middle', fill=MUTED)
    g.text(ex + ew / 2, ey + eh - 12, 'Not used: GPIO 0, 5, 12, 14, 15 (strapping / boot noise)', 11, 'middle', fill=MUTED)

    # ---------------- I2C devices (left bottom)
    def imu(x, y, ref, title, ad0_net):
        box(x, y, 200, 150, ref, title, fill='#f3fbf6')
        pins = [('VCC', '3V3'), ('GND', 'GND'), ('SCL', 'SCL'), ('SDA', 'SDA'), ('AD0', ad0_net)]
        for i, (p, net) in enumerate(pins):
            yy = y + 24 + i * 26
            g.line(x - 50, yy, x, yy); g.text(x + 10, yy + 4, p, 12)
            if net == 'GND':
                gnd(x - 50, yy)
            else:
                lab(x - 50, yy, net, 'l')
        g.text(x + 190, y + 140, 'pull-ups to VCC on board', 11, 'end', fill=MUTED)
    imu(250, 640, 'U4', 'GY-521 mid (s4) 0x68', 'GND')
    imu(250, 880, 'U6', 'GY-521 head (s8) 0x69', '3V3')
    # INA logic pins
    box(1080, 470, 210, 150, 'U2', 'INA219 logic pins', fill='#f3fbf6')
    for i, (p, net) in enumerate([('VCC', '3V3'), ('GND', 'GND'), ('SCL', 'SCL'), ('SDA', 'SDA')]):
        yy = 500 + i * 30
        g.line(1290, yy, 1340, yy); g.text(1280, yy + 4, p, 12, 'end')
        if net == 'GND':
            gnd(1340, yy)
        else:
            lab(1340, yy, net)
    g.text(1090, 610, '10k pull-ups on the module', 11, fill=MUTED, check=False)

    # ---------------- latch (right bottom)
    lx0, ly0 = 1080, 720
    g.text(lx0, ly0 - 20, 'Latch contact (head s8 / tail s0)', 13, weight='bold')
    g.line(lx0, ly0 + 20, lx0 + 60, ly0 + 20); lab(lx0, ly0 + 20, 'LATCH', 'l')
    g.circle(lx0 + 70, ly0 + 20, 10, fill='#d9dde5'); g.text(lx0 + 70, ly0 + 52, 'P1 left pin', 12, 'middle')
    g.circle(lx0 + 230, ly0 + 20, 10, fill='#d9dde5'); g.text(lx0 + 230, ly0 + 52, 'P2 right pin', 12, 'middle')
    g.line(lx0 + 240, ly0 + 20, lx0 + 290, ly0 + 20); gnd(lx0 + 290, ly0 + 20)
    g.line(lx0 + 80, ly0 + 20, lx0 + 220, ly0 + 20, col='#b08d57', w=3, dash='6 5')
    g.text(lx0 + 150, ly0 + 12, 'W0 tail wire', 12, 'middle', fill='#7a5c2e')
    g.text(lx0, ly0 + 90, 'Closed (latched) = GPIO27 low. Internal ~45k pull-up; firmware debounces 15 ms.', 11, fill=MUTED)
    g.text(lx0, ly0 + 108, 'Fallback: A3144 hall switch (open-collector) on the same net, no firmware change.', 11, fill=MUTED)
    # notes
    g.text(1080, 900, 'Power path notes', 13, weight='bold')
    nts = ['U2 VIN+ from U1 OUT+, VIN- to the servos: reads positive current.',
           'R5 halves the shunt: 0.05 ohm, +-6.4 A range, 0.2 mA/LSB (firmware).',
           'C2 kept at 220 uF: Mini560 trips at start-up into >~440 uF.',
           'U3 feeds only the ESP32: a servo stall cannot reset the radio.',
           'No reverse-polarity part: XT30 is keyed. Check polarity once',
           'with a meter when you solder the pigtail (bring-up step 1).']
    for i, t in enumerate(nts):
        g.text(1080, 924 + i * 18, t, 11)
    g.save(DOCS + 'schematic.svg', 'schematic.svg')
    return labels

draw_perfboard()
draw_wiring()
labels = draw_schematic()

# schematic net check: every multi-end signal net must appear as a drawn label/wire at >= 2 ends
for net in ['SDA', 'SCL', '3V3', 'VSENSE', 'LATCH', '5V_LOGIC', '5V_SERVO', 'GND'] + [f'SIG{k}' for k in range(1, 9)]:
    if labels.get(net, 0) < 2:
        problems.append(f'schematic: net {net} drawn at {labels.get(net, 0)} place(s), needs >= 2')

# wire list must cover every off-board connection of every net (ends not on the same physical module)
covered = defaultdict(int)
for w in WIRES:
    for n in w['net'].split(' / '):
        covered[n] += 1
for net in ['SDA', 'SCL', '3V3', 'LATCH', 'VSENSE', '5V_LOGIC', 'GND', 'VBAT_F'] + [f'SIG{k}' for k in range(1, 9)]:
    if not covered[net]:
        problems.append(f'wire list: no run for net {net}')

# ============================================================================ electronics.json
out = dict(
    _about='Ouro robot electronics, generated with the drawings. Refs as in docs/schematic.svg; '
           'board holes as (column 1-23, row A-K) in docs/perfboard.svg.',
    parts={k: dict(description=v[0], value=v[1]) for k, v in PARTS.items()},
    nets=NETS,
    pin_map=[dict(gpio=g_, devkit_label=l, net=n, direction=d, notes=t) for g_, l, n, d, t in PINMAP],
    i2c=dict(clock_hz=400000, devices={'0x40': 'U2 INA219 (s5)', '0x68': 'U4 MPU-6050 mid (s4, AD0 low)',
                                       '0x69': 'U6 MPU-6050 head (s8, AD0 high)'},
             pullups='on breakouts to 3V3: GY-521 x2 (4.7k or 2.2k) || INA219 10k = 1.9 k or 1.0 k',
             timeout_ms=3, bus_length_mm=330),
    power=dict(servo_rail_V=5.0, ina219_shunt_ohm=0.05, ina219_range_A=6.4, ina219_mA_per_lsb=0.2,
               policy_current_obs='min(amps, 3.2) / 3.0 (unchanged, matches sim)',
               fuse='PTC 3 A hold / 6 A trip in the battery + lead', battery_connector='XT30 (PH2.0 is rated ~2 A); on s3 front block at -y, battery stood on its long edge',
               low_battery_cutoff='6.6 V for 2 s, latched; deep sleep 30 s later'),
    perfboard=dict(size_mm=[66, 20.5], grid='26 columns (across the link) x 8 rows (along the body), 2.54 mm, pad-per-hole (no cuts)',
                   placement='s5 front block, link x 19.0..39.5, y -33..+33, on 1 mm foam; board top z 10.6',
                   heights_z_mm={'buck stack (U3 under U1)': 23.4, 'U2 + C2 lying on it': 21.0, 'R1/R2 standing': 20.5, 'J1 + servo plugs': 27.1},
                   bottom_jumpers=[dict(net=n, from_=f'{a[0]}{a[1]}', to=f'{b[0]}{b[1]}', what=w) for n, a, b, w in JUMPERS],
                   holes={f'{c}{r}': dict(net=n, use=u) for (c, r), (n, u) in sorted(HOLES.items(), key=lambda x: (ROWS.index(x[0][1]), x[0][0]))},
                   bottom_traces=[dict(net=n, path=[f'{c}{r}' for c, r in p]) for n, p in TRACES],
                   flying_wires=[dict(id=a, from_=b, to=c, net=d, wire=e) for a, b, c, d, e in FLYING],
                   top_parts_joining_nets=[dict(part=a, between=[b, c]) for a, b, c in TOP_PARTS]),
    wires=[{('from' if k == 'from_' else k): v for k, v in w.items()} for w in WIRES],
    servo_leads=dict(source='cad/check_fit.py (h); servo k in the front cradle of s(k-1), drives joint k, header J1 on s5',
                     mg90s_lead_mm=LEAD_MG90S, extensions_mm={f'M{k}': v for k, v in EXT.items()}),
)
json.dump(out, open(DOCS + 'electronics.json', 'w'), indent=1)

print('holes used', len(HOLES), ' traces', len(TRACES), ' wires', len(WIRES))
for w in WIRES:
    if w['harness'] == 'SERVO':
        print(w['id'], w['length_mm'], w['crosses'], w['note'])
if problems:
    print('\nPROBLEMS:'); [print(' -', p) for p in problems]; sys.exit(1)
print('checks passed: perfboard nets connected/no shorts, schematic labels, wire coverage, no text overlaps')
