"""Files for the builder and the website, all into cad/out/:
  assembled_flat.glb / .stl, assembled_wheel.glb / .stl   every printed and bought part, per-part colours in the GLB
  render_flat.png, render_wheel.png, render_joint_exploded.png, render_latch.png, render_link_top.png, render_link_side.png
Run after make_parts.py:  python export.py   (renders with MuJoCo's offscreen renderer, works headless on macOS)
"""
import os, shutil, tempfile
import numpy as np
import trimesh
import mujoco
from PIL import Image, ImageDraw, ImageFont
import make_parts as P
import assembly as A

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')


def tm(man, colour):
    m = man.to_mesh()
    mesh = trimesh.Trimesh(np.asarray(m.vert_properties)[:, :3].copy(), np.asarray(m.tri_verts).copy(), process=False)
    mat = trimesh.visual.material.PBRMaterial(baseColorFactor=list(colour) + [1.0], metallicFactor=0.0, roughnessFactor=0.7)
    mesh.visual = trimesh.visual.TextureVisuals(material=mat)
    return mesh

def write_assembly(name, items):
    """items: list of (segment, Item). GLB in metres (glTF convention, y up), STL in mm."""
    scene = trimesh.Scene()
    swap = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float) @ np.diag([1e-3, 1e-3, 1e-3, 1])
    allm = []
    for n, (k, it) in enumerate(items):
        if it.solid.is_empty():
            continue
        mesh = tm(it.solid, it.colour)
        allm.append(mesh.copy())
        mesh.apply_transform(swap)
        scene.add_geometry(mesh, node_name=f's{k}_{it.name}_{n}', geom_name=f's{k}_{it.name}_{n}')
    scene.export(os.path.join(OUT, f'{name}.glb'))
    trimesh.util.concatenate(allm).export(os.path.join(OUT, f'{name}.stl'))


# ------------------------------------------------------------------------------------------------ MuJoCo renders
def render(items, cams, size=(1280, 800), floor_z=None, tmpdir=None, ortho=False):
    """items: list of (Item in world mm). cams: dict name -> (pos, xyaxes, fovy) in mm. Returns dict of images."""
    d = tempfile.mkdtemp(dir=tmpdir)
    assets, geoms = [], []
    for n, it in enumerate(items):
        if it.solid.is_empty():
            continue
        f = os.path.join(d, f'm{n}.stl')
        P.to_stl(it.solid, f)
        assets.append(f'<mesh name="m{n}" file="{f}" scale="0.001 0.001 0.001" inertia="shell"/>')
        r, g, b = it.colour
        geoms.append(f'<geom type="mesh" mesh="m{n}" rgba="{r} {g} {b} 1" contype="0" conaffinity="0"/>')
    cam_xml = []
    for cn, (pos, xy, fovy) in cams.items():
        pos_m = ' '.join(f'{v/1000:.5f}' for v in pos)
        orth = ' projection="orthographic"' if ortho else ''
        cam_xml.append(f'<camera name="{cn}" pos="{pos_m}" xyaxes="{" ".join(f"{v:.5f}" for v in xy)}" fovy="{fovy if not ortho else fovy/1000:.5f}"{orth}/>')
    floor = (f'<geom type="plane" size="1 1 0.01" pos="0 0 {floor_z/1000:.5f}" material="desk" contype="0" conaffinity="0"/>'
             if floor_z is not None else '')
    xml = f"""<mujoco><compiler angle="radian"/>
      <visual><global offwidth="{size[0]}" offheight="{size[1]}"/><quality shadowsize="4096" offsamples="8"/>
        <headlight ambient="0.38 0.38 0.38" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1"/>
        <map znear="0.001" zfar="5"/></visual>
      <asset>{''.join(assets)}
        <texture name="grid" type="2d" builtin="checker" rgb1="0.93 0.91 0.86" rgb2="0.88 0.86 0.80" width="512" height="512"/>
        <material name="desk" texture="grid" texrepeat="12 12" reflectance="0.02"/>
        <texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="16" height="16"/></asset>
      <worldbody>
        <light pos="0.3 -0.5 0.9" dir="-0.3 0.5 -0.9" diffuse="0.45 0.45 0.45" castshadow="true"/>
        <light pos="-0.4 0.3 0.6" dir="0.4 -0.3 -0.6" diffuse="0.25 0.25 0.25" castshadow="false"/>
        {floor}{''.join(geoms)}{''.join(cam_xml)}
      </worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    out = {}
    with mujoco.Renderer(model, size[1], size[0]) as r:
        for cn in cams:
            r.update_scene(data, camera=cn)
            out[cn] = r.render().copy()
    shutil.rmtree(d, ignore_errors=True)
    return out

def look(pos, target, up=(0, 0, 1)):
    """Camera xyaxes looking from pos to target."""
    f = np.array(target, float) - np.array(pos, float); f /= np.linalg.norm(f)
    x = np.cross(f, up); x /= np.linalg.norm(x)
    y = np.cross(x, f)
    return list(x) + list(y)

def flat_items(items):
    return [it for _, it in items]

def font(sz):
    for p in ('/System/Library/Fonts/Supplemental/Arial.ttf', '/System/Library/Fonts/Helvetica.ttc', '/Library/Fonts/Arial.ttf'):
        if os.path.exists(p):
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()

def save(img, name, title=None):
    im = Image.fromarray(img)
    if title:
        band = Image.new('RGB', (im.width, im.height + 56), (255, 255, 255)); band.paste(im, (0, 56)); im = band
        d = ImageDraw.Draw(im); d.text((24, 14), title, fill=(40, 40, 40), font=font(24))
    im.save(os.path.join(OUT, name))


def dim_line(d, p0, p1, text, f, off=(0, 0), col=(200, 30, 30)):
    (x0, y0), (x1, y1) = p0, p1
    d.line([p0, p1], fill=col, width=2)
    for (x, y), (xa, ya) in ((p0, p1), (p1, p0)):
        v = np.array([xa - x, ya - y], float); v /= np.linalg.norm(v) + 1e-9; n = np.array([-v[1], v[0]])
        tip = np.array([x, y]); d.polygon([tuple(tip), tuple(tip + 12 * v + 5 * n), tuple(tip + 12 * v - 5 * n)], fill=col)
    d.text(((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1]), text, fill=col, font=f, anchor='mm')


def main():
    os.makedirs(OUT, exist_ok=True)
    tmp = tempfile.mkdtemp()
    flat, Tf = A.assembly([0.0] * 8)
    wheel, Tw = A.assembly([40.0] * 8)
    write_assembly('assembled_flat', flat)
    write_assembly('assembled_wheel', wheel)
    print('wrote assembled_flat/wheel .glb .stl')

    floor = -P.HT - 1.0
    # flat strap
    c = (216, 0, 0)
    imgs = render(flat_items(flat), {'hero': ((470, -380, 300), look((470, -380, 300), (215, 0, 0)), 38)}, floor_z=floor, tmpdir=tmp)
    save(imgs['hero'], 'render_flat.png', 'Ouro, flat strap: 9 links, 8 MG90S, battery s3, ESP32 + IMU s4, power s5')
    # closed wheel: put the wheel on the floor (lowest point at -9 mm in the s0 frame after rotation)
    T0 = np.eye(4)
    pts = np.array([(T @ np.array([0, 0, 0, 1]))[:3] for T in Tw])
    ctr = pts[:9].mean(0)
    # rotate the wheel so the s4/s5 corner is at the bottom (any corner works)
    zmin = min(it.solid.bounding_box()[2] for _, it in wheel)
    items = [A.Item(it.name, it.solid.translate([0, 0, -zmin - P.HT - 1.0 + floor + 0.0]), it.colour, it.group) for _, it in wheel]
    ctr2 = ctr + np.array([0, 0, -zmin - P.HT - 1.0 + floor])
    imgs = render(items, {'wheel': (tuple(ctr2 + np.array([180, -330, 60])), look(ctr2 + np.array([180, -330, 60]), ctr2), 40)},
                  floor_z=floor, tmpdir=tmp)
    save(imgs['wheel'], 'render_wheel.png', 'Closed wheel: every joint at 40 deg, head pins in the tail magnet notches')
    # one joint exploded: link s1 + its servo lifted out of the cradle; horn, horn screw and the next link (s2)
    # pulled out along the servo shaft (+y) and forward; the M2 pivot screw pulled out along -y
    l1 = A.link_items(1, payload=False)
    l2 = A.link_items(2, payload=False)
    ex, lift = [], 30.0
    for it in l1:
        sh = {'servo': (0, 0, lift), 'servo_tie': (0, 0, lift + 16), 'pivot_screw': (0, -42, lift)}.get(it.name, (0, 0, 0))
        if it.group == 'part' or it.name in ('servo', 'servo_tie', 'pivot_screw', 'oring'):
            ex.append(A.Item(it.name, it.solid.translate(list(sh)), it.colour, it.group))
    for it in l2:
        if it.group in ('part', 'rotor'):
            base = it.solid.translate([P.L, 0, lift])
            sh = {'link': (34, 18, 0), 'horn': (0, 16, 0), 'horn_screw': (0, 30, 0), 'spline': (0, 0, 0)}.get(it.name, (0, 0, 0))
            ex.append(A.Item(it.name, base.translate(list(sh)), it.colour, it.group))
    cp = (150, 140, 150)
    imgs = render(ex, {'ex': (cp, look(cp, (55, 5, 12)), 40)}, floor_z=floor, tmpdir=tmp)
    save(imgs['ex'], 'render_joint_exploded.png',
         'One joint exploded: MG90S lifted out of the cradle, zip tie, horn + screw, next link\'s fork, M2 pivot screw (left)')
    # latch close-up: tail + head at 40 deg, cut through the middle of one notch block
    tail = A.link_items(0, payload=False)
    head = A.link_items(8, payload=False)
    H = np.linalg.inv(A.Tx(P.L) @ A.Ry(40))
    li = tail + [A.Item(i.name, i.solid.transform(H[:3, :]), i.colour, i.group) for i in head]
    from components import box
    yc = sum(P.NOTCH_BLOCK) / 2
    cutbox = box(-80, 80, yc, 80, -60, 60)
    li = [A.Item(i.name, i.solid - cutbox, i.colour, i.group) for i in li]
    li = [i for i in li if not i.solid.is_empty()]
    cp = (2, 110, 4)
    imgs = render(li, {'latch': (cp, look(cp, (2, yc, 1)), 20)}, tmpdir=tmp)
    save(imgs['latch'], 'render_latch.png',
         f'Latch section at y = {yc:.1f} (middle of a notch block): M3 pin in the seat, magnet 0.8 mm behind, sense wire below-front')
    # orthographic top and side of one plain link, with dimensions
    li = [i for i in A.link_items(2, payload=False) if i.group in ('part', 'rubber')]
    W, Hh = 1400, 900
    imgs = render(li, {'top': ((24, 0, 300), [1, 0, 0, 0, 1, 0], 110), 'side': ((24, -300, 0), [1, 0, 0, 0, 0, 1], 110)},
                  size=(W, Hh), tmpdir=tmp, ortho=True)
    s = Hh / 110.0                                      # px per mm
    def px(x, y):                                       # world (mm) -> pixel, camera centred on x=24
        return (W / 2 + (x - 24) * s, Hh / 2 - y * s)
    f = font(22)
    for view in ('top', 'side'):
        im = Image.fromarray(imgs[view]); d = ImageDraw.Draw(im)
        if view == 'top':
            dim_line(d, px(0, -46), px(48, -46), 'pitch 48 (joint axis to joint axis)', f, off=(0, 18))
            dim_line(d, px(62, -40), px(62, 40), '80', f, off=(22, 0))
            dim_line(d, px(-12, -33), px(-12, 33), '66 body', f, off=(-50, 0))
            dim_line(d, px(P.FLEX_X0, 44), px(P.FLEX_X1, 44), f'leaf {P.FLEX_X1 - P.FLEX_X0:.0f}', f, off=(0, -18))
            for xx in (0, 48):
                d.line([px(xx, -50), px(xx, 50)], fill=(60, 60, 200), width=1)
            d.text((24, 18), 'Plain link (x4), top view, mm. Blue lines: joint axes. Rear fork on the left, servo cradle right.',
                   fill=(40, 40, 40), font=font(24))
        else:
            dim_line(d, px(-14, -8), px(-14, 8), '16', f, off=(-24, 0))
            dim_line(d, px(48, -13), px(48 + 9, -13), 'R9 tread', f, off=(14, 16))
            dim_line(d, px(0, 22), px(48, 22), '48', f, off=(0, -16))
            d.line([px(-10, -9), px(62, -9)], fill=(120, 120, 120), width=1)
            d.text(px(64, -9), 'desk (O-rings 1 mm proud)', fill=(90, 90, 90), font=font(18), anchor='lm')
            d.line([px(-10, 0), px(62, 0)], fill=(60, 60, 200), width=1)
            d.text(px(-10, 1.5), 'joint axis', fill=(60, 60, 200), font=font(18), anchor='lb')
            dim_line(d, px(P.FLEX_X0, -14), px(P.FLEX_X1, -14), 'leaf 1.0 thick', f, off=(0, 16))
            d.text((24, 18), 'Plain link, side view from -y, mm. Back (+z) up = inside of the wheel; belly prints on the bed.',
                   fill=(40, 40, 40), font=font(24))
        im.save(os.path.join(OUT, f'render_link_{view}.png'))
    shutil.rmtree(tmp, ignore_errors=True)
    print('wrote renders')


if __name__ == '__main__':
    main()
