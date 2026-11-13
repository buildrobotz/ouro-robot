"""Build the Ouro project website into site/dist/ (plain static files, relative links, no server code).

    pip install markdown pillow
    python site/build_site.py
    cd site/dist && python -m http.server 8000      # preview: the 3D viewer needs http, not file://

Content comes from the repo itself: README.md sections, docs/*.md, docs/bom.json, docs/*.svg,
cad/out/*, visual/traj.json. Edit those, rebuild, copy dist/ to your web host.
"""
import json, re, shutil, html, pathlib, io
import markdown
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC, DIST = ROOT / 'site' / 'src', ROOT / 'site' / 'dist'
REPO_URL = 'https://github.com/buildrobotz/ouro-robot'

PAGES = [  # slug, nav label, title, lede
    ('index', 'Overview', 'Ouro, the worm that bites its tail and rolls away',
     'A flat strap of nine printed links and eight $3 hobby servos. It crawls like a caterpillar, rolls itself up, snaps its head onto its tail with magnets and becomes a wheel. You train it with reinforcement learning in MuJoCo on your laptop, then run it on the real robot.'),
    ('build', 'Build plan', 'Build plan', 'From ordering parts to a trained robot rolling on your desk: every step in order, how long it takes and what to watch for.'),
    ('parts', 'Parts', 'Parts and prices', 'Everything to buy for one robot, about $87, with what to look for in each listing.'),
    ('mechanical', 'Mechanical', 'Print and assemble', 'Nine prints from six part files, no supports. Spin the models, download the STLs, then build one joint and the rest follow.'),
    ('electronics', 'Electronics', 'Circuit and wiring', 'One ESP32, eight servos, two IMUs, a current sensor and a 2S battery. Schematic, wiring map, power board and a safe bring-up, step by step.'),
    ('software', 'Software', 'Software', 'Three programs on three computers: firmware on the robot, a radio dongle, and Python on your laptop where the policy runs.'),
    ('learning', 'Simulation and RL', 'Simulate and train', 'The MuJoCo model, servos made as bad as real ones, what the policy sees, what it is rewarded for, and how to train your own.'),
    ('course', 'Learning path', 'Learning path', 'Thirteen one-hour sessions from the simulator to a filmed robot. One thing to do, one thing to learn and one check in each.'),
    ('risks', 'Limits', 'Limits and gotchas', 'What to know before you order parts: the trade-offs in the design, what wears out, and how to keep it safe.'),
]
CAPS = {
    'crawl':   ['1. Caterpillar crawl', 'A hand-written wave: a hump travels tail to head and the strap creeps forward. No learning yet.'],
    'rest':    ['2. Stop', 'It goes flat and still. Nothing about it says wheel.'],
    'bite':    ['3. Bite the tail', 'A learned policy rolls it up from both ends. Magnets catch the head pins from about 10 mm away and the latch closes (green ring).'],
    'roll':    ['4. Roll away', 'Now a nine-sided wheel. A learned policy reshapes it so the weight stays ahead of the contact point. It sees two IMUs, the servo current and the latch, never a joint angle.'],
    'startle': ['5. Startle', 'Three joints at each end curl and the middle two bend back. That pulls the pins out of the magnets, the wheel bursts and it flops flat.'],
}
MD = dict(extensions=['tables', 'fenced_code', 'sane_lists', 'toc'])


def label_table(t):
    """Give every cell a data-label from its column header so wide tables can stack into cards on phones."""
    heads = [re.sub(r'<[^>]+>', '', h).strip() for h in re.findall(r'<th[^>]*>(.*?)</th>', t, re.S)]
    if len(heads) < 3:
        return t
    def row(m):
        cells = iter(heads)
        return re.sub(r'<td(?=[\s>])', lambda c: f'<td data-label="{html.escape(next(cells, ""), quote=True)}"', m.group(0))
    t = re.sub(r'<tr>.*?</tr>', row, t, flags=re.S)
    return t.replace('<table', '<table class="stack"', 1)

def tables(out):
    out = re.sub(r'<table>.*?</table>', lambda m: label_table(m.group(0)), out, flags=re.S)
    return re.sub(r'<table', '<div class="tablewrap"><table', out).replace('</table>', '</table></div>')

def md(text):
    return tables(markdown.markdown(text, **MD))

def fold_reference(body):
    """Inside a '## Reference' / '## Design reference' section, turn each ### subsection into a tap-to-open block."""
    m = re.search(r'<h2 id="(?:design-)?reference[^"]*">.*?</h2>', body, re.S)
    if not m:
        return body
    start = m.end(); nxt = re.search(r'<h2[ >]', body[start:]); end = start + nxt.start() if nxt else len(body)
    sec = body[start:end]
    parts = re.split(r'(<h3 id="[^"]+">.*?</h3>)', sec, flags=re.S)
    out = parts[0]
    for i in range(1, len(parts), 2):
        title = re.sub(r'<[^>]+>', '', parts[i])
        hid = re.search(r'id="([^"]+)"', parts[i]).group(1)
        out += f'<details class="ref" id="{hid}"><summary>{title}</summary>{parts[i + 1]}</details>'
    return body[:start] + out + body[end:]

def toc(body):
    heads = re.findall(r'<h2 id="([^"]+)">(.*?)</h2>', body, re.S)
    if len(heads) < 4:
        heads = re.findall(r'<h[23] id="([^"]+)">(.*?)</h[23]>', body, re.S)
    if len(heads) < 4:
        return ''
    items = ''.join(f'<li><a href="#{i}">{re.sub(r"<[^>]+>", "", t)}</a></li>' for i, t in heads)
    return f'<details class="toc"><summary>On this page</summary><ol>{items}</ol></details>'

def readme_sections():
    text = (ROOT / 'README.md').read_text()
    parts = re.split(r'\n## (\d+)\. ([^\n]+)\n', text)
    return {int(parts[i]): (parts[i + 1], re.sub(r'\n---\s*$', '', parts[i + 2].strip())) for i in range(1, len(parts), 3)}

def doc(name, drop_title=True):
    p = ROOT / 'docs' / name
    if not p.exists():
        return ''
    t = p.read_text()
    return re.sub(r'\A# [^\n]*\n', '', t) if drop_title else t

def page(slug, body, scripts=''):
    i = [p[0] for p in PAGES].index(slug); _, label, title, lede = PAGES[i]
    brz = 'https://buildrobotz.com'
    cur = ' aria-current="page"'
    nav = ''.join(f'<a href="{s}.html"{cur if s == slug else ""}>{l}</a>' for s, l, _, _ in PAGES)
    prev = f'<a href="{PAGES[i-1][0]}.html">&larr; {PAGES[i-1][1]}</a>' if i else '<span></span>'
    nxt = f'<a href="{PAGES[i+1][0]}.html">{PAGES[i+1][1]} &rarr;</a>' if i + 1 < len(PAGES) else '<span></span>'
    return f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(label if slug != "index" else "Ouro")} · Ouro robot</title>
<meta name="description" content="{html.escape(lede)}">
<link rel="icon" href="data:,"><link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Bricolage+Grotesque:opsz,wght@12..96,500;12..96,800&family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<link rel="stylesheet" href="assets/style.css"></head>
<body>
<div class="crumb"><div class="wrap"><a href="{brz}/">BuildRobotz</a><span aria-hidden="true">/</span><a href="{brz}/robots/">Robots</a><span aria-hidden="true">/</span><span>Ouro</span></div></div>
<header class="top"><div class="wrap"><a class="brand" href="index.html">Ouro<span>.</span></a><nav aria-label="Project pages">{nav}</nav></div></header>
<div class="wrap"><main>
<header><div class="eyebrow">BuildRobotz project · {html.escape(label)}</div><h1>{html.escape(title)}</h1><p class="lede">{html.escape(lede)}</p></header>
{toc(body)}{fold_reference(body)}
<div class="pager">{prev}{nxt}</div>
</main></div>
<footer><div class="wrap">Ouro is an open robot project from <a href="{brz}/">BuildRobotz</a>. The design was generated with AI and has not yet been built or tested on hardware; performance figures are MuJoCo simulation results. Build at your own risk. Code, CAD and firmware: <a href="{REPO_URL}">{REPO_URL.split("//")[1]}</a> (code MIT, docs and CAD CC BY-SA 4.0).<br>
<a href="{brz}/robots/">More robots to build</a> · <a href="{brz}/">Learn robotics with BuildRobotz</a></div></footer>
<script>(function(){{var a=document.querySelector('nav [aria-current]');if(a&&a.parentNode.scrollWidth>a.parentNode.clientWidth)a.parentNode.scrollLeft=a.offsetLeft-a.parentNode.clientWidth/2+a.offsetWidth/2;
function op(){{var t=location.hash&&document.getElementById(decodeURIComponent(location.hash.slice(1)));for(var e=t;e;e=e.parentElement)if(e.tagName==='DETAILS')e.open=true;if(t)t.scrollIntoView();}}
addEventListener('hashchange',op);op();}})();</script>
{scripts}
</body></html>'''

def copy_img(src, name, w=1100):
    """Copy a raster into dist/assets, downsized to JPEG; returns the relative URL or None."""
    src = pathlib.Path(src)
    if not src.exists():
        return None
    im = Image.open(src).convert('RGB')
    if im.width > w:
        im = im.resize((w, int(im.height * w / im.width)))
    im.save(DIST / 'assets' / (name + '.jpg'), 'JPEG', quality=80)
    return f'assets/{name}.jpg'

def fig(url, cap, alt=None):
    return f'<figure class="fig"><img src="{url}" alt="{html.escape(alt or cap)}" loading="lazy"><figcaption>{cap}</figcaption></figure>' if url else ''

def status_note():
    return ('<div class="note"><b>Read this first: AI-generated, not yet built</b>This design, its code and these pages were generated with AI, and nobody has built or tested a physical Ouro yet. '
            'The simulation, trained policies, firmware and print files are complete, and every performance number here comes from simulation. '
            'Treat it as a well-worked starting point: check each step as you go, expect to fix things, and build at your own risk, especially around the LiPo battery. '
            'The <a href="risks.html">Limits</a> page lists what to measure and watch.</div>')

# ------------------------------------------------------------------------------------------- pages
def p_index(S):
    films = [('film_bite_0.0', 'START', 'Flat on the desk, 432 mm long.'), ('film_bite_0.4', 'MID-ACTION', 'Both ends lifting, tips first.'),
             ('film_roll_1.0', 'END', 'Head on tail, magnets shut, rolling as a 140 mm wheel.')]
    film = ''.join(f'<figure><img src="{copy_img(ROOT / "visual" / (f + ".png"), f, 640)}" alt="{c}"><figcaption><b>{l}</b><br>{c}</figcaption></figure>' for f, l, c in films)
    facts = json.loads((ROOT / 'site' / 'facts.json').read_text())
    cards = ''.join(f'<a href="{s}.html"><b>{l}</b><span>{d}</span></a>' for s, l, _, d in PAGES[1:])
    return f'''
<section class="stage"><canvas id="cv" width="1040" height="420" role="img" aria-label="Side view replay of the robot crawling, biting its tail, rolling as a wheel and bursting open"></canvas>
<div class="now" id="now"></div><p class="cap" id="cap"></p>
<div class="bar"><button id="play" type="button">Pause</button><input id="scrub" type="range" min="0" max="100" value="0" aria-label="Time">
<div class="read"><span>t <b id="rt">0.0</b> s</span><span>servo current <b id="ra">0.0</b> A</span><span>magnet latch <b id="rl">open</b></span></div></div>
<div class="bar"><div class="phases" id="phases"></div><button id="slow" type="button" aria-pressed="false">Half speed</button></div></section>
<p class="cap" style="padding:0">This is a replay of the MuJoCo run, not a drawing of what it might do. The robot bends in one plane, so the side view is exact.</p>
<dl class="facts">{''.join(f"<div><dt>{k}</dt><dd>{v}</dd></div>" for k, v in facts.items())}</dl>
{status_note()}
<section class="prose"><h2>The surprising moment, in three frames</h2></section>
<div class="film">{film}</div>
<section class="prose">{md(S[1][1])}<h2>Three behaviours</h2>{md(S[8][1])}</section>
<section><div class="prose"><h2>Build it, page by page</h2></div><div class="cards" style="margin-top:12px">{cards}</div></section>'''

def money(x):
    return '' if x is None else f'${x:,.2f}'

def p_parts(S):
    bom = json.loads((ROOT / 'docs' / 'bom.json').read_text())
    rows, tot_a, tot_c, cart, unpriced = '', 0.0, 0.0, 0.0, []
    for it in bom:
        q = it.get('qty_per_robot_num', it.get('qty_per_robot'))
        a, alts = it.get('amazon') or {}, [x for x in (it.get('alt'), it.get('alt2')) if x]
        qn = float(q) if isinstance(q, (int, float)) else float(re.match(r'[\d.]+', str(q)).group()) if re.match(r'[\d.]+', str(q)) else 1.0
        au = a.get('unit_usd'); best = min([x for x in alts if x.get('unit_usd') is not None], key=lambda x: x['unit_usd'], default=None)
        line_a = au * qn if au is not None else None
        cheapest = min([v for v in (au, best and best['unit_usd']) if v is not None], default=None)
        if line_a is not None: tot_a += line_a
        elif best: tot_a += best['unit_usd'] * qn
        if cheapest is not None: tot_c += cheapest * qn
        else: unpriced.append(it['name'])
        if a.get('price_usd') and qn: cart += a['price_usd']
        acell = (f'<a href="{a["url"]}" rel="noopener">{html.escape((a.get("title") or "Amazon listing")[:70])}</a><br>'
                 f'{(("about " + money(a.get("price_usd"))) if a.get("price_usd") else "")}{(" / pack of " + str(a["pack_qty"])) if a.get("price_usd") and a.get("pack_qty") else ""}') if a.get('url') else '&ndash;'
        ccell = '<br>'.join(f'<a href="{x["url"]}" rel="noopener">{html.escape(x.get("vendor", "Vendor"))}</a>: {money(x.get("price_usd"))}{(" / " + str(x["pack_qty"])) if x.get("pack_qty") else ""}'
                            + (f' <span class="pill">ships in {x["ship_days"]} days</span>' if x.get('ship_days') else '') for x in alts) or '&ndash;'
        note = html.escape(it.get('build_note') or a.get('notes') or '')
        rows += (f'<tr><td data-label="#">{it["id"]}</td><td data-label="Part" class="lead"><b>{html.escape(it["name"])}</b><br><span style="color:var(--muted);font-size:.85rem">{note}</span></td>'
                 f'<td class="num" data-label="Per robot">{html.escape(str(q))}</td><td data-label="Amazon">{acell}</td><td data-label="Other sources">{ccell}</td><td class="num" data-label="Line">{money(line_a if line_a is not None else (best["unit_usd"] * qn if best else None))}</td></tr>')
    src = doc('bom-sources.md')
    return f'''
<div class="note"><b>About the prices</b>Prices are typical pack prices and they move, so click through and check before you order. The "per robot" total assumes packs are shared across a group; buying for a single robot costs more but leaves you spares.</div>
<dl class="facts"><div><dt>Per robot, Amazon packs shared</dt><dd>{money(tot_a)}</dd></div><div><dt>Per robot, cheapest source per line</dt><dd>{money(tot_c)}</dd></div><div><dt>Single robot, every pack bought once</dt><dd>about $235</dd></div></dl>
<div class="tablewrap wide"><table class="stack"><thead><tr><th>#</th><th>Part</th><th class="num">Per robot</th><th>Amazon</th><th>Other sources</th><th class="num">Line</th></tr></thead><tbody>{rows}</tbody></table></div>
<section class="prose">{md(doc('parts.md'))}<h2>Sourcing notes</h2>{md(src)}</section>'''

def p_mechanical(S):
    out = ROOT / 'cad' / 'out'; (DIST / 'models').mkdir(exist_ok=True)
    models = []
    for label, n, color in (('Closed wheel', 'assembled_wheel.glb', None), ('Flat strap', 'assembled_flat.glb', None), ('Link (x4)', 'link.stl', '#f2b632'),
                            ('Battery link s3', 'link_s3_battery.stl', '#f2b632'), ('ESP32 link s4', 'link_s4_esp32.stl', '#f2b632'),
                            ('Power link s5', 'link_s5_power.stl', '#f2b632'), ('Tail', 'tail.stl', '#d9560f'), ('Head', 'head.stl', '#d9560f'),
                            ('Test coupon', 'coupon.stl', '#8fae9f')):
        if (out / n).exists():
            shutil.copy(out / n, DIST / 'models' / n); models.append(dict(label=label, file=f'models/{n}', color=color))
    CAPTION = {'render_flat': 'Flat strap with servos and electronics', 'render_wheel': 'Closed wheel, every joint at 40°',
               'render_joint_exploded': 'One joint, exploded: servo, zip tie, horn, fork, pivot screw', 'render_latch': 'Latch section: steel pin in the notch, magnet 0.8 mm behind',
               'render_link_top': 'One link, top view', 'render_link_side': 'One link, side view'}
    gallery = ''.join(fig(copy_img(p, 'cad_' + p.stem), CAPTION.get(p.stem, p.stem.replace('_', ' ').capitalize())) for p in sorted(out.glob('*.png'))) if out.exists() else ''
    dl = ''.join(f'<a class="btn" href="models/{p.name}">{p.name}</a> ' for p in sorted((DIST / 'models').glob('*.stl')))
    if (out / 'check_fit.txt').exists():
        shutil.copy(out / 'check_fit.txt', DIST / 'assets' / 'check_fit.txt'); dl += '<a class="btn" href="assets/check_fit.txt">check_fit.txt (full fit report)</a>'
    return f'''
<section><div class="viewer" data-models='{json.dumps(models)}'><canvas class="view"></canvas><span class="hint">drag to orbit · scroll to zoom</span></div>
<div class="bar vbar" style="border:1px solid var(--rule);border-top:0;border-radius:0 0 6px 6px"></div></section>
<section class="prose"><h4>Download the print files</h4><p>{dl}</p></section>
<div class="gallery">{gallery}</div>
<section class="prose">{md(doc('mechanical.md'))}</section>''', '''<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/GLTFLoader.js"></script>
<script src="assets/viewer.js"></script>'''

def p_electronics(S):
    svgs = ''
    for f, cap in (('schematic.svg', 'Schematic'), ('wiring.svg', 'Wiring map along the strap'), ('perfboard.svg', 'Power board layout')):
        if (ROOT / 'docs' / f).exists():
            shutil.copy(ROOT / 'docs' / f, DIST / 'assets' / f)
            svgs += f'<figure class="fig"><div class="svgwrap"><a href="assets/{f}"><img src="assets/{f}" alt="{cap}"></a></div><figcaption><b>{cap.upper()}</b> · open the image for full size</figcaption></figure>'
    if (ROOT / 'docs' / 'electronics.json').exists():
        shutil.copy(ROOT / 'docs' / 'electronics.json', DIST / 'assets' / 'electronics.json')
    body = doc('electronics.md') or S[5][1]
    body = re.sub(r'!\[[^\]]*\]\((?:\./)?(schematic|wiring|perfboard)\.svg\)', '', body)
    return f'{svgs}<section class="prose">{md(body)}</section>'

def p_software(S):  return f'<section class="prose">{md(doc("software.md"))}</section>'
def p_learning(S):  return f'<section class="prose"><h2>The design in MuJoCo</h2>{md(S[3][1])}<h2>The RL program</h2>{md(S[4][1])}{md(doc("stress.md", False))}</section>'
def p_build(S):     return f'{status_note()}<section class="prose wide" style="max-width:none">{md(S[6][1])}</section>'
def p_course(S):    return f'<section class="prose wide" style="max-width:none">{md(S[7][1])}</section>'
def p_risks(S):     return f'<section class="prose">{md(S[9][1])}</section>'


def main():
    if DIST.exists():
        shutil.rmtree(DIST)
    (DIST / 'assets').mkdir(parents=True)
    for f in ('style.css', 'replay.js', 'viewer.js'):
        shutil.copy(SRC / f, DIST / 'assets' / f)
    traj = (ROOT / 'visual' / 'traj.json').read_text()
    (DIST / 'assets' / 'traj.js').write_text(f'window.OURO_TRAJ={traj};window.OURO_CAPS={json.dumps(CAPS)};')
    S = readme_sections()
    for slug, *_ in PAGES:
        r = globals()['p_' + slug](S)
        body, scripts = r if isinstance(r, tuple) else (r, '')
        if slug == 'index':
            scripts = '<script src="assets/traj.js"></script><script src="assets/replay.js"></script>'
        body = re.sub(r'(src|href)="(?:\./)?(schematic\.svg|wiring\.svg|perfboard\.svg|electronics\.json)"', r'\1="assets/\2"', body)
        body = re.sub(r'(src|href)="\.\./cad/out/([\w-]+)\.png"', r'\1="assets/cad_\2.jpg"', body)
        (DIST / f'{slug}.html').write_text(page(slug, body, scripts))
        if slug == 'index':   # body-only copy for hosts that wrap pages in their own <html>
            full = page(slug, body, scripts)
            head = re.search(r'<title>.*?</title>', full, re.S).group(0) + ''.join(re.findall(r'<link rel="stylesheet"[^>]*>', full))
            (ROOT / 'site' / 'index.fragment.html').write_text(head + re.search(r'<body>(.*)</body>', full, re.S).group(1))
    (DIST / 'CNAME').write_text('ouro.buildrobotz.com\n')   # custom domain for GitHub Pages
    n = sum(1 for _ in DIST.rglob('*') if _.is_file()); size = sum(f.stat().st_size for f in DIST.rglob('*') if f.is_file())
    print(f'built {len(PAGES)} pages, {n} files, {size / 1e6:.1f} MB -> {DIST}')

if __name__ == '__main__':
    main()
