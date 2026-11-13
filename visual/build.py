"""Build ouro.html from template.html + traj.json + three film frames. Run sim/demo.py first."""
import base64, io, json, glob, sys
from PIL import Image

data = json.load(open('traj.json'))
CAPS = {
    'crawl':   ['1. Caterpillar crawl', 'A hand-written wave: a hump travels tail to head and the strap creeps forward. No learning yet.'],
    'rest':    ['2. Stop', 'It goes flat and still. Nothing about it says wheel.'],
    'bite':    ['3. Bite the tail', 'It rolls up from both ends, tips first. Magnets catch the head pins from about 10 mm away and the latch closes (green ring).'],
    'roll':    ['4. Roll away', 'Now a nine-sided wheel. A learned policy reshapes it so the weight stays ahead of the contact point. It sees two IMUs, the servo current and the latch, never a joint angle.'],
    'startle': ['5. Startle', 'Three joints at each end curl and the middle two bend back. That pulls the pins out of the magnets, the wheel bursts and it flops flat.'],
}
def uri(path, w=640):
    im = Image.open(path).convert('RGB'); im = im.resize((w, int(im.height * w / im.width)))
    b = io.BytesIO(); im.save(b, 'JPEG', quality=72)
    return 'data:image/jpeg;base64,' + base64.b64encode(b.getvalue()).decode()
frames = ['film_bite_0.0.png', 'film_bite_0.4.png'] + sorted(glob.glob('film_roll_*.png'))[:1]
labels = [('START', 'Flat on the desk, 432 mm long.'), ('MID-ACTION', 'Both ends lifting. The tips curl first so each servo lifts at most two links.'),
          ('END', 'Head on tail, magnets shut, rolling as a 140 mm wheel.')]
film = ''.join(f'<figure><img src="{uri(f)}" alt="{l[1]}"><figcaption><b>{l[0]}</b><br>{l[1]}</figcaption></figure>' for f, l in zip(frames, labels))
facts = json.load(open('facts.json'))
html = open('template.html').read()
html = html.replace('__FILM__', film).replace('__DATA__', json.dumps(data, separators=(',', ':'))).replace('__CAPS__', json.dumps(CAPS))
html = html.replace('__FACTS__', ''.join(f'<div><dt>{k}</dt><dd>{v}</dd></div>' for k, v in facts.items()))
open('ouro.html', 'w').write(html)
print('ouro.html', len(html) // 1024, 'KB,', len(data['frames']), 'frames,', len(frames), 'film frames')
