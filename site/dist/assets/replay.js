// Side-view replay of the MuJoCo run. Needs window.OURO_TRAJ (traj.js) and the #cv canvas block.
(function(){
  const DATA = window.OURO_TRAJ, CAPS = window.OURO_CAPS, cv = document.getElementById('cv');
  if (!DATA || !cv) return;
  const ctx = cv.getContext('2d'), F = DATA.frames, N = F.length, dt = DATA.dt, marks = DATA.marks;
  const el = id => document.getElementById(id), scrub = el('scrub'); scrub.max = N - 1;
  const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  let i = 0, playing = !matchMedia('(prefers-reduced-motion: reduce)').matches, speed = 1, camX = 0, last = 0, acc = 0;
  const S = 1.9, GY = 360, ph = el('phases');
  marks.forEach(([k, name], n) => { const b = document.createElement('button'); b.type = 'button'; b.textContent = CAPS[name][0];
    b.onclick = () => { i = k; acc = 0; draw(); }; b.id = 'ph' + n; ph.appendChild(b); });
  const phaseAt = k => { let n = 0; marks.forEach((m, j) => { if (k >= m[0]) n = j; }); return n; };
  function draw(){
    const f = F[Math.min(i, N - 1)], P = f.p, W = cv.width, H = cv.height;
    const cx = P.reduce((a, p) => a + p[0], 0) / P.length; camX += (cx - camX) * (playing ? 0.12 : 1);
    const X = x => W / 2 + (x - camX) * S, Y = z => GY - z * S;
    ctx.fillStyle = css('--mat'); ctx.fillRect(0, 0, W, H);
    ctx.strokeStyle = css('--mat-line'); ctx.lineWidth = 1; ctx.font = '11px "IBM Plex Mono", monospace'; ctx.fillStyle = css('--muted');
    const x0 = Math.floor((camX - W / 2 / S) / 50) * 50;
    for (let x = x0; x < camX + W / 2 / S + 50; x += 50) { ctx.beginPath(); ctx.moveTo(X(x), 0); ctx.lineTo(X(x), GY); ctx.stroke();
      if (x % 100 === 0) ctx.fillText((x / 10) + ' cm', X(x) + 4, GY + 16); }
    for (let z = 50; z < 200; z += 50) { ctx.beginPath(); ctx.moveTo(0, Y(z)); ctx.lineTo(W, Y(z)); ctx.stroke(); }
    ctx.strokeStyle = css('--ink'); ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(0, GY + 1); ctx.lineTo(W, GY + 1); ctx.stroke();
    for (let k = 0; k < 9; k++) { const a = P[k], b = P[k + 1];
      ctx.strokeStyle = css(k % 2 ? '--link-b' : '--link-a'); ctx.lineWidth = 16 * S * 0.86;
      const ux = (b[0] - a[0]) / 48, uz = (b[1] - a[1]) / 48;
      ctx.lineCap = 'butt'; ctx.beginPath(); ctx.moveTo(X(a[0] + ux * 6), Y(a[1] + uz * 6)); ctx.lineTo(X(b[0] - ux * 6), Y(b[1] - uz * 6)); ctx.stroke();
      if (k === 8) { const ex = a[0] + ux * 34 - uz * 13, ez = a[1] + uz * 34 + ux * 13;
        ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.arc(X(ex), Y(ez), 7 * S, 0, 7); ctx.fill();
        ctx.fillStyle = css('--rubber'); ctx.beginPath(); ctx.arc(X(ex + ux * 3), Y(ez + uz * 3), 3.2 * S, 0, 7); ctx.fill(); } }
    for (let k = 0; k < 10; k++) { ctx.fillStyle = (k === 0 || k === 9) ? css('--latch') : css('--rubber');
      ctx.beginPath(); ctx.arc(X(P[k][0]), Y(P[k][1]), 9 * S, 0, 7); ctx.fill(); }
    if (f.l) { ctx.strokeStyle = css('--ok'); ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(X(P[0][0]), Y(P[0][1]), 15 * S, 0, 7); ctx.stroke(); }
    const n = phaseAt(i), name = marks[n][1];
    el('now').textContent = CAPS[name][0]; el('cap').textContent = CAPS[name][1];
    marks.forEach((m, j) => el('ph' + j).setAttribute('aria-pressed', j === n));
    el('rt').textContent = (i * dt).toFixed(1); el('ra').textContent = f.a.toFixed(1); el('rl').textContent = f.l ? 'SHUT (wheel)' : 'open';
    scrub.value = i;
  }
  function tick(t){ if (playing) { acc += (t - last) / 1000 * speed; while (acc >= dt) { acc -= dt; i = (i + 1) % N; } draw(); } last = t; requestAnimationFrame(tick); }
  el('play').onclick = () => { playing = !playing; el('play').textContent = playing ? 'Pause' : 'Play'; last = performance.now(); };
  el('slow').onclick = () => { speed = speed === 1 ? 0.5 : 1; el('slow').setAttribute('aria-pressed', speed !== 1); };
  scrub.oninput = () => { i = +scrub.value; acc = 0; draw(); };
  if (!playing) { el('play').textContent = 'Play'; i = Math.min(N - 1, (marks.find(m => m[1] === 'roll') || [0])[0] + 20); }
  camX = F[i].p[4][0]; draw(); requestAnimationFrame(t => { last = t; tick(t); });
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', draw);
})();
