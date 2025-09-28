function deg2rad(d) { return d * Math.PI / 180 }
function rad2deg(r) { return r * 180 / Math.PI }
function clamp(x, a, b) { return Math.max(a, Math.min(b, x)) }

const a1El = document.getElementById('a1');
const a2El = document.getElementById('a2');
const theta1El = document.getElementById('theta1');
const theta2El = document.getElementById('theta2');
const pxEl = document.getElementById('px');
const pyEl = document.getElementById('py');
const fkResultEl = document.getElementById('fkResult');
const ik1_t1 = document.getElementById('ik1_t1');
const ik1_t2 = document.getElementById('ik1_t2');
const ik2_t1 = document.getElementById('ik2_t1');
const ik2_t2 = document.getElementById('ik2_t2');
const gripPulse = document.getElementById('gripPulse');
const gripMmEl = document.getElementById('gripMm');
const gripPulseOpenEl = document.getElementById('gripPulseOpen');
const gripPulseClosedEl = document.getElementById('gripPulseClosed');
const gripMmOpenEl = document.getElementById('gripMmOpen');
const debugEl = document.getElementById('debug');

const canvas = document.getElementById('robotCanvas');
const ctx = canvas.getContext('2d');

function gripPulseToMm(pulse) {
  const pulseOpen = Number(gripPulseOpenEl.value);
  const pulseClosed = Number(gripPulseClosedEl.value);
  const mmOpen = Number(gripMmOpenEl.value);

  const t = (pulse - pulseOpen) / (pulseClosed - pulseOpen);
  const mm = mmOpen * (1 - t);
  return clamp(mm, 0, mmOpen);
}

autoUpdateGrip();

function autoUpdateGrip() {
  const p = Number(gripPulse.value);
  gripMmEl.textContent = gripPulseToMm(p).toFixed(0);
}
[gripPulse, gripPulseOpenEl, gripPulseClosedEl, gripMmOpenEl].forEach(el => {
  el.addEventListener('input', autoUpdateGrip);
});

// drawing helpers
function clearCanvas() { ctx.clearRect(0, 0, canvas.width, canvas.height) }

function drawGrid(maxReach, origin, scale) {
  const step = 20; // ukuran grid dalam mm

  ctx.beginPath();
  // garis vertikal
  for (let x = -maxReach; x <= maxReach; x += step) {
    ctx.moveTo(origin.x + x * scale, origin.y - maxReach * scale);
    ctx.lineTo(origin.x + x * scale, origin.y + maxReach * scale);
  }
  // garis horizontal
  for (let y = -maxReach; y <= maxReach; y += step) {
    ctx.moveTo(origin.x - maxReach * scale, origin.y - y * scale);
    ctx.lineTo(origin.x + maxReach * scale, origin.y - y * scale);
  }
  ctx.strokeStyle = '#eef3fb';
  ctx.lineWidth = 1;
  ctx.stroke();

  // kotak boundary untuk container grid
  ctx.strokeStyle = '#ccc';
  ctx.strokeRect(
    origin.x - maxReach * scale,
    origin.y - maxReach * scale,
    2 * maxReach * scale,
    2 * maxReach * scale
  );
}

function drawArm(theta1_deg, theta2_deg, a1, a2) {
  clearCanvas();

  ctx.save();
  ctx.translate(panX, panY);
  ctx.scale(zoom, zoom);

  const w = canvas.width, h = canvas.height;
  const maxReach = Number(a1) + Number(a2);
  const margin = 40;
  const scale = Math.min((w - margin * 2) / maxReach, (h - margin * 2) / maxReach);

  // origin tetap di bawah
  const origin = { x: w / 2, y: h - margin };
  drawGrid(maxReach, origin, scale);

  const t1 = deg2rad(theta1_deg);
  const t2 = deg2rad(theta2_deg);

  const x1 = a1 * Math.cos(t1);
  const y1 = a1 * Math.sin(t1);
  const x2 = x1 + a2 * Math.cos(t1 + t2);
  const y2 = y1 + a2 * Math.sin(t1 + t2);

  const sx1 = origin.x + x1 * scale;
  const sy1 = origin.y - y1 * scale;
  const sx2 = origin.x + x2 * scale;
  const sy2 = origin.y - y2 * scale;

  // draw reach circle
  ctx.beginPath();
  ctx.arc(origin.x, origin.y, maxReach * scale, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(30,58,138,0.12)';
  ctx.lineWidth = 1;
  ctx.stroke();

  // link 1
  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(sx1, sy1);
  ctx.strokeStyle = '#0b84ff';
  ctx.lineWidth = 8;
  ctx.lineCap = 'round';
  ctx.stroke();

  // joint 1
  ctx.beginPath(); ctx.arc(origin.x, origin.y, 6, 0, Math.PI * 2); ctx.fillStyle = '#111'; ctx.fill();

  // link 2
  ctx.beginPath();
  ctx.moveTo(sx1, sy1);
  ctx.lineTo(sx2, sy2);
  ctx.strokeStyle = '#ff5c3a';
  ctx.lineWidth = 8;
  ctx.lineCap = 'round';
  ctx.stroke();

  // joint 2
  ctx.beginPath(); ctx.arc(sx1, sy1, 6, 0, Math.PI * 2); ctx.fillStyle = '#0a8a5f'; ctx.fill();

  // end effector
  ctx.beginPath(); ctx.arc(sx2, sy2, 6, 0, Math.PI * 2); ctx.fillStyle = '#ffb020'; ctx.fill();

  // labels
  ctx.font = '12px Arial'; ctx.fillStyle = '#222';
  ctx.fillText(`Px: ${x2.toFixed(1)} mm`, 8, 18);
  ctx.fillText(`Py: ${y2.toFixed(1)} mm`, 8, 36);
  ctx.restore();
}

function drawGridDefault() {
  const w = canvas.width, h = canvas.height;
  const margin = 40;
  const maxReach = 400; // nilai default biar ada grid
  const scale = 1;      // default, tidak disesuaikan

  const origin = { x: w / 2, y: h - margin };
  drawGrid(maxReach, origin, scale);
}

// FK
function computeForward(a1, a2, theta1_deg, theta2_deg) {
  const t1 = deg2rad(theta1_deg);
  const t2 = deg2rad(theta2_deg);
  const px = a1 * Math.cos(t1) + a2 * Math.cos(t1 + t2);
  const py = a1 * Math.sin(t1) + a2 * Math.sin(t1 + t2);
  return { px, py };
}

// IK (return solutions in degrees)
function computeInverse(a1, a2, px, py) {
  const r2 = px * px + py * py;
  const denom = 2 * a1 * a2;
  let cosT2 = (r2 - a1 * a1 - a2 * a2) / denom;
  // clamp to [-1,1]
  cosT2 = clamp(cosT2, -1, 1);
  const t2a = Math.acos(cosT2);
  const t2b = -Math.acos(cosT2);

  const t1a = Math.atan2(py, px) - Math.atan2(a2 * Math.sin(t2a), a1 + a2 * Math.cos(t2a));
  const t1b = Math.atan2(py, px) - Math.atan2(a2 * Math.sin(t2b), a1 + a2 * Math.cos(t2b));

  return [{ t1: rad2deg(t1a), t2: rad2deg(t2a) }, { t1: rad2deg(t1b), t2: rad2deg(t2b) }];
}

// event handlers
document.getElementById('btnFK').addEventListener('click', () => {
  const a1 = Number(a1El.value); const a2 = Number(a2El.value);
  const t1 = Number(theta1El.value); const t2 = Number(theta2El.value);
  const { px, py } = computeForward(a1, a2, t1, t2);
  fkResultEl.textContent = `Px = ${px.toFixed(2)} mm, Py = ${py.toFixed(2)} mm`;
  drawArm(t1, t2, a1, a2);
  debugEl.textContent = `Forward: θ1=${t1}°, θ2=${t2}° → Px=${px.toFixed(2)}, Py=${py.toFixed(2)}`;
});

document.getElementById('btnFKReset').addEventListener('click', () => {
  theta1El.value = 0; theta2El.value = 0; fkResultEl.textContent = 'Px = -, Py = -'; clearCanvas(); drawGridDefault();
});

// IK compute
document.getElementById('btnIK').addEventListener('click', () => {
  const a1 = Number(a1El.value); const a2 = Number(a2El.value);
  const px = Number(pxEl.value); const py = Number(pyEl.value);
  const maxReach = a1 + a2; const minReach = Math.abs(a1 - a2);
  const r = Math.hypot(px, py);
  if (r > maxReach + 1e-6 || r < minReach - 1e-6) {
    ik1_t1.textContent = ik1_t2.textContent = ik2_t1.textContent = ik2_t2.textContent = '-';
    debugEl.textContent = 'Target di luar jangkauan robot!';
    alert('Target di luar jangkauan robot (lebih besar dari a1+a2 atau kurang dari |a1-a2|)');
    return;
  }
  const sols = computeInverse(a1, a2, px, py);
  // normalize angles to -180..180
  sols.forEach(s => { s.t1 = ((s.t1 + 180) % 360) - 180; s.t2 = ((s.t2 + 180) % 360) - 180; });
  ik1_t1.textContent = sols[0].t1.toFixed(2); ik1_t2.textContent = sols[0].t2.toFixed(2);
  ik2_t1.textContent = sols[1].t1.toFixed(2); ik2_t2.textContent = sols[1].t2.toFixed(2);
  debugEl.textContent = `IK computed. Sol1: θ1=${sols[0].t1.toFixed(2)}, θ2=${sols[0].t2.toFixed(2)} | Sol2: θ1=${sols[1].t1.toFixed(2)}, θ2=${sols[1].t2.toFixed(2)}`;
});

// Run solutions (direct jump to pose)
document.getElementById('btnRun1').addEventListener('click', () => {
  const t1 = Number(ik1_t1.textContent);
  const t2 = Number(ik1_t2.textContent);
  if (isNaN(t1)) { alert('Hitung IK terlebih dahulu dan pastikan solusi valid.'); return; }
  theta1El.value = t1; theta2El.value = t2;
  const a1 = Number(a1El.value); const a2 = Number(a2El.value);
  const f = computeForward(a1, a2, t1, t2);
  fkResultEl.textContent = `Px = ${f.px.toFixed(2)} mm, Py = ${f.py.toFixed(2)} mm`;
  drawArm(t1, t2, a1, a2);
});

document.getElementById('btnRun2').addEventListener('click', () => {
  const t1 = Number(ik2_t1.textContent);
  const t2 = Number(ik2_t2.textContent);
  if (isNaN(t1)) { alert('Hitung IK terlebih dahulu dan pastikan solusi valid.'); return; }
  theta1El.value = t1; theta2El.value = t2;
  const a1 = Number(a1El.value); const a2 = Number(a2El.value);
  const f = computeForward(a1, a2, t1, t2);
  fkResultEl.textContent = `Px = ${f.px.toFixed(2)} mm, Py = ${f.py.toFixed(2)} mm`;
  drawArm(t1, t2, a1, a2);
});

// reset IK inputs
document.getElementById('btnIKReset').addEventListener('click', () => { pxEl.value = 0; pyEl.value = 0; ik1_t1.textContent = ik1_t2.textContent = ik2_t1.textContent = ik2_t2.textContent = '-'; debugEl.textContent = 'Reset IK'; clearCanvas(); drawGridDefault(); });

document.getElementById('btnClear').addEventListener('click', () => { clearCanvas(); drawGridDefault(); debugEl.textContent = 'Canvas dibersihkan'; });

// snapshot
document.getElementById('btnSnapshot').addEventListener('click', () => {
  const link = document.createElement('a');
  link.download = 'robot-2dof.png';
  link.href = canvas.toDataURL('image/png');
  link.click();
});

// initial draw
clearCanvas(); drawGridDefault();

// Pan & Zoom Support
let panX = 0, panY = 0;
let zoom = 1;
let isDragging = false;
let dragStartX = 0, dragStartY = 0;

canvas.addEventListener('mousedown', e => {
  isDragging = true;
  dragStartX = e.offsetX - panX;
  dragStartY = e.offsetY - panY;
});

canvas.addEventListener('mousemove', e => {
  if (isDragging) {
    panX = e.offsetX - dragStartX;
    panY = e.offsetY - dragStartY;
    redraw();
  }
});

canvas.addEventListener('mouseup', () => isDragging = false);
canvas.addEventListener('mouseleave', () => isDragging = false);

canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const scaleFactor = (e.deltaY < 0) ? 1.1 : 0.9;
  zoom *= scaleFactor;

  // Zoom ke arah pointer
  const mouseX = e.offsetX, mouseY = e.offsetY;
  panX = mouseX - (mouseX - panX) * scaleFactor;
  panY = mouseY - (mouseY - panY) * scaleFactor;

  redraw();
});

// Helper redraw dengan posisi terakhir
function redraw() {
  const a1 = Number(a1El.value);
  const a2 = Number(a2El.value);
  const t1 = Number(theta1El.value);
  const t2 = Number(theta2El.value);
  drawArm(t1, t2, a1, a2);
}