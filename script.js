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

function angleToPulse(angleDeg, minAngleDeg, maxAngleDeg, minPulse, maxPulse) {
  const angle = clamp(angleDeg, minAngleDeg, maxAngleDeg);
  const pulse = minPulse +
                (angle - minAngleDeg) * ((maxPulse - minPulse) / (maxAngleDeg - minAngleDeg));
  return Math.round(pulse);
}

const SERVO_PARAMS = {
    J1: { minAngle: -90, maxAngle: 90, minPulse: 500, maxPulse: 2500, channel: 0 },
    J2: { minAngle: -90, maxAngle: 90, minPulse: 500, maxPulse: 2500, channel: 2 }
};

const COMMAND_SPEED = 1000;

function updateRobotCommand(t1, t2) {
  const p1 = angleToPulse(t1,
                          SERVO_PARAMS.J1.minAngle, SERVO_PARAMS.J1.maxAngle,
                          SERVO_PARAMS.J1.minPulse, SERVO_PARAMS.J1.maxPulse);

  const p2 = angleToPulse(t2,
                          SERVO_PARAMS.J2.minAngle, SERVO_PARAMS.J2.maxAngle,
                          SERVO_PARAMS.J2.minPulse, SERVO_PARAMS.J2.maxPulse);

  const cmd1 = `#${SERVO_PARAMS.J1.channel} P${p1} S${COMMAND_SPEED}`;
  const cmd2 = `#${SERVO_PARAMS.J2.channel} P${p2} S${COMMAND_SPEED}`;
  const command = `${cmd1} ${cmd2}`;

  return command;
}

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

  const t1 = deg2rad(theta1_deg + 90); // Add 90 degrees offset for vertical zero
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

  ctx.beginPath();
  ctx.strokeStyle = 'rgba(255, 92, 58, 0.5)';
  ctx.lineWidth = 1;
  ctx.setLineDash([5, 5]);

  // Horizontal Guide (Py)
  ctx.moveTo(origin.x, sy2);
  ctx.lineTo(sx2, sy2);

  // Vertical Guide (Px)
  ctx.moveTo(sx2, origin.y);
  ctx.lineTo(sx2, sy2);

  ctx.stroke();
  ctx.setLineDash([]);
  ctx.save();
  ctx.fillStyle = '#222';
  ctx.font = '10px Arial';

  const textOffset = 8;

  const midY = (origin.y + sy2) / 2;
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  ctx.fillText(`Px: ${x2.toFixed(1)}`, sx2 + textOffset, midY);

  const midX = (origin.x + sx2) / 2;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.fillText(`Py: ${y2.toFixed(1)}`, midX, sy2 + textOffset);

  ctx.restore();

  // end effector
  ctx.beginPath(); ctx.arc(sx2, sy2, 6, 0, Math.PI * 2); ctx.fillStyle = '#ffb020'; ctx.fill();

  ctx.save();
  ctx.strokeStyle = '#222';
  ctx.fillStyle = '#222';
  ctx.lineWidth = 1;
  ctx.font = '10px Arial';

  // Get angles in radians (using the offsetted t1 for true rotation)
  const t1_rad = deg2rad(theta1_deg + 90);
  const t2_rad = deg2rad(theta2_deg);

  const r1 = 30; // Radius of arc
  const startAngle1 = deg2rad(90); // Start from vertical (your 0 degree reference)
  const endAngle1 = t1_rad;

  // Position text slightly outside the arc
  const textAngle1 = (startAngle1 + endAngle1) / 2;
  const textX1 = origin.x + (r1 * scale + 15) * Math.cos(textAngle1);
  const textY1 = origin.y - (r1 * scale + 15) * Math.sin(textAngle1);
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(`θ1: ${theta1_deg.toFixed(1)}°`, textX1, textY1);

  const r2 = 30; // Radius of arc
  const t1_plus_t2 = t1_rad + t2_rad;

  const textAngle2 = (t1_rad + t1_plus_t2) / 2;
  const textX2 = sx1 + (r2 * scale + 15) * Math.cos(textAngle2);
  const textY2 = sy1 - (r2 * scale + 15) * Math.sin(textAngle2);
  ctx.fillText(`θ2: ${theta2_deg.toFixed(1)}°`, textX2, textY2);

  ctx.restore();

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
  const t1 = deg2rad(theta1_deg + 90); // Add 90 degrees offset for vertical zero
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

  const t1a_std = Math.atan2(py, px) - Math.atan2(a2 * Math.sin(t2a), a1 + a2 * Math.cos(t2a));
  const t1b_std = Math.atan2(py, px) - Math.atan2(a2 * Math.sin(t2b), a1 + a2 * Math.cos(t2b));

  // Map standard angle (X-axis zero) back to desired angle (Y-axis zero)
  const t1a_offset = rad2deg(t1a_std) - 90;
  const t1b_offset = rad2deg(t1b_std) - 90;

  return [{ t1: t1a_offset, t2: rad2deg(t2a) }, { t1: t1b_offset, t2: rad2deg(t2b) }];
}

// event handlers
document.getElementById('btnFK').addEventListener('click', () => {
  const a1 = Number(a1El.value); const a2 = Number(a2El.value);
  const t1 = Number(theta1El.value); const t2 = Number(theta2El.value);
  const { px, py } = computeForward(a1, a2, t1, t2);

  fkResultEl.textContent = `Px = ${px.toFixed(2)} mm, Py = ${py.toFixed(2)} mm`;

  pxEl.value = px.toFixed(2);
  pyEl.value = py.toFixed(2);

  const command = updateRobotCommand(t1, t2);

  drawArm(t1, t2, a1, a2);
  debugEl.textContent = `Forward: θ1=${t1}°, θ2=${t2}° → Px=${px.toFixed(2)}, Py=${py.toFixed(2)}. COMMAND: ${command}`;
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
  const command = updateRobotCommand(t1, t2);
  fkResultEl.textContent = `Px = ${f.px.toFixed(2)} mm, Py = ${f.py.toFixed(2)} mm`;
  drawArm(t1, t2, a1, a2);
  debugEl.textContent = `IK Sol 1 Run. COMMAND: ${command}`;
});

document.getElementById('btnRun2').addEventListener('click', () => {
  const t1 = Number(ik2_t1.textContent);
  const t2 = Number(ik2_t2.textContent);
  if (isNaN(t1)) { alert('Hitung IK terlebih dahulu dan pastikan solusi valid.'); return; }
  theta1El.value = t1; theta2El.value = t2;
  const a1 = Number(a1El.value); const a2 = Number(a2El.value);
  const f = computeForward(a1, a2, t1, t2);
  const command = updateRobotCommand(t1, t2);
  fkResultEl.textContent = `Px = ${f.px.toFixed(2)} mm, Py = ${f.py.toFixed(2)} mm`;
  drawArm(t1, t2, a1, a2);
  debugEl.textContent = `IK Sol 2 Run. COMMAND: ${command}`;
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