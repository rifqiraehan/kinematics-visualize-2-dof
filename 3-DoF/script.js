function deg2rad(d) { return d * Math.PI / 180 }
function rad2deg(r) { return r * 180 / Math.PI }
function clamp(x, a, b) { return Math.max(a, Math.min(b, x)) }

const a1El = document.getElementById('a1');
const a2El = document.getElementById('a2');
const a3El = document.getElementById('a3'); // New: a3 element
const theta1El = document.getElementById('theta1');
const theta2El = document.getElementById('theta2');
const theta3El = document.getElementById('theta3'); // New: theta3 element
const pxEl = document.getElementById('px');
const pyEl = document.getElementById('py');
const phiEl = document.getElementById('phi'); // New: phi element
const fkResultEl = document.getElementById('fkResult');
const ik1_t1 = document.getElementById('ik1_t1');
const ik1_t2 = document.getElementById('ik1_t2');
const ik1_t3 = document.getElementById('ik1_t3'); // New: theta3 result 1
const ik2_t1 = document.getElementById('ik2_t1');
const ik2_t2 = document.getElementById('ik2_t2');
const ik2_t3 = document.getElementById('ik2_t3'); // New: theta3 result 2
const gripPulse = document.getElementById('gripPulse');
const gripMmEl = document.getElementById('gripMm');
const gripPulseOpenEl = document.getElementById('gripPulseOpen');
const gripPulseClosedEl = document.getElementById('gripPulseClosed');
const gripMmOpenEl = document.getElementById('gripMmOpen');
const debugEl = document.getElementById('debug');
const offsetToggleEl = document.getElementById('offsetToggle');
let isOffsetEnabled = offsetToggleEl.checked;

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
    J2: { minAngle: -90, maxAngle: 90, minPulse: 500, maxPulse: 2500, channel: 2 },
    J3: { minAngle: -90, maxAngle: 90, minPulse: 500, maxPulse: 2500, channel: 3 } // New: J3 params
};

const COMMAND_SPEED = 1000;

function getOffset() {
  return isOffsetEnabled ? 90 : 0;
}

function updateRobotCommand(t1, t2, t3) { // Updated to accept t3
  const p1 = angleToPulse(t1,
                          SERVO_PARAMS.J1.minAngle, SERVO_PARAMS.J1.maxAngle,
                          SERVO_PARAMS.J1.minPulse, SERVO_PARAMS.J1.maxPulse);

  const p2 = angleToPulse(t2,
                          SERVO_PARAMS.J2.minAngle, SERVO_PARAMS.J2.maxAngle,
                          SERVO_PARAMS.J2.minPulse, SERVO_PARAMS.J2.maxPulse);

  const p3 = angleToPulse(t3, // New: p3 calculation
                          SERVO_PARAMS.J3.minAngle, SERVO_PARAMS.J3.maxAngle,
                          SERVO_PARAMS.J3.minPulse, SERVO_PARAMS.J3.maxPulse);

  const cmd1 = `#${SERVO_PARAMS.J1.channel} P${p1} S${COMMAND_SPEED}`;
  const cmd2 = `#${SERVO_PARAMS.J2.channel} P${p2} S${COMMAND_SPEED}`;
  const cmd3 = `#${SERVO_PARAMS.J3.channel} P${p3} S${COMMAND_SPEED}`; // New: cmd3
  const command = `${cmd1} ${cmd2} ${cmd3}`; // Updated command string

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

function drawArm(theta1_deg, theta2_deg, theta3_deg, a1, a2, a3) { // Updated to accept a3, theta3
  clearCanvas();

  ctx.save();
  ctx.translate(panX, panY);
  ctx.scale(zoom, zoom);

  const w = canvas.width, h = canvas.height;
  const maxReach = Number(a1) + Number(a2) + Number(a3); // Max reach updated
  const margin = 40;
  const scale = Math.min((w - margin * 2) / (2 * maxReach), (h - margin * 2) / (maxReach));

  // origin tetap di bawah
  const origin = { x: w / 2, y: h - margin };
  drawGrid(maxReach, origin, scale);

  const offset = getOffset();
  const t1 = deg2rad(theta1_deg + offset); // Add 90 degrees offset for vertical zero
  const t2 = deg2rad(theta2_deg);
  const t3 = deg2rad(theta3_deg);

  // Joint 1 position (origin)
  const x0 = 0;
  const y0 = 0;

  // Joint 2 position (End of Link 1)
  const x1 = a1 * Math.cos(t1);
  const y1 = a1 * Math.sin(t1);

  // Joint 3 position (End of Link 2)
  const x2 = x1 + a2 * Math.cos(t1 + t2);
  const y2 = y1 + a2 * Math.sin(t1 + t2);

  // End Effector position (End of Link 3)
  const x3 = x2 + a3 * Math.cos(t1 + t2 + t3);
  const y3 = y2 + a3 * Math.sin(t1 + t2 + t3);

  // Scaled coordinates for drawing
  const sx1 = origin.x + x1 * scale;
  const sy1 = origin.y - y1 * scale;
  const sx2 = origin.x + x2 * scale;
  const sy2 = origin.y - y2 * scale;
  const sx3 = origin.x + x3 * scale; // End effector
  const sy3 = origin.y - y3 * scale; // End effector

  // Max Reach Circle
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

  // link 3 (New)
  ctx.beginPath();
  ctx.moveTo(sx2, sy2);
  ctx.lineTo(sx3, sy3);
  ctx.strokeStyle = '#9d2dff'; // New color for Link 3
  ctx.lineWidth = 8;
  ctx.lineCap = 'round';
  ctx.stroke();

  // joint 3 (New)
  ctx.beginPath(); ctx.arc(sx2, sy2, 6, 0, Math.PI * 2); ctx.fillStyle = '#ff7b00'; ctx.fill();


  ctx.beginPath();
  ctx.strokeStyle = 'rgba(255, 92, 58, 0.5)';
  ctx.lineWidth = 1;
  ctx.setLineDash([5, 5]);

  // Horizontal Guide (Py)
  ctx.moveTo(origin.x, sy3);
  ctx.lineTo(sx3, sy3);

  // Vertical Guide (Px)
  ctx.moveTo(sx3, origin.y);
  ctx.lineTo(sx3, sy3);

  ctx.stroke();
  ctx.setLineDash([]);
  ctx.save();
  ctx.fillStyle = '#222';
  ctx.font = '10px Arial';

  const textOffset = 8;

  const midY = (origin.y + sy3) / 2;
  ctx.textAlign = 'left';
  ctx.textBaseline = 'middle';
  ctx.fillText(`Px: ${x3.toFixed(1)}`, sx3 + textOffset, midY);

  const midX = (origin.x + sx3) / 2;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.fillText(`Py: ${y3.toFixed(1)}`, midX, sy3 + textOffset);

  ctx.restore();

  // end effector (End of Link 3)
  ctx.beginPath(); ctx.arc(sx3, sy3, 6, 0, Math.PI * 2); ctx.fillStyle = '#ffb020'; ctx.fill();

  ctx.save();
  ctx.strokeStyle = '#222';
  ctx.fillStyle = '#222';
  ctx.lineWidth = 1;
  ctx.font = '10px Arial';

  const t1_rad = deg2rad(theta1_deg + offset);
  const t2_rad = deg2rad(theta2_deg);
  const t3_rad = deg2rad(theta3_deg); // New: t3 rad

  const r1 = 30;
  const startAngle1 = deg2rad(90);
  const endAngle1 = t1_rad;

  const textAngle1 = (startAngle1 + endAngle1) / 2;
  const textX1 = origin.x + (r1 * scale + 15) * Math.cos(textAngle1);
  const textY1 = origin.y - (r1 * scale + 15) * Math.sin(textAngle1);
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(`θ1: ${theta1_deg.toFixed(1)}°`, textX1, textY1);

  const r2 = 30;
  const t1_plus_t2 = t1_rad + t2_rad;

  const textAngle2 = (t1_rad + t1_plus_t2) / 2;
  const textX2 = sx1 + (r2 * scale + 15) * Math.cos(textAngle2);
  const textY2 = sy1 - (r2 * scale + 15) * Math.sin(textAngle2);
  ctx.fillText(`θ2: ${theta2_deg.toFixed(1)}°`, textX2, textY2);

  const r3 = 30; // New: Angle text for theta 3
  const t1_plus_t2_plus_t3 = t1_rad + t2_rad + t3_rad;

  const textAngle3 = (t1_plus_t2 + t1_plus_t2_plus_t3) / 2;
  const textX3 = sx2 + (r3 * scale + 15) * Math.cos(textAngle3);
  const textY3 = sy2 - (r3 * scale + 15) * Math.sin(textAngle3);
  ctx.fillText(`θ3: ${theta3_deg.toFixed(1)}°`, textX3, textY3);

  ctx.restore();

  ctx.font = '12px Arial'; ctx.fillStyle = '#222';
  ctx.fillText(`Px: ${x3.toFixed(1)} mm`, 8, 18);
  ctx.fillText(`Py: ${y3.toFixed(1)} mm`, 8, 36);
  ctx.fillText(`φ: ${rad2deg(t1_plus_t2_plus_t3 - deg2rad(offset)).toFixed(1)}°`, 8, 54); // Orientation
  ctx.restore();
}

function drawGridDefault() {
  const w = canvas.width, h = canvas.height;
  const margin = 40;
  const maxReach = 120 + 210 + 100; // Default a1+a2+a3
  const scale = 1;

  const origin = { x: w / 2, y: h - margin };
  drawGrid(maxReach, origin, scale);
}

offsetToggleEl.addEventListener('change', () => {
  isOffsetEnabled = offsetToggleEl.checked;
  redraw();
  // If IK was run, re-run to update angles with new offset
  const a1 = Number(a1El.value); const a2 = Number(a2El.value); const a3 = Number(a3El.value);
  const px = Number(pxEl.value); const py = Number(pyEl.value); const phi = Number(phiEl.value);
  const r = Math.hypot(px, py);
  if (r > 1e-6) {
    const sols = computeInverse(a1, a2, a3, px, py, phi);
    if (sols) {
        sols.forEach(s => { s.t1 = ((s.t1 + 180) % 360) - 180; s.t2 = ((s.t2 + 180) % 360) - 180; s.t3 = ((s.t3 + 180) % 360) - 180; });
        ik1_t1.textContent = sols[0].t1.toFixed(2); ik1_t2.textContent = sols[0].t2.toFixed(2); ik1_t3.textContent = sols[0].t3.toFixed(2);
        ik2_t1.textContent = sols[1].t1.toFixed(2); ik2_t2.textContent = sols[1].t2.toFixed(2); ik2_t3.textContent = sols[1].t3.toFixed(2);
    }
  }
});

// FK - implements KinematicsSolver.Forward from C#
function computeForward(a1, a2, a3, theta1_deg, theta2_deg, theta3_deg) {
  const offset = getOffset();
  const t1 = deg2rad(theta1_deg + offset); // Link 1 angle (base to X axis)
  const t2 = deg2rad(theta2_deg);          // Link 2 angle (relative to Link 1)
  const t3 = deg2rad(theta3_deg);          // Link 3 angle (relative to Link 2)

  const px = a1 * Math.cos(t1) + a2 * Math.cos(t1 + t2) + a3 * Math.cos(t1 + t2 + t3);
  const py = a1 * Math.sin(t1) + a2 * Math.sin(t1 + t2) + a3 * Math.sin(t1 + t2 + t3);
  const phi = rad2deg(t1 + t2 + t3) - offset; // End effector orientation in standard frame
  return { px, py, phi };
}

// IK - implements KinematicsSolver.Inverse from C#
function computeInverse(a1, a2, a3, qx, qy, phiDeg) {
  const phi = deg2rad(phiDeg + getOffset()); // Convert desired end effector angle to angle relative to X-axis

  // Wrist point (P) calculation
  const px = qx - a3 * Math.cos(phi);
  const py = qy - a3 * Math.Sin(phi);

  const r2 = px * px + py * py;
  const denom = 2 * a1 * a2;

  // Check reachability for the wrist point
  let cosT2 = (r2 - a1 * a1 - a2 * a2) / denom;
  if (cosT2 < -1 || cosT2 > 1) return null;

  // Theta 2 solutions (Elbow up/down)
  const t2a = Math.acos(cosT2);
  const t2b = -Math.acos(cosT2);

  // Theta 1 solutions
  const t1a_std = Math.atan2(py, px) - Math.atan2(a2 * Math.sin(t2a), a1 + a2 * Math.cos(t2a));
  const t1b_std = Math.atan2(py, px) - Math.atan2(a2 * Math.sin(t2b), a1 + a2 * Math.cos(t2b));

  // Theta 3 solutions
  const t3a = phi - (t1a_std + t2a);
  const t3b = phi - (t1b_std + t2b);

  // Map standard angle (X-axis zero) back to desired angle (Y-axis zero)
  const offset = getOffset();
  const t1a_offset = rad2deg(t1a_std) - offset;
  const t1b_offset = rad2deg(t1b_std) - offset;

  return [
      { t1: t1a_offset, t2: rad2deg(t2a), t3: rad2deg(t3a) },
      { t1: t1b_offset, t2: rad2deg(t2b), t3: rad2deg(t3b) }
  ];
}

// event handlers
document.getElementById('btnFK').addEventListener('click', () => {
  const a1 = Number(a1El.value); const a2 = Number(a2El.value); const a3 = Number(a3El.value);
  const t1 = Number(theta1El.value); const t2 = Number(theta2El.value); const t3 = Number(theta3El.value);
  const { px, py, phi } = computeForward(a1, a2, a3, t1, t2, t3);

  fkResultEl.textContent = `Px = ${px.toFixed(2)} mm, Py = ${py.toFixed(2)} mm, φ = ${phi.toFixed(2)}°`;

  pxEl.value = px.toFixed(2);
  pyEl.value = py.toFixed(2);
  phiEl.value = phi.toFixed(2); // Update phi input

  const command = updateRobotCommand(t1, t2, t3);

  drawArm(t1, t2, t3, a1, a2, a3);
  debugEl.textContent = `Forward: θ1=${t1}°, θ2=${t2}°, θ3=${t3}° → Px=${px.toFixed(2)}, Py=${py.toFixed(2)}, φ=${phi.toFixed(2)}°.\nCOMMAND:\n${command}`;
});

document.getElementById('btnFKReset').addEventListener('click', () => {
  theta1El.value = 0; theta2El.value = 0; theta3El.value = 0; // Reset theta3
  fkResultEl.textContent = 'Px = -, Py = -, φ = -'; clearCanvas(); drawGridDefault();
});

// IK compute
document.getElementById('btnIK').addEventListener('click', () => {
  const a1 = Number(a1El.value); const a2 = Number(a2El.value); const a3 = Number(a3El.value);
  const px = Number(pxEl.value); const py = Number(pyEl.value); const phi = Number(phiEl.value);

  // Check reachability based on wrist point (P) reach
  const maxReachWrist = a1 + a2;
  const minReachWrist = Math.abs(a1 - a2);
  const qx = px - a3 * Math.cos(deg2rad(phi + getOffset()));
  const qy = py - a3 * Math.sin(deg2rad(phi + getOffset()));
  const r = Math.hypot(qx, qy);

  if (r > maxReachWrist + 1e-6 || r < minReachWrist - 1e-6) {
    ik1_t1.textContent = ik1_t2.textContent = ik1_t3.textContent = ik2_t1.textContent = ik2_t2.textContent = ik2_t3.textContent = '-';
    debugEl.textContent = 'Target (Wrist) di luar jangkauan robot!';
    alert('Target (Wrist) di luar jangkauan robot (lebih besar dari a1+a2 atau kurang dari |a1-a2|)');
    return;
  }

  const sols = computeInverse(a1, a2, a3, px, py, phi);
  if (!sols) {
      ik1_t1.textContent = ik1_t2.textContent = ik1_t3.textContent = ik2_t1.textContent = ik2_t2.textContent = ik2_t3.textContent = '-';
      debugEl.textContent = 'Target di luar jangkauan robot (cos(θ2) tidak valid)!';
      alert('Target di luar jangkauan robot (Perhitungan cos(θ2) tidak valid)');
      return;
  }

  // normalize angles to -180..180
  sols.forEach(s => { s.t1 = ((s.t1 + 180) % 360) - 180; s.t2 = ((s.t2 + 180) % 360) - 180; s.t3 = ((s.t3 + 180) % 360) - 180; });
  ik1_t1.textContent = sols[0].t1.toFixed(2); ik1_t2.textContent = sols[0].t2.toFixed(2); ik1_t3.textContent = sols[0].t3.toFixed(2);
  ik2_t1.textContent = sols[1].t1.toFixed(2); ik2_t2.textContent = sols[1].t2.toFixed(2); ik2_t3.textContent = sols[1].t3.toFixed(2);
  debugEl.textContent = `IK computed. Sol1: θ1=${sols[0].t1.toFixed(2)}, θ2=${sols[0].t2.toFixed(2)}, θ3=${sols[0].t3.toFixed(2)} | Sol2: θ1=${sols[1].t1.toFixed(2)}, θ2=${sols[1].t2.toFixed(2)}, θ3=${sols[1].t3.toFixed(2)}`;
});

// Run solutions (direct jump to pose)
document.getElementById('btnRun1').addEventListener('click', () => {
  const t1 = Number(ik1_t1.textContent);
  const t2 = Number(ik1_t2.textContent);
  const t3 = Number(ik1_t3.textContent); // New: t3
  if (isNaN(t1)) { alert('Hitung IK terlebih dahulu dan pastikan solusi valid.'); return; }
  theta1El.value = t1; theta2El.value = t2; theta3El.value = t3;
  const a1 = Number(a1El.value); const a2 = Number(a2El.value); const a3 = Number(a3El.value);
  const f = computeForward(a1, a2, a3, t1, t2, t3);
  const command = updateRobotCommand(t1, t2, t3);
  fkResultEl.textContent = `Px = ${f.px.toFixed(2)} mm, Py = ${f.py.toFixed(2)} mm, φ = ${f.phi.toFixed(2)}°`;
  drawArm(t1, t2, t3, a1, a2, a3);
  debugEl.textContent = `IK Sol 1 Run. COMMAND: ${command}`;
});

document.getElementById('btnRun2').addEventListener('click', () => {
  const t1 = Number(ik2_t1.textContent);
  const t2 = Number(ik2_t2.textContent);
  const t3 = Number(ik2_t3.textContent); // New: t3
  if (isNaN(t1)) { alert('Hitung IK terlebih dahulu dan pastikan solusi valid.'); return; }
  theta1El.value = t1; theta2El.value = t2; theta3El.value = t3;
  const a1 = Number(a1El.value); const a2 = Number(a2El.value); const a3 = Number(a3El.value);
  const f = computeForward(a1, a2, a3, t1, t2, t3);
  const command = updateRobotCommand(t1, t2, t3);
  fkResultEl.textContent = `Px = ${f.px.toFixed(2)} mm, Py = ${f.py.toFixed(2)} mm, φ = ${f.phi.toFixed(2)}°`;
  drawArm(t1, t2, t3, a1, a2, a3);
  debugEl.textContent = `IK Sol 2 Run. COMMAND: ${command}`;
});

// reset IK inputs
document.getElementById('btnIKReset').addEventListener('click', () => {
  pxEl.value = 0; pyEl.value = 0; phiEl.value = 0; // Reset phi
  ik1_t1.textContent = ik1_t2.textContent = ik1_t3.textContent = ik2_t1.textContent = ik2_t2.textContent = ik2_t3.textContent = '-';
  debugEl.textContent = 'Reset IK'; clearCanvas(); drawGridDefault();
});

document.getElementById('btnClear').addEventListener('click', () => { clearCanvas(); drawGridDefault(); debugEl.textContent = 'Canvas dibersihkan'; });

// snapshot
document.getElementById('btnSnapshot').addEventListener('click', () => {
  const link = document.createElement('a');
  link.download = 'robot-3dof.png';
  link.href = canvas.toDataURL('image/png');
  link.click();
});

// initial draw
clearCanvas(); drawGridDefault();

// Pan & Zoom Support (unchanged)
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
  const a3 = Number(a3El.value); // New: a3
  const t1 = Number(theta1El.value);
  const t2 = Number(theta2El.value);
  const t3 = Number(theta3El.value); // New: t3
  drawArm(t1, t2, t3, a1, a2, a3);
}