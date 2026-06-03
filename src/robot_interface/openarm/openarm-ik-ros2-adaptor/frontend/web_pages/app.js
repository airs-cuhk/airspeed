/**
 * app.js — Main module for IK Validation frontend.
 *
 * Initializes THREE.js scene, loads URDF, connects WebSocket,
 * and renders solver snapshots (joint angles, target markers, telemetry).
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
import { createFloorGrid, createWorldAxes, createJointAxesHelper, createTargetHandleMarker, createVRHeadMarker, createDebugPoseMarker } from '../3d_assets/scene_markers.js';

// ── Constants ──────────────────────────────────────────────────────────
const URDF_URL = (typeof CONFIG !== 'undefined' && CONFIG.URDF_PATH) || '../3d_assets/urdf/openarm_bimanual_copy.urdf';

const LEFT_JOINT_NAMES = Array.from({ length: 7 }, (_, i) => `openarm_left_joint${i + 1}`);
const RIGHT_JOINT_NAMES = Array.from({ length: 7 }, (_, i) => `openarm_right_joint${i + 1}`);
const LEFT_FINGER_JOINTS = ['openarm_left_finger_joint1', 'openarm_left_finger_joint2'];
const RIGHT_FINGER_JOINTS = ['openarm_right_finger_joint1', 'openarm_right_finger_joint2'];
const ALL_ARM_JOINTS = [...LEFT_JOINT_NAMES, ...RIGHT_JOINT_NAMES];
const ALL_FINGER_JOINTS = [...LEFT_FINGER_JOINTS, ...RIGHT_FINGER_JOINTS];

const DEFAULT_POSE_RAD = {
  // Home position from config/robot.yaml home_position_deg, converted to radians
  openarm_left_joint1: 0.5236, openarm_left_joint2: 0.0, openarm_left_joint3: 0.0,
  openarm_left_joint4: 0.7854, openarm_left_joint5: 0.0, openarm_left_joint6: 0.0,
  openarm_left_joint7: -0.5236,
  openarm_right_joint1: -0.5236, openarm_right_joint2: 0.0, openarm_right_joint3: 0.0,
  openarm_right_joint4: 0.7854, openarm_right_joint5: 0.0, openarm_right_joint6: 0.0,
  openarm_right_joint7: 0.5236,
  openarm_left_finger_joint1: 0.0, openarm_left_finger_joint2: 0.0,
  openarm_right_finger_joint1: 0.0, openarm_right_finger_joint2: 0.0,
};

// ── DOM references ──────────────────────────────────────────────────────
const dom = {
  canvas: document.getElementById('robot-canvas'),
  solverStatus: document.getElementById('solver-status'),
  wsStatus: document.getElementById('ws-status'),
  solveRate: document.getElementById('solve-rate'),
  loadingOverlay: document.getElementById('loading-overlay'),
  loadingText: document.getElementById('loading-text'),
  errorBanner: document.getElementById('error-banner'),
  errorBannerText: document.getElementById('error-banner-text'),
  logBody: document.getElementById('log-body'),
  jointCanvas: document.getElementById('joint-canvas'),
  latencyCanvas: document.getElementById('latency-canvas'),
  vrConnectBtn: document.getElementById('vr-connect-btn'),
  vrDisconnectBtn: document.getElementById('vr-disconnect-btn'),
  resetHomeBtn: document.getElementById('reset-home-btn'),
};

// ── State ───────────────────────────────────────────────────────────────
const state = {
  robot: null,
  socket: null,
  socketAttempts: 0,
  solveCount: 0,
  lastSolveTimestamp: 0,
  latencyHistory: [],      // last 200 solve times
  errorHistory: { left: [], right: [] },  // per-arm error mm
  vrMarkers: null,         // { left, right, head } THREE.js objects
};

// ── THREE.js Scene Setup ────────────────────────────────────────────────
const renderer = new THREE.WebGLRenderer({
  antialias: true,
  canvas: dom.canvas,
});
renderer.setPixelRatio(window.devicePixelRatio);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf1eddc);
scene.fog = new THREE.Fog(0xf1eddc, 2.8, 8.5);

const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 100);
camera.position.set(2.6, -2.4, 1.9);
camera.up.set(0, 0, 1);

const controls = new OrbitControls(camera, dom.canvas);
controls.target.set(0.0, 0.0, 0.85);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 1.2;
controls.maxDistance = 8.0;
controls.update();

// Lights
const fillLight = new THREE.DirectionalLight(0xfff8dd, 3.1);
fillLight.position.set(2.0, -2.2, 3.0);
scene.add(fillLight);

const rimLight = new THREE.DirectionalLight(0x9dbef9, 1.6);
rimLight.position.set(-2.2, 2.0, 1.8);
scene.add(rimLight);

const ambientLight = new THREE.AmbientLight(0x99a4b5, 1.7);
scene.add(ambientLight);

// Floor grid (XY plane, Z-up)
const grid = new THREE.GridHelper(3.0, 12, 0x6b7280, 0x9aa3af);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.001;
scene.add(grid);

// World axes
const worldAxes = new THREE.AxesHelper(0.45);
worldAxes.renderOrder = 100;
worldAxes.material.depthTest = false;
worldAxes.material.depthWrite = false;
scene.add(worldAxes);

// VR controller markers (teal=left, brick=right, blue=head)
const vrLeftHandle = createTargetHandleMarker('left', 0x008d7c, 0x004d42);
const vrRightHandle = createTargetHandleMarker('right', 0xc44d24, 0x7a2f12);
const vrHeadMarker = createVRHeadMarker(0x60a5fa);
vrLeftHandle.visible = false;
vrRightHandle.visible = false;
scene.add(vrLeftHandle);
scene.add(vrRightHandle);
scene.add(vrHeadMarker);
state.vrMarkers = { left: vrLeftHandle, right: vrRightHandle, head: vrHeadMarker };

// IK target command markers (wireframe octahedron showing where solver was told to reach)
const ikLeftTarget = createDebugPoseMarker('desired', 0x008d7c);
const ikRightTarget = createDebugPoseMarker('desired', 0xc44d24);
ikLeftTarget.visible = false;
ikRightTarget.visible = false;
scene.add(ikLeftTarget);
scene.add(ikRightTarget);
state.ikTargets = { left: ikLeftTarget, right: ikRightTarget };

// VR pinned origin axis (shows coordinate system after B-button calibration)
const vrOriginGroup = new THREE.Group();
const vrOriginSphere = new THREE.Mesh(
  new THREE.SphereGeometry(0.025, 12, 12),
  new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.9 }),
);
const vrOriginAxes = new THREE.AxesHelper(0.35);
vrOriginAxes.material.depthTest = false;
vrOriginAxes.material.depthWrite = false;
vrOriginAxes.renderOrder = 120;
vrOriginGroup.add(vrOriginSphere);
vrOriginGroup.add(vrOriginAxes);
vrOriginGroup.visible = false;
scene.add(vrOriginGroup);
state.vrOriginMarker = vrOriginGroup;

// ── Resize Handler ──────────────────────────────────────────────────────
function resize() {
  const parent = dom.canvas.parentElement;
  if (!parent) return;
  const w = parent.clientWidth;
  const h = parent.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

window.addEventListener('resize', resize);

// ── URDF Loading ────────────────────────────────────────────────────────
async function loadRobot() {
  const loader = new URDFLoader();
  loader.parseCollision = false;

  state.robot = await loader.loadAsync(URDF_URL);
  state.robot.name = 'openarm-bimanual';

  // Set default pose from config
  for (const [name, value] of Object.entries(DEFAULT_POSE_RAD)) {
    state.robot.setJointValue(name, value);
  }

  // Attach joint axis helpers
  attachUrdfJointAxes(state.robot);

  scene.add(state.robot);
  resize();
}

function attachUrdfJointAxes(robot) {
  const joints = Object.values(robot?.joints ?? {});
  joints.forEach((joint) => {
    if (joint.children.some((child) => child.name === 'joint-axis-helper')) {
      return;
    }
    joint.add(createJointAxesHelper());
  });
}

// ── Snapshot Rendering ──────────────────────────────────────────────────
function updateSnapshot(snapshot) {
  if (!snapshot) return;

  // Solver warmup status
  const warmupComplete = snapshot.solver?.warmup_complete ?? false;

  // Solver status pill
  if (warmupComplete) {
    dom.solverStatus.textContent = 'Ready';
    dom.solverStatus.className = 'status-pill status-ok';
  } else {
    dom.solverStatus.textContent = 'Warming up';
    dom.solverStatus.className = 'status-pill status-pending';
  }

  // Error banner
  const leftArm = snapshot.arms?.left;
  const rightArm = snapshot.arms?.right;
  const hasError = (leftArm && leftArm.success === false) || (rightArm && rightArm.success === false);
  if (hasError) {
    dom.errorBanner.hidden = false;
    const reasons = [];
    if (leftArm?.failure_reason) reasons.push(`Left: ${leftArm.failure_reason}`);
    if (rightArm?.failure_reason) reasons.push(`Right: ${rightArm.failure_reason}`);
    dom.errorBannerText.textContent = reasons.join(' | ');
  } else {
    dom.errorBanner.hidden = true;
  }

  // Update robot joint angles from snapshot
  if (state.robot && leftArm && rightArm) {
    applyJointAngles(leftArm.joint_radians, rightArm.joint_radians);
  }

  // Update VR status panel
  updateVRStatus(snapshot.vr_status);

  // Update VR telemetry panel
  updateVRTelemetry(snapshot.vr_status);

  // Update VR controller markers in 3D scene
  updateVRMarkers(snapshot.vr_status);

  // Update IK target command markers
  updateIKTargets(snapshot.arms);

  // Update control source display
  updateControlSource(snapshot.control_source);

  // Solve rate
  const now = performance.now();
  if (state.lastSolveTimestamp > 0 && snapshot.meta?.solve_index > state.solveCount) {
    const dt = now - state.lastSolveTimestamp;
    if (dt > 0) {
      const rate = 1000 / dt;
      dom.solveRate.textContent = `${rate.toFixed(0)} Hz`;
    }
  }
  state.solveCount = snapshot.meta?.solve_index ?? state.solveCount;
  state.lastSolveTimestamp = now;

  // Update audit panels
  updateAuditPanels(snapshot);
}

function applyJointAngles(leftRads, rightRads) {
  if (!state.robot) return;

  // Left arm joints (7) + finger joints (2)
  if (Array.isArray(leftRads)) {
    leftRads.forEach((val, i) => {
      if (i < LEFT_JOINT_NAMES.length) {
        state.robot.setJointValue(LEFT_JOINT_NAMES[i], val);
      } else if (i - 7 < LEFT_FINGER_JOINTS.length) {
        state.robot.setJointValue(LEFT_FINGER_JOINTS[i - 7], val);
      }
    });
  }

  // Right arm joints (7) + finger joints (2)
  if (Array.isArray(rightRads)) {
    rightRads.forEach((val, i) => {
      if (i < RIGHT_JOINT_NAMES.length) {
        state.robot.setJointValue(RIGHT_JOINT_NAMES[i], val);
      } else if (i - 7 < RIGHT_FINGER_JOINTS.length) {
        state.robot.setJointValue(RIGHT_FINGER_JOINTS[i - 7], val);
      }
    });
  }
}

// ── Audit Panels ────────────────────────────────────────────────────────
function updateAuditPanels(snapshot) {
  const leftArm = snapshot.arms?.left;
  const rightArm = snapshot.arms?.right;
  const audit = snapshot.audit;

  // Solve log table
  if (audit && leftArm && dom.logBody) {
    addLogRow(snapshot.meta.solve_index, leftArm, rightArm);
  }

  // Joint bar chart
  if (leftArm?.joint_radians && rightArm?.joint_radians) {
    drawJointChart(leftArm.joint_radians, rightArm.joint_radians);
  }

  // Latency chart
  if (audit?.last_solve_time_ms !== undefined) {
    state.latencyHistory.push(audit.last_solve_time_ms);
    if (state.latencyHistory.length > 200) state.latencyHistory.shift();
    drawLatencyChart();
  }

  // Error metrics
  if (leftArm?.position_error_mm !== undefined) {
    state.errorHistory.left.push(leftArm.position_error_mm);
    if (state.errorHistory.left.length > 1000) state.errorHistory.left.shift();
  }
  if (rightArm?.position_error_mm !== undefined) {
    state.errorHistory.right.push(rightArm.position_error_mm);
    if (state.errorHistory.right.length > 1000) state.errorHistory.right.shift();
  }
  updateErrorMetrics();
}

function addLogRow(solveIndex, leftArm, rightArm) {
  const row = document.createElement('tr');
  const time = new Date().toLocaleTimeString();
  const target = leftArm.target_active
    ? leftArm.target_position.map(v => v.toFixed(2)).join(', ')
    : '—';
  const solveMs = leftArm.solve_time_ms?.toFixed(1) ?? '—';
  const posErr = leftArm.position_error_mm?.toFixed(2) ?? '—';
  const oriErr = leftArm.orientation_error_deg?.toFixed(2) ?? '—';
  const status = leftArm.success === false ? 'FAIL' : 'OK';

  row.innerHTML = `
    <td>${solveIndex}</td>
    <td>${time}</td>
    <td>${target}</td>
    <td>${solveMs}</td>
    <td>${posErr}</td>
    <td>${oriErr}</td>
    <td>${status}</td>
  `;

  dom.logBody.prepend(row);

  // Keep only last 100 rows
  while (dom.logBody.children.length > 100) {
    dom.logBody.lastElementChild.remove();
  }
}

function drawJointChart(leftRads, rightRads) {
  const canvas = dom.jointCanvas;
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width = canvas.clientWidth * window.devicePixelRatio;
  const h = canvas.height = canvas.clientHeight * window.devicePixelRatio;
  ctx.clearRect(0, 0, w, h);

  const allRads = [...leftRads, ...rightRads];
  const barCount = allRads.length;
  const maxAbs = Math.max(Math.max(...allRads.map(Math.abs)), Math.PI);
  const barWidth = (w / barCount) * 0.7;
  const gap = (w / barCount) * 0.3;
  const midY = h / 2;

  allRads.forEach((val, i) => {
    const x = i * (barWidth + gap) + gap / 2;
    const barH = (val / maxAbs) * (midY - 4);
    const color = i < 9 ? '#008d7c' : '#c44d24';

    ctx.fillStyle = color;
    ctx.fillRect(x, midY - barH, barWidth, barH);
  });

  // Center line
  ctx.strokeStyle = 'rgba(23,32,43,0.12)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, midY);
  ctx.lineTo(w, midY);
  ctx.stroke();
}

function drawLatencyChart() {
  const canvas = dom.latencyCanvas;
  if (!canvas || state.latencyHistory.length < 2) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width = canvas.clientWidth * window.devicePixelRatio;
  const h = canvas.height = canvas.clientHeight * window.devicePixelRatio;
  ctx.clearRect(0, 0, w, h);

  const data = state.latencyHistory;
  const maxMs = Math.max(Math.max(...data), 1);
  const stepX = w / (data.length - 1);

  // Line
  ctx.strokeStyle = '#008d7c';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  data.forEach((ms, i) => {
    const x = i * stepX;
    const y = h - (ms / maxMs) * (h - 8) - 4;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Y-axis labels
  ctx.fillStyle = '#596271';
  ctx.font = `${10 * window.devicePixelRatio}px IBM Plex Mono`;
  ctx.textAlign = 'left';
  ctx.fillText(`0`, 2, h - 2);
  ctx.fillText(`${maxMs.toFixed(1)}ms`, 2, 12);
}

function updateErrorMetrics() {
  for (const side of ['left', 'right']) {
    const data = state.errorHistory[side];
    if (data.length === 0) continue;
    const sorted = [...data].sort((a, b) => a - b);
    const min = sorted[0];
    const max = sorted[sorted.length - 1];
    const p50 = sorted[Math.floor(sorted.length * 0.5)];
    const p99 = sorted[Math.floor(sorted.length * 0.99)];

    const prefix = side;
    const el = (id) => document.getElementById(id);
    if (el(`${prefix}-pos-min`)) el(`${prefix}-pos-min`).textContent = min.toFixed(3);
    if (el(`${prefix}-pos-p50`)) el(`${prefix}-pos-p50`).textContent = p50.toFixed(3);
    if (el(`${prefix}-pos-p99`)) el(`${prefix}-pos-p99`).textContent = p99.toFixed(3);
    if (el(`${prefix}-pos-max`)) el(`${prefix}-pos-max`).textContent = max.toFixed(3);
  }
}

// ── VR / Keyboard Status ────────────────────────────────────────────────
function updateVRStatus(vrStatus) {
  const el = document.getElementById('vr-status');
  if (!el) return;
  if (!vrStatus) {
    el.textContent = 'VR: N/A';
    el.className = 'status-pill status-warn';
    return;
  }

  const ros2Installed = vrStatus.ros2_installed ?? false;
  const subscriberRunning = vrStatus.subscriber_running ?? false;
  const connected = vrStatus.connected;
  const calState = vrStatus.calibration?.state ?? 'waiting';

  // Update connect/disconnect button states
  if (dom.vrConnectBtn && dom.vrDisconnectBtn) {
    dom.vrConnectBtn.disabled = subscriberRunning;
    dom.vrDisconnectBtn.disabled = !subscriberRunning;
  }

  if (!ros2Installed) {
    el.textContent = 'VR: ROS2 missing';
    el.className = 'status-pill status-error';
  } else if (subscriberRunning && connected) {
    const labels = { waiting: 'Press B', ready: 'Press A', active: 'Active' };
    el.textContent = `VR: ${labels[calState] ?? calState}`;
    el.className = calState === 'active' ? 'status-pill status-ok' : 'status-pill status-pending';
  } else if (subscriberRunning && !connected) {
    el.textContent = 'VR: Listening';
    el.className = 'status-pill status-pending';
  } else {
    el.textContent = 'VR: Off';
    el.className = 'status-pill status-idle';
  }
}

function updateVRTelemetry(vrStatus) {
  const telStatus = document.getElementById('vr-tel-status');
  if (!telStatus) return;

  if (!vrStatus) {
    telStatus.textContent = 'off';
    telStatus.className = 'status-pill status-idle';
    return;
  }

  // Top-level status
  const running = vrStatus.subscriber_running ?? false;
  const connected = vrStatus.connected ?? false;
  if (running && connected) {
    telStatus.textContent = 'live';
    telStatus.className = 'status-pill status-ok';
  } else if (running) {
    telStatus.textContent = 'listening';
    telStatus.className = 'status-pill status-pending';
  } else {
    telStatus.textContent = 'off';
    telStatus.className = 'status-pill status-idle';
  }

  const poses = vrStatus.poses ?? {};

  // Helper: format [x,y,z] to compact string
  const fmt3 = (arr) => {
    if (!Array.isArray(arr) || arr.length < 3) return '—';
    return arr.map(v => v.toFixed(3)).join(', ');
  };
  const fmt4 = (arr) => {
    if (!Array.isArray(arr) || arr.length < 4) return '—';
    return arr.map(v => v.toFixed(3)).join(', ');
  };

  // Left / Right controllers
  const buttons = vrStatus.buttons ?? {};
  for (const side of ['left', 'right']) {
    const pose = poses[side];
    const badge = document.getElementById(`vr-${side}-badge`);
    const posEl = document.getElementById(`vr-${side}-pos`);
    const quatEl = document.getElementById(`vr-${side}-quat`);
    const ageEl = document.getElementById(`vr-${side}-age`);

    if (!pose || pose.stale || !pose.position) {
      if (badge) { badge.textContent = pose?.stale ? 'stale' : 'missing'; badge.dataset.state = pose?.stale ? 'stale' : 'missing'; }
      if (posEl) posEl.textContent = '—';
      if (quatEl) quatEl.textContent = '—';
      if (ageEl) ageEl.textContent = pose ? `${pose.age_ms ?? '—'} ms` : '—';
    } else {
      if (badge) { badge.textContent = 'live'; badge.dataset.state = 'live'; }
      if (posEl) posEl.textContent = fmt3(pose.position);
      if (quatEl) quatEl.textContent = fmt4(pose.orientation_wxyz);
      if (ageEl) ageEl.textContent = `${pose.age_ms ?? '—'} ms`;
    }

    // Buttons
    const btnEl = document.getElementById(`vr-${side}-btns`);
    const btn = buttons[side];
    if (btnEl) {
      if (btn && !btn.stale && btn.values.length > 0) {
        const activeClass = side === 'left' ? 'active' : 'active-brick';
        btnEl.innerHTML = btn.values.map(v =>
          `<span class="vr-btn-dot${v > 0.5 ? ' ' + activeClass : ''}"></span>`
        ).join('');
      } else {
        btnEl.textContent = '—';
      }
    }
  }

  // Head
  const headPose = poses.head;
  const headBadge = document.getElementById('vr-head-badge');
  const headPos = document.getElementById('vr-head-pos');
  const headAge = document.getElementById('vr-head-age');

  if (!headPose || headPose.stale || !headPose.position) {
    if (headBadge) { headBadge.textContent = headPose?.stale ? 'stale' : 'missing'; headBadge.dataset.state = headPose?.stale ? 'stale' : 'missing'; }
    if (headPos) headPos.textContent = '—';
    if (headAge) headAge.textContent = headPose ? `${headPose.age_ms ?? '—'} ms` : '—';
  } else {
    if (headBadge) { headBadge.textContent = 'live'; headBadge.dataset.state = 'live'; }
    if (headPos) headPos.textContent = fmt3(headPose.position);
    if (headAge) headAge.textContent = `${headPose.age_ms ?? '—'} ms`;
  }

  // Calibration & last message
  const calState = vrStatus.calibration?.state ?? '—';
  const calEl = document.getElementById('vr-cal-state');
  if (calEl) calEl.textContent = calState;

  const lastMsg = vrStatus.last_message_age_ms;
  const lastMsgEl = document.getElementById('vr-last-msg');
  if (lastMsgEl) lastMsgEl.textContent = lastMsg != null ? `${lastMsg} ms` : '—';
}

function updateVRMarkers(vrStatus) {
  if (!state.vrMarkers) return;

  const poses = vrStatus?.poses;
  if (!poses) {
    state.vrMarkers.left.visible = false;
    state.vrMarkers.right.visible = false;
    state.vrMarkers.head.visible = false;
    return;
  }

  for (const side of ['left', 'right']) {
    const pose = poses[side];
    const marker = state.vrMarkers[side];
    if (pose && pose.position && !pose.stale) {
      marker.visible = true;
      marker.position.set(pose.position[0], pose.position[1], pose.position[2]);
      if (pose.orientation_wxyz) {
        const [w, x, y, z] = pose.orientation_wxyz;
        marker.quaternion.set(x, y, z, w);
      }
    } else {
      marker.visible = false;
    }
  }

  const headPose = poses.head;
  const headMarker = state.vrMarkers.head;
  if (headPose && headPose.position && !headPose.stale) {
    headMarker.visible = true;
    headMarker.position.set(headPose.position[0], headPose.position[1], headPose.position[2]);
  } else {
    headMarker.visible = false;
  }

  // Update pinned VR origin axis marker
  updateVROriginAxis(vrStatus?.calibration);
}

function updateVROriginAxis(calibration) {
  const marker = state.vrOriginMarker;
  if (!marker) return;
  if (calibration?.origin_pinned && calibration.origin_position) {
    marker.visible = true;
    marker.position.set(
      calibration.origin_position[0],
      calibration.origin_position[1],
      calibration.origin_position[2],
    );
    if (calibration.origin_quaternion_wxyz) {
      const [w, x, y, z] = calibration.origin_quaternion_wxyz;
      marker.quaternion.set(x, y, z, w);
    }
  } else {
    marker.visible = false;
  }
}

function updateIKTargets(arms) {
  if (!state.ikTargets) return;
  for (const side of ['left', 'right']) {
    const arm = arms?.[side];
    const marker = state.ikTargets[side];
    // Show marker when target is active (IK solving) or when FK home position is available (READY state)
    const hasTarget = arm && Array.isArray(arm.target_position) &&
      arm.target_position.some(v => v !== 0);
    if (hasTarget) {
      marker.visible = true;
      marker.position.set(arm.target_position[0], arm.target_position[1], arm.target_position[2]);
      if (Array.isArray(arm.target_quaternion_wxyz)) {
        const [w, x, y, z] = arm.target_quaternion_wxyz;
        marker.quaternion.set(x, y, z, w);
      }
    } else {
      marker.visible = false;
    }
  }
}

function updateControlSource(source) {
  const el = document.getElementById('control-source');
  if (!el) return;
  el.textContent = source ?? 'idle';
}

// ── WebSocket ───────────────────────────────────────────────────────────
function websocketUrl() {
  // Use CONFIG.WS_URL if available (set in config.js), fallback to same-host
  if (typeof CONFIG !== 'undefined' && CONFIG.WS_URL) {
    return CONFIG.WS_URL;
  }
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  return `${protocol}//${window.location.host}:5200/ws`;
}

function connectWebSocket() {
  dom.wsStatus.textContent = 'Connecting';
  dom.wsStatus.className = 'status-pill status-pending';

  state.socket = new WebSocket(websocketUrl());

  state.socket.addEventListener('open', () => {
    state.socketAttempts = 0;
    dom.wsStatus.textContent = 'Connected';
    dom.wsStatus.className = 'status-pill status-ok';
  });

  state.socket.addEventListener('message', (event) => {
    const snapshot = JSON.parse(event.data);
    updateSnapshot(snapshot);
  });

  state.socket.addEventListener('close', () => {
    dom.wsStatus.textContent = 'Reconnecting';
    dom.wsStatus.className = 'status-pill status-warn';
    state.socketAttempts += 1;
    const retryDelay = Math.min(3000, 500 * state.socketAttempts);
    setTimeout(connectWebSocket, retryDelay);
  });

  state.socket.addEventListener('error', () => {
    dom.wsStatus.textContent = 'Error';
    dom.wsStatus.className = 'status-pill status-error';
  });
}

// ── VR Connect / Disconnect / Reset Buttons ─────────────────────────────
dom.vrConnectBtn?.addEventListener('click', () => {
  if (state.socket && state.socket.readyState === WebSocket.OPEN) {
    state.socket.send(JSON.stringify({ type: 'vr_connect' }));
  }
});

dom.vrDisconnectBtn?.addEventListener('click', () => {
  if (state.socket && state.socket.readyState === WebSocket.OPEN) {
    state.socket.send(JSON.stringify({ type: 'vr_disconnect' }));
  }
});

dom.resetHomeBtn?.addEventListener('click', () => {
  if (state.socket && state.socket.readyState === WebSocket.OPEN) {
    state.socket.send(JSON.stringify({ type: 'reset_home' }));
  }
});

// ── Export Buttons ──────────────────────────────────────────────────────
document.getElementById('export-json')?.addEventListener('click', () => {
  window.location.href = '/api/audit?format=json';
});

document.getElementById('export-csv')?.addEventListener('click', () => {
  window.location.href = '/api/audit?format=csv';
});

// ── Animation Loop ──────────────────────────────────────────────────────
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

// ── Bootstrap ───────────────────────────────────────────────────────────
async function init() {
  dom.loadingOverlay.hidden = false;
  dom.loadingText.textContent = 'Loading robot model...';

  try {
    await loadRobot();
    // Hide loading overlay once robot model is loaded — the 3D scene is usable
    dom.loadingOverlay.hidden = true;
  } catch (err) {
    dom.loadingText.textContent = `Failed to load URDF: ${err.message}`;
    console.error('URDF load failed:', err);
    return;
  }

  connectWebSocket();
  animate();
}

init();
