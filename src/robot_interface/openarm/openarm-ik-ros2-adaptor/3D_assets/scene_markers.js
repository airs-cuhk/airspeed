/**
 * scene_markers.js — Reusable THREE.js marker and helper factories.
 *
 * Extracted from exp-IK-super-fast-validation/web/app.js.
 * Each export returns a self-contained THREE.Object3D ready for scene.add().
 */

import * as THREE from 'three';
import { buildControllerHandleDisplay } from './handle_marker.js';

// ── Constants ────────────────────────────────────────────────────────────
const AXIS_COLOR = 0x111111;
const DEFAULT_JOINT_AXIS_SIZE = 0.04;
const DEFAULT_WORLD_AXIS_SIZE = 0.45;
const DEFAULT_TARGET_AXIS_SIZE = 0.13;
const DEFAULT_TARGET_LABEL_OFFSET = 0.055;

// ── Arrow helper with no-depth-write (always visible) ────────────────────
function createAxisArrow(length, direction, color) {
  const arrow = new THREE.ArrowHelper(
    direction.clone().normalize(),
    new THREE.Vector3(0, 0, 0),
    length,
    color,
    Math.max(length * 0.18, 0.045),
    Math.max(length * 0.12, 0.03),
  );
  arrow.line.material.depthTest = false;
  arrow.line.material.depthWrite = false;
  arrow.cone.material.depthTest = false;
  arrow.cone.material.depthWrite = false;
  arrow.renderOrder = 150;
  return arrow;
}

// ── Small RGB axes attached to each URDF joint ──────────────────────────
export function createJointAxesHelper(size = DEFAULT_JOINT_AXIS_SIZE) {
  const helper = new THREE.AxesHelper(size);
  helper.name = 'joint-axis-helper';
  helper.renderOrder = 130;
  helper.material.depthTest = false;
  helper.material.depthWrite = false;
  helper.material.transparent = true;
  helper.material.opacity = 0.85;
  return helper;
}

// ── Labeled axes group (3 arrows + projected HTML labels) ───────────────
//
// `registerLabel` is a callback so the caller can decide how labels are
// projected (CSS, sprite, etc.).  Signature: (anchorObject, label, scope, localPosition)
export function createLabeledAxes(
  size,
  labels = ['x', 'y', 'z'],
  scope = 'target',
  labelOffset = 0.14,
  registerLabel = null,
) {
  const axesGroup = new THREE.Group();
  axesGroup.add(createAxisArrow(size, new THREE.Vector3(1, 0, 0), AXIS_COLOR));
  axesGroup.add(createAxisArrow(size, new THREE.Vector3(0, 1, 0), AXIS_COLOR));
  axesGroup.add(createAxisArrow(size, new THREE.Vector3(0, 0, 1), AXIS_COLOR));

  if (typeof registerLabel === 'function') {
    const entries = [
      { label: labels[0], position: new THREE.Vector3(size + labelOffset, 0, 0) },
      { label: labels[1], position: new THREE.Vector3(0, size + labelOffset, 0) },
      { label: labels[2], position: new THREE.Vector3(0, 0, size + labelOffset) },
    ];
    entries.forEach(({ label, position }) => {
      registerLabel(axesGroup, label, scope, position);
    });
  }
  return axesGroup;
}

// ── World-space axes indicator ──────────────────────────────────────────
export function createWorldAxes(
  size = DEFAULT_WORLD_AXIS_SIZE,
  position = new THREE.Vector3(-0.85, -0.85, 0.03),
  registerLabel = null,
) {
  const axes = createLabeledAxes(size, ['X', 'Y', 'Z'], 'world', 0.14, registerLabel);
  axes.name = 'space-axes';
  axes.position.copy(position);
  return axes;
}

// ── VR controller target handle marker ──────────────────────────────────
export function createTargetHandleMarker(
  key,
  color,
  emissive,
  { axisSize = DEFAULT_TARGET_AXIS_SIZE, labelOffset = DEFAULT_TARGET_LABEL_OFFSET, registerLabel = null } = {},
) {
  const marker = new THREE.Group();
  marker.add(
    buildControllerHandleDisplay(key, {
      unlit: true,
      color,
      emissive,
      emissiveIntensity: 0.22,
      roughness: 0.3,
      metalness: 0.16,
      crownColor: key === 'left' ? 0x0d4a53 : 0x723117,
      crownEmissive: key === 'left' ? 0x03252a : 0x311108,
      crownEmissiveIntensity: 0.12,
      faceColor: key === 'left' ? 0x16616c : 0x8a3c1c,
      faceEmissive: key === 'left' ? 0x063138 : 0x3f1709,
      faceEmissiveIntensity: 0.12,
    }),
  );
  const axes = createLabeledAxes(axisSize, ['x', 'y', 'z'], 'target', labelOffset, registerLabel);
  axes.rotation.y = -Math.PI / 2;
  marker.add(axes);
  return marker;
}

// ── Reach-boundary point cloud ──────────────────────────────────────────
export function createReachBoundaryPointCloud(color) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
  const material = new THREE.PointsMaterial({
    color,
    size: 0.024,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.38,
    depthWrite: false,
  });
  const points = new THREE.Points(geometry, material);
  points.visible = false;
  points.renderOrder = 95;
  return points;
}

export function updateReachBoundaryPointCloud(points, boundaryShell) {
  const seen = new Set();
  const vertices = [];
  (Array.isArray(boundaryShell) ? boundaryShell : []).forEach((sample) => {
    const position = sample?.position_m;
    if (!Array.isArray(position) || position.length !== 3) return;
    const key = position.map((v) => Number(v).toFixed(6)).join(':');
    if (seen.has(key)) return;
    seen.add(key);
    vertices.push(...position);
  });
  const positions = new Float32Array(vertices);
  const attribute = new THREE.BufferAttribute(positions, 3);
  points.geometry.setAttribute('position', attribute);
  attribute.needsUpdate = true;
  if (positions.length > 0) {
    points.geometry.computeBoundingSphere();
  } else {
    points.geometry.boundingSphere = null;
  }
  points.visible = positions.length > 0;
}

// ── Debug pose marker (desired = wireframe octahedron, achieved = solid box + edges) ──
export function createDebugPoseMarker(kind, color) {
  const group = new THREE.Group();
  let visual;
  if (kind === 'desired') {
    visual = new THREE.Mesh(
      new THREE.OctahedronGeometry(0.038, 0),
      new THREE.MeshBasicMaterial({ color, wireframe: true, transparent: true, opacity: 0.92 }),
    );
  } else {
    visual = new THREE.Mesh(
      new THREE.BoxGeometry(0.032, 0.032, 0.032),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.9 }),
    );
    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(0.036, 0.036, 0.036)),
      new THREE.LineBasicMaterial({ color: 0x111111, transparent: true, opacity: 0.8 }),
    );
    group.add(edges);
  }
  group.add(visual);
  group.visible = false;
  return group;
}

// ── Floor grid (lies in XY plane at z ≈ 0) ─────────────────────────────
export function createFloorGrid(size = 3.0, divisions = 12) {
  const grid = new THREE.GridHelper(size, divisions, 0x6b7280, 0x9aa3af);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.001;
  return grid;
}

// ── VR head-tracking sphere ─────────────────────────────────────────────
export function createVRHeadMarker(color = 0x60a5fa) {
  const mesh = new THREE.Mesh(
    new THREE.SphereGeometry(0.04, 16, 16),
    new THREE.MeshBasicMaterial({ color }),
  );
  mesh.visible = false;
  return mesh;
}
