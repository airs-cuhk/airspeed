import * as THREE from 'three';

const DEFAULT_CONTROLLER_AXIS_SIZE = 0.12;

function createHandleMaterial(config) {
  if (config.unlit === true) {
    return new THREE.MeshBasicMaterial({
      color: config.color,
    });
  }
  return new THREE.MeshStandardMaterial(config);
}

export function buildControllerHandleModel(key, config = {}) {
  const handSign = key === 'left' ? -1 : 1;
  const visualRollZ = key === 'left' ? -Math.PI / 2 : Math.PI / 2;
  const visualRollOffsetZ = config.visualRollOffsetZ ?? 0;
  const color = config.color ?? (key === 'left' ? 0x067a89 : 0xc2551a);
  const emissive = config.emissive ?? (key === 'left' ? 0x024d56 : 0x7f3210);
  const emissiveIntensity = config.emissiveIntensity ?? 0.24;
  const roughness = config.roughness ?? 0.28;
  const metalness = config.metalness ?? 0.16;
  const crownColor = config.crownColor ?? 0xf5efe2;
  const crownEmissive = config.crownEmissive ?? 0x000000;
  const crownEmissiveIntensity = config.crownEmissiveIntensity ?? 0.0;
  const crownRoughness = config.crownRoughness ?? 0.35;
  const crownMetalness = config.crownMetalness ?? 0.1;
  const faceColor = config.faceColor ?? 0xf0f4f7;
  const faceEmissive = config.faceEmissive ?? 0x7baec2;
  const faceEmissiveIntensity = config.faceEmissiveIntensity ?? 0.16;
  const faceRoughness = config.faceRoughness ?? 0.18;
  const faceMetalness = config.faceMetalness ?? 0.2;

  const group = new THREE.Group();
  const orientationGroup = new THREE.Group();
  orientationGroup.rotation.y = -Math.PI / 2;
  const visualGroup = new THREE.Group();
  visualGroup.rotation.z = visualRollZ + visualRollOffsetZ;

  const grip = new THREE.Mesh(
    new THREE.CylinderGeometry(0.028, 0.034, 0.18, 24),
    createHandleMaterial({
      unlit: config.unlit,
      color,
      emissive,
      emissiveIntensity,
      roughness,
      metalness,
    })
  );
  grip.rotation.set(Math.PI / 2, 0, 0);

  const crown = new THREE.Mesh(
    new THREE.TorusGeometry(0.055, 0.012, 12, 30),
    createHandleMaterial({
      unlit: config.unlit,
      color: crownColor,
      emissive: crownEmissive,
      emissiveIntensity: crownEmissiveIntensity,
      roughness: crownRoughness,
      metalness: crownMetalness,
    })
  );
  crown.position.set(handSign * 0.018, 0.045, -0.03);
  crown.rotation.y = Math.PI / 2;

  const face = new THREE.Mesh(
    new THREE.BoxGeometry(0.05, 0.045, 0.055),
    createHandleMaterial({
      unlit: config.unlit,
      color: faceColor,
      emissive: faceEmissive,
      emissiveIntensity: faceEmissiveIntensity,
      roughness: faceRoughness,
      metalness: faceMetalness,
    })
  );
  face.position.set(handSign * 0.02, 0.05, -0.048);
  face.rotation.y = handSign * Math.PI * 0.1;

  const trigger = new THREE.Mesh(
    new THREE.BoxGeometry(0.024, 0.05, 0.022),
    createHandleMaterial({
      unlit: config.unlit,
      color: 0x2a2f34,
      roughness: 0.45,
    })
  );
  trigger.position.set(handSign * 0.016, -0.006, -0.058);
  trigger.rotation.y = handSign * Math.PI * 0.14;

  visualGroup.add(grip);
  visualGroup.add(crown);
  visualGroup.add(face);
  visualGroup.add(trigger);
  orientationGroup.add(visualGroup);
  group.add(orientationGroup);
  return group;
}

export function buildControllerHandleDisplay(key, config = {}) {
  const display = new THREE.Group();
  display.rotation.x = Math.PI / 2;
  display.add(buildControllerHandleModel(key, config));

  if (config.axes !== false) {
    display.add(new THREE.AxesHelper(config.axisSize ?? DEFAULT_CONTROLLER_AXIS_SIZE));
  }

  return display;
}
