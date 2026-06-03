/**
 * config.js — Frontend configuration.
 *
 * Edit this file when deploying the frontend to a different machine or port.
 * All paths are relative to web_pages/ (where index.html lives).
 */

const CONFIG = {
  // WebSocket URL for IK solver snapshot stream (50 Hz).
  // Default: same host as this page, port 5200, path /ws.
  WS_URL: (() => {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${protocol}//${window.location.hostname}:5200/ws`;
  })(),

  // Path to the URDF model, relative to web_pages/index.html.
  URDF_PATH: '../3d_assets/urdf/openarm_bimanual_copy.urdf',
};

// Freeze to prevent accidental mutation
Object.freeze(CONFIG);
