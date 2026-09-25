import { state } from './state.js';

export const PIPELINE_PHASES = ['detect', 'segment', 'plan', 'pick', 'transport', 'place'];

function _getImageBounds(rect) {
  const robotCamImg = document.getElementById('robot-cam-img');
  let imgWidth = rect.width, imgHeight = rect.height, offsetX = 0, offsetY = 0;
  if (robotCamImg && !robotCamImg.classList.contains('hidden') &&
      robotCamImg.naturalWidth > 0 && robotCamImg.naturalHeight > 0) {
    const ia = robotCamImg.naturalWidth / robotCamImg.naturalHeight;
    const ca = rect.width / rect.height;
    if (ca > ia) {
      imgHeight = rect.height; imgWidth = rect.height * ia;
      offsetX = (rect.width - imgWidth) / 2;
    } else {
      imgWidth = rect.width; imgHeight = rect.width / ia;
      offsetY = (rect.height - imgHeight) / 2;
    }
  }
  return { imgWidth, imgHeight, offsetX, offsetY };
}

function _toPx(x, y, b) {
  return { px: b.offsetX + (x / 1000) * b.imgWidth, py: b.offsetY + (y / 1000) * b.imgHeight };
}

function _pillLabel(ctx, text, x, y, color) {
  ctx.font = 'bold 11px sans-serif';
  const w = ctx.measureText(text).width;
  ctx.fillStyle = 'rgba(0,0,0,0.65)';
  ctx.beginPath();
  ctx.roundRect(x - 4, y - 11, w + 8, 16, 3);
  ctx.fill();
  ctx.fillStyle = color;
  ctx.fillText(text, x, y);
}

export function redrawOverlay() {
  const overlayCanvas = document.getElementById('overlay-canvas');
  if (!overlayCanvas) return;
  const rect = overlayCanvas.parentElement.getBoundingClientRect();
  overlayCanvas.width = rect.width;
  overlayCanvas.height = rect.height;
  const ctx = overlayCanvas.getContext('2d');
  const b = _getImageBounds(rect);

  if (state.overlayState.sam3 && state.overlayData.sam3Mask) {
    _drawSam3(ctx, b, state.overlayData.sam3Mask);
  }
  if (state.overlayState.graspPose && state.overlayData.graspPose) {
    _drawGraspPose(ctx, b, state.overlayData.graspPose);
  }
  if (state.overlayState.geminiPoint && state.overlayData.pickCoords) {
    _drawGeminiPoint(ctx, b, state.overlayData.pickCoords);
  }
  if (state.overlayState.placeLocation && state.overlayData.placeCoords) {
    _drawPlaceLocation(ctx, b, state.overlayData.placeCoords);
  }
  if (state.overlayData.livePoints) {
    _drawLivePoints(ctx, b, state.overlayData.livePoints);
  }
}

function _drawSam3(ctx, b, maskData) {
  if (!maskData.contours || maskData.contours.length === 0) return;
  ctx.save();
  ctx.fillStyle = 'rgba(0,0,0,0.55)';
  ctx.fillRect(b.offsetX, b.offsetY, b.imgWidth, b.imgHeight);
  ctx.globalCompositeOperation = 'destination-out';
  for (const contour of maskData.contours) {
    if (contour.length < 3) continue;
    ctx.beginPath();
    const f = _toPx(contour[0][0], contour[0][1], b);
    ctx.moveTo(f.px, f.py);
    for (let i = 1; i < contour.length; i++) {
      const p = _toPx(contour[i][0], contour[i][1], b);
      ctx.lineTo(p.px, p.py);
    }
    ctx.closePath();
    ctx.fill();
  }
  ctx.globalCompositeOperation = 'source-over';
  ctx.strokeStyle = '#A100FF';
  ctx.lineWidth = 2;
  for (const contour of maskData.contours) {
    if (contour.length < 3) continue;
    ctx.beginPath();
    const f = _toPx(contour[0][0], contour[0][1], b);
    ctx.moveTo(f.px, f.py);
    for (let i = 1; i < contour.length; i++) {
      const p = _toPx(contour[i][0], contour[i][1], b);
      ctx.lineTo(p.px, p.py);
    }
    ctx.closePath();
    ctx.stroke();
  }
  ctx.restore();
}

function _drawGraspPose(ctx, b, pose) {
  const { px, py } = _toPx(pose.x, pose.y, b);
  const angle = ((pose.angle_deg || 0) * Math.PI) / 180;
  const L = 26;
  const C = '#00E5FF';
  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(angle);
  ctx.strokeStyle = C;
  ctx.lineWidth = 2.5;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(-L, 0); ctx.lineTo(L, 0);
  ctx.moveTo(L, 0); ctx.lineTo(L - 7, -5);
  ctx.moveTo(L, 0); ctx.lineTo(L - 7, 5);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(0, 0, 5, 0, 2 * Math.PI);
  ctx.fillStyle = C;
  ctx.fill();
  ctx.restore();
  const label = pose.quality != null ? `Grasp ${Math.round(pose.quality * 100)}%` : 'Grasp';
  _pillLabel(ctx, label, px + L + 10, py + 4, C);
}

function _drawCrosshairMarker(ctx, px, py, color, label) {
  const R = 7, EXT = 4;
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(px, py, R, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(px, py, 2.5, 0, 2 * Math.PI);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.beginPath();
  ctx.moveTo(px - R - EXT, py); ctx.lineTo(px - R, py);
  ctx.moveTo(px + R, py);       ctx.lineTo(px + R + EXT, py);
  ctx.moveTo(px, py - R - EXT); ctx.lineTo(px, py - R);
  ctx.moveTo(px, py + R);       ctx.lineTo(px, py + R + EXT);
  ctx.stroke();
  if (label) _pillLabel(ctx, label, px + R + EXT + 4, py + 4, color);
}

function _drawGeminiPoint(ctx, b, coords) {
  const { px, py } = _toPx(coords.x, coords.y, b);
  const label = coords.label ? `Pick: ${coords.label}` : 'Pick';
  _drawCrosshairMarker(ctx, px, py, '#FF9800', label);
}

function _drawPlaceLocation(ctx, b, coords) {
  const { px, py } = _toPx(coords.x, coords.y, b);
  _drawCrosshairMarker(ctx, px, py, '#00C853', null);
}

function _drawLivePoints(ctx, b, points) {
  const R = 7;
  for (const c of points) {
    const { px: cx, py: cy } = _toPx(c.x, c.y, b);
    ctx.beginPath();
    ctx.arc(cx, cy, R, 0, 2 * Math.PI);
    ctx.fillStyle = 'rgba(0,120,255,0.9)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(255,255,255,0.95)';
    ctx.lineWidth = 2.5;
    ctx.stroke();
    if (c.label) {
      ctx.font = 'bold 13px sans-serif';
      const lx = cx + R + 8, ly = cy + 4;
      const metrics = ctx.measureText(c.label);
      const padX = 5, padY = 3;
      ctx.fillStyle = 'rgba(0,0,0,0.65)';
      ctx.beginPath();
      ctx.roundRect(lx - padX, ly - 10 - padY, metrics.width + padX * 2, 14 + padY * 2, 4);
      ctx.fill();
      ctx.fillStyle = 'rgba(255,255,255,0.95)';
      ctx.fillText(c.label, lx, ly);
    }
  }
}

// Pipeline management

export function setPipelinePhase(phase) {
  state.currentPipelinePhase = phase;
  const idx = PIPELINE_PHASES.indexOf(phase);
  if (idx > 0) {
    PIPELINE_PHASES.slice(0, idx).forEach(p => {
      if (!state.donePipelinePhases.includes(p)) state.donePipelinePhases.push(p);
    });
  }
  _updatePipelineBar();
}

export function completePipelinePhase(phase) {
  if (!state.donePipelinePhases.includes(phase)) state.donePipelinePhases.push(phase);
  state.currentPipelinePhase = null;
  _updatePipelineBar();
}

export function resetPipeline() {
  state.currentPipelinePhase = null;
  state.donePipelinePhases = [];
  _updatePipelineBar();
}

function _updatePipelineBar() {
  const bar = document.getElementById('pipeline-bar');
  if (!bar) return;
  bar.querySelectorAll('.pipeline-step').forEach(el => {
    const step = el.dataset.step;
    el.classList.toggle('active', step === state.currentPipelinePhase);
    el.classList.toggle('done', state.donePipelinePhases.includes(step));
  });
}

export function clearAllOverlayData() {
  state.overlayData.sam3Mask = null;
  state.overlayData.graspPose = null;
  state.overlayData.pickCoords = null;
  state.overlayData.placeCoords = null;
  resetPipeline();
  redrawOverlay();
}

export function setFrankaOverlayUI(visible) {
  const controls = document.getElementById('overlay-controls');
  const bar = document.getElementById('pipeline-bar');
  if (controls) controls.classList.toggle('hidden', !visible);
  if (bar) bar.classList.toggle('hidden', !visible || !state.overlayState.pipeline);
  if (!visible) {
    state.overlayData.livePoints = null;
    redrawOverlay();
  }
}

export function initOverlayCheckboxes() {
  const checks = [
    ['ov-sam3',     'sam3'],
    ['ov-grasp',    'graspPose'],
    ['ov-place',    'placeLocation'],
    ['ov-gemini',   'geminiPoint'],
    ['ov-pipeline', 'pipeline'],
  ];
  checks.forEach(([id, key]) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.onchange = () => {
      state.overlayState[key] = el.checked;
      if (key === 'pipeline') {
        const bar = document.getElementById('pipeline-bar');
        if (bar) bar.classList.toggle('hidden', !el.checked || state.selectedAgentName !== 'franka');
      }
      redrawOverlay();
    };
  });
}
