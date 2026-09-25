// Migrated to modules/ — see ui/modules/app.js
// --- Main Application Logic (Proactive Agent) ---
let localStorage = window.localStorage;

let statusLabel = document.getElementById('status-label');
let micBtn = document.getElementById('micBtn');
let textInput = document.getElementById('textInput');
let sendBtn = document.getElementById('sendBtn');
let videoPreview = document.getElementById('video-preview');
let videoPlaceholder = document.getElementById('video-placeholder');
let robotCamImg = document.getElementById('robot-cam-img');
let connectBtn = document.getElementById('connectBtn');
let chatLog = document.getElementById('chat-log');
let overlayCanvas = document.getElementById('overlay-canvas');
let chatToggle = document.getElementById('chatToggle');
let chatSidebar = document.getElementById('chat-sidebar');

// --- Model Selector & Runtime Configs ---
let modelSelectorBtn = document.getElementById('modelSelectorBtn');
let modelSelectorLabel = document.getElementById('modelSelectorLabel');
let modelDropdown = document.getElementById('modelDropdown');
let customModelModal = document.getElementById('custom-model-modal');
let customModelInput = document.getElementById('custom-model-input');
let customModalitySelect = document.getElementById('custom-modality-select');
let customModelCancel = document.getElementById('custom-model-cancel');
let customModelApply = document.getElementById('custom-model-apply');

// Endpoint Selector DOM
let endpointSelectorBtn = document.getElementById('endpointSelectorBtn');
let endpointSelectorLabel = document.getElementById('endpointSelectorLabel');
let endpointDropdown = document.getElementById('endpointDropdown');

// Audio Toggle DOM
let audioToggleBtn = document.getElementById('audioToggleBtn');

// Agent Selector DOM
let agentSelectorBtn = document.getElementById('agentSelectorBtn');
let agentSelectorLabel = document.getElementById('agentSelectorLabel');
let agentDropdown = document.getElementById('agentDropdown');

const ENDPOINT_MODELS = {
  gemini_live_api: [
    { model: 'models/gemini-robotics-er-2-streaming-preview', label: 'models/gemini-robotics-er-2-streaming-preview', modality: 'TEXT', textOnly: true },
    { model: 'models/gemini-3.1-flash-live-preview', label: 'models/gemini-3.1-flash-live-preview', modality: 'AUDIO', textOnly: false },
    { model: 'models/gemini-2.5-flash-native-audio-latest', label: 'models/gemini-2.5-flash-native-audio-latest', modality: 'AUDIO', textOnly: false }
  ]
};

let selectedEndpointType =
  localStorage.getItem('lite_endpoint_type') || 'gemini_live_api';

let selectedModel =
  localStorage.getItem('lite_model') ||
  ENDPOINT_MODELS[selectedEndpointType][0].model;

let selectedAudioEnabled = localStorage.getItem('lite_audio_enabled') !==
  'false';  // defaults to true
let customBaseModality =
  localStorage.getItem('lite_custom_modality') || 'TEXT';

// Re-validate selectedModel against available options for the selected endpoint
let availableModels = ENDPOINT_MODELS[selectedEndpointType] || [];
let modelExists = availableModels.some(m => m.model === selectedModel) || selectedModel === 'custom';
if (!modelExists && availableModels.length > 0) {
  selectedModel = availableModels[0].model;
  localStorage.setItem('lite_model', selectedModel);
}

// Populate models dropdown dynamically
function populateModels(endpoint) {
  if (!modelDropdown) return;
  modelDropdown.innerHTML = '';
  const models = ENDPOINT_MODELS[endpoint] || [];
  models.forEach(m => {
    const opt = document.createElement('div');
    opt.className = 'model-option';
    if (m.model === selectedModel) {
      opt.classList.add('active');
      if (modelSelectorLabel) modelSelectorLabel.textContent = m.label;
    }
    opt.dataset.model = m.model;
    opt.dataset.modality = m.modality;
    opt.dataset.textOnly = m.textOnly ? 'true' : 'false';
    opt.textContent = m.label;
    bindModelOption(opt);
    modelDropdown.appendChild(opt);
  });

  // Add Custom Option
  const customOpt = document.createElement('div');
  customOpt.className = 'model-option model-option-custom';
  if (selectedModel === 'custom') {
    customOpt.classList.add('active');
  }
  customOpt.dataset.model = 'custom';
  customOpt.textContent = 'Custom...';
  bindModelOption(customOpt);
  modelDropdown.appendChild(customOpt);
}

// Binds the click handler to a model option element.
function bindModelOption(opt) {
  opt.onclick = () => {
    const model = opt.dataset.model;
    if (model === 'custom') {
      modelDropdown.classList.add('hidden');
      if (customModelInput) customModelInput.value = selectedModel;
      if (customModalitySelect) customModalitySelect.value = customBaseModality;
      customModelModal.classList.remove('hidden');
      if (customModelInput) customModelInput.focus();
      return;
    }
    selectedModel = model;
    localStorage.setItem('lite_model', selectedModel);
    modelSelectorLabel.textContent = opt.textContent;

    modelDropdown.querySelectorAll('.model-option')
      .forEach(o => o.classList.remove('active'));
    opt.classList.add('active');
    modelDropdown.classList.add('hidden');
    updateAudioUI();
  };
}

// Initialize endpoint and model selectors
if (endpointSelectorLabel) {
  if (endpointDropdown) {
    const activeOpt = Array.from(endpointDropdown.querySelectorAll('.model-option'))
      .find(opt => opt.dataset.endpoint === selectedEndpointType);
    if (activeOpt) {
      endpointSelectorLabel.textContent = activeOpt.textContent;
      endpointDropdown.querySelectorAll('.model-option').forEach(o => {
        o.classList.toggle('active', o.dataset.endpoint === selectedEndpointType);
      });
    }
  }
}

populateModels(selectedEndpointType);

// Toggle endpoint dropdown
if (endpointSelectorBtn) {
  endpointSelectorBtn.onclick = (e) => {
    e.stopPropagation();
    endpointDropdown.classList.toggle('hidden');
  };
}

// Toggle model dropdown
if (modelSelectorBtn) {
  modelSelectorBtn.onclick = (e) => {
    e.stopPropagation();
    modelDropdown.classList.toggle('hidden');
  };
}

// Handle endpoint selection
if (endpointDropdown) {
  endpointDropdown.querySelectorAll('.model-option').forEach(opt => {
    opt.onclick = () => {
      const ep = opt.dataset.endpoint;
      selectedEndpointType = ep;
      localStorage.setItem('lite_endpoint_type', ep);
      if (endpointSelectorLabel) {
        endpointSelectorLabel.textContent = opt.textContent;
      }
      endpointDropdown.querySelectorAll('.model-option').forEach(o => o.classList.remove('active'));
      opt.classList.add('active');
      endpointDropdown.classList.add('hidden');

      // Refresh model dropdown
      populateModels(ep);
      // Select the first model as default
      const defaultModel = ENDPOINT_MODELS[ep][0].model;
      selectModelVisual(defaultModel);
    };
  });
}

// Close dropdowns on outside click
document.addEventListener('click', (e) => {
  if (modelDropdown && !modelDropdown.contains(e.target) && e.target !== modelSelectorBtn) {
    modelDropdown.classList.add('hidden');
  }
  if (endpointDropdown && !endpointDropdown.contains(e.target) && e.target !== endpointSelectorBtn) {
    endpointDropdown.classList.add('hidden');
  }
});

// Custom model modal handlers
if (customModelCancel) {
  customModelCancel.onclick = () => customModelModal.classList.add('hidden');
}
if (customModelApply) {
  customModelApply.onclick = () => {
    const customName = customModelInput.value.trim();
    if (!customName) return;
    selectedModel = customName;
    localStorage.setItem('lite_model', selectedModel);
    modelSelectorLabel.textContent = selectedModel;
    if (modelDropdown) {
      modelDropdown.querySelectorAll('.model-option')
        .forEach(o => o.classList.remove('active'));
    }
    // Custom modality from select
    if (customModalitySelect) {
      customBaseModality = customModalitySelect.value;
      localStorage.setItem('lite_custom_modality', customBaseModality);
    }
    customModelModal.classList.add('hidden');
    updateAudioUI();
  };
}

// Handle Audio Toggle button clicks
if (audioToggleBtn) {
  audioToggleBtn.onclick = () => {
    if (audioToggleBtn.disabled) return;
    selectedAudioEnabled = !selectedAudioEnabled;
    localStorage.setItem('lite_audio_enabled', selectedAudioEnabled);
    updateAudioUI();
  };
}

function updateAudioUI() {
  if (!audioToggleBtn) return;

  const ICON_AUDIO_ON = '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><path d="M19.07 4.93a10 10 0 010 14.14"/><path d="M15.54 8.46a5 5 0 010 7.07"/></svg>';
  const ICON_AUDIO_OFF = '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><line x1="23" y1="9" x2="17" y2="15"/><line x1="17" y1="9" x2="23" y2="15"/></svg>';

  // Resolve base modality of selected model
  let baseModality = 'TEXT';
  if (selectedModel === 'custom') {
    baseModality = customBaseModality;
  } else if (modelDropdown) {
    const selectedOpt =
      Array.from(modelDropdown.querySelectorAll('.model-option'))
        .find(opt => opt.dataset.model === selectedModel);
    if (selectedOpt) {
      baseModality = selectedOpt.dataset.modality || 'TEXT';
    }
  }

  if (baseModality === 'AUDIO') {
    // Natively AUDIO model -> force Speaker to ON and disable it
    audioToggleBtn.disabled = true;
    audioToggleBtn.classList.add('active');
    audioToggleBtn.innerHTML = ICON_AUDIO_ON;
    audioToggleBtn.title = 'Audio is required for this model';
  } else {
    // Natively TEXT model -> restore user's toggle state and enable it
    audioToggleBtn.disabled = false;
    audioToggleBtn.classList.toggle('active', selectedAudioEnabled);
    audioToggleBtn.innerHTML = selectedAudioEnabled ? ICON_AUDIO_ON : ICON_AUDIO_OFF;
    audioToggleBtn.title =
      selectedAudioEnabled ? 'Click to mute audio' : 'Click to unmute audio';
  }
}

// Run initial update
updateAudioUI();

function selectModelVisual(model) {
  selectedModel = model;
  localStorage.setItem('lite_model', selectedModel);

  if (modelDropdown) {
    let activeText = selectedModel;
    modelDropdown.querySelectorAll('.model-option')
      .forEach(o => {
        const isActive = o.dataset.model === selectedModel;
        o.classList.toggle('active', isActive);
        if (isActive) activeText = o.textContent;
      });
    if (modelSelectorLabel) modelSelectorLabel.textContent = activeText;
  } else if (modelSelectorLabel) {
    modelSelectorLabel.textContent = selectedModel;
  }
  updateAudioUI();
}



// Helper to dynamically resolve session configs — always Live API
function getResolvedSessionConfig() {
  let baseModality = 'TEXT';
  if (selectedModel === 'custom') {
    baseModality = customBaseModality;
  } else if (modelDropdown) {
    const selectedOpt =
      Array.from(modelDropdown.querySelectorAll('.model-option'))
        .find(opt => opt.dataset.model === selectedModel);
    if (selectedOpt) {
      baseModality = selectedOpt.dataset.modality || 'TEXT';
    }
  }

  let resolvedModality = 'TEXT';
  let resolvedUseTts = false;

  if (baseModality === 'AUDIO') {
    resolvedModality = 'AUDIO';
    resolvedUseTts = selectedAudioEnabled;
  } else {
    resolvedModality = 'TEXT';
    resolvedUseTts = selectedAudioEnabled;
  }

  return {
    model: selectedModel,
    response_modality: resolvedModality,
    use_tts: resolvedUseTts,
    endpoint_type: selectedEndpointType
  };
}

// Fetch server defaults if not in localStorage

async function initServerDefaults() {
  try {
    const resp = await fetch('/api/server_defaults');
    if (!resp.ok) throw new Error();
    const defaults = await resp.json();
    // Dynamically rebuild the model dropdown based on available server models
    if (defaults.available_models && modelDropdown) {
      modelDropdown.innerHTML = '';
      const allKnown = Object.values(ENDPOINT_MODELS).flat();
      defaults.available_models.forEach(modelName => {
        const found = allKnown.find(m => m.model === modelName);
        const label = found ? found.label : modelName;
        const modality = found ?
          found.modality :
          (modelName.includes('audio') || modelName.includes('live') ? 'AUDIO' : 'TEXT');
        const textOnly = found ? found.textOnly : false;

        const opt = document.createElement('div');
        opt.className = 'model-option';
        opt.dataset.model = modelName;
        opt.dataset.modality = modality;
        opt.dataset.textOnly = textOnly ? 'true' : 'false';
        opt.textContent = label;
        bindModelOption(opt);
        modelDropdown.appendChild(opt);
      });
      // Add custom option back
      const customOpt = document.createElement('div');
      customOpt.className = 'model-option model-option-custom';
      customOpt.dataset.model = 'custom';
      customOpt.innerHTML = 'Custom&hellip;';
      bindModelOption(customOpt);
      modelDropdown.appendChild(customOpt);
    }

    // Always fetch server default model if nothing explicitly preferred by user
    // or if the preferred model is not available
    const localModel = localStorage.getItem('lite_model');
    if (!localModel ||
      (defaults.available_models &&
        !defaults.available_models.includes(localModel))) {
      selectedModel = defaults.model;
      localStorage.setItem('lite_model', selectedModel);
    } else {
      selectedModel = localModel;
    }

    // Set default audio enabled based on server-configured use_tts
    if (!localStorage.getItem('lite_audio_enabled_set')) {
      selectedAudioEnabled = defaults.use_tts;
      localStorage.setItem('lite_audio_enabled', selectedAudioEnabled);
      localStorage.setItem('lite_audio_enabled_set', 'true');
    }

    if (modelDropdown) {
      let activeText = selectedModel;
      modelDropdown.querySelectorAll('.model-option').forEach(opt => {
        const isActive = opt.dataset.model === selectedModel;
        opt.classList.toggle('active', isActive);
        if (isActive) activeText = opt.textContent;
      });
      if (modelSelectorLabel) modelSelectorLabel.textContent = activeText;
    }
    updateAudioUI();
  } catch (e) {
    console.warn('Failed to fetch server defaults, using local defaults');
  }
}
initServerDefaults();

let currentGeminiMessageDiv = null;
let currentUserMessageDiv = null;
let currentThinkingContentDiv = null;
let videoSource = 'computer';  // "computer" or "robot"
let toolCallCounter = 0;
let lastAckRow = null;  // Reference to the current aggregated ack heartbeat row
let ackCount = 0;       // Number of consecutive ack calls in the current row

// ─────────────────────────────────────────────────────────────────────────────
// OVERLAY SYSTEM (Franka only)
// ─────────────────────────────────────────────────────────────────────────────

const overlayState = {
  sam3: true,
  graspPose: true,
  placeLocation: true,
  geminiPoint: true,
  pipeline: true,
};

const overlayData = {
  sam3Mask: null,      // { contours: [[[x,y],...], ...] } coords 0-1000 — from future backend msg
  graspPose: null,     // { x, y, angle_deg, quality } coords 0-1000 — from future backend msg
  pickCoords: null,    // { x, y, label } — captured from tool_call pick
  placeCoords: null,   // { x, y, label } — captured from tool_call place
  livePoints: null,    // existing draw_points server data
};

const PIPELINE_PHASES = ['detect', 'segment', 'plan', 'pick', 'transport', 'place'];
let currentPipelinePhase = null;
let donePipelinePhases = [];

// ── Coord helpers ────────────────────────────────────────────────

function _getImageBounds(rect) {
  let imgWidth = rect.width, imgHeight = rect.height, offsetX = 0, offsetY = 0;
  if (!robotCamImg.classList.contains('hidden') &&
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

// ── Master redraw ────────────────────────────────────────────────

function redrawOverlay() {
  const rect = overlayCanvas.parentElement.getBoundingClientRect();
  overlayCanvas.width = rect.width;
  overlayCanvas.height = rect.height;
  const ctx = overlayCanvas.getContext('2d');
  const b = _getImageBounds(rect);

  // SAM3 first — uses destination-out compositing (must be drawn before other overlays)
  if (overlayState.sam3 && overlayData.sam3Mask) {
    _drawSam3(ctx, b, overlayData.sam3Mask);
  }

  if (overlayState.graspPose && overlayData.graspPose) {
    _drawGraspPose(ctx, b, overlayData.graspPose);
  }

  if (overlayState.geminiPoint && overlayData.pickCoords) {
    _drawGeminiPoint(ctx, b, overlayData.pickCoords);
  }

  if (overlayState.placeLocation && overlayData.placeCoords) {
    _drawPlaceLocation(ctx, b, overlayData.placeCoords);
  }

  if (overlayData.livePoints) {
    _drawLivePoints(ctx, b, overlayData.livePoints);
  }
}

// ── Individual draw functions ────────────────────────────────────

function _drawSam3(ctx, b, maskData) {
  if (!maskData.contours || maskData.contours.length === 0) return;
  ctx.save();
  // Dark overlay over entire image area
  ctx.fillStyle = 'rgba(0,0,0,0.55)';
  ctx.fillRect(b.offsetX, b.offsetY, b.imgWidth, b.imgHeight);
  // Punch transparent hole for the object
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
  // Purple border around object
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
  const R = 7;
  const EXT = 4;
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

// ── Pipeline management ──────────────────────────────────────────

function setPipelinePhase(phase) {
  currentPipelinePhase = phase;
  const idx = PIPELINE_PHASES.indexOf(phase);
  if (idx > 0) {
    PIPELINE_PHASES.slice(0, idx).forEach(p => {
      if (!donePipelinePhases.includes(p)) donePipelinePhases.push(p);
    });
  }
  _updatePipelineBar();
}

function completePipelinePhase(phase) {
  if (!donePipelinePhases.includes(phase)) donePipelinePhases.push(phase);
  currentPipelinePhase = null;
  _updatePipelineBar();
}

function resetPipeline() {
  currentPipelinePhase = null;
  donePipelinePhases = [];
  _updatePipelineBar();
}

function _updatePipelineBar() {
  const bar = document.getElementById('pipeline-bar');
  if (!bar) return;
  bar.querySelectorAll('.pipeline-step').forEach(el => {
    const step = el.dataset.step;
    el.classList.toggle('active', step === currentPipelinePhase);
    el.classList.toggle('done', donePipelinePhases.includes(step));
  });
}

function _clearAllOverlayData() {
  overlayData.sam3Mask = null;
  overlayData.graspPose = null;
  overlayData.pickCoords = null;
  overlayData.placeCoords = null;
  resetPipeline();
  redrawOverlay();
}

// Show/hide franka overlay UI elements
function setFrankaOverlayUI(visible) {
  const controls = document.getElementById('overlay-controls');
  const bar = document.getElementById('pipeline-bar');
  if (controls) controls.classList.toggle('hidden', !visible);
  if (bar) bar.classList.toggle('hidden', !visible || !overlayState.pipeline);
  if (!visible) {
    overlayData.livePoints = null;
    redrawOverlay();
  }
}

// ── Checkbox listeners ───────────────────────────────────────────
(function initOverlayCheckboxes() {
  const checks = [
    ['ov-sam3',      'sam3'],
    ['ov-grasp',     'graspPose'],
    ['ov-place',     'placeLocation'],
    ['ov-gemini',    'geminiPoint'],
    ['ov-pipeline',  'pipeline'],
  ];
  checks.forEach(([id, key]) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.onchange = () => {
      overlayState[key] = el.checked;
      if (key === 'pipeline') {
        const bar = document.getElementById('pipeline-bar');
        if (bar) {
          bar.classList.toggle('hidden', !el.checked || selectedAgentName !== 'franka');
        }
      }
      redrawOverlay();
    };
  });
})();

// --- Latency Tracking ---
let MAX_HISTORY = 50;  // Sliding window for CI computation
let querySendTime = null;
let ttftRecorded = false;
let ttfcRecorded = false;

// Client-measured latency histories (sliding window, milliseconds).
let ttftHistory = [];  // Time to First Token (any model response)
let ttfcHistory = [];  // Time to First Function Call (tool invocation)
let ttltHistory = [];  // Time to Last Token (turn_complete)
// Server-reported latency breakdown histories (milliseconds).
let srvClientHistory = [];
let srvServerHistory = [];
let srvPrefillHistory = [];
let srvDecodeHistory = [];
let srvTtftHistory = [];
let srvClientTtftHistory = [];

function pushCapped(arr, val) {
  arr.push(val);
  if (arr.length > MAX_HISTORY) arr.shift();
}

function resetLatencyTimer() {
  querySendTime = performance.now();
  ttftRecorded = false;
  ttfcRecorded = false;
}

function computeCI(values) {
  const n = values.length;
  if (n === 0) return null;
  const mean = values.reduce((a, b) => a + b, 0) / n;
  if (n === 1) return { mean, lo: mean, hi: mean, n };
  const variance = values.reduce((s, v) => s + (v - mean) ** 2, 0) / (n - 1);
  const stderr = Math.sqrt(variance / n);
  const z = 1.96;  // 95% CI
  return { mean, lo: mean - z * stderr, hi: mean + z * stderr, n };
}

function formatCI(ci) {
  if (!ci) return '--';
  const m = (ci.mean / 1000).toFixed(2);
  if (ci.n === 1) return m + 's';
  const lo = (Math.max(0, ci.lo) / 1000).toFixed(2);
  const hi = (ci.hi / 1000).toFixed(2);
  return m + 's [' + lo + ', ' + hi + ']';
}

function formatCIms(ci) {
  if (!ci) return '--';
  const m = Math.round(ci.mean);
  if (ci.n === 1) return m + 'ms';
  const lo = Math.round(Math.max(0, ci.lo));
  const hi = Math.round(ci.hi);
  return m + 'ms [' + lo + ', ' + hi + ']';
}

function recordTTFT() {
  if (!ttftRecorded && querySendTime !== null) {
    const ttft = performance.now() - querySendTime;
    pushCapped(ttftHistory, ttft);
    const ci = computeCI(ttftHistory);
    const el = document.getElementById('ttft-val');
    const countEl = document.getElementById('ttft-count');
    if (el) el.textContent = formatCI(ci);
    if (countEl) countEl.textContent = 'n=' + ci.n;
    ttftRecorded = true;
  }
}

function recordTTFC() {
  if (!ttfcRecorded && querySendTime !== null) {
    const ttfc = performance.now() - querySendTime;
    pushCapped(ttfcHistory, ttfc);
    const ci = computeCI(ttfcHistory);
    const el = document.getElementById('ttfc-val');
    const countEl = document.getElementById('ttfc-count');
    if (el) el.textContent = formatCI(ci);
    if (countEl) countEl.textContent = 'n=' + ci.n;
    ttfcRecorded = true;
  }
}

function recordTTLT() {
  if (querySendTime !== null) {
    const ttlt = performance.now() - querySendTime;
    pushCapped(ttltHistory, ttlt);
    const ci = computeCI(ttltHistory);
    const el = document.getElementById('ttlt-val');
    const countEl = document.getElementById('ttlt-count');
    if (el) el.textContent = formatCI(ci);
    if (countEl) countEl.textContent = 'n=' + ci.n;
  }
}

// Helper to update a metric's DOM elements.
function updateMetric(prefix, history) {
  const ci = computeCI(history);
  const el = document.getElementById(prefix + '-val');
  const countEl = document.getElementById(prefix + '-count');
  if (el) el.textContent = ci ? formatCIms(ci) : '--';
  if (countEl) countEl.textContent = ci ? 'n=' + ci.n : '';
}

function updateServerLatency(sl, serverTtftMs) {
  // Only use client TTFT if it was freshly recorded for the current turn.
  const clientTtftMs = ttftRecorded && ttftHistory.length > 0 ?
    ttftHistory[ttftHistory.length - 1] :
    null;

  // ① Client Overhead = Client TTFT - Server TTFT
  if (clientTtftMs !== null && serverTtftMs > 0) {
    const val = Math.max(0, clientTtftMs - serverTtftMs);
    pushCapped(srvClientHistory, val);
    updateMetric('srv-client', srvClientHistory);
  }

  // ② Server Overhead = Server TTFT - known inference
  if (serverTtftMs > 0 && sl.request_ttft_ms > 0) {
    const known = (sl.prefill_queue_ms || 0) + (sl.prefill_ms || 0) +
      (sl.decode_queue_ms || 0) + (sl.decode_ttft_ms || 0);
    const val = Math.max(0, serverTtftMs - known);
    pushCapped(srvServerHistory, val);
    updateMetric('srv-server', srvServerHistory);
  }

  // ③ Prefill
  if (sl.prefill_ms > 0) {
    pushCapped(srvPrefillHistory, sl.prefill_ms);
    updateMetric('srv-prefill', srvPrefillHistory);
  }

  // ④ Decode
  if (sl.decode_ttft_ms > 0) {
    pushCapped(srvDecodeHistory, sl.decode_ttft_ms);
    updateMetric('srv-decode', srvDecodeHistory);
  }

  // Reference: Srv TTFT
  if (sl.request_ttft_ms > 0) {
    pushCapped(srvTtftHistory, sl.request_ttft_ms);
    updateMetric('srv-ttft', srvTtftHistory);
  }

  // Reference: Client TTFT
  if (clientTtftMs !== null) {
    pushCapped(srvClientTtftHistory, clientTtftMs);
    updateMetric('srv-client-ttft', srvClientTtftHistory);
  }
}

function formatTokenCount(n) {
  if (n >= 1000) return (n / 1000).toFixed(1) + 'K';
  return String(n);
}

function updateTokenUsage(tu) {
  if (!tu) return;

  // Footer bar: CTX utilization %, input, and cumulative output tokens
  const ctxEl = document.getElementById('tok-ctx-pct');
  const inputBarEl = document.getElementById('tok-input-bar');
  const outputBarEl = document.getElementById('tok-output-bar');
  if (ctxEl) {
    const pct = tu.context_window_utilization_pct;
    ctxEl.textContent = pct != null ? pct.toFixed(1) + '%' : '--';
    // Color-code: green < 50%, yellow 50-80%, red > 80%
    if (pct != null) {
      ctxEl.style.color = pct >= 80 ? '#e74c3c' : pct >= 50 ? '#f39c12' : '';
    }
  }
  if (inputBarEl) {
    inputBarEl.textContent = formatTokenCount(tu.prompt_token_count);
  }
  if (outputBarEl) {
    outputBarEl.textContent = formatTokenCount(tu.cumulative_output_tokens);
  }

  // Detail panel: per-field breakdown
  const setEl = (id, val) => {
    const el = document.getElementById(id);
    if (el) el.textContent = val;
  };
  setEl('tok-prompt-val', formatTokenCount(tu.prompt_token_count));
  setEl('tok-response-val', formatTokenCount(tu.response_token_count));
  setEl('tok-thoughts-val', formatTokenCount(tu.thoughts_token_count));
  setEl(
    'tok-total-val',
    formatTokenCount(tu.total_token_count) + ' / ' +
    formatTokenCount(tu.context_window_limit));
  setEl('tok-cumulative-val', formatTokenCount(tu.cumulative_output_tokens));

  // Per-modality breakdown (compact format)
  const modality = tu.prompt_tokens_by_modality || {};
  const parts = Object.entries(modality)
    .filter(([, v]) => v > 0)
    .map(([k, v]) => k + ':' + formatTokenCount(v));
  setEl('tok-modality-val', parts.length > 0 ? parts.join(' ') : '--');
}

let mediaHandler = new MediaHandler();
let audioFlushPending = false;
let geminiClient = new GeminiClient({
  onOpen: () => {
    statusLabel.className = 'status-label connected';
    connectBtn.classList.add('danger');
    connectBtn.title = 'Disconnect from Agent';
    connectBtn.disabled = false;
    connectBtn.textContent = 'Disconnect';
    if (modelSelectorBtn) modelSelectorBtn.disabled = true;
    if (audioToggleBtn) audioToggleBtn.disabled = true;
    if (agentSelectorBtn) agentSelectorBtn.disabled = true;

    // Apply the current video source mode — derive from agent, not stale videoSource
    applyVideoSource(selectedAgentName === 'human' ? 'computer' : selectedAgentName);

    // Start session timer
    startSessionTimer();
  },
  onMessage: (event) => {
    if (typeof event.data === 'string') {
      try {
        const msg = JSON.parse(event.data);
        handleJsonMessage(msg);
      } catch (e) {
        console.error('Parse error:', e);
      }
    } else {
      if (audioFlushPending) {
        mediaHandler.stopAudioPlayback();
        audioFlushPending = false;
      }
      mediaHandler.playAudio(event.data);
    }
  },
  onClose: (e) => {
    console.log('WS Closed:', e);
    statusLabel.className = 'status-label disconnected';
    statusLabel.textContent = 'Disconnected';
    stopSessionTimer();
    resetUI();
  },
  onError: (e) => {
    console.error('WS Error:', e);
    statusLabel.className = 'status-label error';
    statusLabel.textContent = 'Error';
  },
});

// --- Video Source Toggle ---

function applyVideoSource(source, { notifyServer = true } = {}) {
  videoSource = source;
  setFrankaOverlayUI(selectedAgentName === 'franka' && source !== 'computer');

  // Tell the server which mode we're in (skip when the server itself initiated)
  if (notifyServer && geminiClient.isConnected()) {
    geminiClient.send(JSON.stringify({ type: 'video_source', source: source }));
  }

  const videoContainer = robotCamImg.parentElement;

  if (source === 'computer') {
    // Computer mode: webcam → Gemini directly
    robotCamImg.classList.add('hidden');
    robotCamImg.src = '';
    videoContainer.classList.remove('atari-aspect');
    videoContainer.classList.remove('spot-aspect');

    // Start webcam
    videoPlaceholder.classList.add('hidden');
    videoPreview.classList.remove('hidden');
    if (!mediaHandler.videoStream) {
      mediaHandler
        .startVideo(
          videoPreview,
          (base64Data) => {
            if (geminiClient.isConnected()) {
              geminiClient.sendImage(base64Data);
            }
          })
        .catch(e => {
          console.error('Camera start failed:', e);
          videoPlaceholder.textContent = '⚠️ Camera blocked — allow access in browser';
          videoPlaceholder.classList.remove('hidden');
          videoPreview.classList.add('hidden');
        });
    }
  } else {
    // Robot / Atari mode: camera poller → Gemini
    mediaHandler.stopVideo(videoPreview);
    videoPreview.classList.add('hidden');
    videoPlaceholder.classList.add('hidden');

    // Atari uses 8/5 aspect ratio
    if (source === 'atari') {
      videoContainer.classList.add('atari-aspect');
      videoContainer.classList.remove('spot-aspect');
    } else {
      videoContainer.classList.remove('atari-aspect');
      videoContainer.classList.toggle('spot-aspect', source === 'spot');
    }

    // Show robot cam (stitched MJPEG from camera poller)
    robotCamImg.classList.remove('hidden');
    robotCamImg.src = '/api/camera?' + Date.now();

    // Retry after a short delay
    if (geminiClient.isConnected()) {
      setTimeout(() => {
        robotCamImg.src = '/api/camera?' + Date.now();
      }, 2000);
    }
  }
}

// --- Agent Selector Click Bindings ---
if (agentSelectorBtn) {
  agentSelectorBtn.onclick = (e) => {
    e.stopPropagation();
    agentDropdown.classList.toggle('hidden');
  };
}

document.addEventListener('click', (e) => {
  if (agentDropdown && !agentDropdown.contains(e.target) &&
    e.target !== agentSelectorBtn) {
    agentDropdown.classList.add('hidden');
  }
});

if (agentDropdown) {
  agentDropdown.querySelectorAll('.agent-option').forEach(opt => {
    opt.onclick = () => {
      const agent = opt.dataset.agent;
      selectAgent(agent);

      if (agent === 'human') {
        applyVideoSource('computer');
      } else if (agent === 'spot') {
        applyVideoSource('spot');
      } else if (agent === 'franka') {
        applyVideoSource('franka');
      }

      agentDropdown.classList.add('hidden');
    };
  });
}

function selectAgent(type) {
  if (typeof selectedAgentName !== 'undefined') {
    selectedAgentName = type;
  }

  // Clear active state from all agent dropdown options
  if (agentDropdown) {
    agentDropdown.querySelectorAll('.agent-option')
      .forEach(o => o.classList.remove('active'));
    const activeOpt =
      Array.from(agentDropdown.querySelectorAll('.agent-option'))
        .find(o => o.dataset.agent === type);
    if (activeOpt) {
      activeOpt.classList.add('active');
      if (agentSelectorLabel) {
        agentSelectorLabel.textContent = activeOpt.textContent;
      }
    }
  }
  // Show overlay controls as soon as Franka is selected, even without video stream
  setFrankaOverlayUI(type === 'franka');
}

// --- Chat Sidebar Toggle ---
chatToggle.onclick = () => {
  chatSidebar.classList.toggle('collapsed');
};

// --- Message Handling ---

function handleJsonMessage(msg) {
  if (msg.type === 'heartbeat_sent') {
    resetLatencyTimer();
    return;
  }
  // Future backend messages for SAM3 segmentation and grasp planning
  if (msg.type === 'segmentation_mask') {
    overlayData.sam3Mask = msg.data || null;
    setPipelinePhase('segment');
    redrawOverlay();
    return;
  }
  if (msg.type === 'grasp_pose') {
    overlayData.graspPose = msg.data || null;
    setPipelinePhase('plan');
    redrawOverlay();
    return;
  }
  if (msg.type === 'video_source') {
    applyVideoSource(msg.source, { notifyServer: false });
    return;
  }
  if (msg.type === 'run_instruction') {
    // Server-side run_instruction is blocking — show overlay until user clicks Done.
    const overlay = document.getElementById('instruction-overlay');
    const textEl = document.getElementById('instruction-text');
    if (overlay && textEl) {
      textEl.textContent = msg.instruction || '';
      overlay.style.display = 'flex';
    }
    return;
  }
  if (msg.type === 'interrupted') {
    audioFlushPending = true;
    currentGeminiMessageDiv = null;
    currentUserMessageDiv = null;
    currentThinkingContentDiv = null;
  } else if (msg.type === 'turn_complete') {
    recordTTLT();
    currentGeminiMessageDiv = null;
    currentUserMessageDiv = null;
    currentThinkingContentDiv = null;
  } else if (msg.type === 'telemetry') {
    if (msg.server_latency || msg.server_ttft_ms) {
      updateServerLatency(
        msg.server_latency || {}, msg.server_ttft_ms || 0);
    }
    if (msg.token_usage) {
      updateTokenUsage(msg.token_usage);
    }

  } else if (msg.type === 'draw_points') {
    recordTTFT();
    overlayData.livePoints = msg.points;
    redrawOverlay();
  } else if (msg.type === 'tool_call_start') {
    if (selectedAgentName === 'franka') {
      if (msg.name === 'pick' && msg.args) {
        overlayData.pickCoords = { x: msg.args.x, y: msg.args.y, label: msg.args.label || '' };
        overlayData.placeCoords = null;
        overlayData.sam3Mask = null;
        overlayData.graspPose = null;
        setPipelinePhase('pick');
        redrawOverlay();
      } else if (msg.name === 'place' && msg.args) {
        overlayData.pickCoords = null;
        overlayData.placeCoords = { x: msg.args.x, y: msg.args.y };
        setPipelinePhase('place');
        redrawOverlay();
      }
    }
  } else if (msg.type === 'clear_overlay') {
    overlayData.livePoints = null;
    redrawOverlay();
  } else if (msg.type === 'user' || msg.type === 'user_transcript') {
    // User message breaks ack streak
    lastAckRow = null;
    ackCount = 0;
    currentThinkingContentDiv = null;
    // Reset latency timer on voice input (user_transcript from server).
    if (msg.type === 'user_transcript' && querySendTime === null) {
      resetLatencyTimer();
    }
    if (currentUserMessageDiv) {
      currentUserMessageDiv.textContent += msg.text;
      const isAtBottomUser =
        chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
      if (isAtBottomUser) chatLog.scrollTop = chatLog.scrollHeight;
    } else {
      currentUserMessageDiv = appendMessage('user', msg.text);
    }
  } else if (msg.type === 'text_input') {
    if (typeof msg.text === 'string' &&
      msg.text.trimStart().startsWith('[HEARTBEAT]')) {
      return;
    }
    currentThinkingContentDiv = null;
    appendMessage('system', `📨 ${msg.text}`);
  } else if (msg.type === 'gemini' || msg.type === 'gemini_transcript') {
    // Gemini message breaks ack streak
    lastAckRow = null;
    ackCount = 0;
    currentThinkingContentDiv = null;
    recordTTFT();
    if (currentGeminiMessageDiv) {
      currentGeminiMessageDiv.textContent += msg.text;
      const isAtBottomGemini =
        chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
      if (isAtBottomGemini) chatLog.scrollTop = chatLog.scrollHeight;
    } else {
      currentGeminiMessageDiv = appendMessage('gemini', msg.text);
    }
  } else if (msg.type === 'gemini_thought') {
    // Gemini thought message breaks ack streak
    lastAckRow = null;
    ackCount = 0;
    recordTTFT();
    currentGeminiMessageDiv = null;
    currentUserMessageDiv = null;
    if (currentThinkingContentDiv) {
      currentThinkingContentDiv.textContent += msg.text;
      const isAtBottom =
        chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
      if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
    } else {
      currentThinkingContentDiv = appendThoughtMessage(msg.text);
    }
  } else if (msg.type === 'tool_call') {
    currentThinkingContentDiv = null;
    recordTTFT();
    recordTTFC();
    toolCallCounter++;
    if (msg.name === 'ack') {
      // Aggregate consecutive ack calls into a single heartbeat row
      ackCount++;
      if (lastAckRow) {
        // Update existing heartbeat row
        const countSpan = lastAckRow.querySelector('.hb-count');
        if (countSpan) countSpan.textContent = `x${ackCount}`;
        // Scroll if at bottom
        const isAtBottom =
          chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight <
          50;
        if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
      } else {
        // Create new heartbeat row
        const row = document.createElement('div');
        row.className = 'message tool_call heartbeat-row';
        row.innerHTML = `<span class="hb-icon">❤️</span>` +
          `<span class="hb-count">x${ackCount}</span>`;
        const isAtBottom =
          chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight <
          50;
        chatLog.appendChild(row);
        if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
        lastAckRow = row;
      }
    } else {
      // Non-ack tool call — reset ack aggregation
      lastAckRow = null;
      ackCount = 0;
      const args = msg.args ? JSON.stringify(msg.args) : '';
      const result =
        msg.result !== undefined ? formatFunctionResult(msg.result) : '';

      if (result) {
        // Render as collapsible details
        const details = document.createElement('details');
        details.className = 'message tool_call_details';
        details.open = true; // Open by default

        const summary = document.createElement('summary');
        summary.className = 'tool_call_summary';
        summary.textContent = `🔧 [${toolCallCounter}] ${msg.name}(${args})`;
        details.appendChild(summary);

        const contentDiv = document.createElement('div');
        contentDiv.className = 'tool_response_content';
        contentDiv.textContent = result;
        details.appendChild(contentDiv);

        const isAtBottom =
          chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
        chatLog.appendChild(details);
        if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
      } else {
        // Fallback if no result yet
        appendMessage(
          'tool_call', `🔧 [${toolCallCounter}] ${msg.name}(${args})`);
      }

      if (msg.name === 'run_instruction') {
        updateRobotStatus(
          'executing', 'Executing: ' + (msg.args?.instruction || '...'));
      } else if (msg.name === 'stop') {
        updateRobotStatus('idle', 'Stopped');
      } else if (msg.name === 'reset') {
        updateRobotStatus('idle', 'Resetting...');
      }

      // Overlay for home/stop (tool_call_start handles pick/place)
      if (selectedAgentName === 'franka') {
        if (msg.name === 'home' || msg.name === 'stop') {
          _clearAllOverlayData();
        }
      }
    }
  } else if (msg.type === 'tool_response') {
    currentThinkingContentDiv = null;
    // Tool response breaks ack streak
    lastAckRow = null;
    ackCount = 0;
    appendMessage(
      'tool_response', `✅ ${msg.name}: ${formatFunctionResult(msg.result)}`);
    // Pipeline advancement (Franka)
    if (selectedAgentName === 'franka') {
      if (msg.name === 'pick') {
        overlayData.pickCoords = null;
        completePipelinePhase('pick');
        setPipelinePhase('transport');
        redrawOverlay();
      } else if (msg.name === 'place') {
        overlayData.placeCoords = null;
        redrawOverlay();
        PIPELINE_PHASES.forEach(p => { if (!donePipelinePhases.includes(p)) donePipelinePhases.push(p); });
        currentPipelinePhase = null;
        _updatePipelineBar();
        setTimeout(() => { _clearAllOverlayData(); }, 3000);
      }
    }
  }
}

// --- Overlay Drawing ---

function drawPointsOverlay(points) {
  const rect = overlayCanvas.parentElement.getBoundingClientRect();
  overlayCanvas.width = rect.width;
  overlayCanvas.height = rect.height;

  // Compute actual image bounds within the container (object-fit: contain).
  let imgWidth = rect.width;
  let imgHeight = rect.height;
  let offsetX = 0;
  let offsetY = 0;

  if (!robotCamImg.classList.contains('hidden') &&
    robotCamImg.naturalWidth > 0 && robotCamImg.naturalHeight > 0) {
    const imgAspect = robotCamImg.naturalWidth / robotCamImg.naturalHeight;
    const containerAspect = rect.width / rect.height;

    if (containerAspect > imgAspect) {
      imgHeight = rect.height;
      imgWidth = rect.height * imgAspect;
      offsetX = (rect.width - imgWidth) / 2;
    } else {
      imgWidth = rect.width;
      imgHeight = rect.width / imgAspect;
      offsetY = (rect.height - imgHeight) / 2;
    }
  }

  const ctx = overlayCanvas.getContext('2d');
  const POINT_RADIUS = 7;

  for (const c of points) {
    const cx = offsetX + (c.x / 1000) * imgWidth;
    const cy = offsetY + (c.y / 1000) * imgHeight;

    // Draw filled point with contrast outline
    ctx.beginPath();
    ctx.arc(cx, cy, POINT_RADIUS, 0, 2 * Math.PI);
    ctx.fillStyle = 'rgba(0, 120, 255, 0.9)';
    ctx.fill();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.95)';
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Draw label with background pill
    if (c.label) {
      ctx.font = 'bold 13px sans-serif';
      const labelX = cx + POINT_RADIUS + 8;
      const labelY = cy + 4;
      const metrics = ctx.measureText(c.label);
      const padX = 5, padY = 3;

      // Background pill
      ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
      const pillX = labelX - padX;
      const pillY = labelY - 10 - padY;
      const pillW = metrics.width + padX * 2;
      const pillH = 14 + padY * 2;
      ctx.beginPath();
      ctx.roundRect(pillX, pillY, pillW, pillH, 4);
      ctx.fill();

      // Label text
      ctx.fillStyle = 'rgba(255, 255, 255, 0.95)';
      ctx.fillText(c.label, labelX, labelY);
    }
  }
}

function clearOverlay() {
  const rect = overlayCanvas.parentElement.getBoundingClientRect();
  overlayCanvas.width = rect.width;
  overlayCanvas.height = rect.height;
  // Setting width/height clears the canvas.
}

let robotState = 'idle';
let robotStateText = '';

function updateRobotStatus(state, text) {
  robotState = state;
  robotStateText = text;
  refreshStatusLabel();
}

function refreshStatusLabel() {
  if (!statusLabel) return;
  // Robot executing takes priority over connection state display
  if (robotState === 'executing' && geminiClient.isConnected()) {
    statusLabel.className = 'status-label executing';
    statusLabel.textContent = robotStateText;
    return;
  }
  // Otherwise show connection state (timer is updated separately)
  if (geminiClient.isConnected()) {
    statusLabel.className = 'status-label connected';
    if (!sessionStartTime) {
      statusLabel.textContent = 'Connected';
    }
  }
}

function appendMessage(type, text) {
  const msgDiv = document.createElement('div');
  msgDiv.className = `message ${type}`;
  msgDiv.textContent = text;
  const isAtBottom =
    chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
  chatLog.appendChild(msgDiv);
  if (isAtBottom) {
    chatLog.scrollTop = chatLog.scrollHeight;
  }
  return msgDiv;
}

function appendThoughtMessage(text) {
  const details = document.createElement('details');
  details.className = 'message thought';
  details.open = false; // Start collapsed

  const summary = document.createElement('summary');
  summary.textContent = 'Thinking Process';
  details.appendChild(summary);

  const contentDiv = document.createElement('div');
  contentDiv.className = 'thought-content';
  contentDiv.textContent = text;
  details.appendChild(contentDiv);

  const isAtBottom =
    chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
  chatLog.appendChild(details);
  if (isAtBottom) {
    chatLog.scrollTop = chatLog.scrollHeight;
  }
  return contentDiv;
}

// --- Connect ---

connectBtn.onclick = async () => {
  if (geminiClient.isConnected()) {
    geminiClient.disconnect();
  } else {
    // Clear previous conversation history when starting a new connection
    chatLog.innerHTML = '';
    const memoryLog = document.getElementById('memory-log');
    if (memoryLog) memoryLog.innerHTML = '';

    statusLabel.className = 'status-label connecting';
    statusLabel.textContent = 'Connecting…';
    connectBtn.disabled = true;

    try {
      await mediaHandler.initializeAudio();

      const sessionCfg = getResolvedSessionConfig();
      const connParams = {
        agent_name: selectedAgentName,
        model: sessionCfg.model,
        response_modality: sessionCfg.response_modality,
        use_tts: sessionCfg.use_tts,
        endpoint_type: sessionCfg.endpoint_type,
        thinking_level: document.getElementById('config-thinking-level') ?
          document.getElementById('config-thinking-level').value : undefined,
      };
      geminiClient.connect(connParams);
    } catch (error) {
      console.error('Connection error:', error);
      statusLabel.className = 'status-label error';
      statusLabel.textContent = 'Error';
      connectBtn.disabled = false;
    }
  }
};

micBtn.onclick = async () => {
  if (mediaHandler.isRecording) {
    mediaHandler.stopAudio();
    micBtn.classList.remove('active');
    micBtn.title = 'Start Microphone';
  } else {
    try {
      await mediaHandler.startAudio((data) => {
        if (geminiClient.isConnected()) {
          geminiClient.send(data);
        }
      });
      micBtn.classList.add('active');
      micBtn.title = 'Stop Microphone';
    } catch (e) {
      alert('Could not start audio capture');
    }
  }
};

sendBtn.onclick = sendText;
textInput.onkeypress = (e) => {
  if (e.key === 'Enter') sendText();
};

const imageBtn = document.getElementById('imageBtn');
const imageFileInput = document.getElementById('imageFileInput');

if (imageBtn && imageFileInput) {
  imageBtn.onclick = () => {
    if (!geminiClient.isConnected()) return;
    imageFileInput.value = '';
    imageFileInput.click();
  };

  imageFileInput.onchange = () => {
    const file = imageFileInput.files[0];
    if (!file || !geminiClient.isConnected()) return;
    const reader = new FileReader();
    reader.onload = (e) => {
      const dataUrl = e.target.result;
      // dataUrl = "data:image/jpeg;base64,<data>"
      const commaIdx = dataUrl.indexOf(',');
      const mimeType = dataUrl.substring(5, dataUrl.indexOf(';'));
      const base64Data = dataUrl.substring(commaIdx + 1);
      const caption = textInput.value.trim();
      geminiClient.send(JSON.stringify({
        type: 'user_image',
        mime_type: mimeType,
        data: base64Data,
        caption: caption,
      }));
      // Show thumbnail in chat log
      const imgEl = document.createElement('img');
      imgEl.src = dataUrl;
      imgEl.style.cssText = 'max-width:180px;max-height:120px;border-radius:4px;display:block;margin-top:4px;';
      const wrapper = document.createElement('div');
      wrapper.className = 'message user';
      if (caption) {
        const captionEl = document.createElement('div');
        captionEl.textContent = caption;
        wrapper.appendChild(captionEl);
        textInput.value = '';
      }
      wrapper.appendChild(imgEl);
      const chatLog = document.getElementById('chat-log');
      if (chatLog) { chatLog.appendChild(wrapper); chatLog.scrollTop = chatLog.scrollHeight; }
    };
    reader.readAsDataURL(file);
  };
}

function sendText() {
  const text = textInput.value;
  if (text && geminiClient.isConnected()) {
    resetLatencyTimer();
    geminiClient.sendText(text);
    appendMessage('user', text);
    textInput.value = '';
  }
}

function resetUI() {
  connectBtn.classList.remove('danger');
  connectBtn.textContent = 'Connect';
  connectBtn.title = 'Connect to Agent';
  if (modelSelectorBtn) modelSelectorBtn.disabled = false;
  if (audioToggleBtn) audioToggleBtn.disabled = false;
  if (agentSelectorBtn) agentSelectorBtn.disabled = false;
  updateAudioUI();

  mediaHandler.stopAudio();
  mediaHandler.stopVideo(videoPreview);
  robotCamImg.classList.add('hidden');
  robotCamImg.src = '';
  videoPlaceholder.classList.remove('hidden');

  micBtn.classList.remove('active');
  micBtn.title = 'Start Microphone';
  connectBtn.disabled = false;
  updateRobotStatus('idle', 'Stopped');
  toolCallCounter = 0;
  lastAckRow = null;
  ackCount = 0;

  // Hide the run_instruction overlay if still showing.
  const overlay = document.getElementById('instruction-overlay');
  if (overlay) overlay.style.display = 'none';

  // Clear all franka overlays and pipeline
  _clearAllOverlayData();
  setFrankaOverlayUI(false);
}

// --- run_instruction "Done" button ---
(function () {
  const doneBtn = document.getElementById('instruction-done-btn');
  if (!doneBtn) return;
  doneBtn.addEventListener('click', () => {
    if (geminiClient && geminiClient.isConnected()) {
      geminiClient.send(JSON.stringify({ type: 'instruction_done' }));
    }
    const overlay = document.getElementById('instruction-overlay');
    if (overlay) overlay.style.display = 'none';
  });
})();

// --- Settings Modal and Theme Toggle ---

let settingsBtn = document.getElementById('settings-btn');
let settingsModal = document.getElementById('settings-modal');
let closeModalBtn = document.getElementById('close-modal-btn');
let themeRadios = document.querySelectorAll('input[name="theme"]');
let modalTabs = document.querySelectorAll('.modal-sidebar li[data-tab]');

let selectedAgentName = 'human';  // Track the currently selected agent

// Tab switching
modalTabs.forEach(tab => {
  tab.onclick = () => {
    modalTabs.forEach(t => t.classList.remove('active'));
    tab.classList.add('active');
    const tabName = tab.dataset.tab;
    document.querySelectorAll('.modal-panel')
      .forEach(p => p.classList.add('hidden'));
    document.getElementById(`modal-panel-${tabName}`)
      .classList.remove('hidden');
  };
});

// Fetch and display agent config
async function loadAgentConfig(name, model) {
  try {
    let url = `/api/agent_config/${encodeURIComponent(name)}?endpoint_type=${encodeURIComponent(selectedEndpointType)}`;
    if (model) {
      url += `&model=${encodeURIComponent(model)}`;
    }
    const resp = await fetch(url);
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const cfg = await resp.json();

    document.getElementById('config-title').textContent =
      `${cfg.name} Agent Config`;
    const sessionCfg = getResolvedSessionConfig();
    document.getElementById('config-model').textContent = sessionCfg.model;
    document.getElementById('config-modality').textContent =
      sessionCfg.response_modality;
    document.getElementById('config-tts').textContent =
      sessionCfg.use_tts ? 'Enabled' : 'Disabled';
    document.getElementById('config-tts-voice').textContent = cfg.tts_voice;
    document.getElementById('config-endpoint').textContent = cfg.endpoint_type;
    document.getElementById('config-service').textContent = cfg.service_address;

    // Tools
    const toolsDiv = document.getElementById('config-tools');
    toolsDiv.innerHTML = '';
    if (cfg.tools && cfg.tools.length > 0) {
      const disabledSet = new Set(cfg.disabled_tools || []);
      cfg.tools.forEach(tool => {
        const label = document.createElement('label');
        label.className = 'tool-checkbox-label';
        label.title = tool.description || '';
        const cb = document.createElement('input');
        cb.type = 'checkbox';
        cb.className = 'tool-checkbox';
        cb.dataset.toolName = tool.name;
        cb.checked = !disabledSet.has(tool.name);
        label.appendChild(cb);
        const span = document.createElement('span');
        span.textContent = tool.name;
        label.appendChild(span);
        toolsDiv.appendChild(label);
      });
    } else {
      toolsDiv.textContent = 'None';
    }

    // Instructions
    const devPromptEl = document.getElementById('config-developer-prompt');
    if (devPromptEl) devPromptEl.value = cfg.developer_instruction || '';

    const hbTextEl = document.getElementById('config-heartbeat-text');
    if (hbTextEl) hbTextEl.value = cfg.heartbeat_text || '';

    const thinkingLevelEl = document.getElementById('config-thinking-level');
    if (thinkingLevelEl && cfg.thinking_level) {
      thinkingLevelEl.value = cfg.thinking_level;
    }

    document.getElementById('config-heartbeat-interval').value =
      cfg.heartbeat_interval_seconds !== undefined ? cfg.heartbeat_interval_seconds : '';
    document.getElementById('config-heartbeat-min-delay').value =
      cfg.heartbeat_min_delay_seconds !== undefined ? cfg.heartbeat_min_delay_seconds : '';
    document.getElementById('config-use-event-driven-heartbeat').checked =
      !!cfg.use_event_driven_heartbeat;

  } catch (e) {
    console.error('Failed to load agent config:', e);
    const devPromptEl = document.getElementById('config-developer-prompt');
    if (devPromptEl) devPromptEl.value = 'Error loading config';
  }
}

if (settingsBtn) {
  settingsBtn.onclick = () => {
    settingsModal.classList.remove('hidden');
    // Reset to Agent Config tab
    modalTabs.forEach(t => t.classList.remove('active'));
    document.getElementById('modal-tab-config').classList.add('active');
    document.querySelectorAll('.modal-panel')
      .forEach(p => p.classList.add('hidden'));
    document.getElementById('modal-panel-config').classList.remove('hidden');
    loadAgentConfig(selectedAgentName, selectedModel);
  };
}

let updatePromptBtn = document.getElementById('update-prompt-btn');
if (updatePromptBtn) {
  updatePromptBtn.onclick = async () => {
    const newDeveloperPrompt =
      document.getElementById('config-developer-prompt') ? document.getElementById('config-developer-prompt').value : '';
    const newHeartbeatText =
      document.getElementById('config-heartbeat-text') ? document.getElementById('config-heartbeat-text').value : '';
    const heartbeatInterval =
      document.getElementById('config-heartbeat-interval').value;
    const heartbeatMinDelay =
      document.getElementById('config-heartbeat-min-delay').value;
    const useEventDrivenHeartbeat =
      document.getElementById('config-use-event-driven-heartbeat').checked;
    updatePromptBtn.disabled = true;
    updatePromptBtn.textContent = 'Updating...';
    try {
      const resp = await fetch('/api/config/instructions', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          agent_name: selectedAgentName,
          developer_instruction: newDeveloperPrompt,
          heartbeat_text: newHeartbeatText,
          heartbeat_interval_seconds: heartbeatInterval !== '' ? parseFloat(heartbeatInterval) : null,
          heartbeat_min_delay_seconds: heartbeatMinDelay !== '' ? parseFloat(heartbeatMinDelay) : null,
          use_event_driven_heartbeat: useEventDrivenHeartbeat,
          disabled_tools: Array.from(
            document.querySelectorAll('.tool-checkbox:not(:checked)')
          ).map(cb => cb.dataset.toolName),
        }),
      });
      const data = await resp.json();
      if (data.success) {
        updatePromptBtn.textContent = 'Updated ✅';
        setTimeout(() => {
          loadAgentConfig(selectedAgentName, selectedModel);
        }, 1000);
      } else {
        updatePromptBtn.textContent = 'Failed ❌';
      }
    } catch (e) {
      console.error('Failed to update instructions:', e);
      updatePromptBtn.textContent = 'Error ❌';
    }
    setTimeout(() => {
      updatePromptBtn.disabled = false;
      updatePromptBtn.textContent = 'Update Instructions';
    }, 2000);
  };
}

let resetPromptBtn = document.getElementById('reset-prompt-btn');
if (resetPromptBtn) {
  resetPromptBtn.onclick = async () => {
    resetPromptBtn.disabled = true;
    resetPromptBtn.textContent = 'Resetting...';
    try {
      const resp = await fetch(`/api/config/instructions?agent_name=${encodeURIComponent(selectedAgentName)}`, {
        method: 'DELETE',
      });
      const data = await resp.json();
      if (data.success) {
        resetPromptBtn.textContent = 'Reset ✅';
        setTimeout(() => {
          loadAgentConfig(selectedAgentName, selectedModel);
        }, 1000);
      } else {
        resetPromptBtn.textContent = 'Failed ❌';
      }
    } catch (e) {
      console.error('Failed to reset instructions:', e);
      resetPromptBtn.textContent = 'Error ❌';
    }
    setTimeout(() => {
      resetPromptBtn.disabled = false;
      resetPromptBtn.textContent = 'Reset to Default';
    }, 2000);
  };
}

if (closeModalBtn) {
  closeModalBtn.onclick = () => {
    settingsModal.classList.add('hidden');
  };
}

// Close modal on click outside
window.onclick = (event) => {
  if (event.target === settingsModal) {
    settingsModal.classList.add('hidden');
  }
};

themeRadios.forEach(radio => {
  radio.onchange = (e) => {
    const theme = e.target.value;
    applyTheme(theme);
    localStorage.setItem('theme', theme);
  };
});

function applyTheme(theme) {
  if (theme === 'dark') {
    document.body.classList.add('dark-theme');
  } else {
    document.body.classList.remove('dark-theme');
  }
}

// Load theme on startup
let savedTheme = localStorage.getItem('theme') || 'light';
applyTheme(savedTheme);

// Update radio button state
themeRadios.forEach(radio => {
  if (radio.value === savedTheme) {
    radio.checked = true;
  }
});

// --- Keyboard Shortcuts ---
document.addEventListener('keydown', (e) => {
  // Don't intercept when typing in input fields
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') {
    if (e.key === 'Escape') {
      e.target.blur();
    }
    return;
  }

  switch (e.key) {
    case ' ':
      e.preventDefault();
      micBtn.click();
      break;
    case 'Escape':
      if (!settingsModal.classList.contains('hidden')) {
        settingsModal.classList.add('hidden');
      } else if (geminiClient.isConnected()) {
        // Send stop command
        geminiClient.sendText('stop');
      }
      break;
    case '1':
      selectAgent('human');
      applyVideoSource('computer');
      break;
    case '2':
      selectAgent('spot');
      applyVideoSource('spot');
      break;
    case '3':
      selectAgent('franka');
      applyVideoSource('robot');
      break;
    case '4':
      selectAgent('franka_vla');
      applyVideoSource('robot');
      break;
    case 't':
    case 'T':
      const currentTheme =
        document.body.classList.contains('dark-theme') ? 'light' : 'dark';
      applyTheme(currentTheme);
      localStorage.setItem('theme', currentTheme);
      themeRadios.forEach(r => {
        r.checked = r.value === currentTheme;
      });
      break;
    case '?':
      const helpModal = document.getElementById('shortcuts-modal');
      if (helpModal) helpModal.classList.toggle('hidden');
      break;
  }
});

function escapeHtml(str) {
  const div = document.createElement('div');
  div.textContent = str;
  return div.innerHTML;
}

function formatFunctionResult(value, depth = 0) {
  if (value === null || value === undefined) return '';
  if (typeof value !== 'object') return String(value);

  const indent = '  '.repeat(depth);
  if (Array.isArray(value)) {
    return value.map(item => {
      const text = formatFunctionResult(item, depth + 1);
      const lines = text.split('\n');
      return `${indent}- ${lines[0]}${lines.slice(1).map(line => `\n${indent}  ${line}`).join('')}`;
    }).join('\n');
  }

  return Object.entries(value).map(([key, item]) => {
    const label = key.replaceAll('_', ' ');
    if (item !== null && typeof item === 'object') {
      return `${indent}${label}:\n${formatFunctionResult(item, depth + 1)}`;
    }
    return `${indent}${label}: ${formatFunctionResult(item)}`;
  }).join('\n');
}

// --- Session Timer (integrated into status label) ---
let sessionStartTime = null;
let sessionTimerInterval = null;

function startSessionTimer() {
  sessionStartTime = Date.now();
  updateSessionTimer();
  sessionTimerInterval = setInterval(updateSessionTimer, 1000);
}

function stopSessionTimer() {
  if (sessionTimerInterval) {
    clearInterval(sessionTimerInterval);
    sessionTimerInterval = null;
  }
  sessionStartTime = null;
}

function updateSessionTimer() {
  if (!sessionStartTime || !statusLabel) return;
  const elapsed = Math.floor((Date.now() - sessionStartTime) / 1000);
  const h = Math.floor(elapsed / 3600);
  const m = Math.floor((elapsed % 3600) / 60);
  const s = elapsed % 60;
  const pad = n => String(n).padStart(2, '0');
  const time = h > 0 ? `${pad(h)}:${pad(m)}:${pad(s)}` : `${pad(m)}:${pad(s)}`;
  statusLabel.textContent = `Connected ${time}`;
}

// --- Transparent History Viewer ---

function prettyPrintPartJs(part) {
  if (!part) return '';
  const isThought = part.thought || part.raw_thought || part.rawThought;
  const prefix = isThought ? 'Thought: ' : '';
  if (part.text) {
    return `${prefix}${part.text}`;
  } else if (part.inlineData) {
    const mime = part.inlineData.mimeType || 'unknown';
    const len = (part.inlineData.data || '').length;
    if (mime.startsWith('image/') || mime.includes('jpeg') ||
      mime.includes('png')) {
      return `<div class="memory-part-media"><span class="chip">🖼️ Frame</span><img src="data:image/jpeg;base64,${part.inlineData.data}" class="memory-inline-img" /></div>`;
    }
    return `<span class="chip">📦 Data</span> <code>[InlineData: ${mime} (${len} bytes)]</code>`;
  } else if (part.audioTranscription) {
    return `<span class="chip">🗣️ Transcript</span> <em>"${part.audioTranscription.text}"</em>`;
  } else if (part.audio_transcription) {
    return `<span class="chip">🗣️ Transcript</span> <em>"${part.audio_transcription.text}"</em>`;
  } else if (part.functionCall) {
    const fc = part.functionCall;
    const args = JSON.stringify(fc.args || {});
    return `<span class="chip code-chip">🔧 Call</span> <code>${fc.name}(${args})</code>`;
  } else if (part.functionResponse) {
    const fr = part.functionResponse;
    const resp = escapeHtml(formatFunctionResult(fr.response || {}))
      .replaceAll('\n', '<br>');
    return `<span class="chip code-chip">✅ Result</span> <code>${fr.name} -> ${resp}</code>`;
  } else if (part.fileData) {
    return `<span class="chip">📁 File</span> <code>[FileData: ${part.fileData.mimeType}]</code>`;
  }
  return `<code>[Unknown Part]</code>`;
}



// --- Latency Detail Toggle ---
let latencyDetailToggle = document.getElementById('latencyDetailToggle');
let latencyDetail = document.getElementById('latency-detail');
if (latencyDetailToggle) {
  latencyDetailToggle.onclick = () => {
    latencyDetail.classList.toggle('hidden');
  };
}
