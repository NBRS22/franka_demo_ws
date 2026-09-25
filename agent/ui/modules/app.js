import { state } from './state.js';
import { ENDPOINT_MODELS, initServerDefaults } from './config.js';
import { initModelSelector, syncModelDropdown, updateAudioUI, bindModelOption } from './model-selector.js';
import { initAgentSelector } from './agent-selector.js';
import { initOverlayCheckboxes } from './overlay.js';
import { initConnection, onWsOpen, onWsMessage, onWsClose, onWsError } from './connection.js';
import { initSettingsModal } from './settings-modal.js';

// ── Bootstrap state from localStorage ────────────────────────────────────────

state.selectedEndpointType = localStorage.getItem('lite_endpoint_type') || 'gemini_live_api';
state.selectedAudioEnabled = localStorage.getItem('lite_audio_enabled') !== 'false';
state.customBaseModality = localStorage.getItem('lite_custom_modality') || 'TEXT';

const localModel = localStorage.getItem('lite_model');
const available = ENDPOINT_MODELS[state.selectedEndpointType] || [];
if (localModel && (available.some(m => m.model === localModel) || localModel === 'custom')) {
  state.selectedModel = localModel;
} else if (available.length > 0) {
  state.selectedModel = available[0].model;
  localStorage.setItem('lite_model', state.selectedModel);
} else {
  state.selectedModel = '';
}

// ── Instantiate shared objects (GeminiClient & MediaHandler are globals from <script> tags) ──

state.mediaHandler = new MediaHandler();   // eslint-disable-line no-undef
state.geminiClient = new GeminiClient({    // eslint-disable-line no-undef
  onOpen: onWsOpen,
  onMessage: onWsMessage,
  onClose: onWsClose,
  onError: onWsError,
});

// ── Wire up all modules ───────────────────────────────────────────────────────

initModelSelector();
initAgentSelector();
initOverlayCheckboxes();
initConnection();
initSettingsModal();

// ── Fetch server defaults and re-sync dropdowns ───────────────────────────────

initServerDefaults(bindModelOption).then(() => {
  syncModelDropdown();
  updateAudioUI();
});
