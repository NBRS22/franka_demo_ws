import { state } from './state.js';
import { ENDPOINT_MODELS, getBaseModality } from './config.js';

const ICON_AUDIO_ON = '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><path d="M19.07 4.93a10 10 0 010 14.14"/><path d="M15.54 8.46a5 5 0 010 7.07"/></svg>';
const ICON_AUDIO_OFF = '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5"/><line x1="23" y1="9" x2="17" y2="15"/><line x1="17" y1="9" x2="23" y2="15"/></svg>';

export function updateAudioUI() {
  const audioToggleBtn = document.getElementById('audioToggleBtn');
  if (!audioToggleBtn) return;

  const baseModality = getBaseModality();

  if (baseModality === 'AUDIO') {
    audioToggleBtn.disabled = true;
    audioToggleBtn.classList.add('active');
    audioToggleBtn.innerHTML = ICON_AUDIO_ON;
    audioToggleBtn.title = 'Audio is required for this model';
  } else {
    audioToggleBtn.disabled = false;
    audioToggleBtn.classList.toggle('active', state.selectedAudioEnabled);
    audioToggleBtn.innerHTML = state.selectedAudioEnabled ? ICON_AUDIO_ON : ICON_AUDIO_OFF;
    audioToggleBtn.title = state.selectedAudioEnabled ? 'Click to mute audio' : 'Click to unmute audio';
  }
}

export function bindModelOption(opt) {
  const modelDropdown = document.getElementById('modelDropdown');
  const customModelModal = document.getElementById('custom-model-modal');
  const customModelInput = document.getElementById('custom-model-input');
  const customModalitySelect = document.getElementById('custom-modality-select');
  const modelSelectorLabel = document.getElementById('modelSelectorLabel');

  opt.onclick = () => {
    const model = opt.dataset.model;
    if (model === 'custom') {
      if (modelDropdown) modelDropdown.classList.add('hidden');
      if (customModelInput) customModelInput.value = state.selectedModel;
      if (customModalitySelect) customModalitySelect.value = state.customBaseModality;
      if (customModelModal) customModelModal.classList.remove('hidden');
      if (customModelInput) customModelInput.focus();
      return;
    }
    state.selectedModel = model;
    localStorage.setItem('lite_model', state.selectedModel);
    if (modelSelectorLabel) modelSelectorLabel.textContent = opt.textContent;
    if (modelDropdown) {
      modelDropdown.querySelectorAll('.model-option').forEach(o => o.classList.remove('active'));
    }
    opt.classList.add('active');
    if (modelDropdown) modelDropdown.classList.add('hidden');
    updateAudioUI();
  };
}

export function populateModels(endpoint) {
  const modelDropdown = document.getElementById('modelDropdown');
  const modelSelectorLabel = document.getElementById('modelSelectorLabel');
  if (!modelDropdown) return;

  modelDropdown.innerHTML = '';
  const models = ENDPOINT_MODELS[endpoint] || [];
  models.forEach(m => {
    const opt = document.createElement('div');
    opt.className = 'model-option';
    if (m.model === state.selectedModel) {
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

  const customOpt = document.createElement('div');
  customOpt.className = 'model-option model-option-custom';
  if (state.selectedModel === 'custom') customOpt.classList.add('active');
  customOpt.dataset.model = 'custom';
  customOpt.textContent = 'Custom...';
  bindModelOption(customOpt);
  modelDropdown.appendChild(customOpt);
}

export function selectModelVisual(model) {
  const modelDropdown = document.getElementById('modelDropdown');
  const modelSelectorLabel = document.getElementById('modelSelectorLabel');

  state.selectedModel = model;
  localStorage.setItem('lite_model', state.selectedModel);

  if (modelDropdown) {
    let activeText = state.selectedModel;
    modelDropdown.querySelectorAll('.model-option').forEach(o => {
      const isActive = o.dataset.model === state.selectedModel;
      o.classList.toggle('active', isActive);
      if (isActive) activeText = o.textContent;
    });
    if (modelSelectorLabel) modelSelectorLabel.textContent = activeText;
  } else if (modelSelectorLabel) {
    modelSelectorLabel.textContent = state.selectedModel;
  }
  updateAudioUI();
}

export function syncModelDropdown() {
  const modelDropdown = document.getElementById('modelDropdown');
  const modelSelectorLabel = document.getElementById('modelSelectorLabel');
  if (!modelDropdown) return;

  let activeText = state.selectedModel;
  modelDropdown.querySelectorAll('.model-option').forEach(opt => {
    bindModelOption(opt);
    const isActive = opt.dataset.model === state.selectedModel;
    opt.classList.toggle('active', isActive);
    if (isActive) activeText = opt.textContent;
  });
  if (modelSelectorLabel) modelSelectorLabel.textContent = activeText;
}

export function initModelSelector() {
  const modelSelectorBtn = document.getElementById('modelSelectorBtn');
  const modelDropdown = document.getElementById('modelDropdown');
  const endpointSelectorBtn = document.getElementById('endpointSelectorBtn');
  const endpointSelectorLabel = document.getElementById('endpointSelectorLabel');
  const endpointDropdown = document.getElementById('endpointDropdown');
  const audioToggleBtn = document.getElementById('audioToggleBtn');
  const customModelModal = document.getElementById('custom-model-modal');
  const customModelInput = document.getElementById('custom-model-input');
  const customModalitySelect = document.getElementById('custom-modality-select');
  const customModelCancel = document.getElementById('custom-model-cancel');
  const customModelApply = document.getElementById('custom-model-apply');
  const modelSelectorLabel = document.getElementById('modelSelectorLabel');

  // Init endpoint label
  if (endpointSelectorLabel && endpointDropdown) {
    const activeOpt = Array.from(endpointDropdown.querySelectorAll('.model-option'))
      .find(opt => opt.dataset.endpoint === state.selectedEndpointType);
    if (activeOpt) {
      endpointSelectorLabel.textContent = activeOpt.textContent;
      endpointDropdown.querySelectorAll('.model-option').forEach(o => {
        o.classList.toggle('active', o.dataset.endpoint === state.selectedEndpointType);
      });
    }
  }

  populateModels(state.selectedEndpointType);

  // Toggle endpoint dropdown
  if (endpointSelectorBtn) {
    endpointSelectorBtn.onclick = (e) => {
      e.stopPropagation();
      if (endpointDropdown) endpointDropdown.classList.toggle('hidden');
    };
  }

  // Toggle model dropdown
  if (modelSelectorBtn) {
    modelSelectorBtn.onclick = (e) => {
      e.stopPropagation();
      if (modelDropdown) modelDropdown.classList.toggle('hidden');
    };
  }

  // Handle endpoint selection
  if (endpointDropdown) {
    endpointDropdown.querySelectorAll('.model-option').forEach(opt => {
      opt.onclick = () => {
        const ep = opt.dataset.endpoint;
        state.selectedEndpointType = ep;
        localStorage.setItem('lite_endpoint_type', ep);
        if (endpointSelectorLabel) endpointSelectorLabel.textContent = opt.textContent;
        endpointDropdown.querySelectorAll('.model-option').forEach(o => o.classList.remove('active'));
        opt.classList.add('active');
        endpointDropdown.classList.add('hidden');
        populateModels(ep);
        const defaultModel = (ENDPOINT_MODELS[ep] || [])[0]?.model;
        if (defaultModel) selectModelVisual(defaultModel);
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

  // Custom model modal
  if (customModelCancel) {
    customModelCancel.onclick = () => { if (customModelModal) customModelModal.classList.add('hidden'); };
  }
  if (customModelApply) {
    customModelApply.onclick = () => {
      if (!customModelInput) return;
      const customName = customModelInput.value.trim();
      if (!customName) return;
      state.selectedModel = customName;
      localStorage.setItem('lite_model', state.selectedModel);
      if (modelSelectorLabel) modelSelectorLabel.textContent = state.selectedModel;
      if (modelDropdown) modelDropdown.querySelectorAll('.model-option').forEach(o => o.classList.remove('active'));
      if (customModalitySelect) {
        state.customBaseModality = customModalitySelect.value;
        localStorage.setItem('lite_custom_modality', state.customBaseModality);
      }
      if (customModelModal) customModelModal.classList.add('hidden');
      updateAudioUI();
    };
  }

  // Audio toggle
  if (audioToggleBtn) {
    audioToggleBtn.onclick = () => {
      if (audioToggleBtn.disabled) return;
      state.selectedAudioEnabled = !state.selectedAudioEnabled;
      localStorage.setItem('lite_audio_enabled', state.selectedAudioEnabled);
      updateAudioUI();
    };
  }

  updateAudioUI();
}
