import { state } from './state.js';

export const ENDPOINT_MODELS = {
  gemini_live_api: [
    { model: 'models/gemini-robotics-er-2-streaming-preview', label: 'models/gemini-robotics-er-2-streaming-preview', modality: 'TEXT', textOnly: true },
    { model: 'models/gemini-3.1-flash-live-preview', label: 'models/gemini-3.1-flash-live-preview', modality: 'AUDIO', textOnly: false },
    { model: 'models/gemini-2.5-flash-native-audio-latest', label: 'models/gemini-2.5-flash-native-audio-latest', modality: 'AUDIO', textOnly: false },
  ],
};

export function getBaseModality() {
  if (state.selectedModel === 'custom') return state.customBaseModality;
  const dropdown = document.getElementById('modelDropdown');
  if (dropdown) {
    const opt = Array.from(dropdown.querySelectorAll('.model-option'))
      .find(o => o.dataset.model === state.selectedModel);
    if (opt) return opt.dataset.modality || 'TEXT';
  }
  return 'TEXT';
}

export function getResolvedSessionConfig() {
  const baseModality = getBaseModality();
  return {
    model: state.selectedModel,
    response_modality: baseModality === 'AUDIO' ? 'AUDIO' : 'TEXT',
    use_tts: state.selectedAudioEnabled,
    endpoint_type: state.selectedEndpointType,
  };
}

export async function initServerDefaults(bindModelOptionFn) {
  try {
    const resp = await fetch('/api/server_defaults');
    if (!resp.ok) throw new Error();
    const defaults = await resp.json();

    const modelDropdown = document.getElementById('modelDropdown');
    if (defaults.available_models && modelDropdown) {
      modelDropdown.innerHTML = '';
      const allKnown = Object.values(ENDPOINT_MODELS).flat();
      defaults.available_models.forEach(modelName => {
        const found = allKnown.find(m => m.model === modelName);
        const label = found ? found.label : modelName;
        const modality = found ? found.modality
          : (modelName.includes('audio') || modelName.includes('live') ? 'AUDIO' : 'TEXT');
        const textOnly = found ? found.textOnly : false;
        const opt = document.createElement('div');
        opt.className = 'model-option';
        opt.dataset.model = modelName;
        opt.dataset.modality = modality;
        opt.dataset.textOnly = textOnly ? 'true' : 'false';
        opt.textContent = label;
        if (bindModelOptionFn) bindModelOptionFn(opt);
        modelDropdown.appendChild(opt);
      });
      const customOpt = document.createElement('div');
      customOpt.className = 'model-option model-option-custom';
      customOpt.dataset.model = 'custom';
      customOpt.innerHTML = 'Custom&hellip;';
      if (bindModelOptionFn) bindModelOptionFn(customOpt);
      modelDropdown.appendChild(customOpt);
    }

    const localModel = localStorage.getItem('lite_model');
    if (!localModel || (defaults.available_models && !defaults.available_models.includes(localModel))) {
      state.selectedModel = defaults.model;
      localStorage.setItem('lite_model', state.selectedModel);
    } else {
      state.selectedModel = localModel;
    }

    if (!localStorage.getItem('lite_audio_enabled_set')) {
      state.selectedAudioEnabled = defaults.use_tts;
      localStorage.setItem('lite_audio_enabled', state.selectedAudioEnabled);
      localStorage.setItem('lite_audio_enabled_set', 'true');
    }
  } catch (e) {
    console.warn('Failed to fetch server defaults, using local defaults');
  }
}
