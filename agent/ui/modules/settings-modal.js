import { state } from './state.js';
import { getResolvedSessionConfig } from './config.js';
import { applyVideoSource, selectAgent } from './agent-selector.js';

export function applyTheme(theme) {
  if (theme === 'dark') {
    document.body.classList.add('dark-theme');
  } else {
    document.body.classList.remove('dark-theme');
  }
}

export async function loadAgentConfig(name, model) {
  try {
    let url = `/api/agent_config/${encodeURIComponent(name)}?endpoint_type=${encodeURIComponent(state.selectedEndpointType)}`;
    if (model) url += `&model=${encodeURIComponent(model)}`;
    const resp = await fetch(url);
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const cfg = await resp.json();

    const titleEl = document.getElementById('config-title');
    if (titleEl) titleEl.textContent = `${cfg.name} Agent Config`;

    const sessionCfg = getResolvedSessionConfig();
    const setCode = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
    setCode('config-model', sessionCfg.model);
    setCode('config-modality', sessionCfg.response_modality);
    setCode('config-tts', sessionCfg.use_tts ? 'Enabled' : 'Disabled');
    setCode('config-tts-voice', cfg.tts_voice);
    setCode('config-endpoint', cfg.endpoint_type);
    setCode('config-service', cfg.service_address);

    const toolsDiv = document.getElementById('config-tools');
    if (toolsDiv) {
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
    }

    const devPromptEl = document.getElementById('config-developer-prompt');
    if (devPromptEl) devPromptEl.value = cfg.developer_instruction || '';

    const hbTextEl = document.getElementById('config-heartbeat-text');
    if (hbTextEl) hbTextEl.value = cfg.heartbeat_text || '';

    const thinkingLevelEl = document.getElementById('config-thinking-level');
    if (thinkingLevelEl && cfg.thinking_level) thinkingLevelEl.value = cfg.thinking_level;

    const hbInt = document.getElementById('config-heartbeat-interval');
    if (hbInt) hbInt.value = cfg.heartbeat_interval_seconds !== undefined ? cfg.heartbeat_interval_seconds : '';

    const hbDelay = document.getElementById('config-heartbeat-min-delay');
    if (hbDelay) hbDelay.value = cfg.heartbeat_min_delay_seconds !== undefined ? cfg.heartbeat_min_delay_seconds : '';

    const hbEvent = document.getElementById('config-use-event-driven-heartbeat');
    if (hbEvent) hbEvent.checked = !!cfg.use_event_driven_heartbeat;

  } catch (e) {
    console.error('Failed to load agent config:', e);
    const devPromptEl = document.getElementById('config-developer-prompt');
    if (devPromptEl) devPromptEl.value = 'Error loading config';
  }
}

export function initSettingsModal() {
  const settingsBtn = document.getElementById('settings-btn');
  const settingsModal = document.getElementById('settings-modal');
  const closeModalBtn = document.getElementById('close-modal-btn');
  const themeRadios = document.querySelectorAll('input[name="theme"]');
  const modalTabs = document.querySelectorAll('.modal-sidebar li[data-tab]');
  const updatePromptBtn = document.getElementById('update-prompt-btn');
  const resetPromptBtn = document.getElementById('reset-prompt-btn');
  const latencyDetailToggle = document.getElementById('latencyDetailToggle');
  const latencyDetail = document.getElementById('latency-detail');

  // Tab switching
  modalTabs.forEach(tab => {
    tab.onclick = () => {
      modalTabs.forEach(t => t.classList.remove('active'));
      tab.classList.add('active');
      document.querySelectorAll('.modal-panel').forEach(p => p.classList.add('hidden'));
      const panel = document.getElementById(`modal-panel-${tab.dataset.tab}`);
      if (panel) panel.classList.remove('hidden');
    };
  });

  // Open settings modal
  if (settingsBtn) {
    settingsBtn.onclick = () => {
      if (!settingsModal) return;
      settingsModal.classList.remove('hidden');
      modalTabs.forEach(t => t.classList.remove('active'));
      const configTab = document.getElementById('modal-tab-config');
      if (configTab) configTab.classList.add('active');
      document.querySelectorAll('.modal-panel').forEach(p => p.classList.add('hidden'));
      const configPanel = document.getElementById('modal-panel-config');
      if (configPanel) configPanel.classList.remove('hidden');
      loadAgentConfig(state.selectedAgentName, state.selectedModel);
    };
  }

  // Update prompt
  if (updatePromptBtn) {
    updatePromptBtn.onclick = async () => {
      const devPromptEl = document.getElementById('config-developer-prompt');
      const hbTextEl = document.getElementById('config-heartbeat-text');
      const hbIntEl = document.getElementById('config-heartbeat-interval');
      const hbDelayEl = document.getElementById('config-heartbeat-min-delay');
      const hbEventEl = document.getElementById('config-use-event-driven-heartbeat');

      updatePromptBtn.disabled = true;
      updatePromptBtn.textContent = 'Updating...';
      try {
        const resp = await fetch('/api/config/instructions', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            agent_name: state.selectedAgentName,
            developer_instruction: devPromptEl ? devPromptEl.value : '',
            heartbeat_text: hbTextEl ? hbTextEl.value : '',
            heartbeat_interval_seconds: hbIntEl && hbIntEl.value !== '' ? parseFloat(hbIntEl.value) : null,
            heartbeat_min_delay_seconds: hbDelayEl && hbDelayEl.value !== '' ? parseFloat(hbDelayEl.value) : null,
            use_event_driven_heartbeat: hbEventEl ? hbEventEl.checked : false,
            disabled_tools: Array.from(document.querySelectorAll('.tool-checkbox:not(:checked)')).map(cb => cb.dataset.toolName),
          }),
        });
        const data = await resp.json();
        updatePromptBtn.textContent = data.success ? 'Updated ✅' : 'Failed ❌';
        if (data.success) {
          setTimeout(() => loadAgentConfig(state.selectedAgentName, state.selectedModel), 1000);
        }
      } catch (e) {
        console.error('Failed to update instructions:', e);
        updatePromptBtn.textContent = 'Error ❌';
      }
      setTimeout(() => { updatePromptBtn.disabled = false; updatePromptBtn.textContent = 'Update Instructions'; }, 2000);
    };
  }

  // Reset prompt
  if (resetPromptBtn) {
    resetPromptBtn.onclick = async () => {
      resetPromptBtn.disabled = true;
      resetPromptBtn.textContent = 'Resetting...';
      try {
        const resp = await fetch(`/api/config/instructions?agent_name=${encodeURIComponent(state.selectedAgentName)}`, { method: 'DELETE' });
        const data = await resp.json();
        resetPromptBtn.textContent = data.success ? 'Reset ✅' : 'Failed ❌';
        if (data.success) {
          setTimeout(() => loadAgentConfig(state.selectedAgentName, state.selectedModel), 1000);
        }
      } catch (e) {
        console.error('Failed to reset instructions:', e);
        resetPromptBtn.textContent = 'Error ❌';
      }
      setTimeout(() => { resetPromptBtn.disabled = false; resetPromptBtn.textContent = 'Reset to Default'; }, 2000);
    };
  }

  // Close modal
  if (closeModalBtn) {
    closeModalBtn.onclick = () => { if (settingsModal) settingsModal.classList.add('hidden'); };
  }
  window.onclick = (event) => {
    if (event.target === settingsModal) settingsModal.classList.add('hidden');
  };

  // Theme
  themeRadios.forEach(radio => {
    radio.onchange = (e) => {
      applyTheme(e.target.value);
      localStorage.setItem('theme', e.target.value);
    };
  });
  const savedTheme = localStorage.getItem('theme') || 'light';
  applyTheme(savedTheme);
  themeRadios.forEach(radio => { radio.checked = radio.value === savedTheme; });

  // Latency detail toggle
  if (latencyDetailToggle && latencyDetail) {
    latencyDetailToggle.onclick = () => latencyDetail.classList.toggle('hidden');
  }

  // Keyboard shortcuts
  document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') {
      if (e.key === 'Escape') e.target.blur();
      return;
    }

    switch (e.key) {
      case ' ':
        e.preventDefault();
        document.getElementById('micBtn')?.click();
        break;
      case 'Escape':
        if (settingsModal && !settingsModal.classList.contains('hidden')) {
          settingsModal.classList.add('hidden');
        } else if (state.geminiClient && state.geminiClient.isConnected()) {
          state.geminiClient.sendText('stop');
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
      case 'T': {
        const currentTheme = document.body.classList.contains('dark-theme') ? 'light' : 'dark';
        applyTheme(currentTheme);
        localStorage.setItem('theme', currentTheme);
        themeRadios.forEach(r => { r.checked = r.value === currentTheme; });
        break;
      }
      case '?': {
        const helpModal = document.getElementById('shortcuts-modal');
        if (helpModal) helpModal.classList.toggle('hidden');
        break;
      }
    }
  });
}
