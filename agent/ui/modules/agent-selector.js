import { state } from './state.js';
import { setFrankaOverlayUI } from './overlay.js';

export function applyVideoSource(source, { notifyServer = true } = {}) {
  state.videoSource = source;
  setFrankaOverlayUI(state.selectedAgentName === 'franka' && source !== 'computer');

  if (notifyServer && state.geminiClient && state.geminiClient.isConnected()) {
    state.geminiClient.send(JSON.stringify({ type: 'video_source', source }));
  }

  const robotCamImg = document.getElementById('robot-cam-img');
  const videoPreview = document.getElementById('video-preview');
  const videoPlaceholder = document.getElementById('video-placeholder');
  const videoContainer = robotCamImg ? robotCamImg.parentElement : null;

  if (source === 'computer') {
    if (robotCamImg) { robotCamImg.classList.add('hidden'); robotCamImg.src = ''; }
    if (videoContainer) {
      videoContainer.classList.remove('atari-aspect');
      videoContainer.classList.remove('spot-aspect');
    }
    if (videoPlaceholder) videoPlaceholder.classList.add('hidden');
    if (videoPreview) videoPreview.classList.remove('hidden');
    if (state.mediaHandler && !state.mediaHandler.videoStream) {
      state.mediaHandler.startVideo(videoPreview, (base64Data) => {
        if (state.geminiClient && state.geminiClient.isConnected()) {
          state.geminiClient.sendImage(base64Data);
        }
      }).catch(e => {
        console.error('Camera start failed:', e);
        const placeholder = document.getElementById('video-placeholder');
        if (placeholder) {
          placeholder.textContent = '⚠️ Camera blocked — allow access in browser';
          placeholder.classList.remove('hidden');
        }
        if (videoPreview) videoPreview.classList.add('hidden');
      });
    }
  } else {
    if (state.mediaHandler) state.mediaHandler.stopVideo(videoPreview);
    if (videoPreview) videoPreview.classList.add('hidden');
    if (videoPlaceholder) videoPlaceholder.classList.add('hidden');

    if (videoContainer) {
      if (source === 'atari') {
        videoContainer.classList.add('atari-aspect');
        videoContainer.classList.remove('spot-aspect');
      } else {
        videoContainer.classList.remove('atari-aspect');
        videoContainer.classList.toggle('spot-aspect', source === 'spot');
      }
    }

    if (robotCamImg) {
      robotCamImg.classList.remove('hidden');
      robotCamImg.src = '/api/camera?' + Date.now();
      if (state.geminiClient && state.geminiClient.isConnected()) {
        setTimeout(() => { robotCamImg.src = '/api/camera?' + Date.now(); }, 2000);
      }
    }
  }
}

export function selectAgent(type) {
  state.selectedAgentName = type;

  const agentDropdown = document.getElementById('agentDropdown');
  const agentSelectorLabel = document.getElementById('agentSelectorLabel');

  if (agentDropdown) {
    agentDropdown.querySelectorAll('.agent-option').forEach(o => o.classList.remove('active'));
    const activeOpt = Array.from(agentDropdown.querySelectorAll('.agent-option'))
      .find(o => o.dataset.agent === type);
    if (activeOpt) {
      activeOpt.classList.add('active');
      if (agentSelectorLabel) agentSelectorLabel.textContent = activeOpt.textContent;
    }
  }
  setFrankaOverlayUI(type === 'franka');
}

export function initAgentSelector() {
  const agentSelectorBtn = document.getElementById('agentSelectorBtn');
  const agentDropdown = document.getElementById('agentDropdown');

  if (agentSelectorBtn) {
    agentSelectorBtn.onclick = (e) => {
      e.stopPropagation();
      if (agentDropdown) agentDropdown.classList.toggle('hidden');
    };
  }

  document.addEventListener('click', (e) => {
    if (agentDropdown && !agentDropdown.contains(e.target) && e.target !== agentSelectorBtn) {
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
        } else {
          applyVideoSource('robot');
        }
        agentDropdown.classList.add('hidden');
      };
    });
  }
}
