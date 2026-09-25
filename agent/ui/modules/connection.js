import { state } from './state.js';
import { getResolvedSessionConfig } from './config.js';
import { updateAudioUI } from './model-selector.js';
import { applyVideoSource } from './agent-selector.js';
import { clearAllOverlayData, setFrankaOverlayUI } from './overlay.js';
import { resetLatencyTimer } from './latency.js';
import { appendMessage } from './chat-log.js';
import { handleJsonMessage } from './message-handler.js';

// ── WebSocket callbacks (called from app.js via GeminiClient) ───────────────

export function onWsOpen() {
  const statusLabel = document.getElementById('status-label');
  const connectBtn = document.getElementById('connectBtn');
  const modelSelectorBtn = document.getElementById('modelSelectorBtn');
  const audioToggleBtn = document.getElementById('audioToggleBtn');
  const agentSelectorBtn = document.getElementById('agentSelectorBtn');

  if (statusLabel) statusLabel.className = 'status-label connected';
  if (connectBtn) {
    connectBtn.classList.add('danger');
    connectBtn.title = 'Disconnect from Agent';
    connectBtn.disabled = false;
    connectBtn.textContent = 'Disconnect';
  }
  if (modelSelectorBtn) modelSelectorBtn.disabled = true;
  if (audioToggleBtn) audioToggleBtn.disabled = true;
  if (agentSelectorBtn) agentSelectorBtn.disabled = true;

  applyVideoSource(state.selectedAgentName === 'human' ? 'computer' : state.selectedAgentName);
  startSessionTimer();
}

export function onWsMessage(event) {
  if (typeof event.data === 'string') {
    try {
      const msg = JSON.parse(event.data);
      handleJsonMessage(msg);
    } catch (e) {
      console.error('Parse error:', e);
    }
  } else {
    if (state.audioFlushPending) {
      if (state.mediaHandler) state.mediaHandler.stopAudioPlayback();
      state.audioFlushPending = false;
    }
    if (state.mediaHandler) state.mediaHandler.playAudio(event.data);
  }
}

export function onWsClose(e) {
  console.log('WS Closed:', e);
  const statusLabel = document.getElementById('status-label');
  if (statusLabel) { statusLabel.className = 'status-label disconnected'; statusLabel.textContent = 'Disconnected'; }
  stopSessionTimer();
  resetUI();
}

export function onWsError(e) {
  console.error('WS Error:', e);
  const statusLabel = document.getElementById('status-label');
  if (statusLabel) { statusLabel.className = 'status-label error'; statusLabel.textContent = 'Error'; }
}

// ── Robot status ─────────────────────────────────────────────────────────────

export function updateRobotStatus(robotState, text) {
  state.robotState = robotState;
  state.robotStateText = text;
  refreshStatusLabel();
}

export function refreshStatusLabel() {
  const statusLabel = document.getElementById('status-label');
  if (!statusLabel) return;
  if (state.robotState === 'executing' && state.geminiClient && state.geminiClient.isConnected()) {
    statusLabel.className = 'status-label executing';
    statusLabel.textContent = state.robotStateText;
    return;
  }
  if (state.geminiClient && state.geminiClient.isConnected()) {
    statusLabel.className = 'status-label connected';
    if (!state.sessionStartTime) statusLabel.textContent = 'Connected';
  }
}

// ── Session timer ─────────────────────────────────────────────────────────────

export function startSessionTimer() {
  state.sessionStartTime = Date.now();
  updateSessionTimer();
  state.sessionTimerInterval = setInterval(updateSessionTimer, 1000);
}

export function stopSessionTimer() {
  if (state.sessionTimerInterval) {
    clearInterval(state.sessionTimerInterval);
    state.sessionTimerInterval = null;
  }
  state.sessionStartTime = null;
}

function updateSessionTimer() {
  const statusLabel = document.getElementById('status-label');
  if (!state.sessionStartTime || !statusLabel) return;
  const elapsed = Math.floor((Date.now() - state.sessionStartTime) / 1000);
  const h = Math.floor(elapsed / 3600);
  const m = Math.floor((elapsed % 3600) / 60);
  const s = elapsed % 60;
  const pad = n => String(n).padStart(2, '0');
  const time = h > 0 ? `${pad(h)}:${pad(m)}:${pad(s)}` : `${pad(m)}:${pad(s)}`;
  statusLabel.textContent = `Connected ${time}`;
}

// ── Reset UI ──────────────────────────────────────────────────────────────────

export function resetUI() {
  const connectBtn = document.getElementById('connectBtn');
  const modelSelectorBtn = document.getElementById('modelSelectorBtn');
  const audioToggleBtn = document.getElementById('audioToggleBtn');
  const agentSelectorBtn = document.getElementById('agentSelectorBtn');
  const micBtn = document.getElementById('micBtn');
  const videoPreview = document.getElementById('video-preview');
  const robotCamImg = document.getElementById('robot-cam-img');
  const videoPlaceholder = document.getElementById('video-placeholder');
  const instructionOverlay = document.getElementById('instruction-overlay');

  if (connectBtn) { connectBtn.classList.remove('danger'); connectBtn.textContent = 'Connect'; connectBtn.title = 'Connect to Agent'; connectBtn.disabled = false; }
  if (modelSelectorBtn) modelSelectorBtn.disabled = false;
  if (audioToggleBtn) audioToggleBtn.disabled = false;
  if (agentSelectorBtn) agentSelectorBtn.disabled = false;
  updateAudioUI();

  if (state.mediaHandler) {
    state.mediaHandler.stopAudio();
    state.mediaHandler.stopVideo(videoPreview);
  }
  if (robotCamImg) { robotCamImg.classList.add('hidden'); robotCamImg.src = ''; }
  if (videoPlaceholder) videoPlaceholder.classList.remove('hidden');
  if (micBtn) { micBtn.classList.remove('active'); micBtn.title = 'Start Microphone'; }
  if (instructionOverlay) instructionOverlay.style.display = 'none';

  updateRobotStatus('idle', 'Stopped');
  state.toolCallCounter = 0;
  state.lastAckRow = null;
  state.ackCount = 0;

  clearAllOverlayData();
  setFrankaOverlayUI(false);
}

// ── Event bindings ────────────────────────────────────────────────────────────

export function initConnection() {
  const connectBtn = document.getElementById('connectBtn');
  const micBtn = document.getElementById('micBtn');
  const sendBtn = document.getElementById('sendBtn');
  const textInput = document.getElementById('textInput');
  const chatToggle = document.getElementById('chatToggle');
  const chatSidebar = document.getElementById('chat-sidebar');
  const imageBtn = document.getElementById('imageBtn');
  const imageFileInput = document.getElementById('imageFileInput');
  const statusLabel = document.getElementById('status-label');

  if (connectBtn) {
    connectBtn.onclick = async () => {
      if (state.geminiClient && state.geminiClient.isConnected()) {
        state.geminiClient.disconnect();
      } else {
        const chatLog = document.getElementById('chat-log');
        if (chatLog) chatLog.innerHTML = '';
        const memoryLog = document.getElementById('memory-log');
        if (memoryLog) memoryLog.innerHTML = '';

        if (statusLabel) { statusLabel.className = 'status-label connecting'; statusLabel.textContent = 'Connecting…'; }
        connectBtn.disabled = true;

        try {
          if (state.mediaHandler) await state.mediaHandler.initializeAudio();
          const sessionCfg = getResolvedSessionConfig();
          const thinkingLevelEl = document.getElementById('config-thinking-level');
          const connParams = {
            agent_name: state.selectedAgentName,
            model: sessionCfg.model,
            response_modality: sessionCfg.response_modality,
            use_tts: sessionCfg.use_tts,
            endpoint_type: sessionCfg.endpoint_type,
            thinking_level: thinkingLevelEl ? thinkingLevelEl.value : undefined,
          };
          if (state.geminiClient) state.geminiClient.connect(connParams);
        } catch (error) {
          console.error('Connection error:', error);
          if (statusLabel) { statusLabel.className = 'status-label error'; statusLabel.textContent = 'Error'; }
          connectBtn.disabled = false;
        }
      }
    };
  }

  if (micBtn) {
    micBtn.onclick = async () => {
      if (state.mediaHandler && state.mediaHandler.isRecording) {
        state.mediaHandler.stopAudio();
        micBtn.classList.remove('active');
        micBtn.title = 'Start Microphone';
      } else {
        try {
          if (state.mediaHandler) {
            await state.mediaHandler.startAudio((data) => {
              if (state.geminiClient && state.geminiClient.isConnected()) {
                state.geminiClient.send(data);
              }
            });
          }
          micBtn.classList.add('active');
          micBtn.title = 'Stop Microphone';
        } catch (e) {
          alert('Could not start audio capture');
        }
      }
    };
  }

  if (sendBtn) sendBtn.onclick = sendText;
  if (textInput) textInput.onkeypress = (e) => { if (e.key === 'Enter') sendText(); };

  if (chatToggle && chatSidebar) {
    chatToggle.onclick = () => chatSidebar.classList.toggle('collapsed');
  }

  // Image upload
  if (imageBtn && imageFileInput) {
    imageBtn.onclick = () => {
      if (!state.geminiClient || !state.geminiClient.isConnected()) return;
      imageFileInput.value = '';
      imageFileInput.click();
    };

    imageFileInput.onchange = () => {
      const file = imageFileInput.files[0];
      if (!file || !state.geminiClient || !state.geminiClient.isConnected()) return;
      const reader = new FileReader();
      reader.onload = (e) => {
        const dataUrl = e.target.result;
        const commaIdx = dataUrl.indexOf(',');
        const mimeType = dataUrl.substring(5, dataUrl.indexOf(';'));
        const base64Data = dataUrl.substring(commaIdx + 1);
        const caption = textInput ? textInput.value.trim() : '';
        state.geminiClient.send(JSON.stringify({ type: 'user_image', mime_type: mimeType, data: base64Data, caption }));

        const imgEl = document.createElement('img');
        imgEl.src = dataUrl;
        imgEl.style.cssText = 'max-width:180px;max-height:120px;border-radius:4px;display:block;margin-top:4px;';
        const wrapper = document.createElement('div');
        wrapper.className = 'message user';
        if (caption) {
          const captionEl = document.createElement('div');
          captionEl.textContent = caption;
          wrapper.appendChild(captionEl);
          if (textInput) textInput.value = '';
        }
        wrapper.appendChild(imgEl);
        const chatLog = document.getElementById('chat-log');
        if (chatLog) { chatLog.appendChild(wrapper); chatLog.scrollTop = chatLog.scrollHeight; }
      };
      reader.readAsDataURL(file);
    };
  }

  // run_instruction "Done" button
  const doneBtn = document.getElementById('instruction-done-btn');
  if (doneBtn) {
    doneBtn.addEventListener('click', () => {
      if (state.geminiClient && state.geminiClient.isConnected()) {
        state.geminiClient.send(JSON.stringify({ type: 'instruction_done' }));
      }
      const overlay = document.getElementById('instruction-overlay');
      if (overlay) overlay.style.display = 'none';
    });
  }
}

export function sendText() {
  const textInput = document.getElementById('textInput');
  const text = textInput ? textInput.value : '';
  if (text && state.geminiClient && state.geminiClient.isConnected()) {
    resetLatencyTimer();
    state.geminiClient.sendText(text);
    appendMessage('user', text);
    if (textInput) textInput.value = '';
  }
}
