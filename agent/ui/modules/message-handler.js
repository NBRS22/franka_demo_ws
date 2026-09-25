import { state } from './state.js';
import {
  redrawOverlay, setPipelinePhase, completePipelinePhase,
  clearAllOverlayData, PIPELINE_PHASES,
} from './overlay.js';
import { resetLatencyTimer, recordTTFT, recordTTFC, recordTTLT, updateServerLatency, updateTokenUsage } from './latency.js';
import { appendMessage, appendThoughtMessage, formatFunctionResult } from './chat-log.js';
import { applyVideoSource } from './agent-selector.js';

export function handleJsonMessage(msg) {
  const chatLog = document.getElementById('chat-log');

  if (msg.type === 'heartbeat_sent') {
    resetLatencyTimer();
    return;
  }

  if (msg.type === 'segmentation_mask') {
    state.overlayData.sam3Mask = msg.data || null;
    setPipelinePhase('segment');
    redrawOverlay();
    return;
  }

  if (msg.type === 'grasp_pose') {
    state.overlayData.graspPose = msg.data || null;
    setPipelinePhase('plan');
    redrawOverlay();
    return;
  }

  if (msg.type === 'video_source') {
    applyVideoSource(msg.source, { notifyServer: false });
    return;
  }

  if (msg.type === 'run_instruction') {
    const overlay = document.getElementById('instruction-overlay');
    const textEl = document.getElementById('instruction-text');
    if (overlay && textEl) {
      textEl.textContent = msg.instruction || '';
      overlay.style.display = 'flex';
    }
    return;
  }

  if (msg.type === 'interrupted') {
    state.audioFlushPending = true;
    state.currentGeminiMessageDiv = null;
    state.currentUserMessageDiv = null;
    state.currentThinkingContentDiv = null;

  } else if (msg.type === 'turn_complete') {
    recordTTLT();
    state.currentGeminiMessageDiv = null;
    state.currentUserMessageDiv = null;
    state.currentThinkingContentDiv = null;

  } else if (msg.type === 'telemetry') {
    if (msg.server_latency || msg.server_ttft_ms) {
      updateServerLatency(msg.server_latency || {}, msg.server_ttft_ms || 0);
    }
    if (msg.token_usage) {
      updateTokenUsage(msg.token_usage);
    }

  } else if (msg.type === 'draw_points') {
    recordTTFT();
    state.overlayData.livePoints = msg.points;
    redrawOverlay();

  } else if (msg.type === 'tool_call_start') {
    if (state.selectedAgentName === 'franka') {
      if (msg.name === 'pick' && msg.args) {
        state.overlayData.pickCoords = { x: msg.args.x, y: msg.args.y, label: msg.args.label || '' };
        state.overlayData.placeCoords = null;
        state.overlayData.sam3Mask = null;
        state.overlayData.graspPose = null;
        setPipelinePhase('pick');
        redrawOverlay();
      } else if (msg.name === 'place' && msg.args) {
        state.overlayData.pickCoords = null;
        state.overlayData.placeCoords = { x: msg.args.x, y: msg.args.y };
        setPipelinePhase('place');
        redrawOverlay();
      }
    }

  } else if (msg.type === 'clear_overlay') {
    state.overlayData.livePoints = null;
    redrawOverlay();

  } else if (msg.type === 'user' || msg.type === 'user_transcript') {
    state.lastAckRow = null;
    state.ackCount = 0;
    state.currentThinkingContentDiv = null;
    if (msg.type === 'user_transcript' && state.querySendTime === null) {
      resetLatencyTimer();
    }
    if (state.currentUserMessageDiv) {
      state.currentUserMessageDiv.textContent += msg.text;
      if (chatLog && chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50) {
        chatLog.scrollTop = chatLog.scrollHeight;
      }
    } else {
      state.currentUserMessageDiv = appendMessage('user', msg.text);
    }

  } else if (msg.type === 'text_input') {
    if (typeof msg.text === 'string' && msg.text.trimStart().startsWith('[HEARTBEAT]')) return;
    state.currentThinkingContentDiv = null;
    appendMessage('system', `📨 ${msg.text}`);

  } else if (msg.type === 'gemini' || msg.type === 'gemini_transcript') {
    state.lastAckRow = null;
    state.ackCount = 0;
    state.currentThinkingContentDiv = null;
    recordTTFT();
    if (state.currentGeminiMessageDiv) {
      state.currentGeminiMessageDiv.textContent += msg.text;
      if (chatLog && chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50) {
        chatLog.scrollTop = chatLog.scrollHeight;
      }
    } else {
      state.currentGeminiMessageDiv = appendMessage('gemini', msg.text);
    }

  } else if (msg.type === 'gemini_thought') {
    state.lastAckRow = null;
    state.ackCount = 0;
    recordTTFT();
    state.currentGeminiMessageDiv = null;
    state.currentUserMessageDiv = null;
    if (state.currentThinkingContentDiv) {
      state.currentThinkingContentDiv.textContent += msg.text;
      if (chatLog && chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50) {
        chatLog.scrollTop = chatLog.scrollHeight;
      }
    } else {
      state.currentThinkingContentDiv = appendThoughtMessage(msg.text);
    }

  } else if (msg.type === 'tool_call') {
    state.currentThinkingContentDiv = null;
    recordTTFT();
    recordTTFC();
    state.toolCallCounter++;

    if (msg.name === 'ack') {
      state.ackCount++;
      if (state.lastAckRow) {
        const countSpan = state.lastAckRow.querySelector('.hb-count');
        if (countSpan) countSpan.textContent = `x${state.ackCount}`;
        if (chatLog && chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50) {
          chatLog.scrollTop = chatLog.scrollHeight;
        }
      } else {
        const row = document.createElement('div');
        row.className = 'message tool_call heartbeat-row';
        row.innerHTML = `<span class="hb-icon">❤️</span><span class="hb-count">x${state.ackCount}</span>`;
        if (chatLog) {
          const isAtBottom = chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
          chatLog.appendChild(row);
          if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
        }
        state.lastAckRow = row;
      }
    } else {
      state.lastAckRow = null;
      state.ackCount = 0;
      const args = msg.args ? JSON.stringify(msg.args) : '';
      const result = msg.result !== undefined ? formatFunctionResult(msg.result) : '';

      if (result) {
        const details = document.createElement('details');
        details.className = 'message tool_call_details';
        details.open = true;
        const summary = document.createElement('summary');
        summary.className = 'tool_call_summary';
        summary.textContent = `🔧 [${state.toolCallCounter}] ${msg.name}(${args})`;
        details.appendChild(summary);
        const contentDiv = document.createElement('div');
        contentDiv.className = 'tool_response_content';
        contentDiv.textContent = result;
        details.appendChild(contentDiv);
        if (chatLog) {
          const isAtBottom = chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
          chatLog.appendChild(details);
          if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
        }
      } else {
        appendMessage('tool_call', `🔧 [${state.toolCallCounter}] ${msg.name}(${args})`);
      }

      if (msg.name === 'run_instruction') {
        updateRobotStatus('executing', 'Executing: ' + (msg.args?.instruction || '...'));
      } else if (msg.name === 'stop') {
        updateRobotStatus('idle', 'Stopped');
      } else if (msg.name === 'reset') {
        updateRobotStatus('idle', 'Resetting...');
      }

      if (state.selectedAgentName === 'franka') {
        if (msg.name === 'home' || msg.name === 'stop') {
          clearAllOverlayData();
        }
      }
    }

  } else if (msg.type === 'tool_response') {
    state.currentThinkingContentDiv = null;
    state.lastAckRow = null;
    state.ackCount = 0;
    appendMessage('tool_response', `✅ ${msg.name}: ${formatFunctionResult(msg.result)}`);

    if (state.selectedAgentName === 'franka') {
      if (msg.name === 'pick') {
        state.overlayData.pickCoords = null;
        completePipelinePhase('pick');
        setPipelinePhase('transport');
        redrawOverlay();
      } else if (msg.name === 'place') {
        state.overlayData.placeCoords = null;
        redrawOverlay();
        PIPELINE_PHASES.forEach(p => {
          if (!state.donePipelinePhases.includes(p)) state.donePipelinePhases.push(p);
        });
        state.currentPipelinePhase = null;
        // trigger bar update via import
        import('./overlay.js').then(m => m.redrawOverlay());
        setTimeout(() => clearAllOverlayData(), 3000);
      }
    }
  }
}

function updateRobotStatus(robotState, text) {
  state.robotState = robotState;
  state.robotStateText = text;
  // refreshStatusLabel is in connection.js — import lazily to avoid circular
  import('./connection.js').then(m => m.refreshStatusLabel());
}
