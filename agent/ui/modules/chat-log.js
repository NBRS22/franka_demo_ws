export function appendMessage(type, text) {
  const chatLog = document.getElementById('chat-log');
  if (!chatLog) return null;
  const msgDiv = document.createElement('div');
  msgDiv.className = `message ${type}`;
  msgDiv.textContent = text;
  const isAtBottom = chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
  chatLog.appendChild(msgDiv);
  if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
  return msgDiv;
}

export function appendThoughtMessage(text) {
  const chatLog = document.getElementById('chat-log');
  if (!chatLog) return null;
  const details = document.createElement('details');
  details.className = 'message thought';
  details.open = false;

  const summary = document.createElement('summary');
  summary.textContent = 'Thinking Process';
  details.appendChild(summary);

  const contentDiv = document.createElement('div');
  contentDiv.className = 'thought-content';
  contentDiv.textContent = text;
  details.appendChild(contentDiv);

  const isAtBottom = chatLog.scrollHeight - chatLog.scrollTop - chatLog.clientHeight < 50;
  chatLog.appendChild(details);
  if (isAtBottom) chatLog.scrollTop = chatLog.scrollHeight;
  return contentDiv;
}

export function escapeHtml(str) {
  const div = document.createElement('div');
  div.textContent = str;
  return div.innerHTML;
}

export function formatFunctionResult(value, depth = 0) {
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

export function prettyPrintPartJs(part) {
  if (!part) return '';
  const isThought = part.thought || part.raw_thought || part.rawThought;
  const prefix = isThought ? 'Thought: ' : '';
  if (part.text) {
    return `${prefix}${part.text}`;
  } else if (part.inlineData) {
    const mime = part.inlineData.mimeType || 'unknown';
    const len = (part.inlineData.data || '').length;
    if (mime.startsWith('image/') || mime.includes('jpeg') || mime.includes('png')) {
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
    const resp = escapeHtml(formatFunctionResult(fr.response || {})).replaceAll('\n', '<br>');
    return `<span class="chip code-chip">✅ Result</span> <code>${fr.name} -> ${resp}</code>`;
  } else if (part.fileData) {
    return `<span class="chip">📁 File</span> <code>[FileData: ${part.fileData.mimeType}]</code>`;
  }
  return `<code>[Unknown Part]</code>`;
}
