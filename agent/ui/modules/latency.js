import { state } from './state.js';

export function pushCapped(arr, val) {
  arr.push(val);
  if (arr.length > state.MAX_HISTORY) arr.shift();
}

export function resetLatencyTimer() {
  state.querySendTime = performance.now();
  state.ttftRecorded = false;
  state.ttfcRecorded = false;
}

export function computeCI(values) {
  const n = values.length;
  if (n === 0) return null;
  const mean = values.reduce((a, b) => a + b, 0) / n;
  if (n === 1) return { mean, lo: mean, hi: mean, n };
  const variance = values.reduce((s, v) => s + (v - mean) ** 2, 0) / (n - 1);
  const stderr = Math.sqrt(variance / n);
  const z = 1.96;
  return { mean, lo: mean - z * stderr, hi: mean + z * stderr, n };
}

export function formatCI(ci) {
  if (!ci) return '--';
  const m = (ci.mean / 1000).toFixed(2);
  if (ci.n === 1) return m + 's';
  const lo = (Math.max(0, ci.lo) / 1000).toFixed(2);
  const hi = (ci.hi / 1000).toFixed(2);
  return m + 's [' + lo + ', ' + hi + ']';
}

export function formatCIms(ci) {
  if (!ci) return '--';
  const m = Math.round(ci.mean);
  if (ci.n === 1) return m + 'ms';
  const lo = Math.round(Math.max(0, ci.lo));
  const hi = Math.round(ci.hi);
  return m + 'ms [' + lo + ', ' + hi + ']';
}

export function recordTTFT() {
  if (!state.ttftRecorded && state.querySendTime !== null) {
    const ttft = performance.now() - state.querySendTime;
    pushCapped(state.ttftHistory, ttft);
    const ci = computeCI(state.ttftHistory);
    const el = document.getElementById('ttft-val');
    const countEl = document.getElementById('ttft-count');
    if (el) el.textContent = formatCI(ci);
    if (countEl) countEl.textContent = 'n=' + ci.n;
    state.ttftRecorded = true;
  }
}

export function recordTTFC() {
  if (!state.ttfcRecorded && state.querySendTime !== null) {
    const ttfc = performance.now() - state.querySendTime;
    pushCapped(state.ttfcHistory, ttfc);
    const ci = computeCI(state.ttfcHistory);
    const el = document.getElementById('ttfc-val');
    const countEl = document.getElementById('ttfc-count');
    if (el) el.textContent = formatCI(ci);
    if (countEl) countEl.textContent = 'n=' + ci.n;
    state.ttfcRecorded = true;
  }
}

export function recordTTLT() {
  if (state.querySendTime !== null) {
    const ttlt = performance.now() - state.querySendTime;
    pushCapped(state.ttltHistory, ttlt);
    const ci = computeCI(state.ttltHistory);
    const el = document.getElementById('ttlt-val');
    const countEl = document.getElementById('ttlt-count');
    if (el) el.textContent = formatCI(ci);
    if (countEl) countEl.textContent = 'n=' + ci.n;
  }
}

function updateMetric(prefix, history) {
  const ci = computeCI(history);
  const el = document.getElementById(prefix + '-val');
  const countEl = document.getElementById(prefix + '-count');
  if (el) el.textContent = ci ? formatCIms(ci) : '--';
  if (countEl) countEl.textContent = ci ? 'n=' + ci.n : '';
}

export function updateServerLatency(sl, serverTtftMs) {
  const clientTtftMs = state.ttftRecorded && state.ttftHistory.length > 0
    ? state.ttftHistory[state.ttftHistory.length - 1]
    : null;

  if (clientTtftMs !== null && serverTtftMs > 0) {
    pushCapped(state.srvClientHistory, Math.max(0, clientTtftMs - serverTtftMs));
    updateMetric('srv-client', state.srvClientHistory);
  }

  if (serverTtftMs > 0 && sl.request_ttft_ms > 0) {
    const known = (sl.prefill_queue_ms || 0) + (sl.prefill_ms || 0) +
      (sl.decode_queue_ms || 0) + (sl.decode_ttft_ms || 0);
    pushCapped(state.srvServerHistory, Math.max(0, serverTtftMs - known));
    updateMetric('srv-server', state.srvServerHistory);
  }

  if (sl.prefill_ms > 0) {
    pushCapped(state.srvPrefillHistory, sl.prefill_ms);
    updateMetric('srv-prefill', state.srvPrefillHistory);
  }

  if (sl.decode_ttft_ms > 0) {
    pushCapped(state.srvDecodeHistory, sl.decode_ttft_ms);
    updateMetric('srv-decode', state.srvDecodeHistory);
  }

  if (sl.request_ttft_ms > 0) {
    pushCapped(state.srvTtftHistory, sl.request_ttft_ms);
    updateMetric('srv-ttft', state.srvTtftHistory);
  }

  if (clientTtftMs !== null) {
    pushCapped(state.srvClientTtftHistory, clientTtftMs);
    updateMetric('srv-client-ttft', state.srvClientTtftHistory);
  }
}

export function formatTokenCount(n) {
  if (n >= 1000) return (n / 1000).toFixed(1) + 'K';
  return String(n);
}

export function updateTokenUsage(tu) {
  if (!tu) return;

  const ctxEl = document.getElementById('tok-ctx-pct');
  const inputBarEl = document.getElementById('tok-input-bar');
  const outputBarEl = document.getElementById('tok-output-bar');

  if (ctxEl) {
    const pct = tu.context_window_utilization_pct;
    ctxEl.textContent = pct != null ? pct.toFixed(1) + '%' : '--';
    if (pct != null) {
      ctxEl.style.color = pct >= 80 ? '#e74c3c' : pct >= 50 ? '#f39c12' : '';
    }
  }
  if (inputBarEl) inputBarEl.textContent = formatTokenCount(tu.prompt_token_count);
  if (outputBarEl) outputBarEl.textContent = formatTokenCount(tu.cumulative_output_tokens);

  const setEl = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
  setEl('tok-prompt-val', formatTokenCount(tu.prompt_token_count));
  setEl('tok-response-val', formatTokenCount(tu.response_token_count));
  setEl('tok-thoughts-val', formatTokenCount(tu.thoughts_token_count));
  setEl('tok-total-val', formatTokenCount(tu.total_token_count) + ' / ' + formatTokenCount(tu.context_window_limit));
  setEl('tok-cumulative-val', formatTokenCount(tu.cumulative_output_tokens));

  const modality = tu.prompt_tokens_by_modality || {};
  const parts = Object.entries(modality).filter(([, v]) => v > 0).map(([k, v]) => k + ':' + formatTokenCount(v));
  setEl('tok-modality-val', parts.length > 0 ? parts.join(' ') : '--');
}
