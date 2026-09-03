"""Cloud TTS 3P client.

Uses the public Google Cloud Text-to-Speech REST API instead of gRPC.
"""

import asyncio
import collections.abc
import logging
import urllib.request
import urllib.parse
import json
import base64
import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_QUOTA_PROJECT = "robotics-hri"


def apply_audio_gain(audio_data: bytes, gain: float) -> bytes:
  """Scales PCM LINEAR16 audio samples by gain and clips to int16 range."""
  if gain == 1.0:
    return audio_data
  samples = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
  samples *= gain
  np.clip(samples, -32768, 32767, out=samples)
  return samples.astype(np.int16).tobytes()


class Tts3pClient:
  """Streaming-interface TTS client using the public Cloud TTS REST API."""

  def __init__(
      self,
      voice_name: str = "en-US-Chirp3-HD-Puck",
      language_code: str = "en-US",
      quota_project: str = _DEFAULT_QUOTA_PROJECT,
      api_key: str | None = None,
      audio_gain: float = 1.0,
  ):
    self._voice_name = voice_name
    self._language_code = language_code
    self._quota_project = quota_project
    self._api_key = api_key
    self._audio_gain = audio_gain

  def _get_headers_and_url(self) -> tuple[dict[str, str], str]:
    headers = {"Content-Type": "application/json"}
    if not self._api_key:
      raise ValueError("api_key is required for Tts3pClient")
    url = f"https://texttospeech.googleapis.com/v1/text:synthesize?key={self._api_key}"
    return headers, url

  def synthesize(self, text: str) -> bytes:
    """Synthesize text to audio using the REST API."""
    headers, url = self._get_headers_and_url()
    body = {
        "input": {"text": text},
        "voice": {
            "languageCode": self._language_code,
            "name": self._voice_name,
        },
        "audioConfig": {
            "audioEncoding": "LINEAR16",
            "sampleRateHertz": 24000,
        },
    }

    req = urllib.request.Request(
        url,
        data=json.dumps(body).encode("utf-8"),
        headers=headers,
        method="POST",
    )

    try:
      with urllib.request.urlopen(req) as response:
        if response.status == 200:
          data = json.loads(response.read().decode("utf-8"))
          audio_data = base64.b64decode(data["audioContent"])
          logger.info(
              "TTS 3P REST synthesized %d bytes for text: %s...",
              len(audio_data),
              text[:50],
          )
          return apply_audio_gain(audio_data, self._audio_gain)
        else:
          raise RuntimeError(f"TTS REST request failed with status: {response.status}")
    except Exception as e:
      logger.error("TTS REST error: %s", e)
      raise

  async def synthesize_stream(
      self, text: str
  ) -> collections.abc.AsyncIterator[bytes]:
    """Synthesize text to audio, yielding chunks asynchronously."""
    # Since REST API is unary, we fetch the whole audio and slice it to simulate streaming.
    try:
      audio_data = await asyncio.get_running_loop().run_in_executor(
          None, self.synthesize, text
      )
    except Exception as e:
      logger.error("Failed to synthesize TTS: %s", e)
      return

    chunk_size = 4096
    for i in range(0, len(audio_data), chunk_size):
      yield audio_data[i : i + chunk_size]
      await asyncio.sleep(0.01)
