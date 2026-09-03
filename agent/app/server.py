"""FastAPI application: routes, WebSocket session, and app factory."""

import asyncio
import base64
import json
import logging
import os
import re
import time

import fastapi
from fastapi import responses as fastapi_responses
from fastapi import staticfiles as fastapi_staticfiles
from fastapi.middleware import cors as cors_middleware

from agents import agent as agent_lib
from app.config import ServerConfig, resolve_thinking_level
from embodiment import human as human_embodiment_lib
from embodiment.franka import franka_embodiment as franka_embodiment_lib
from embodiment.franka import franka_vla_embodiment as franka_vla_embodiment_lib
from embodiment.spot import spot_embodiment as spot_embodiment_lib
from model import tts_client as tts_client_lib
from session import config as session_config
from session import manager as session_manager

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _resolve_ui_dir() -> str | None:
  this_dir = os.path.dirname(os.path.abspath(__file__))
  candidate = os.path.join(os.path.dirname(this_dir), "ui")
  if os.path.isdir(candidate):
    return candidate
  return None


def _validate_agent_name(name: object) -> str:
  if not isinstance(name, str) or not name.strip():
    raise fastapi.HTTPException(status_code=400, detail="agent_name is required")
  name = name.strip()
  try:
    agent_lib.from_name(name)
  except ValueError as exc:
    raise fastapi.HTTPException(status_code=400, detail=str(exc)) from exc
  return name


def _set_optional_override(
    overrides: dict[str, str],
    agent_name: str,
    value: object,
) -> None:
  if value is None or value == "":
    overrides.pop(agent_name, None)
    return
  if not isinstance(value, str):
    raise fastapi.HTTPException(status_code=400, detail="Instruction values must be strings")
  overrides[agent_name] = value


# ---------------------------------------------------------------------------
# Router
# ---------------------------------------------------------------------------

router = fastapi.APIRouter()


@router.get("/")
async def root(request: fastapi.Request):
  ui_dir = request.app.state.ui_dir
  if not ui_dir:
    return fastapi_responses.PlainTextResponse("UI not found")
  return fastapi_responses.FileResponse(os.path.join(ui_dir, "index.html"))


@router.get("/api/camera")
async def api_camera(request: fastapi.Request):
  """Return the latest camera frame as MJPEG stream."""
  poller = request.app.state.active_poller_ref
  if not poller:
    return fastapi.Response(status_code=204)

  async def _mjpeg_gen():
    async for frame in poller.get_stream():
      yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"

  return fastapi.responses.StreamingResponse(
      _mjpeg_gen(),
      media_type="multipart/x-mixed-replace; boundary=frame",
      headers={"Cache-Control": "no-cache", "Connection": "keep-alive"},
  )


@router.post("/api/send")
async def api_send(request: fastapi.Request):
  """Inject text into the active session (inter-agent messaging)."""
  body = await request.json()
  text = body.get("text", "")
  active_session = request.app.state.active_session
  if not active_session or not text:
    return fastapi.responses.JSONResponse(
        status_code=400, content={"error": "No active session or empty text"}
    )
  await active_session.get_text_queue().put(text)
  return {"status": "ok"}


@router.get("/api/server_defaults")
async def api_server_defaults(request: fastapi.Request):
  config = request.app.state.config
  return {
      "models": list(session_config.KNOWN_MODELS.keys()),
      "default_model": config.model,
      "default_modality": config.response_modality,
      "use_tts": config.use_tts,
  }


@router.post("/api/config/instructions")
async def update_instructions(request: fastapi.Request):
  """Update runtime instructions used when the next agent session starts."""
  body = await request.json()
  agent_name = _validate_agent_name(body.get("agent_name"))
  config = request.app.state.config

  _set_optional_override(config.custom_si, agent_name, body.get("system_instruction"))
  _set_optional_override(config.custom_di, agent_name, body.get("developer_instruction"))
  _set_optional_override(config.custom_heartbeat_text, agent_name, body.get("heartbeat_text"))

  if "disabled_tools" in body:
    disabled = body["disabled_tools"]
    if isinstance(disabled, list):
      if disabled:
        config.custom_disabled_tools[agent_name] = [str(t) for t in disabled]
      else:
        config.custom_disabled_tools.pop(agent_name, None)

  for field in ("heartbeat_interval_seconds", "heartbeat_min_delay_seconds"):
    value = body.get(field)
    if value is None:
      continue
    try:
      value = float(value)
    except (TypeError, ValueError) as exc:
      raise fastapi.HTTPException(status_code=400, detail=f"{field} must be a number") from exc
    if value <= 0:
      raise fastapi.HTTPException(status_code=400, detail=f"{field} must be greater than zero")
    setattr(config, field, value)

  if "use_event_driven_heartbeat" in body:
    value = body["use_event_driven_heartbeat"]
    if not isinstance(value, bool):
      raise fastapi.HTTPException(status_code=400, detail="use_event_driven_heartbeat must be a boolean")
    config.use_event_driven_heartbeat = value

  return {
      "success": True,
      "agent_name": agent_name,
      "requires_reconnect": request.app.state.active_session is not None,
  }


@router.delete("/api/config/instructions")
async def reset_instructions(request: fastapi.Request, agent_name: str):
  """Reset one agent's overrides to startup values."""
  agent_name = _validate_agent_name(agent_name)
  config = request.app.state.config
  config.custom_si.pop(agent_name, None)
  config.custom_di.pop(agent_name, None)
  config.custom_heartbeat_text.pop(agent_name, None)
  config.custom_disabled_tools.pop(agent_name, None)

  defaults = request.app.state.instruction_defaults
  config.heartbeat_interval_seconds = defaults["heartbeat_interval_seconds"]
  config.heartbeat_min_delay_seconds = defaults["heartbeat_min_delay_seconds"]
  config.use_event_driven_heartbeat = defaults["use_event_driven_heartbeat"]
  return {
      "success": True,
      "agent_name": agent_name,
      "requires_reconnect": request.app.state.active_session is not None,
  }


@router.get("/api/agent_config/{name}")
async def api_agent_config(
    request: fastapi.Request,
    name: str,
    endpoint_type: str = "gemini_live_api",
    model: str | None = None,
):
  """Return the resolved agent configuration."""
  config = request.app.state.config
  try:
    agent = agent_lib.from_name(name)
    agent_si = config.custom_si.get(name, "")
    agent_di = config.custom_di.get(name, "")
    agent_hb = (
        config.custom_heartbeat_text.get(name, "")
        or session_manager.get_default_heartbeat_text()
    )
    if agent_si:
      agent.system_instruction = agent_si
    if agent_di:
      agent.developer_instruction = agent_di
  except ValueError:
    raise fastapi.HTTPException(status_code=400, detail=f"Unknown agent name: {name}")

  decls = []
  for t in agent.tools:
    if "functionDeclarations" in t:
      decls.extend(t["functionDeclarations"])
    else:
      decls.append(t)

  return {
      "name": agent.name,
      "system_instruction": agent.system_instruction,
      "developer_instruction": agent.developer_instruction,
      "is_modified": bool(agent_si or agent_di or config.custom_heartbeat_text.get(name, "")),
      "tools": decls,
      "model": config.model,
      "response_modality": config.response_modality,
      "use_tts": config.use_tts,
      "tts_voice": config.tts_voice,
      "tts_language_code": config.tts_language_code,
      "endpoint_type": endpoint_type,
      "service_address": "generativelanguage.googleapis.com",
      "heartbeat_text": agent_hb,
      "heartbeat_interval_seconds": config.heartbeat_interval_seconds,
      "heartbeat_min_delay_seconds": config.heartbeat_min_delay_seconds,
      "use_event_driven_heartbeat": config.use_event_driven_heartbeat,
      "thinking_level": resolve_thinking_level(model or config.model),
      "disabled_tools": config.custom_disabled_tools.get(name, []),
  }


# ---------------------------------------------------------------------------
# WebSocket
# ---------------------------------------------------------------------------

@router.websocket("/ws")
async def websocket_endpoint(
    websocket: fastapi.WebSocket,
    agent_name: str = "human",
    custom_si: str = "",
    enabled_tools: str = "",
    response_modality: str | None = None,
    model: str | None = None,
    use_tts: bool | None = None,
    thinking_level: str | None = None,
    endpoint_type: str = "gemini_live_api",
):
  """WebSocket: Proactive Agent session with Gemini Live API."""
  app = websocket.app
  config = app.state.config

  await websocket.accept()

  async with app.state.session_lock:
    if app.state.active_session is not None:
      await websocket.close(code=1008, reason="Only one active session allowed")
      return

    if agent_name == "spot":
      current_embodiment = spot_embodiment_lib.SpotEmbodiment(robot_url=config.robot_url)
      await current_embodiment.initialize()
      app.state.active_poller_ref = current_embodiment.poller
    elif agent_name == "franka":
      current_embodiment = franka_embodiment_lib.FrankaEmbodiment(robot_url=config.robot_url)
      app.state.active_poller_ref = current_embodiment.poller
    elif agent_name == "franka_vla":
      current_embodiment = franka_vla_embodiment_lib.FrankaVlaEmbodiment(robot_url=config.robot_url)
      app.state.active_poller_ref = current_embodiment.poller
    else:
      current_embodiment = await human_embodiment_lib.HumanEmbodiment.create()
      app.state.active_poller_ref = getattr(current_embodiment, "poller", None)

    session_cfg = session_config.SessionConfig(
        agent_name=agent_name,
        custom_si=custom_si,
        enabled_tools=enabled_tools.split(",") if enabled_tools else [],
        response_modality=response_modality,
        model=model or config.model,
        use_tts=use_tts,
        thinking_level=thinking_level,
        endpoint_type=endpoint_type,
    )

    custom_si_override = session_cfg.custom_si or config.custom_si.get(agent_name, "")
    custom_di_override = session_cfg.custom_di or config.custom_di.get(agent_name, "")
    heartbeat_text_override = (
        session_cfg.heartbeat_text or config.custom_heartbeat_text.get(agent_name, "")
    )
    resolved_model = session_cfg.model or config.model
    assert resolved_model is not None
    resolved_thinking_level = resolve_thinking_level(resolved_model, session_cfg.thinking_level)

    agent = agent_lib.from_name(agent_name)
    if custom_si_override:
      agent.system_instruction = custom_si_override
    if custom_di_override:
      agent.developer_instruction = custom_di_override

    agent_tools = (
        current_embodiment.get_tools() if agent_name == "spot" else agent.tools
    )
    if session_cfg.enabled_tools:
      enabled = set(session_cfg.enabled_tools)
      agent_tools = [
          {**group, "functionDeclarations": [
              d for d in group.get("functionDeclarations", [])
              if d.get("name") in enabled
          ]}
          for group in agent_tools if group.get("functionDeclarations")
      ]
      agent_tools = [g for g in agent_tools if g["functionDeclarations"]]
    elif config.custom_disabled_tools.get(agent_name):
      disabled = set(config.custom_disabled_tools[agent_name])
      agent_tools = [
          {**group, "functionDeclarations": [
              d for d in group.get("functionDeclarations", [])
              if d.get("name") not in disabled
          ]}
          for group in agent_tools if group.get("functionDeclarations")
      ]
      agent_tools = [g for g in agent_tools if g["functionDeclarations"]]

    modality = session_cfg.response_modality or config.response_modality
    resolved_use_tts = (
        session_cfg.use_tts if session_cfg.use_tts is not None else config.use_tts
    )

    app.state.active_session = session_manager.SessionManager(
        model=resolved_model,
        embodiment_instance=current_embodiment,
        tools=agent_tools,
        system_instruction=agent.system_instruction,
        developer_instruction=agent.developer_instruction,
        response_modality=modality,
        dump_video_dir=config.dump_video_dir or None,
        api_key=config.api_key,
        heartbeat_interval_seconds=config.heartbeat_interval_seconds,
        heartbeat_enabled=config.heartbeat_enabled,
        heartbeat_min_delay_seconds=config.heartbeat_min_delay_seconds,
        agent_peers=config.agent_peers,
        peer_name=agent_name,
        use_event_driven_heartbeat=config.use_event_driven_heartbeat,
        heartbeat_safety_timeout_seconds=config.heartbeat_safety_timeout_seconds,
        media_resolution=config.media_resolution,
        heartbeat_text=heartbeat_text_override,
        enable_send_message_to_user=config.enable_send_message_to_user,
        endpoint_type=session_cfg.endpoint_type,
        thinking_level=resolved_thinking_level,
    )
    app.state.session_agent_name = agent_name

  try:
    async def audio_output(data):
      try:
        await websocket.send_bytes(data)
      except Exception:  # pylint: disable=broad-except
        pass

    text_output_cb = None
    if resolved_use_tts:
      tts_client = tts_client_lib.Tts3pClient(
          voice_name=config.tts_voice,
          language_code=config.tts_language_code,
          api_key=config.tts_api_key,
          audio_gain=config.tts_audio_gain,
      )

      async def text_output(text):
        filtered = re.sub(r"```.*?```", "", text, flags=re.DOTALL)
        if not filtered.strip():
          return
        total_bytes = 0
        first_chunk_sent = False
        t_start = 0.0
        try:
          async for chunk in tts_client.synthesize_stream(filtered):
            if not first_chunk_sent:
              t_start = time.time()
              first_chunk_sent = True
            if chunk:
              await audio_output(chunk)
              total_bytes += len(chunk)
        except Exception as e:
          logger.error("TTS streaming failed: %s", e)
        if total_bytes > 0 and t_start > 0.0:
          remaining = max(0.0, total_bytes / 48000.0 - (time.time() - t_start))
          await asyncio.sleep(remaining)

      text_output_cb = text_output

    async def receive_from_client():
      try:
        while True:
          message = await websocket.receive()
          if message.get("type") == "websocket.disconnect":
            break
          if message.get("bytes"):
            await current_embodiment.get_audio_queue().put(message["bytes"])
          elif message.get("text"):
            text = message["text"]
            try:
              payload = json.loads(text)
              if isinstance(payload, dict):
                if payload.get("type") == "image":
                  if getattr(current_embodiment, "poller", None) is None:
                    await current_embodiment.get_video_queue().put(
                        base64.b64decode(payload["data"])
                    )
                  continue
                if payload.get("type") in ("video_source", ):
                  continue
                if payload.get("type") == "instruction_done":
                  if hasattr(current_embodiment, "on_instruction_done"):
                    current_embodiment.on_instruction_done()
                  continue
                if payload.get("type") == "user_image":
                  sentinel = json.dumps({
                      "__type": "user_image",
                      "mime_type": payload.get("mime_type", "image/jpeg"),
                      "data": payload["data"],
                      "caption": payload.get("caption", ""),
                  })
                  await current_embodiment.get_text_queue().put(sentinel)
                  continue
            except json.JSONDecodeError:
              pass
            await current_embodiment.get_text_queue().put(text)
      except (fastapi.WebSocketDisconnect, asyncio.CancelledError):
        pass
      except Exception as e:
        logger.error("Receive error: %s", e)

    async def run_session():
      try:
        async for event in app.state.active_session.start_session(
            audio_output_callback=audio_output,
            text_output_callback=text_output_cb,
        ):
          if event:
            try:
              await websocket.send_json(event)
            except (fastapi.WebSocketDisconnect, RuntimeError):
              break
      except Exception as e:
        logger.error("Session error: %s", e)

    if hasattr(current_embodiment, "set_ui_callback"):
      current_embodiment.set_ui_callback(websocket.send_json)

    if getattr(current_embodiment, "poller", None) is not None:
      try:
        await websocket.send_json({"type": "video_source", "source": "realsense"})
      except Exception:  # pylint: disable=broad-except
        pass

    receive_task = asyncio.create_task(receive_from_client())
    session_task = asyncio.create_task(run_session())
    done, pending = await asyncio.wait(
        [session_task, receive_task], return_when=asyncio.FIRST_COMPLETED
    )
    for t in pending:
      t.cancel()
    if pending:
      await asyncio.gather(*pending, return_exceptions=True)

  except Exception as e:
    logger.error("Session error: %s", e, exc_info=True)
  finally:
    app.state.active_poller_ref = None
    try:
      await current_embodiment.close()
    except Exception as exc:
      logger.warning("Failed to close embodiment: %s", exc)
    async with app.state.session_lock:
      app.state.active_session = None
    try:
      await websocket.close()
    except Exception:  # pylint: disable=broad-except
      pass


# ---------------------------------------------------------------------------
# App factory
# ---------------------------------------------------------------------------

def create_app(config: ServerConfig) -> fastapi.FastAPI:
  """Create and configure the FastAPI application."""
  ui_dir = _resolve_ui_dir()
  application = fastapi.FastAPI()

  application.state.config = config
  application.state.ui_dir = ui_dir
  application.state.active_poller_ref = None
  application.state.active_session = None
  application.state.session_agent_name = None

  @application.on_event("startup")
  def startup_event():
    application.state.session_lock = asyncio.Lock()
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s"))
    for log in (logging.getLogger(), logging.getLogger("__main__")):
      log.setLevel(logging.INFO)
      log.addHandler(ch)
      log.propagate = False
  application.state.instruction_defaults = {
      "heartbeat_interval_seconds": config.heartbeat_interval_seconds,
      "heartbeat_min_delay_seconds": config.heartbeat_min_delay_seconds,
      "use_event_driven_heartbeat": config.use_event_driven_heartbeat,
  }

  application.add_middleware(
      cors_middleware.CORSMiddleware,
      allow_origins=["*"],
      allow_credentials=True,
      allow_methods=["*"],
      allow_headers=["*"],
  )

  if ui_dir:
    application.mount(
        "/static",
        fastapi_staticfiles.StaticFiles(directory=ui_dir, follow_symlink=True),
        name="static",
    )

  application.include_router(router)
  return application
