"""
server.py — ZMQ REP server loop.

Binds a ZMQ REP socket, receives msgpack requests, runs SAM3 inference under BF16
autocast, and sends back the segmentation result to the client.
"""
from __future__ import annotations

import contextlib
import io
import logging
import time
import torch
import zmq
import msgpack

from PIL import Image

from .config import ServerConfig
from .inference import extract_best_mask, run_inference
from .model import ModelBundle
from .protocol import decode_request, encode_error, encode_ok

logger = logging.getLogger(__name__)


def _autocast(device: str):
    """BF16 autocast on CUDA, no-op otherwise."""
    if device == "cuda" and torch.cuda.is_bf16_supported():
        return torch.autocast(device_type="cuda", dtype=torch.bfloat16)
    return contextlib.nullcontext()


def serve(config: ServerConfig, bundle: ModelBundle) -> None:
    ctx    = zmq.Context()
    socket = ctx.socket(zmq.REP)
    socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1 s timeout — lets Ctrl+C through
    socket.bind(f"tcp://*:{config.port}")
    logger.info("ZMQ REP socket bound on tcp://*:%d", config.port)

    req_count = 0

    try:
        while True:
            logger.debug("Waiting for next request…")
            try:
                raw = socket.recv()
            except zmq.error.Again:
                continue  # timeout expired, loop back and check for KeyboardInterrupt
            req_count += 1

            logger.info("── Request #%d  (%d bytes) ──────────────────────────", req_count, len(raw))

            # ── Health check ───────────────────────────────────────────────────
            # Not part of decode_request's schema (no image/point_x/point_y/text),
            # so it must be special-cased before the normal validation below.
            try:
                peek = msgpack.unpackb(raw, raw=False)
            except Exception:
                peek = None

            if isinstance(peek, dict) and peek.get("action") == "health":
                logger.info("  health check")
                socket.send(msgpack.packb({"status": "ok"}, use_bin_type=True))
                continue

            # ── Parse ──────────────────────────────────────────────────────────
            try:
                req = decode_request(raw)
            except Exception as exc:
                logger.warning("Rejected malformed request: %s", exc)
                socket.send(encode_error(f"Bad request: {exc}"))
                continue

            image = Image.open(io.BytesIO(req["image"])).convert("RGB")
            img_w, img_h = image.size

            logger.info("  image     : %dx%d px  (%.1f KB)",
                     img_w, img_h, len(req["image"]) / 1024)
            logger.info("  point     : (%.1f, %.1f)", req["point_x"], req["point_y"])
            logger.info("  text      : %r", req["text"])
            logger.info("  threshold : %.3f", req["threshold"])

            # ── Inference ──────────────────────────────────────────────────────
            logger.debug("Starting inference…")
            t0 = time.perf_counter()
            try:
                with _autocast(config.device):
                    state = run_inference(
                        bundle,
                        image=image,
                        point_x=req["point_x"],
                        point_y=req["point_y"],
                        text=req["text"],
                        threshold=req["threshold"],
                    )
            except Exception as exc:
                logger.error("Inference failed: %s", exc, exc_info=True)
                socket.send(encode_error(f"Inference error: {exc}"))
                continue

            elapsed_ms = (time.perf_counter() - t0) * 1000

            # ── Build response ─────────────────────────────────────────────────
            has_mask, mask_np, score = extract_best_mask(state)

            if has_mask:
                logger.info("  result    : mask found  score=%.4f  %d px",
                         score, int(mask_np.sum()))
            else:
                logger.info("  result    : no mask above threshold")

            logger.info("  inference : %.0f ms", elapsed_ms)

            resp = encode_ok(has_mask, mask_np, score)
            logger.info("  response  : %d bytes  →  sent", len(resp))

            socket.send(resp)

    except KeyboardInterrupt:
        logger.info("Shutdown requested (Ctrl+C) — stopping server…")
    finally:
        socket.close()
        ctx.term()
        logger.info("ZMQ socket closed. Server stopped after %d request(s).", req_count)
