"""
protocol.py — ZMQ msgpack request/response serialization.

Defines the wire format between the SAM3 server and any ZMQ client: decodes incoming
requests, validates required fields, and encodes segmentation results or error messages.
"""

from __future__ import annotations

from typing import Optional

import msgpack
import numpy as np


_REQUIRED = ("image", "point_x", "point_y", "text")


def decode_request(raw: bytes) -> dict:
    req = msgpack.unpackb(raw, raw=False)
    missing = [f for f in _REQUIRED if f not in req]
    if missing:
        raise ValueError(f"Missing required fields: {missing}")
    req["point_x"]   = float(req["point_x"])
    req["point_y"]   = float(req["point_y"])
    req["text"]      = str(req["text"]).strip() or "visual"
    req["threshold"] = float(req.get("threshold", 0.05))
    return req


def encode_ok(has_mask: bool, mask_np: Optional[np.ndarray] = None, score: Optional[float] = None) -> bytes:
    payload: dict = {"status": "ok", "has_mask": has_mask}
    if has_mask:
        payload["mask"]       = mask_np.tobytes()
        payload["mask_shape"] = list(mask_np.shape)
        payload["score"]      = score
    return msgpack.packb(payload, use_bin_type=True)


def encode_error(msg: str) -> bytes:
    return msgpack.packb({"status": "error", "error_msg": msg}, use_bin_type=True)
