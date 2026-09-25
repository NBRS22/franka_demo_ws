"""
client.py — ZMQ REQ client for the SAM3 segmentation server.

Encodes the image and point prompt as a msgpack request, sends it to the server,
and returns the decoded mask as a numpy array.
"""
from __future__ import annotations

import io
import zmq
import msgpack
import numpy as np

from PIL import Image


class Sam3Client:
    """
    ZMQ REQ client for the SAM3 segmentation server.

    Result dict:
        has_mask    bool
        mask        (H, W) bool numpy array  
        score       float                    
    """

    def __init__(self, host: str = "localhost", port: int = 5555, timeout_ms: int = 30_000):
        self._ctx    = zmq.Context()
        self._socket = self._ctx.socket(zmq.REQ)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.connect(f"tcp://{host}:{port}")

    def segment(
        self,
        image: str | Image.Image,
        point_x: float,
        point_y: float,
        text: str,
        threshold: float = 0.05,
    ) -> dict:
        """
        Send a segmentation request and return the decoded result.

        Args:
            image      path to an image file or a PIL Image
            point_x    click x in pixels
            point_y    click y in pixels
            text       object description
            threshold  confidence threshold

        Raises:
            RuntimeError  if the server returns an error
            zmq.error.Again  if the server does not respond within timeout
        """
        image_bytes = self._encode_image(image)
        raw_req = msgpack.packb(
            {
                "image":     image_bytes,
                "point_x":   float(point_x),
                "point_y":   float(point_y),
                "text":      text,
                "threshold": float(threshold),
            },
            use_bin_type=True,
        )
        self._socket.send(raw_req)
        raw_resp = self._socket.recv()
        return self._decode_response(raw_resp)

    def close(self) -> None:
        self._socket.close()
        self._ctx.term()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    @staticmethod
    def _encode_image(image: str | Image.Image) -> bytes:
        if isinstance(image, str):
            image = Image.open(image).convert("RGB")
        buf = io.BytesIO()
        image.save(buf, format="JPEG", quality=95)
        return buf.getvalue()

    @staticmethod
    def _decode_response(raw: bytes) -> dict:
        resp = msgpack.unpackb(raw, raw=False)
        if resp.get("status") == "error":
            raise RuntimeError(resp.get("error_msg", "Unknown server error"))
        if resp.get("has_mask"):
            shape = resp["mask_shape"]
            resp["mask"] = np.frombuffer(resp["mask"], dtype=bool).reshape(shape)
        return resp
