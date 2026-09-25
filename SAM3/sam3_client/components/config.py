"""
config.py — Client configuration dataclass and CLI argument parser.

Defines ClientConfig with all runtime parameters.
"""
from __future__ import annotations

import argparse

from dataclasses import dataclass


@dataclass
class ClientConfig:
    image: str
    x: float
    y: float
    text: str
    host: str = "localhost"
    port: int = 5557
    threshold: float = 0.05
    timeout: int = 30_000
    output: str = "sam3_client/output/result.png"
    no_display: bool = False


def parse_args() -> ClientConfig:
    p = argparse.ArgumentParser(description="SAM3 ZMQ client")
    p.add_argument("image", help="path to the input image")
    p.add_argument("x", type=float, help="click x in pixels")
    p.add_argument("y", type=float, help="click y in pixels")
    p.add_argument("text", help="object label, e.g. 'person'")
    p.add_argument("--host", default="localhost", help="SAM3 server IP")
    p.add_argument("--port", type=int,   default=5557,   help="SAM3 server port")
    p.add_argument("--threshold", type=float, default=0.05,   help="confidence threshold")
    p.add_argument("--timeout", type=int,   default=30_000, help="recv timeout in ms")
    p.add_argument("--output", default="sam3_client/output/result.png", help="output image path")
    p.add_argument("--no-display", action="store_true", help="skip plt.show()")
    args = p.parse_args()
    return ClientConfig(
        image=args.image,
        x=args.x,
        y=args.y,
        text=args.text,
        host=args.host,
        port=args.port,
        threshold=args.threshold,
        timeout=args.timeout,
        output=args.output,
        no_display=args.no_display,
    )
