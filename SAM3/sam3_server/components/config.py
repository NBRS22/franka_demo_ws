"""
config.py — Server configuration dataclass and CLI argument parser.

Defines ServerConfig with all runtime parameters.
"""
from __future__ import annotations

import argparse

from dataclasses import dataclass


@dataclass
class ServerConfig:
    port: int = 5557
    device: str = "cuda"
    default_threshold: float = 0.05
    warmup: bool = True


def parse_args() -> ServerConfig:
    p = argparse.ArgumentParser(description="SAM3 ZMQ segmentation server")
    p.add_argument("--port",      type=int,   default=5557,   help="ZMQ REP port")
    p.add_argument("--device",    type=str,   default="cuda", help="torch device")
    p.add_argument("--threshold", type=float, default=0.05,   help="confidence threshold")
    p.add_argument("--no-warmup", action="store_true",         help="skip warm-up inference")
    args = p.parse_args()
    return ServerConfig(
        port=args.port,
        device=args.device,
        default_threshold=args.threshold,
        warmup=not args.no_warmup,
    )
