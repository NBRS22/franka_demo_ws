"""
__main__.py — Entry point for the SAM3 ZMQ segmentation server.

Validates the runtime environment, loads the model, runs the optional warm-up, then starts the blocking ZMQ REP server loop.
"""
from __future__ import annotations

import logging
import torch
import sys

from .components.config import parse_args
from .components.model import load_model, warmup_model
from .components.server import serve

_handler = logging.StreamHandler(sys.stdout)
_handler.setFormatter(logging.Formatter(
    fmt="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
))

logger = logging.getLogger("sam3_server")
logger.setLevel(logging.INFO)
logger.addHandler(_handler)
logger.propagate = False


def main() -> None:
    config = parse_args()

    logger.info("SAM3 ZMQ Server — starting up")
    logger.info("port : %d", config.port)
    logger.info("device : %s", config.device)
    logger.info("threshold : %.3f", config.default_threshold)
    logger.info("warmup : %s", config.warmup)

    if config.device == "cuda":
        if not torch.cuda.is_available():
            logger.error("CUDA requested but not available — aborting.")
            sys.exit(1)
        logger.info("GPU: %s", torch.cuda.get_device_name(0))

    bundle = load_model(config)

    if config.warmup:
        warmup_model(bundle)

    logger.info("Server ready — waiting for connections on port %d", config.port)
    serve(config, bundle)


if __name__ == "__main__":
    main()
