"""
__main__.py — CLI entry point for the SAM3 test client.

Sends a segmentation request to the SAM3 ZMQ server with an image, a click point, 
and a text label, then displays and saves the resulting mask.
"""
from __future__ import annotations

import argparse
import logging
import sys

from .components.client import Sam3Client
from .components.config import parse_args
from .components.visualize import show_result

_handler = logging.StreamHandler(sys.stdout)
_handler.setFormatter(logging.Formatter(
    fmt="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
))

logger = logging.getLogger("sam3_client")
logger.setLevel(logging.INFO)
logger.addHandler(_handler)
logger.propagate = False


def main() -> None:
    cfg = parse_args()

    logger.info("SAM3 ZMQ Client — starting up")
    logger.info("server : tcp://%s:%d", cfg.host, cfg.port)
    logger.info("image : %s", cfg.image)
    logger.info("point : (%.1f, %.1f)", cfg.x, cfg.y)
    logger.info("text : %r", cfg.text)
    logger.info("threshold : %.3f", cfg.threshold)

    try:
        logger.info("Connecting to tcp://%s:%d…", cfg.host, cfg.port)
        with Sam3Client(host=cfg.host, port=cfg.port, timeout_ms=cfg.timeout) as client:
            logger.info("Sending request…")
            result = client.segment(
                image=cfg.image,
                point_x=cfg.x,
                point_y=cfg.y,
                text=cfg.text,
                threshold=cfg.threshold,
            )

        if result["has_mask"]:
            logger.info("Mask found  score=%.4f  %d px", result["score"], int(result["mask"].sum()))
        else:
            logger.warning("No mask above threshold.")

        output = None if cfg.no_display and not cfg.output else cfg.output
        show_result(image=cfg.image, result=result, point_x=cfg.x, point_y=cfg.y, text=cfg.text, output_path=output, display=not cfg.no_display)

    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
        sys.exit(0)
    except RuntimeError as exc:
        logger.error("Server error: %s", exc)
        sys.exit(1)
    except Exception as exc:
        logger.error("Unexpected error: %s", exc, exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
