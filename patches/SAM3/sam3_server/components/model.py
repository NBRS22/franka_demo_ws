"""
model.py — SAM3 model loading and warm-up.

Builds the SAM3 model and its processor into a ModelBundle, and runs one dummy
inference at startup to trigger CUDA JIT compilation before the first real request.
"""
from __future__ import annotations

import logging
import torch
import time

from dataclasses import dataclass
from PIL import Image

from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor
from .config import ServerConfig

logger = logging.getLogger(__name__)


@dataclass
class ModelBundle:
    model: torch.nn.Module
    processor: Sam3Processor
    device: str


def load_model(config: ServerConfig) -> ModelBundle:
    logger.info("Loading SAM3 on %s…", config.device)
    t0 = time.perf_counter()

    model = build_sam3_image_model()
    model = model.to(config.device)
    model.eval()

    n_params = sum(p.numel() for p in model.parameters()) / 1e6
    logger.info("Model parameters: %.0f M", n_params)

    processor = Sam3Processor(
        model,
        device=config.device,
        confidence_threshold=config.default_threshold,
    )

    if config.device == "cuda":
        mem_gb = torch.cuda.memory_allocated() / 1e9
        logger.info("GPU memory allocated: %.2f GB", mem_gb)

    logger.info("Model ready in %.1f s", time.perf_counter() - t0)
    return ModelBundle(model=model, processor=processor, device=config.device)


def warmup_model(bundle: ModelBundle) -> None:
    """One dummy inference to trigger CUDA JIT compilation."""
    from .inference import run_inference

    logger.info("Warming up CUDA kernels…")
    t0 = time.perf_counter()
    dummy = Image.new("RGB", (256, 256), color=(0, 0, 0))
    try:
        run_inference(bundle, dummy, 128.0, 128.0, "visual", 0.05)
        logger.info("Warm-up done in %.1f s", time.perf_counter() - t0)
    except Exception as exc:
        logger.warning("Warm-up failed (non-fatal): %s", exc)
