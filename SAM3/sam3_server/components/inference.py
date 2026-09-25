"""
inference.py — SAM3 point + text inference pipeline.

Runs the full grounding forward pass and returns the single highest-scoring mask with its confidence score.
"""
from __future__ import annotations

import numpy as np
import torch

from typing import Optional, Tuple
from PIL import Image

from .model import ModelBundle


@torch.inference_mode()
def run_inference(
    bundle: ModelBundle,
    image: Image.Image,
    point_x: float,
    point_y: float,
    text: str,
    threshold: float,
) -> dict:
    """
    Full SAM3 point + text inference.

    Returns the processor state dict with keys:
        masks        (N, 1, H, W) bool tensor
        scores       (N,)         float tensor
        boxes        (N, 4)       float tensor  [x1, y1, x2, y2] pixels
        masks_logits (N, 1, H, W) float tensor  (raw sigmoid output)
    """
    W, H = image.size
    device = bundle.device

    bundle.processor.confidence_threshold = threshold

    # 1. Image encoder
    state = bundle.processor.set_image(image)

    # 2. Text encoder
    state["backbone_out"].update(
        bundle.model.backbone.forward_text([text], device=device)
    )

    # 3. Geometric prompt — single positive click, normalised to [0, 1]
    state["geometric_prompt"] = bundle.model._get_dummy_prompt()

    point = torch.tensor(
        [[[point_x / W, point_y / H]]], device=device, dtype=torch.float32
    )
    label     = torch.tensor([[1]], device=device, dtype=torch.long)
    attn_mask = torch.zeros(1, 1, device=device, dtype=torch.bool)

    state["geometric_prompt"].append_points(point, label, mask=attn_mask)

    # 4. Grounding head (encoder + decoder + segmentation head)
    return bundle.processor._forward_grounding(state)


def extract_best_mask(
    state: dict,
) -> Tuple[bool, Optional[np.ndarray], Optional[float]]:
    """
    Keep only the single highest-scoring mask (Option A).

    Returns:
        has_mask   bool
        mask_np    (H, W) bool numpy array, or None
        score      float, or None
    """
    masks  = state.get("masks")
    scores = state.get("scores")

    if masks is None or masks.numel() == 0:
        return False, None, None

    best_idx = int(scores.argmax())
    mask_np  = masks[best_idx, 0].cpu().numpy()
    score    = float(scores[best_idx])

    return True, mask_np, score
