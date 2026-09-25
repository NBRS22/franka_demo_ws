"""
visualize.py — Mask visualization helper for the SAM3 test client.

Renders the segmentation result as a side-by-side matplotlib figure showing
the original image with the click point and the best mask overlay.
"""
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def show_result(image: str | Image.Image, result: dict, point_x: float, point_y: float, text: str,
    output_path: str | None = "sam3_client/output/result.png", display: bool = True) -> None:
    """
    Side-by-side display: original image with click  |  best mask overlay.

    Args:
        image        path or PIL Image
        result       dict returned by Sam3Client.segment()
        point_x/y    click coordinates in pixels
        text         text prompt used
        output_path  save path - None to skip saving
    """
    if not display:
        plt.switch_backend("Agg")  # headless: never open a GUI window (plt.show would block)
    if isinstance(image, str):
        image = Image.open(image).convert("RGB")

    img_np = np.array(image)
    W, H   = image.size

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f'SAM3  —  "{text}"  +  clic positif', fontsize=13, fontweight="bold")

    # ── Left: original image + click ──────────────────────────────────────────
    axes[0].imshow(img_np)
    axes[0].scatter(
        point_x, point_y,
        c="red", s=220, zorder=5, marker="*",
        edgecolors="white", linewidths=0.8,
        label="Clic positif",
    )
    axes[0].set_title(f"Input Image ({W}×{H})")
    axes[0].legend(loc="upper right", fontsize=9)
    axes[0].axis("off")

    # ── Right: mask overlay ────────────────────────────────────────────────────
    axes[1].imshow(img_np)

    if not result.get("has_mask"):
        axes[1].set_title("No mask returned")
    else:
        mask_np = result["mask"]          
        score   = result["score"]

        overlay = np.zeros((*mask_np.shape, 4), dtype=np.float32)
        overlay[mask_np] = [0.18, 0.80, 0.18, 0.50]
        axes[1].imshow(overlay)

        axes[1].contour(
            mask_np.astype(float), levels=[0.5],
            colors=["lime"], linewidths=1.8,
        )
        axes[1].scatter(
            point_x, point_y,
            c="red", s=220, zorder=6, marker="*",
            edgecolors="white", linewidths=0.8,
        )

        axes[1].set_title(
            f"Masque  score={score:.3f}  {int(mask_np.sum())} px"
        )

    axes[1].axis("off")
    plt.tight_layout()

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Saved → {output_path}")

    if display:
        plt.show()
    else:
        plt.close(fig)
