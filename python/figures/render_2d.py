#!/usr/bin/env python3
"""Render the 2D figure: a chart, what K and D do to it, and it straightened.

    uv run python/figures/render_2d.py --out data/generated/app2d.png

This is a *renderer*, not an app. Interactive exploration lives in the web
viewer (https://cyhunblr.github.io/learn_camera_intrinsics/),
which does the same maths in JavaScript. This script exists so the figures in
the README and docs can be regenerated, and so the C++ half has an exact twin
to be checked against.
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import make_K                                 # noqa: E402
from camintrinsics.patterns import (checkerboard_image,           # noqa: E402
                                    distort_image, grid_image,
                                    photo_like, radial_target,
                                    undistort_image)
from camintrinsics.plots import (plot_distortion_field,           # noqa: E402
                                 plot_distortion_profile)
from camintrinsics.renderer import (draw_crosshair,            # noqa: E402
                                    draw_text_block, hstack_labeled,
                                    save_image)
from camintrinsics.presets import (PRESETS, kd_hud_lines,         # noqa: E402
                                   preset_model)

W, H = 440, 330
CHARTS = {
    "grid": lambda: grid_image(W, H, step=28),
    "checker": lambda: checkerboard_image(W, H, squares=8, margin=18),
    "radial": lambda: radial_target(W, H, rings=8, spokes=24),
    "street": lambda: photo_like(W, H),
}
CHART_KEYS = list(CHARTS)

CAPTION = [
    "fx,fy zoom the image.  cx,cy slide it.  k1<0 barrel, k1>0 pincushion.",
    "p1,p2 are tangential: they tilt the pattern instead of squeezing it.",
]


def compose(chart: np.ndarray, K, D, alpha: float, preset_name: str,
            caption: bool = True) -> np.ndarray:
    """Build one full frame of the app from the current parameters."""
    K_ideal = make_K(W / 2.0, W / 2.0, W / 2.0, H / 2.0)
    cam = distort_image(chart, K, D, K_ideal)
    und, K_new, roi = undistort_image(cam, K, D, alpha=alpha)

    cam_annot = cam.copy()
    draw_crosshair(cam_annot, K)

    und_annot = und.copy()
    x, y, rw, rh = roi
    if rw > 0 and rh > 0:
        cv2.rectangle(und_annot, (x, y), (x + rw, y + rh), (90, 230, 90), 1)

    top = hstack_labeled(
        [chart, cam_annot, und_annot],
        ["1) ideal pinhole  (the ground truth)",
         "2) camera view  =  K and D applied",
         f"3) undistorted  alpha={alpha:.1f}  (green = valid ROI)"],
    )

    profile = plot_distortion_profile(D, W, H)
    field = plot_distortion_field(K, D, W, H, step=34)
    hud = np.full((H, W, 3), (24, 24, 28), np.uint8)
    lines = kd_hud_lines(K, D, W, H)
    lines += ["", f"preset: {preset_name}",
              f"K_new: fx {K_new[0, 0]:6.1f}  fy {K_new[1, 1]:6.1f}",
              f"       cx {K_new[0, 2]:6.1f}  cy {K_new[1, 2]:6.1f}",
              f"valid ROI: {rw}x{rh} of {W}x{H}"]
    draw_text_block(hud, lines, org=(12, 26), scale=0.42, line_h=19, bg=False)
    bottom = hstack_labeled(
        [profile, field, hud],
        ["4) radial profile  r' vs r", "5) displacement field (D only)",
         "6) the numbers"],
    )

    width = max(top.shape[1], bottom.shape[1])
    pad = np.full((10, width, 3), (24, 24, 28), np.uint8)
    parts = [top, pad, bottom]
    if caption:
        bar = np.full((48, width, 3), (16, 16, 20), np.uint8)
        draw_text_block(bar, CAPTION, org=(14, 22), scale=0.44, line_h=19, bg=False)
        parts += [bar]
    return np.vstack(parts)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default="data/generated/app2d.png")
    ap.add_argument("--chart", choices=CHART_KEYS, default="grid")
    ap.add_argument("--preset", choices=list(PRESETS), default="webcam (mild barrel)")
    ap.add_argument("--alpha", type=float, default=0.0)
    args = ap.parse_args()

    K, D = preset_model(args.preset, W, H)
    frame = compose(CHARTS[args.chart](), K, D, args.alpha, args.preset)

    return 0 if save_image(args.out, frame) else 1


if __name__ == "__main__":
    raise SystemExit(main())
