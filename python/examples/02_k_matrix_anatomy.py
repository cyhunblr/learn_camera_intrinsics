#!/usr/bin/env python3
"""Example 2 - what each entry of K actually does.

Run:  uv run python/examples/02_k_matrix_anatomy.py [--save out.png]

Renders the same 3D scene four times, changing exactly one thing each time, and
prints the field of view alongside.  The point to internalise: K contains no
rotation and no translation.  It cannot move the camera.  It only decides how
the already-projected ray lands on the sensor grid.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import fov_deg, make_K                       # noqa: E402
from camintrinsics.renderer import (draw_crosshair, draw_text_block,  # noqa: E402
                                    hstack_labeled, orbit_pose, render,
                                    save_image)
from camintrinsics.scene import default_scene                   # noqa: E402

W, H = 420, 320
BASE_F = 320.0


def shot(K, caption):
    img = np.full((H, W, 3), (24, 24, 28), np.uint8)
    render(img, default_scene(), orbit_pose([0, 0, 4.0], 4.4, 14.0, 10.0), K, None)
    draw_crosshair(img, K)
    hf, vf, _ = fov_deg(K, W, H)
    draw_text_block(img, [f"fx {K[0,0]:.0f}  fy {K[1,1]:.0f}",
                          f"cx {K[0,2]:.0f}  cy {K[1,2]:.0f}  skew {K[0,1]:.0f}",
                          f"FOV {hf:.1f} x {vf:.1f} deg"],
                    org=(10, 20), scale=0.42, line_h=16)
    return img, caption


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    variants = [
        (make_K(BASE_F, BASE_F, W / 2, H / 2), "baseline"),
        (make_K(BASE_F * 2, BASE_F * 2, W / 2, H / 2), "fx,fy x2  ->  zoom in, FOV halves"),
        (make_K(BASE_F, BASE_F * 1.7, W / 2, H / 2), "fy x1.7  ->  non-square pixels"),
        (make_K(BASE_F, BASE_F, W / 2 - 120, H / 2 + 60), "cx,cy moved -> image shifts, camera does not"),
    ]
    tiles = [shot(K, c) for K, c in variants]
    row1 = hstack_labeled([tiles[0][0], tiles[1][0]], [tiles[0][1], tiles[1][1]])
    row2 = hstack_labeled([tiles[2][0], tiles[3][0]], [tiles[2][1], tiles[3][1]])
    grid = np.vstack([row1, row2])

    print(__doc__)
    print(f"{'variant':46s} {'hFOV':>7s} {'vFOV':>7s} {'dFOV':>7s}")
    for K, cap in variants:
        hf, vf, df = fov_deg(K, W, H)
        print(f"{cap:46s} {hf:7.2f} {vf:7.2f} {df:7.2f}")

    print("\nthings worth noticing")
    print("  * fx and fy are focal lengths measured IN PIXELS, so they change")
    print("    when you resize the image even though the lens did not change.")
    print("  * fx != fy only means the pixels are not square (or someone")
    print("    stretched the image). Modern sensors give fx/fy within ~1%.")
    print("  * moving cx,cy slides the image; it does NOT rotate the camera.")
    print("    A rotation changes what is visible at infinity, a cx shift does not.")
    print("  * skew is the shear between the sensor axes. It is 0 for every")
    print("    digital sensor you will meet, and cv2.projectPoints ignores it")
    print("    entirely -- see docs/course/02_the_K_matrix.md.")

    if args.save:
        if not save_image(args.save, grid):
            return 1
    else:
        print("\n(pass --save out.png to write the figure)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
