#!/usr/bin/env python3
"""Example 5 - undistortion, the alpha knob, and the K that comes out of it.

Run:  uv run python/examples/05_undistort_and_alpha.py [--save out.png]

Undistorting is not a filter you apply and forget.  It produces a *different
camera*: a new K, a new field of view, and a region of the output that contains
no real data.  Using the original K on an undistorted image is one of the most
common bugs in production perception code.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import fov_deg, make_D, make_K, project_points  # noqa: E402
from camintrinsics.patterns import (checkerboard_image,            # noqa: E402
                                    distort_image, grid_image)
from camintrinsics.renderer import hstack_labeled, save_image      # noqa: E402

W, H = 480, 360
K = make_K(fx=W * 0.52, fy=W * 0.52, cx=W / 2 - 6, cy=H / 2 + 4)
D = make_D(k1=-0.34, k2=0.11, p1=0.0008, p2=-0.0006)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    ideal = grid_image(W, H, step=30)
    K_ideal = make_K(W / 2, W / 2, W / 2, H / 2)
    raw = distort_image(ideal, K, D, K_ideal)

    print(__doc__)

    def alpha_table(Kc, Dc, title, collect=False):
        print(f"\n{title}")
        print(f"  original:  fx {Kc[0,0]:.1f}  cx {Kc[0,2]:.1f}  cy {Kc[1,2]:.1f}"
              f"   hFOV {fov_deg(Kc, W, H)[0]:.1f} deg")
        header = (f"  {'alpha':>6s} {'fx_new':>9s} {'fy_new':>9s} {'cx_new':>9s} "
                  f"{'cy_new':>9s} {'hFOV':>7s} {'valid ROI':>12s} {'kept':>7s}")
        print(header)
        print("  " + "-" * (len(header) - 2))
        tiles, labels = [], []
        for alpha in (0.0, 0.5, 1.0):
            K_new, roi = cv2.getOptimalNewCameraMatrix(Kc, Dc, (W, H), alpha, (W, H))
            x, y, rw, rh = roi
            kept = 100.0 * (rw * rh) / (W * H)
            print(f"  {alpha:6.1f} {K_new[0,0]:9.1f} {K_new[1,1]:9.1f} "
                  f"{K_new[0,2]:9.1f} {K_new[1,2]:9.1f} "
                  f"{fov_deg(K_new, W, H)[0]:7.1f} {f'{rw}x{rh}':>12s} {kept:6.1f}%")
            if collect:
                vis = cv2.undistort(raw, Kc, Dc, None, K_new)
                if rw > 0 and rh > 0:
                    cv2.rectangle(vis, (x, y), (x + rw, y + rh), (90, 230, 90), 2)
                tiles.append(vis)
                labels.append(f"undistorted, alpha = {alpha:.1f}")
        return tiles, labels

    made, made_labels = alpha_table(K, D, "BARREL lens  (k1 = -0.34)", collect=True)
    tiles = [raw] + made
    labels = ["what the camera records"] + made_labels
    D_pin = make_D(k1=+0.30, k2=-0.05)
    alpha_table(K, D_pin, "PINCUSHION lens  (k1 = +0.30), same K")

    print("\nreading those tables")
    print("  alpha = 0  the output holds only real pixels: OpenCV inscribes the")
    print("             largest rectangle inside the undistorted image outline.")
    print("             The reported ROI is then the whole frame.")
    print("  alpha = 1  every input pixel is kept: OpenCV takes the bounding")
    print("             rectangle instead. Black curved borders appear and the")
    print("             valid ROI shrinks. Outside it, pixels are invented.")
    print("\n  Which way fx_new moves is decided by the distortion, not by alpha:")
    print("    barrel     the periphery was squeezed, so undistorting spreads it")
    print("               out -- fx_new DROPS and the FOV grows, at every alpha.")
    print("    pincushion the periphery was stretched, so undistorting pulls it")
    print("               back in -- fx_new RISES and the FOV shrinks.")
    print("  alpha only decides how much of that new image you keep.")
    print("\n  In every case cx_new and cy_new move too. If you keep using the old")
    print("  K on the undistorted image, every projection is wrong by the")
    print("  difference -- a constant offset that looks like a calibration error.")

    # --- prove the point numerically -------------------------------------
    print("\n" + "=" * 70)
    print("numerical proof: one 3D point, three ways of getting its pixel")
    print("=" * 70)
    Pw = np.array([[0.8, -0.35, 4.0]])
    K_new, _ = cv2.getOptimalNewCameraMatrix(K, D, (W, H), 0.0, (W, H))
    uv_raw = project_points(Pw, K, D)[0]
    uv_correct = project_points(Pw, K_new, None)[0]      # undistorted image
    uv_wrong = project_points(Pw, K, None)[0]            # old K, no distortion
    print(f"  in the raw (distorted) image, using K and D : "
          f"({uv_raw[0]:7.2f}, {uv_raw[1]:7.2f})   <- correct")
    print(f"  in the undistorted image, using K_new       : "
          f"({uv_correct[0]:7.2f}, {uv_correct[1]:7.2f})   <- correct")
    print(f"  in the undistorted image, using the old K   : "
          f"({uv_wrong[0]:7.2f}, {uv_wrong[1]:7.2f})   <- WRONG")
    print(f"  the bug is worth {np.linalg.norm(uv_correct - uv_wrong):.1f} px here, and it grows")
    print("  toward the image corners.")
    print("\n  Rule of thumb: an undistorted image has D = [0,0,0,0,0] and K_new.")
    print("  Carry both together, or do not undistort at all and keep projecting")
    print("  with (K, D) -- which is usually faster anyway.")

    if args.save:
        fig = np.vstack([hstack_labeled(tiles[:2], labels[:2]),
                         hstack_labeled(tiles[2:], labels[2:])])
        if not save_image(args.save, fig):
            return 1
    else:
        print("\n(pass --save out.png to write the figure)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
