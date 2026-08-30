#!/usr/bin/env python3
"""Example 3 - reading a D vector: what each coefficient looks like.

Run:  uv run python/examples/03_distortion_anatomy.py [--save out.png]

Six lenses, one chart, one figure.  Each column changes exactly one coefficient
so you can learn the visual signature of k1, k2, k3, p1 and p2 well enough to
guess a D vector by eye.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import (distort_normalized, make_D, make_K,  # noqa: E402
                           max_distorted_radius, max_valid_radius)
from camintrinsics.patterns import distort_image, grid_image     # noqa: E402
from camintrinsics.plots import plot_distortion_profile          # noqa: E402
from camintrinsics.renderer import hstack_labeled, save_image    # noqa: E402

W, H = 300, 240
CASES = [
    ("no distortion",        make_D()),
    ("k1 = -0.35  barrel",   make_D(k1=-0.35)),
    ("k1 = +0.35  pincushion", make_D(k1=+0.35)),
    ("k1=-0.4 k2=+0.25  moustache", make_D(k1=-0.40, k2=+0.25)),
    ("p1 = +0.05  tangential", make_D(p1=+0.05)),
    ("p2 = +0.05  tangential", make_D(p2=+0.05)),
]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default=None)
    args = ap.parse_args()

    chart = grid_image(W, H, step=24)
    K = make_K(W * 0.55, W * 0.55, W / 2, H / 2)
    K_ideal = make_K(W / 2, W / 2, W / 2, H / 2)

    print(__doc__)
    print("r' sampled along the +x axis (so p1, whose term is 2*p1*x*y,")
    print("contributes nothing there -- tangential effects are not radial):\n")
    header = f"{'lens':32s} {'r=0.3':>9s} {'r=0.6':>9s} {'r=1.0':>9s} {'folds at r':>11s}"
    print(header)
    print("-" * len(header))
    images, labels = [], []
    for name, D in CASES:
        rs = [0.3, 0.6, 1.0]
        rd = [distort_normalized([[r, 0.0]], D)[0, 0] for r in rs]
        print(f"{name:32s} " + " ".join(f"{v:9.4f}" for v in rd)
              + f" {max_valid_radius(D):11.2f}")
        img = distort_image(chart, K, D, K_ideal)
        images.append(img)
        labels.append(name)

    rows = []
    for i in range(0, len(CASES), 3):
        strip = hstack_labeled(images[i:i + 3], labels[i:i + 3])
        plots = hstack_labeled(
            [cv2.resize(plot_distortion_profile(D, 300, 200), (W, 190))
             for _, D in CASES[i:i + 3]],
            ["r' vs r"] * len(CASES[i:i + 3]))
        rows += [strip, plots]
    fig = np.vstack(rows)

    print("\nhow to read a D vector by eye")
    print("  k1 < 0            barrel: the frame edges bow outward, corners pull in.")
    print("  k1 > 0            pincushion: edges bow inward.")
    print("  k1 < 0, k2 > 0    'moustache': barrel near the centre, pincushion at")
    print("                    the edge. Very common in wide zooms; a single k1")
    print("                    cannot model it, which is why k2 exists.")
    print("  p1, p2 != 0       the lens is not centred on the sensor. The pattern")
    print("                    goes lopsided instead of symmetric. Real values are")
    print("                    tiny (1e-4 .. 1e-3); anything near 0.01 means your")
    print("                    calibration is fitting noise.")
    print("  k3                only matters for very wide lenses; on a normal lens")
    print("                    it is poorly constrained and often best fixed to 0")
    print("                    (cv2.CALIB_FIX_K3).")
    fold = make_D(k1=-0.5)
    print("\n  invertibility: undistortion only exists below the radius where")
    print("  r'(r) stops increasing. For k1 = -0.50 the curve peaks at")
    print(f"  r = {max_valid_radius(fold):.3f}, so no point with r' above "
          f"{max_distorted_radius(fold):.3f} can ever")
    print("  be undistorted -- cv2.undistort returns silent garbage there.")

    if args.save:
        if not save_image(args.save, fig):
            return 1
    else:
        print("\n(pass --save out.png to write the figure)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
