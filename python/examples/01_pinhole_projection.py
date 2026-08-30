#!/usr/bin/env python3
"""Example 1 - the pinhole projection, one arithmetic step at a time.

Run:  uv run python/examples/01_pinhole_projection.py

Take one 3D point in the camera frame and walk it all the way to a pixel,
printing every intermediate value.  Then check the answer against
``cv2.projectPoints``.  If you understand this file, you understand the forward
camera model; everything else in the repo is detail on top of it.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import (distort_normalized, make_D, make_K,  # noqa: E402
                           normalized_to_pixel, project_points, split_K)

K = make_K(fx=800.0, fy=800.0, cx=640.0, cy=360.0)
D = make_D(k1=-0.28, k2=0.09, p1=0.001, p2=-0.0015, k3=0.0)
P = np.array([[0.9, -0.4, 3.0]])          # a point 3 m ahead, right and up

print(__doc__)
print("K =\n", K, "\nD =", D, "\nP_cam =", P[0], "\n")

# ---- step 1: perspective divide ------------------------------------------
X, Y, Z = P[0]
x, y = X / Z, Y / Z
print("1) perspective divide   x = X/Z, y = Y/Z")
print(f"   x = {X:+.3f} / {Z:.3f} = {x:+.6f}")
print(f"   y = {Y:+.3f} / {Z:.3f} = {y:+.6f}")
print("   -> 'normalized image coordinates': the ray, with the depth thrown")
print("      away. Every point on this ray gives the same (x, y).\n")

# ---- step 2: lens distortion ---------------------------------------------
r2 = x * x + y * y
k1, k2, p1, p2, k3 = D
radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
xd, yd = distort_normalized([[x, y]], D)[0]
print("2) distortion (still unitless, still resolution independent)")
print(f"   r^2 = {r2:.6f}   radial gain = {radial:.6f}")
print(f"   tangential dx = {2 * p1 * x * y + p2 * (r2 + 2 * x * x):+.6f}")
print(f"   x' = {xd:+.6f}   y' = {yd:+.6f}")
print(f"   the lens moved the point by {np.hypot(xd - x, yd - y):.6f} in normalized units\n")

# ---- step 3: intrinsics ---------------------------------------------------
fx, fy, cx, cy, s = split_K(K)
u, v = normalized_to_pixel([[xd, yd]], K)[0]
print("3) intrinsics: scale by the focal length, shift by the principal point")
print(f"   u = fx*x' + s*y' + cx = {fx:.1f}*{xd:+.6f} + {cx:.1f} = {u:8.3f}")
print(f"   v = fy*y' + cy        = {fy:.1f}*{yd:+.6f} + {cy:.1f} = {v:8.3f}\n")

# ---- cross-check ----------------------------------------------------------
mine = project_points(P, K, D)[0]
ocv = cv2.projectPoints(P, np.zeros(3), np.zeros(3), K, D)[0].reshape(2)
print("cross-check")
print(f"   this repo      : ({mine[0]:.6f}, {mine[1]:.6f})")
print(f"   cv2.projectPoints: ({ocv[0]:.6f}, {ocv[1]:.6f})")
print(f"   difference     : {np.abs(mine - ocv).max():.2e} px\n")

# ---- the lesson about depth ----------------------------------------------
print("depth is not recoverable from one pixel:")
for d in (1.0, 3.0, 10.0, 100.0):
    q = project_points(P / Z * d, K, D)[0]
    print(f"   the same ray at Z = {d:6.1f} m -> pixel ({q[0]:.3f}, {q[1]:.3f})")
print("\n   Identical pixels. A single camera measures direction, not distance.")
print("   That is why you need stereo, a known plane, motion, or a depth sensor.")

# ---- what breaks --------------------------------------------------------
print("\npoints behind the camera (Z <= 0) have no projection:")
behind = np.array([[0.9, -0.4, -3.0]])
print("   project_points -> ", project_points(behind, K, D)[0],
      " (NaN, on purpose)")
print("   cv2.projectPoints ->",
      cv2.projectPoints(behind, np.zeros(3), np.zeros(3), K, D)[0].reshape(2),
      " (a plausible-looking lie: the 'mirror' point)")
print("   Always clip on Z before you trust a projection.")
