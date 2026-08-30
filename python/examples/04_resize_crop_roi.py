#!/usr/bin/env python3
"""Example 4 - the bookkeeping trap: resizing, cropping and ROIs.

Run:  uv run python/examples/04_resize_crop_roi.py

Calibration gives you a K that belongs to *one specific image size*.  The moment
a preprocessing step resizes, crops, pads or letterboxes the frame, that K is
wrong -- and nothing crashes.  Your reprojection is just quietly off by a few
pixels, which is exactly enough to ruin a 3D detection.

This example does the bookkeeping correctly and proves it numerically: the same
world point must land on the same *physical* spot in every version of the image.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import (crop_K, flip_D, flip_K, make_D,      # noqa: E402
                           make_K, project_points, scale_K, split_K)

W, H = 1920, 1080
K = make_K(fx=1450.0, fy=1452.0, cx=962.4, cy=531.7)
D = make_D(k1=-0.21, k2=0.06, p1=0.0004, p2=-0.0002)
pts = np.array([[0.0, 0.0, 5.0], [1.8, -1.1, 6.5], [-2.4, 0.9, 4.0]])

print(__doc__)
print(f"calibrated at {W}x{H}:  fx {K[0,0]:.1f}  fy {K[1,1]:.1f} "
      f" cx {K[0,2]:.1f}  cy {K[1,2]:.1f}\n")
base = project_points(pts, K, D)
for p, uv in zip(pts, base):
    print(f"   world {p} -> pixel ({uv[0]:8.2f}, {uv[1]:8.2f})")


def report(name, K2, expect, tol=1e-6):
    got = project_points(pts, K2, D)
    err = np.abs(got - expect).max()
    fx, fy, cx, cy, _ = split_K(K2)
    status = "OK  " if err < tol else "FAIL"
    print(f"\n{status} {name}")
    print(f"     K -> fx {fx:8.2f}  fy {fy:8.2f}  cx {cx:8.2f}  cy {cy:8.2f}")
    print(f"     max deviation from the expected physical location: {err:.3e} px")
    return got


# --------------------------------------------------------------------------
print("\n" + "=" * 74)
print("1) resize to half.  Everything in the first two rows of K scales.")
print("=" * 74)
K_half = scale_K(K, 0.5)
report("half resolution, K scaled correctly", K_half, base * 0.5)

print("\n   the wrong way -- scale the focal lengths but forget cx, cy:")
K_bad = K.copy()
K_bad[0, 0] *= 0.5
K_bad[1, 1] *= 0.5
bad = project_points(pts, K_bad, D)
print(f"     error: up to {np.abs(bad - base * 0.5).max():.1f} px at half resolution")
print("     -> a silent, constant offset. It survives every downstream stage,")
print("        and it looks exactly like a small extrinsic calibration error.")

print("\n   non-uniform resize (1920x1080 -> 640x480 without keeping the aspect):")
K_stretch = scale_K(K, 640 / W, 480 / H)
report("letterbox-free stretch", K_stretch, base * [640 / W, 480 / H])
fx, fy, *_ = split_K(K_stretch)
print(f"     fx/fy is now {fx / fy:.3f}: the stretch faked non-square pixels.")

# --------------------------------------------------------------------------
print("\n" + "=" * 74)
print("2) crop.  Focal lengths do NOT change -- the lens did not change.")
print("=" * 74)
x0, y0 = 320, 180
K_crop = crop_K(K, x0, y0)
report(f"crop starting at ({x0}, {y0})", K_crop, base - [x0, y0])
print("     Cropping narrows the field of view without touching fx or fy.")
print("     Digital zoom = crop + resize, so it does change fx: crop then scale.")

K_zoom = scale_K(crop_K(K, x0, y0), W / (W - 2 * x0))
print(f"\n     2x digital zoom back to {W} wide: fx {K_zoom[0,0]:.1f} "
      f"(was {K[0,0]:.1f})")

# --------------------------------------------------------------------------
print("\n" + "=" * 74)
print("3) padding and letterboxing (the classic ML-preprocessing bug)")
print("=" * 74)
pad_l, pad_t = 64, 40
K_pad = crop_K(K, -pad_l, -pad_t)      # padding is a crop with a negative origin
report(f"pad {pad_l} left, {pad_t} top", K_pad, base + [pad_l, pad_t])
print("     A letterbox resize is 'scale, then pad'. Apply scale_K first,")
print("     then crop_K with negative offsets, in that order.")

# --------------------------------------------------------------------------
print("\n" + "=" * 74)
print("4) horizontal flip (data augmentation that changes the camera model)")
print("=" * 74)
K_flip, D_flip = flip_K(K, W, H, horizontal=True), flip_D(D, horizontal=True)
pts_mirrored = pts * [-1.0, 1.0, 1.0]     # mirroring the image mirrors the world
expect = np.stack([(W - 1) - base[:, 0], base[:, 1]], axis=1)
print(f"     cx {K[0,2]:.1f} -> {K_flip[0,2]:.1f}      p2 {D[3]:+.4f} -> "
      f"{D_flip[3]:+.4f}")
err_ok = np.abs(project_points(pts_mirrored, K_flip, D_flip) - expect).max()
err_bad = np.abs(project_points(pts_mirrored, K_flip, D) - expect).max()
print(f"     K and D both mirrored : {err_ok:.3e} px  <- exact")
print(f"     K mirrored, D left as-is: {err_bad:.3e} px  <- silent bias")
print("     The radial terms are even functions, so they survive a mirror. The")
print("     tangential ones are not: a horizontal flip needs p2 -> -p2.")
print("     Mirroring also flips handedness: if you flip images for augmentation")
print("     you must mirror the extrinsics too, or your poses become left-handed.")

# --------------------------------------------------------------------------
print("\n" + "=" * 74)
print("5) what does NOT change: D")
print("=" * 74)
print("     D lives in normalized coordinates, before K is applied, so resizing")
print("     and cropping leave every coefficient untouched. If someone hands you")
print("     'the D for 640x480', they are confused -- D has no resolution.")
print("\n     (Undistorting, however, DOES change K -- see example 05.)")
