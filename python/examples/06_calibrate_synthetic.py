#!/usr/bin/env python3
"""Example 6 - calibrate a camera whose true K and D you already know.

Run:  uv run python/examples/06_calibrate_synthetic.py

Real calibration has no ground truth: you get numbers and a reprojection error,
and you have to decide whether to trust them.  Here we synthesise the whole
experiment, so we can measure how far the recovered K and D actually are from
the truth -- and watch that error change with noise, with the number of views,
and with how the boards were held.

The last section is the important one: it reproduces the single most common
real-world calibration mistake and shows that the reprojection error does *not*
warn you about it.
"""

import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import make_D, make_K                          # noqa: E402

W, H = 1280, 960
K_TRUE = make_K(fx=980.0, fy=982.0, cx=641.5, cy=478.2)
D_TRUE = make_D(k1=-0.24, k2=0.08, p1=0.0006, p2=-0.0004, k3=0.0)
ROWS, COLS, SQUARE = 6, 9, 0.025          # inner corners, 25 mm squares

OBJP = np.zeros((ROWS * COLS, 3), np.float32)
OBJP[:, :2] = np.mgrid[0:COLS, 0:ROWS].T.reshape(-1, 2) * SQUARE


def make_views(n_views, rng, tilt_deg=32.0, z_range=(0.35, 0.75), noise_px=0.0):
    """Synthesise ``n_views`` observations of the board through the true camera."""
    obj_pts, img_pts = [], []
    for _ in range(n_views):
        rvec = rng.uniform(-1, 1, 3)
        rvec = rvec / np.linalg.norm(rvec) * np.radians(rng.uniform(0, tilt_deg))
        z = rng.uniform(*z_range)
        centre = np.array([rng.uniform(-0.06, 0.06), rng.uniform(-0.05, 0.05), z])
        tvec = centre - cv2.Rodrigues(rvec)[0] @ (OBJP.mean(0).astype(np.float64))
        uv = cv2.projectPoints(OBJP, rvec, tvec, K_TRUE, D_TRUE)[0].reshape(-1, 2)
        if not (uv.min() > 8 and uv[:, 0].max() < W - 8 and uv[:, 1].max() < H - 8):
            continue                       # the board must be fully visible
        if noise_px:
            uv = uv + rng.normal(0, noise_px, uv.shape)
        obj_pts.append(OBJP.copy())
        img_pts.append(uv.astype(np.float32).reshape(-1, 1, 2))
    return obj_pts, img_pts


def run(obj_pts, img_pts, flags=0):
    rms, K, D, _, _ = cv2.calibrateCamera(obj_pts, img_pts, (W, H), None, None,
                                          flags=flags)
    return rms, K, np.asarray(D).ravel()


def report(name, rms, K, D, n):
    dfx = K[0, 0] - K_TRUE[0, 0]
    dfy = K[1, 1] - K_TRUE[1, 1]
    dcx = K[0, 2] - K_TRUE[0, 2]
    dcy = K[1, 2] - K_TRUE[1, 2]
    dk1 = D[0] - D_TRUE[0]
    print(f"{name:34s} {n:4d} {rms:8.4f} {dfx:+9.2f} {dfy:+9.2f} "
          f"{dcx:+8.2f} {dcy:+8.2f} {dk1:+9.4f}")


HEADER = (f"{'experiment':34s} {'#img':>4s} {'RMS px':>8s} {'d fx':>9s} "
          f"{'d fy':>9s} {'d cx':>8s} {'d cy':>8s} {'d k1':>9s}")

print(__doc__)
print("ground truth")
print(f"  fx {K_TRUE[0,0]:.1f}  fy {K_TRUE[1,1]:.1f}  cx {K_TRUE[0,2]:.1f} "
      f" cy {K_TRUE[1,2]:.1f}")
print(f"  D  {np.array2string(D_TRUE, precision=4)}\n")
print("errors below are (recovered - true)\n")
print(HEADER)
print("-" * len(HEADER))

# 1. how many views do you need? ------------------------------------------
for n in (3, 6, 12, 25):
    rng = np.random.RandomState(7)
    o, i = make_views(n, rng, noise_px=0.25)
    rms, K, D = run(o, i)
    report(f"{n} well-tilted views, 0.25 px noise", rms, K, D, len(o))

print()
# 2. how much does corner noise hurt? -------------------------------------
for noise in (0.0, 0.1, 0.5, 1.5):
    rng = np.random.RandomState(11)
    o, i = make_views(20, rng, noise_px=noise)
    rms, K, D = run(o, i)
    report(f"20 views, corner noise {noise:.2f} px", rms, K, D, len(o))

print()
# 3. the mistake everybody makes ------------------------------------------
rng = np.random.RandomState(3)
o, i = make_views(20, rng, tilt_deg=2.0, z_range=(0.5, 0.55), noise_px=0.25)
rms_flat, K_flat, D_flat = run(o, i)
report("20 views, board held FLAT", rms_flat, K_flat, D_flat, len(o))

rng = np.random.RandomState(3)
o, i = make_views(20, rng, tilt_deg=35.0, z_range=(0.3, 0.8), noise_px=0.25)
rms_tilt, K_tilt, D_tilt = run(o, i)
report("20 views, board TILTED + depth", rms_tilt, K_tilt, D_tilt, len(o))

print("\n" + "=" * 78)
print("what to take away")
print("=" * 78)
print(f"  The flat-board run reports RMS = {rms_flat:.3f} px, which looks excellent,")
print(f"  yet fx is off by {K_flat[0,0] - K_TRUE[0,0]:+.1f} px -- "
      f"{100 * abs(K_flat[0,0] / K_TRUE[0,0] - 1):.0f}% wrong. Note that")
print("  cx and cy came out fine: the degeneracy is specifically between focal")
print("  length and board distance, which a flat board cannot separate.")
print(f"  The tilted run reports a similar RMS = {rms_tilt:.3f} px and recovers fx to")
print(f"  {K_tilt[0,0] - K_TRUE[0,0]:+.1f} px.")
print()
print("  Reprojection error measures how well the model fits the data you gave it.")
print("  It cannot tell you the data was uninformative. With every board at the")
print("  same distance and angle, focal length and board distance trade off against")
print("  each other almost perfectly -- the optimiser is free to pick the wrong")
print("  combination and still fit every corner.")
print()
print("  Practical checklist")
print("    * tilt the board 30-45 deg in several directions, not just flat on")
print("    * vary the distance so the board fills 1/3 to 3/4 of the frame")
print("    * push the board into all four corners: that is where D is measured")
print("    * 15-25 good views beats 60 sloppy ones")
print("    * fix k3 (cv2.CALIB_FIX_K3) unless the lens is genuinely very wide")
print("    * sanity-check the result: cx, cy within a few % of the image centre,")
print("      fx/fy within ~1%, and fx consistent with the lens spec and sensor")

# 4. fixing k3 on a lens that does not need it ----------------------------
print("\n" + "=" * 78)
print("bonus: is a free k3 worth it on a lens whose true k3 is 0?")
print("=" * 78)
print("One calibration cannot answer that -- the difference hides in the spread")
print("across repeats. So run the same experiment 12 times with different noise")
print("and different board poses, and look at the variance.\n")

hdr = (f"{'':14s} {'RMS px':>8s} {'fx bias':>9s} {'fx std':>8s} "
       f"{'k1 bias':>9s} {'k1 std':>8s} {'k3 mean':>9s} {'k3 std':>8s}")
print(hdr)
print("-" * len(hdr))
for flags, label in ((0, "free k1,k2,k3"), (cv2.CALIB_FIX_K3, "k3 fixed to 0")):
    fxs, k1s, k3s, rmss = [], [], [], []
    for seed in range(12):
        rng = np.random.RandomState(seed)
        o, i = make_views(15, rng, noise_px=0.4)
        rms, K, D = run(o, i, flags=flags)
        rmss.append(rms)
        fxs.append(K[0, 0])
        k1s.append(D[0])
        k3s.append(D[4])
    print(f"{label:14s} {np.mean(rmss):8.4f} {np.mean(fxs) - K_TRUE[0,0]:+9.2f} "
          f"{np.std(fxs):8.2f} {np.mean(k1s) - D_TRUE[0]:+9.4f} {np.std(k1s):8.4f} "
          f"{np.mean(k3s):+9.4f} {np.std(k3s):8.4f}")

print("\n  The reprojection error is identical to four decimals, and fx barely")
print("  cares. But look at k3: its true value is 0, and the free fit scatters it")
print("  across a range far wider than the coefficient itself -- it is almost")
print("  unconstrained by this data. That noise does not stay contained: it leaks")
print("  into k1, whose spread grows by roughly 60%.")
print()
print("  So the argument for CALIB_FIX_K3 is not 'lower error'. It is that an")
print("  unconstrained parameter buys you nothing and destabilises the ones you")
print("  actually use. Add k3 only when the lens is wide enough to need it, and")
print("  check that it comes out repeatable across recalibrations.")
