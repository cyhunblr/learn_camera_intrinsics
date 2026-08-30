"""Worked solutions. Read these *after* you have attempted exercises.py.

Each one is written the way you would actually write it in production code,
not golfed -- clarity is the point.
"""

import numpy as np


def ex01_build_K(fx, fy, cx, cy, skew=0.0):
    return np.array([[fx, skew, cx],
                     [0.0, fy, cy],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def ex02_project_pinhole(points_cam, K):
    P = np.atleast_2d(np.asarray(points_cam, dtype=np.float64))
    z = P[:, 2]
    valid = z > 0
    uv = np.full((P.shape[0], 2), np.nan)
    # The perspective divide, then K. Note we never form K^-1 or a 4x4 matrix:
    # the two-line closed form is clearer and faster.
    x = P[valid, 0] / z[valid]
    y = P[valid, 1] / z[valid]
    uv[valid, 0] = K[0, 0] * x + K[0, 1] * y + K[0, 2]
    uv[valid, 1] = K[1, 1] * y + K[1, 2]
    return uv


def ex03_distort(xy, k1, k2, p1, p2, k3):
    xy = np.atleast_2d(np.asarray(xy, dtype=np.float64))
    x, y = xy[:, 0], xy[:, 1]
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([xd, yd], axis=1)


def ex04_fov_degrees(K, width, height):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    # Two half-angles per axis, because cx is not necessarily width/2.
    hfov = np.degrees(np.arctan2(cx, fx) + np.arctan2(width - cx, fx))
    vfov = np.degrees(np.arctan2(cy, fy) + np.arctan2(height - cy, fy))
    return float(hfov), float(vfov)


def ex05_K_after_resize(K, sx, sy):
    out = np.asarray(K, dtype=np.float64).copy()
    out[0, :] *= sx        # fx, skew and cx are all horizontal pixel lengths
    out[1, :] *= sy        # fy and cy are vertical pixel lengths
    return out


def ex06_K_after_crop(K, x0, y0):
    out = np.asarray(K, dtype=np.float64).copy()
    out[0, 2] -= x0        # only the origin moved; the lens is unchanged
    out[1, 2] -= y0
    return out


def ex07_undistort_point(xyd, k1, k2, p1, p2, k3, iters=20):
    xyd = np.atleast_2d(np.asarray(xyd, dtype=np.float64))
    x0, y0 = xyd[:, 0], xyd[:, 1]
    x, y = x0.copy(), y0.copy()
    for _ in range(iters):
        r2 = x * x + y * y
        radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
        dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        x = (x0 - dx) / radial
        y = (y0 - dy) / radial
    return np.stack([x, y], axis=1)


def ex08_K_from_hfov(hfov_deg, width, height):
    fx = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
    return ex01_build_K(fx, fx, width / 2.0, height / 2.0)


def ex09_classify_distortion(k1, k2, k3, r=1.0):
    r2 = r * r
    rd = r * (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3)
    if rd < r - 1e-9:
        return "barrel"
    if rd > r + 1e-9:
        return "pincushion"
    return "none"


def ex10_pipeline_K(K, crop_x0, crop_y0, scale):
    # A pixel meets the crop first (its coordinates shift), then the resize
    # (everything scales). So: crop, then scale -- not the other way round.
    return ex05_K_after_resize(ex06_K_after_crop(K, crop_x0, crop_y0),
                               scale, scale)
