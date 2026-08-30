"""Camera intrinsics from scratch: the K matrix and the D vector.

Everything in this module is written with plain NumPy so that you can read the
maths instead of trusting a black box.  Every function has an OpenCV twin that
is used in the tests to prove the implementations agree.

Coordinate conventions used throughout the repository
-----------------------------------------------------
* World and camera frames are right-handed.
* The camera looks down its **+Z** axis, **+X** points right, **+Y** points
  down.  This is the OpenCV convention (*not* the OpenGL one, where the camera
  looks down -Z).
* Image coordinates ``(u, v)`` are in pixels, ``u`` to the right, ``v`` down,
  with ``(0, 0)`` at the centre of the top-left pixel.
"""

from __future__ import annotations

import numpy as np

__all__ = [
    "make_K",
    "make_D",
    "split_K",
    "K_from_fov",
    "fov_deg",
    "scale_K",
    "crop_K",
    "flip_K",
    "flip_D",
    "pixel_to_normalized",
    "normalized_to_pixel",
    "distort_normalized",
    "undistort_normalized",
    "project_points",
    "backproject_pixels",
    "distortion_profile",
    "max_valid_radius",
    "max_distorted_radius",
    "is_invertible_over_image",
    "optimal_new_camera_matrix",
]


# --------------------------------------------------------------------------
# 1. Building and inspecting K
# --------------------------------------------------------------------------
def make_K(fx: float, fy: float, cx: float, cy: float, skew: float = 0.0) -> np.ndarray:
    """Assemble the 3x3 intrinsic matrix.

    ::

            | fx  s  cx |
        K = |  0 fy  cy |
            |  0  0   1 |

    ``fx``/``fy`` are focal lengths **in pixels**, ``cx``/``cy`` the principal
    point in pixels, ``s`` the skew (0 for every sane modern sensor).
    """
    return np.array(
        [[fx, skew, cx],
         [0.0, fy, cy],
         [0.0, 0.0, 1.0]], dtype=np.float64
    )


def make_D(k1: float = 0.0, k2: float = 0.0, p1: float = 0.0,
           p2: float = 0.0, k3: float = 0.0, k4: float = 0.0,
           k5: float = 0.0, k6: float = 0.0) -> np.ndarray:
    """Assemble the OpenCV distortion vector ``[k1, k2, p1, p2, k3, ...]``.

    Note the deliberately confusing OpenCV ordering: the two **tangential**
    terms ``p1, p2`` sit *between* the radial terms ``k2`` and ``k3``.  Getting
    this order wrong is the single most common camera-calibration bug.

    Returns a length-5 vector when the rational terms ``k4..k6`` are all zero,
    otherwise a length-8 vector.
    """
    if k4 == 0.0 and k5 == 0.0 and k6 == 0.0:
        return np.array([k1, k2, p1, p2, k3], dtype=np.float64)
    return np.array([k1, k2, p1, p2, k3, k4, k5, k6], dtype=np.float64)


def split_K(K: np.ndarray) -> tuple[float, float, float, float, float]:
    """Return ``(fx, fy, cx, cy, skew)`` from a 3x3 intrinsic matrix."""
    K = np.asarray(K, dtype=np.float64)
    return float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]), float(K[0, 1])


def _pad_D(D) -> np.ndarray:
    """Pad any accepted distortion vector to a canonical length-8 array."""
    if D is None:
        return np.zeros(8, dtype=np.float64)
    d = np.asarray(D, dtype=np.float64).ravel()
    if d.size > 8:  # 12/14-coefficient models (thin prism, tilt) are out of scope
        d = d[:8]
    out = np.zeros(8, dtype=np.float64)
    out[: d.size] = d
    return out


# --------------------------------------------------------------------------
# 2. Field of view: the intuition behind fx and fy
# --------------------------------------------------------------------------
def fov_deg(K: np.ndarray, width: int, height: int) -> tuple[float, float, float]:
    """Return ``(horizontal, vertical, diagonal)`` field of view in degrees.

    This is the *ideal pinhole* FOV computed from the image borders; it ignores
    distortion.  Note that the principal point is usually **not** exactly in the
    centre, so the FOV is the sum of two asymmetric half-angles.
    """
    fx, fy, cx, cy, _ = split_K(K)
    left, right = np.arctan2(cx, fx), np.arctan2(width - cx, fx)
    top, bottom = np.arctan2(cy, fy), np.arctan2(height - cy, fy)
    hfov = np.degrees(left + right)
    vfov = np.degrees(top + bottom)
    # Diagonal from the corner-most normalized radius.
    corners = np.array([[-cx / fx, -cy / fy],
                        [(width - cx) / fx, -cy / fy],
                        [-cx / fx, (height - cy) / fy],
                        [(width - cx) / fx, (height - cy) / fy]])
    dfov = 2.0 * np.degrees(np.arctan(np.max(np.linalg.norm(corners, axis=1))))
    return float(hfov), float(vfov), float(dfov)


def K_from_fov(hfov_deg_: float, width: int, height: int,
               square_pixels: bool = True) -> np.ndarray:
    """Build a centred K from a horizontal field of view.

    Handy when you have no calibration but know the lens spec, e.g. "90 deg
    horizontal FOV on a 1920x1080 sensor".
    """
    fx = (width / 2.0) / np.tan(np.radians(hfov_deg_) / 2.0)
    fy = fx if square_pixels else (height / 2.0) / np.tan(np.radians(hfov_deg_) / 2.0)
    return make_K(fx, fy, width / 2.0, height / 2.0)


# --------------------------------------------------------------------------
# 3. What happens to K when you resize, crop or flip the image
# --------------------------------------------------------------------------
def scale_K(K: np.ndarray, sx: float, sy: float | None = None) -> np.ndarray:
    """K for the same camera after resizing the image by ``(sx, sy)``.

    **Every** entry of the first two rows scales -- including ``cx``/``cy`` and
    the skew.  Forgetting to scale the principal point is the classic "my
    projection is off by a few pixels after downscaling" bug.
    """
    sy = sx if sy is None else sy
    K = np.asarray(K, dtype=np.float64).copy()
    K[0, :] *= sx
    K[1, :] *= sy
    return K


def crop_K(K: np.ndarray, x0: float, y0: float) -> np.ndarray:
    """K after cropping with the new image origin at ``(x0, y0)``.

    Cropping moves the principal point but leaves the focal lengths alone --
    the lens did not change, only which part of its image circle you keep.
    """
    K = np.asarray(K, dtype=np.float64).copy()
    K[0, 2] -= x0
    K[1, 2] -= y0
    return K


def flip_K(K: np.ndarray, width: int, height: int,
           horizontal: bool = True, vertical: bool = False) -> np.ndarray:
    """K after mirroring the image. Mirroring reflects the principal point.

    Pair this with :func:`flip_D`: a mirror is only an exact operation on the
    camera model if the tangential coefficients are mirrored too.
    """
    K = np.asarray(K, dtype=np.float64).copy()
    if horizontal:
        K[0, 2] = (width - 1) - K[0, 2]
        K[0, 1] = -K[0, 1]
    if vertical:
        K[1, 2] = (height - 1) - K[1, 2]
    return K


def flip_D(D, horizontal: bool = True, vertical: bool = False) -> np.ndarray:
    """D after mirroring the image.

    The radial terms are even in ``x`` and ``y`` and survive a mirror untouched.
    The tangential ones do not: substituting ``x -> -x`` into the model shows
    that the mirrored lens is described by ``p2 -> -p2`` (and ``p1 -> -p1`` for a
    vertical mirror).  Flip the image without flipping D and you keep a small,
    systematic error that no amount of re-calibration downstream will explain.
    """
    d = _pad_D(D).copy()
    if horizontal:
        d[3] = -d[3]        # p2
    if vertical:
        d[2] = -d[2]        # p1
    return d[:5] if not np.any(d[5:]) else d


# --------------------------------------------------------------------------
# 4. K as a coordinate change: pixels <-> normalized image coordinates
# --------------------------------------------------------------------------
def normalized_to_pixel(xy: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Apply K: normalized image coordinates -> pixels.  ``xy`` is (N, 2)."""
    xy = np.atleast_2d(np.asarray(xy, dtype=np.float64))
    fx, fy, cx, cy, s = split_K(K)
    u = fx * xy[:, 0] + s * xy[:, 1] + cx
    v = fy * xy[:, 1] + cy
    return np.stack([u, v], axis=1)


def pixel_to_normalized(uv: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Apply K^-1: pixels -> normalized image coordinates.  ``uv`` is (N, 2).

    The closed form of the inverse (cheaper and clearer than ``inv(K)``)::

        y = (v - cy) / fy
        x = (u - cx - s*y) / fx
    """
    uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
    fx, fy, cx, cy, s = split_K(K)
    y = (uv[:, 1] - cy) / fy
    x = (uv[:, 0] - cx - s * y) / fx
    return np.stack([x, y], axis=1)


# --------------------------------------------------------------------------
# 5. The D vector: forward (distort) and inverse (undistort) lens model
# --------------------------------------------------------------------------
def distort_normalized(xy: np.ndarray, D) -> np.ndarray:
    """Apply the OpenCV plumb-bob / Brown-Conrady model in normalized space.

    With ``r^2 = x^2 + y^2``::

        radial = (1 + k1 r^2 + k2 r^4 + k3 r^6) / (1 + k4 r^2 + k5 r^4 + k6 r^6)
        x' = x * radial + 2 p1 x y + p2 (r^2 + 2 x^2)
        y' = y * radial + p1 (r^2 + 2 y^2) + 2 p2 x y

    Distortion happens **before** K is applied, which is why it lives in
    normalized coordinates and is completely independent of image resolution.
    """
    xy = np.atleast_2d(np.asarray(xy, dtype=np.float64))
    k1, k2, p1, p2, k3, k4, k5, k6 = _pad_D(D)
    x, y = xy[:, 0], xy[:, 1]
    r2 = x * x + y * y
    r4, r6 = r2 * r2, r2 * r2 * r2
    radial = (1 + k1 * r2 + k2 * r4 + k3 * r6) / (1 + k4 * r2 + k5 * r4 + k6 * r6)
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.stack([xd, yd], axis=1)


def undistort_normalized(xyd: np.ndarray, D, iters: int = 30,
                         tol: float = 1e-12) -> np.ndarray:
    """Invert :func:`distort_normalized`, or return NaN if it cannot be done.

    The distortion model has no closed-form inverse, so it has to be solved
    numerically.  OpenCV uses a *fixed-point* iteration: subtract the tangential
    part, divide out the radial gain, repeat.  It is simple, it is what
    exercise 7 asks you to write, and it is worth understanding.

    It is also fragile.  Near the corners of a strong wide-angle lens it stops
    contracting -- for ``k1 = -0.34, k2 = 0.11`` at the corner of a 640x480
    frame it oscillates between r = 1.44 and r = 1.56 forever, and twenty
    iterations leave a residual of 8e-2 in normalized units, roughly 26 px.

    So this function uses Newton's method instead: finding ``(x, y)`` with
    ``distort(x, y) = (xd, yd)`` is a 2x2 root find with an analytic Jacobian,
    and it converges quadratically.

    Newton is fast but it is not magic, and two things can still go wrong.  A
    point past the model's fold radius has **no** solution at all, and a Newton
    step can happily walk to the polynomial's *other* root -- for ``k1 = -0.5``,
    asking to undistort ``r' = 0.82`` returns ``x = -1.72``, which satisfies the
    equation exactly and is physically meaningless.  Both cases are therefore
    rejected up front (by radius) and afterwards (by residual), and rejected
    points come back as **NaN**, the same convention :func:`project_points`
    uses for points behind the camera.

    Never let a solver hand back a number it does not believe in; that is the
    failure this whole chapter is about.
    """
    xyd = np.atleast_2d(np.asarray(xyd, dtype=np.float64))
    out = np.full(xyd.shape, np.nan)
    if xyd.shape[0] == 0:
        return out

    k1, k2, p1, p2, k3, k4, k5, k6 = _pad_D(D)
    rational = bool(k4 or k5 or k6)

    # No r maps to a radius past the fold, so do not pretend otherwise.
    solvable = np.hypot(xyd[:, 0], xyd[:, 1]) <= max_distorted_radius(D)
    if not np.any(solvable):
        return out
    tx, ty = xyd[solvable, 0], xyd[solvable, 1]
    x, y = tx.copy(), ty.copy()

    for _ in range(iters):
        r2 = x * x + y * y
        r4, r6 = r2 * r2, r2 * r2 * r2
        num = 1 + k1 * r2 + k2 * r4 + k3 * r6
        dnum = k1 + 2 * k2 * r2 + 3 * k3 * r4
        if rational:
            den = 1 + k4 * r2 + k5 * r4 + k6 * r6
            dden = k4 + 2 * k5 * r2 + 3 * k6 * r4
            radial = num / den
            drad = (dnum * den - num * dden) / (den * den)   # quotient rule
        else:
            radial, drad = num, dnum

        fx = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        fy = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y

        j00 = radial + 2 * x * x * drad + 2 * p1 * y + 6 * p2 * x
        j01 = 2 * x * y * drad + 2 * p1 * x + 2 * p2 * y
        j11 = radial + 2 * y * y * drad + 6 * p1 * y + 2 * p2 * x

        rx, ry = tx - fx, ty - fy
        det = j00 * j11 - j01 * j01          # the Jacobian is symmetric here
        # A near-singular Jacobian means Newton has no usable step. Take none
        # rather than a clamped one -- clamping |det| would keep the magnitude
        # at ~1/eps and, for a negative det, point the step the wrong way.
        good = np.abs(det) > 1e-12
        dx = np.where(good, (j11 * rx - j01 * ry) / np.where(good, det, 1.0), 0.0)
        dy = np.where(good, (-j01 * rx + j00 * ry) / np.where(good, det, 1.0), 0.0)
        x += dx
        y += dy
        if np.all(np.abs(dx) < tol) and np.all(np.abs(dy) < tol):
            break

    # Believe the answer only if it actually solves the equation we were given.
    back = distort_normalized(np.stack([x, y], axis=1), D)
    converged = np.hypot(back[:, 0] - tx, back[:, 1] - ty) <= 1e-9
    idx = np.flatnonzero(solvable)[converged]
    out[idx, 0] = x[converged]
    out[idx, 1] = y[converged]
    return out


def distortion_profile(D, r_max: float = 1.0, n: int = 256) -> tuple[np.ndarray, np.ndarray]:
    """Radial-only distortion curve: returns ``(r, r_distorted)``.

    Plotting ``r'`` against ``r`` is the fastest way to read a D vector:
    ``r' < r`` is barrel distortion (image pulled inward, straight lines bow
    outward), ``r' > r`` is pincushion.
    """
    r = np.linspace(0.0, r_max, n)
    pts = np.stack([r, np.zeros_like(r)], axis=1)
    rd = distort_normalized(pts, D)[:, 0]
    return r, rd


def max_valid_radius(D, r_max: float = 3.0, n: int = 2000) -> float:
    """Largest normalized radius before the radial model stops being monotonic.

    Beyond this radius the model folds back on itself, undistortion becomes
    ambiguous and remapped images grow the smeared "petals" you sometimes see
    at the edge of an over-extrapolated undistort.
    """
    r, rd = distortion_profile(D, r_max=r_max, n=n)
    drop = np.nonzero(np.diff(rd) <= 0)[0]
    return float(r_max if drop.size == 0 else r[drop[0]])


def max_distorted_radius(D, r_max: float = 3.0, n: int = 2000) -> float:
    """The largest ``r'`` the radial model can ever produce, i.e. ``r'(r_fold)``.

    Any image point whose distorted radius exceeds this value **cannot be
    undistorted** -- no ``r`` maps to it.  ``cv2.undistort`` will not warn you;
    it will quietly return garbage, which is how a plausible-looking D vector
    produces a black or shredded undistorted image.

    If the model never folds within ``r_max`` the value at the search cap is
    returned, i.e. "no practical limit".
    """
    r_fold = max_valid_radius(D, r_max=r_max, n=n)
    return float(distort_normalized([[r_fold, 0.0]], D)[0, 0])


def is_invertible_over_image(K: np.ndarray, D, width: int, height: int) -> bool:
    """Can every pixel of a ``width x height`` image be undistorted?

    Checks the four corners -- they always carry the largest radius -- against
    :func:`max_distorted_radius`.
    """
    corners = np.array([[0.0, 0.0], [width, 0.0], [0.0, height], [width, height]])
    r = np.linalg.norm(pixel_to_normalized(corners, K), axis=1).max()
    return bool(r <= max_distorted_radius(D))


def optimal_new_camera_matrix(K: np.ndarray, D, width: int, height: int,
                              alpha: float = 0.0):
    """The K of the undistorted image, and the region of it holding real data.

    Same algorithm as ``cv2.getOptimalNewCameraMatrix``: sample a 9x9 grid over
    the image, undistort it, and take the largest rectangle that fits *inside*
    the result (``alpha = 0``, no invalid pixels) or the smallest that *contains*
    it (``alpha = 1``, no lost pixels), interpolating between the two.

    Written out here rather than called because OpenCV's version undistorts that
    grid with its default five fixed-point iterations, which do not converge on
    a strong lens.  For the repository's own "action cam" preset at
    ``alpha = 0`` that costs it 13% in ``fx`` -- see
    ``docs/course/05_undistortion.md``.  This uses :func:`undistort_normalized`, so the
    figures, the C++ half and the web viewer all agree.

    Returns ``(K_new, (x, y, w, h))``.
    """
    n = 9
    us = np.linspace(0.0, width - 1.0, n)
    vs = np.linspace(0.0, height - 1.0, n)
    grid = np.array([[u, v] for v in vs for u in us])
    xy = undistort_normalized(pixel_to_normalized(grid, K), D).reshape(n, n, 2)
    if not np.all(np.isfinite(xy)):
        raise ValueError(
            "this lens cannot be undistorted over the whole image: some pixels "
            "lie past the model's fold radius (see max_distorted_radius)")

    outer = (xy[:, :, 0].min(), xy[:, :, 1].min(),
             xy[:, :, 0].max(), xy[:, :, 1].max())
    inner = (xy[:, 0, 0].max(), xy[0, :, 1].max(),
             xy[:, -1, 0].min(), xy[-1, :, 1].min())
    x0, y0, x1, y1 = (i * (1 - alpha) + o * alpha for i, o in zip(inner, outer))

    fx = (width - 1) / max(x1 - x0, 1e-9)
    fy = (height - 1) / max(y1 - y0, 1e-9)
    K_new = make_K(fx, fy, -fx * x0, -fy * y0)

    # The valid region is the inner rectangle expressed in the new pixel grid.
    rx0 = max(0, int(np.ceil(fx * inner[0] + K_new[0, 2])))
    ry0 = max(0, int(np.ceil(fy * inner[1] + K_new[1, 2])))
    rx1 = min(width, int(np.floor(fx * inner[2] + K_new[0, 2])))
    ry1 = min(height, int(np.floor(fy * inner[3] + K_new[1, 2])))
    return K_new, (rx0, ry0, max(0, rx1 - rx0), max(0, ry1 - ry0))


# --------------------------------------------------------------------------
# 6. Full forward model: 3D camera-frame point -> pixel
# --------------------------------------------------------------------------
def project_points(points_cam: np.ndarray, K: np.ndarray, D=None,
                   return_mask: bool = False):
    """Project 3D points **given in the camera frame** to pixels.

    The complete pipeline, in the order the light travels::

        (X, Y, Z)  --perspective divide-->  (x, y) = (X/Z, Y/Z)
                   --distortion D-------->  (x', y')
                   --intrinsics K-------->  (u, v)

    Points with ``Z <= 0`` are behind the camera; their projection is
    meaningless and is returned as NaN (set ``return_mask=True`` to get the
    validity mask alongside).
    """
    P = np.atleast_2d(np.asarray(points_cam, dtype=np.float64))
    z = P[:, 2]
    valid = z > 1e-9
    xy = np.full((P.shape[0], 2), np.nan)
    xy[valid] = P[valid, :2] / z[valid, None]
    xy[valid] = distort_normalized(xy[valid], D)
    uv = np.full((P.shape[0], 2), np.nan)
    uv[valid] = normalized_to_pixel(xy[valid], K)
    return (uv, valid) if return_mask else uv


def backproject_pixels(uv: np.ndarray, K: np.ndarray, D=None,
                       depth: float | np.ndarray = 1.0) -> np.ndarray:
    """Inverse of :func:`project_points`: pixel -> 3D ray point at ``depth``.

    A single pixel does not determine a 3D point, only the ray it lies on.  You
    need one extra piece of information (a depth, a plane, a second view) to
    pin the point down -- this is the whole reason monocular 3D is hard.
    """
    xyd = pixel_to_normalized(uv, K)
    xy = undistort_normalized(xyd, D)
    d = np.asarray(depth, dtype=np.float64).reshape(-1, 1) if np.ndim(depth) else float(depth)
    ones = np.ones((xy.shape[0], 1))
    return np.hstack([xy, ones]) * d
