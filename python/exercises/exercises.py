"""Exercises - implement each function, then run the checker.

    uv run python/exercises/check.py            # scorecard for all exercises
    uv run python/exercises/check.py 3          # just exercise 3
    uv run pytest                               # same thing under pytest

Rules of the game
  * Use NumPy. Do **not** call cv2 -- the point is to write the maths yourself.
  * Do not import camintrinsics either; that would just be copying the answer.
  * Every function has a worked solution in solutions.py. Look only after you
    have tried, and compare approaches rather than just diffing.

Conventions (identical to the rest of the repo)
  * camera looks down +Z, +X right, +Y down
  * ``xy`` means normalized image coordinates, ``uv`` means pixels
  * arrays of points are ``(N, 2)`` or ``(N, 3)``
"""

import numpy as np

TODO = "replace this line with your implementation"


# --------------------------------------------------------------------------
def ex01_build_K(fx, fy, cx, cy, skew=0.0):
    """Return the 3x3 intrinsic matrix as a float64 NumPy array.

    >>> ex01_build_K(800, 800, 320, 240)[0, 2]
    320.0
    """
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex02_project_pinhole(points_cam, K):
    """Project (N, 3) camera-frame points to (N, 2) pixels. No distortion.

    Points with ``Z <= 0`` must come back as ``np.nan`` -- they are behind the
    camera and have no image.
    """
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex03_distort(xy, k1, k2, p1, p2, k3):
    """Apply the OpenCV plumb-bob model to (N, 2) normalized coordinates.

    With ``r^2 = x^2 + y^2``::

        radial = 1 + k1 r^2 + k2 r^4 + k3 r^6
        x' = x*radial + 2 p1 x y + p2 (r^2 + 2 x^2)
        y' = y*radial + p1 (r^2 + 2 y^2) + 2 p2 x y

    Returns an (N, 2) array.
    """
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex04_fov_degrees(K, width, height):
    """Return ``(hfov, vfov)`` in degrees for an ideal pinhole camera.

    Careful: the principal point is generally **not** at the image centre, so
    the horizontal FOV is the sum of two different half-angles, one measured to
    the left edge and one to the right.
    """
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex05_K_after_resize(K, sx, sy):
    """Return the K of the same camera after the image is resized by (sx, sy).

    Think about which entries are lengths in pixels and which are not.
    """
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex06_K_after_crop(K, x0, y0):
    """Return the K after cropping so the new image starts at pixel (x0, y0)."""
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex07_undistort_point(xyd, k1, k2, p1, p2, k3, iters=20):
    """Invert ex03: recover the ideal normalized coordinates from distorted ones.

    There is no closed form. Use fixed-point iteration: start from ``x = xd``,
    then repeatedly remove the tangential part and divide out the radial gain,
    recomputing ``r^2`` from your current estimate each time.
    """
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex08_K_from_hfov(hfov_deg, width, height):
    """Build a centred, square-pixel K from a horizontal field of view."""
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex09_classify_distortion(k1, k2, k3, r=1.0):
    """Return "barrel", "pincushion" or "none" for the radius ``r``.

    Compare the distorted radius against ``r`` itself, with a 1e-9 tolerance.
    Note that the answer can depend on ``r``: a moustache lens is barrel near
    the centre and pincushion at the edge.
    """
    raise NotImplementedError(TODO)


# --------------------------------------------------------------------------
def ex10_pipeline_K(K, crop_x0, crop_y0, scale):
    """The real-world one: a frame is cropped, *then* the crop is resized.

    Given the K calibrated for the full-resolution image, return the K that is
    valid for the final image. Order matters -- work out which operation the
    pixel coordinates meet first.
    """
    raise NotImplementedError(TODO)
