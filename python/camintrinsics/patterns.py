"""Synthetic 2D test images plus the two image warps that D controls.

The images here are *ideal pinhole* images: they are what a perfect, perfectly
undistorted camera with intrinsics ``K_ideal`` would record.  Feeding them
through :func:`distort_image` produces what a real lens would have delivered,
which lets you see distortion without owning the lens.
"""

from __future__ import annotations

import cv2
import numpy as np

from .intrinsics import (make_K, normalized_to_pixel,
                         optimal_new_camera_matrix, pixel_to_normalized,
                         undistort_normalized)

__all__ = ["grid_image", "checkerboard_image", "radial_target", "photo_like",
           "distort_image", "undistort_image", "distortion_field"]

BG = (26, 26, 30)


# --------------------------------------------------------------------------
# Test charts
# --------------------------------------------------------------------------
def grid_image(width: int = 800, height: int = 600, step: int = 40,
               color=(200, 200, 200), accent=(60, 170, 250)) -> np.ndarray:
    """A ruled grid: the reference chart for spotting bent straight lines."""
    img = np.full((height, width, 3), BG, np.uint8)
    for x in range(0, width + 1, step):
        c = accent if (x // step) % 5 == 0 else color
        cv2.line(img, (x, 0), (x, height), c, 1 if c is color else 2, cv2.LINE_AA)
    for y in range(0, height + 1, step):
        c = accent if (y // step) % 5 == 0 else color
        cv2.line(img, (0, y), (width, y), c, 1 if c is color else 2, cv2.LINE_AA)
    cv2.rectangle(img, (1, 1), (width - 2, height - 2), (90, 230, 90), 2)
    return img


def checkerboard_image(width: int = 800, height: int = 600, squares: int = 10,
                       margin: int = 40) -> np.ndarray:
    """A checkerboard plate, the pattern real calibrations are built on."""
    img = np.full((height, width, 3), BG, np.uint8)
    side = min(width - 2 * margin, height - 2 * margin) // squares
    x0 = (width - side * squares) // 2
    y0 = (height - side * squares) // 2
    for r in range(squares):
        for c in range(squares):
            if (r + c) % 2 == 0:
                continue
            cv2.rectangle(img, (x0 + c * side, y0 + r * side),
                          (x0 + (c + 1) * side, y0 + (r + 1) * side),
                          (235, 235, 235), -1)
    cv2.rectangle(img, (x0, y0), (x0 + side * squares, y0 + side * squares),
                  (60, 170, 250), 2)
    return img


def radial_target(width: int = 800, height: int = 600, rings: int = 10,
                  spokes: int = 24) -> np.ndarray:
    """Concentric circles + radial spokes, centred on the image centre.

    Radial distortion moves points *along* the spokes and leaves their angle
    untouched, so this chart separates the radial terms (circles change size)
    from the tangential ones (the whole pattern goes lopsided).
    """
    img = np.full((height, width, 3), BG, np.uint8)
    cx, cy = width / 2.0, height / 2.0
    rmax = np.hypot(cx, cy)
    for i in range(1, rings + 1):
        r = rmax * i / rings
        cv2.circle(img, (int(cx), int(cy)), int(r), (200, 200, 200), 1, cv2.LINE_AA)
    for k in range(spokes):
        a = 2 * np.pi * k / spokes
        cv2.line(img, (int(cx), int(cy)),
                 (int(cx + rmax * np.cos(a)), int(cy + rmax * np.sin(a))),
                 (110, 110, 110), 1, cv2.LINE_AA)
    cv2.drawMarker(img, (int(cx), int(cy)), (60, 60, 235), cv2.MARKER_CROSS, 20, 2)
    return img


def photo_like(width: int = 800, height: int = 600) -> np.ndarray:
    """A cartoon 'street scene': straight edges everywhere, like a real road."""
    img = np.full((height, width, 3), (60, 45, 35), np.uint8)
    cv2.rectangle(img, (0, 0), (width, int(height * 0.45)), (150, 110, 70), -1)
    for i in range(-4, 5):                                   # lane markings
        x = width // 2 + i * width // 9
        cv2.line(img, (width // 2, int(height * 0.45)), (x, height),
                 (230, 230, 230), 3, cv2.LINE_AA)
    for bx, bw, bh in [(40, 120, 0.30), (200, 90, 0.22), (520, 140, 0.34),
                       (690, 90, 0.20)]:                     # buildings
        top = int(height * 0.45 - height * bh)
        cv2.rectangle(img, (bx, top), (bx + bw, int(height * 0.45)), (95, 95, 105), -1)
        cv2.rectangle(img, (bx, top), (bx + bw, int(height * 0.45)), (40, 40, 45), 2)
        for wy in range(top + 12, int(height * 0.45) - 10, 26):
            for wx in range(bx + 10, bx + bw - 14, 24):
                cv2.rectangle(img, (wx, wy), (wx + 12, wy + 14), (70, 190, 230), -1)
    cv2.line(img, (0, int(height * 0.45)), (width, int(height * 0.45)),
             (230, 230, 230), 2, cv2.LINE_AA)
    return img


# --------------------------------------------------------------------------
# The two warps
# --------------------------------------------------------------------------
def distort_image(ideal: np.ndarray, K: np.ndarray, D, K_ideal=None,
                  size=None) -> np.ndarray:
    """Render what a lens with ``(K, D)`` would have recorded.

    For every pixel of the **output** (the real, distorted image) we ask "which
    ideal pixel does this see?":

    1. ``K^-1``  : output pixel  -> *distorted* normalized coords
    2. undistort : -> ideal normalized coords
    3. ``K_ideal``: -> a location in the ideal source image
    4. ``remap`` : sample the source there

    Note that the map is built **backwards**, from destination to source.  Every
    image warp works this way: forward-mapping leaves holes, backward-mapping
    does not.

    Output pixels whose radius lies past the distortion model's fold point are
    left at the border colour: no ideal point maps there, so any value would be
    a fabrication.  ``cv2.undistort`` fabricates one instead, which is where the
    swirling "petals" at the edge of an over-distorted image come from.
    """
    h, w = ideal.shape[:2]
    if K_ideal is None:
        K_ideal = make_K(w / 2.0, w / 2.0, w / 2.0, h / 2.0)
    ow, oh = size if size is not None else (w, h)
    vv, uu = np.mgrid[0:oh, 0:ow].astype(np.float64)
    uv = np.stack([uu.ravel(), vv.ravel()], axis=1)
    xy = undistort_normalized(pixel_to_normalized(uv, K), D)
    src = normalized_to_pixel(xy, K_ideal)
    # undistort_normalized returns NaN where no ideal point maps; -1 sends
    # remap to its border colour instead of fabricating a pixel.
    src[~np.isfinite(src)] = -1.0
    mx = src[:, 0].reshape(oh, ow).astype(np.float32)
    my = src[:, 1].reshape(oh, ow).astype(np.float32)
    return cv2.remap(ideal, mx, my, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=(12, 12, 14))


def undistort_image(distorted: np.ndarray, K: np.ndarray, D, alpha: float = 0.0,
                    size=None):
    """Straighten an image, returning ``(image, K_new, roi)``.

    ``alpha`` is the trade-off knob of :func:`cv2.getOptimalNewCameraMatrix`:

    * ``alpha = 0`` -- zoom in until no invalid (black) pixel remains.  You lose
      field of view.
    * ``alpha = 1`` -- keep every original pixel.  You keep the FOV but gain
      black curved borders, and ``K_new`` has a smaller focal length.

    Either way **K changes**, and using the old K on the undistorted image is a
    silent, very common source of reprojection error.
    """
    h, w = distorted.shape[:2]
    ow, oh = size if size is not None else (w, h)
    K_new, roi = optimal_new_camera_matrix(K, D, w, h, float(alpha))
    # cv2.undistort itself is exact -- it maps destination to source with the
    # *forward* model. Only the K_new above needed replacing.
    out = cv2.undistort(distorted, K, np.asarray(D, float), None, K_new)
    return out, K_new, roi


def distortion_field(K: np.ndarray, D, width: int, height: int, step: int = 40):
    """Per-pixel displacement introduced by D, as ``(points, deltas)`` arrays.

    ``delta = distorted_pixel - ideal_pixel`` at the same normalized ray, i.e.
    "how far did the lens push this point, in pixels".
    """
    vv, uu = np.mgrid[step // 2:height:step, step // 2:width:step]
    uv = np.stack([uu.ravel().astype(float), vv.ravel().astype(float)], axis=1)
    xy = pixel_to_normalized(uv, K)              # treat as ideal rays
    from .intrinsics import distort_normalized
    uv_d = normalized_to_pixel(distort_normalized(xy, D), K)
    return uv, uv_d - uv
