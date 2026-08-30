"""Small OpenCV-drawn diagnostic plots -- no matplotlib, so they can live
inside a live interactive window at 60 fps."""

from __future__ import annotations

import cv2
import numpy as np

from .intrinsics import distortion_profile, split_K
from .patterns import distortion_field
from .scene import COLORS

__all__ = ["plot_distortion_profile", "plot_distortion_field"]


def plot_distortion_profile(D, width: int = 380, height: int = 260,
                            r_max: float = 1.2) -> np.ndarray:
    """Plot ``r' vs r``: the whole radial model on one curve.

    The grey diagonal is "no distortion".  Curve below it -> barrel (points
    pulled toward the centre); above -> pincushion.  Where the curve stops
    rising, the model has folded over and undistortion is no longer unique.
    """
    img = np.full((height, width, 3), (24, 24, 28), np.uint8)
    m = 38
    x0, y0, x1, y1 = m, height - m, width - 12, 12
    r, rd = distortion_profile(D, r_max=r_max, n=400)
    y_max = max(r_max, float(np.nanmax(rd)) * 1.05, 1e-6)

    def to_px(rx, ry):
        return (int(x0 + (x1 - x0) * rx / r_max),
                int(y0 + (y1 - y0) * ry / y_max))

    for g in np.linspace(0, r_max, 7):                     # grid
        cv2.line(img, to_px(g, 0), to_px(g, y_max), (44, 44, 48), 1)
    for g in np.linspace(0, y_max, 7):
        cv2.line(img, to_px(0, g), to_px(r_max, g), (44, 44, 48), 1)
    cv2.line(img, to_px(0, 0), to_px(r_max, min(r_max, y_max)),
             (110, 110, 110), 1, cv2.LINE_AA)               # identity
    pts = np.array([to_px(a, b) for a, b in zip(r, rd)], np.int32)
    cv2.polylines(img, [pts], False, COLORS["orange"], 2, cv2.LINE_AA)

    cv2.line(img, (x0, y0), (x1, y0), (150, 150, 150), 1)
    cv2.line(img, (x0, y0), (x0, y1), (150, 150, 150), 1)
    cv2.putText(img, "r (normalized)", (x0 + 4, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (170, 170, 170), 1, cv2.LINE_AA)
    cv2.putText(img, "r'", (8, y1 + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (170, 170, 170), 1, cv2.LINE_AA)
    tag = "barrel" if rd[-1] < r[-1] else ("pincushion" if rd[-1] > r[-1] else "none")
    cv2.putText(img, f"radial: {tag}", (x0 + 4, y1 + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLORS["orange"], 1, cv2.LINE_AA)
    return img


def plot_distortion_field(K, D, width: int, height: int, step: int = 46,
                          gain: float = 1.0, canvas=None) -> np.ndarray:
    """Arrows showing where D pushes each pixel, with a colour-coded magnitude.

    Long arrows near the corners and none at the centre is the signature of pure
    radial distortion; a field that does not vanish at the principal point means
    tangential terms (``p1``, ``p2``) are in play.
    """
    img = (np.full((height, width, 3), (24, 24, 28), np.uint8)
           if canvas is None else canvas)
    uv, delta = distortion_field(K, D, width, height, step)
    mag = np.linalg.norm(delta, axis=1)
    mmax = max(float(mag.max()), 1e-6)
    for (u, v), (du, dv), m in zip(uv, delta, mag):
        c = cv2.applyColorMap(np.uint8([[255 * m / mmax]]),
                              cv2.COLORMAP_TURBO)[0, 0].tolist()
        a = (int(u), int(v))
        b = (int(u + du * gain), int(v + dv * gain))
        cv2.arrowedLine(img, a, b, c, 1, cv2.LINE_AA, tipLength=0.3)
    _, _, cx, cy, _ = split_K(K)
    cv2.drawMarker(img, (int(cx), int(cy)), COLORS["white"], cv2.MARKER_CROSS, 16, 1)
    cv2.putText(img, f"max shift {mmax:.1f} px", (10, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (210, 210, 210), 1, cv2.LINE_AA)
    return img
