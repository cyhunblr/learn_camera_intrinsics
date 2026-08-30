"""Named lens presets and the on-screen summary of a camera model.

The presets are shared by the figure renderers here and by the web viewer in
``web/``, so a lens called "action cam" means the same thing everywhere.
"""

from __future__ import annotations

__all__ = ["PRESETS", "preset_model", "kd_hud_lines"]


#: Named starting points, roughly matching real hardware classes.
PRESETS = {
    "ideal pinhole":      dict(f=1.00, k1=0.00,  k2=0.00,  p1=0.0,   p2=0.0,   k3=0.0),
    "webcam (mild barrel)": dict(f=0.85, k1=-0.18, k2=0.05, p1=0.001, p2=-0.001, k3=0.0),
    "action cam (strong barrel)": dict(f=0.45, k1=-0.35, k2=0.12, p1=0.0, p2=0.0, k3=0.0),
    "tele (pincushion)":  dict(f=1.80, k1=0.22,  k2=-0.06, p1=0.0,   p2=0.0,   k3=0.0),
    "decentred lens":     dict(f=0.90, k1=-0.12, k2=0.02,  p1=0.012, p2=0.009, k3=0.0),
}


def preset_model(name: str, width: int, height: int):
    """Build ``(K, D)`` for a named preset at the given image size.

    Mirrors ``ci::presetModel``. Note ``f`` scales the *width* for both focal
    lengths -- the presets describe square pixels.
    """
    from .intrinsics import make_D, make_K
    p = PRESETS[name]
    K = make_K(p["f"] * width, p["f"] * width, width / 2.0, height / 2.0)
    D = make_D(p["k1"], p["k2"], p["p1"], p["p2"], p["k3"])
    return K, D


def kd_hud_lines(K, D, width: int, height: int):
    """Human-readable summary of the current K and D, for the on-screen HUD."""
    from .intrinsics import (fov_deg, is_invertible_over_image,
                             max_distorted_radius, split_K)
    fx, fy, cx, cy, s = split_K(K)
    hf, vf, df = fov_deg(K, width, height)
    d = list(map(float, D))
    lines = [
        f"K = [ {fx:7.1f} {s:6.2f} {cx:7.1f} ]",
        f"    [ {0:7.1f} {fy:6.1f} {cy:7.1f} ]",
        f"    [ {0:7.1f} {0:6.1f} {1:7.1f} ]",
        f"D  k1 {d[0]:+.3f}   k2 {d[1]:+.3f}   k3 {d[4]:+.3f}",
        f"   p1 {d[2]:+.4f}  p2 {d[3]:+.4f}",
        f"FOV  h {hf:5.1f}  v {vf:5.1f}  d {df:5.1f} deg",
        f"fx/fy {fx / fy:5.3f}   principal offset "
        f"({cx - width / 2:+.0f}, {cy - height / 2:+.0f}) px",
    ]
    if not is_invertible_over_image(K, D, width, height):
        lines.append(f"!! corners exceed r'max={max_distorted_radius(D):.2f}:")
        lines.append("   undistort is undefined there")
    return lines
