"""Tiny 3D scene description: everything is a subdivided 3D polyline.

Why subdivide?  A straight line in the world stays straight under a *pinhole*
projection, but lens distortion bends it.  If you store only the two endpoints
you can never see that bend -- you would draw a straight segment between two
correctly-curved endpoints.  So every primitive here is emitted as a polyline
with many intermediate points, and the renderer projects each of them.

That single detail is the difference between a demo that teaches distortion and
one that hides it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

__all__ = ["Polyline", "COLORS", "line", "polygon", "axes", "ground_grid",
           "cube", "checkerboard", "frustum", "camera_gizmo", "sphere_wire",
           "default_scene"]

# BGR, because OpenCV.
COLORS = {
    "white":  (235, 235, 235),
    "grey":   (120, 120, 120),
    "dim":    (70, 70, 70),
    "red":    (60, 60, 235),
    "green":  (90, 210, 90),
    "blue":   (235, 160, 60),
    "yellow": (60, 210, 235),
    "cyan":   (220, 220, 60),
    "magenta": (200, 60, 200),
    "orange": (40, 140, 250),
}


@dataclass
class Polyline:
    """An ordered list of 3D points to be drawn as a connected path."""
    points: np.ndarray                       # (N, 3) float64, world frame
    color: tuple = COLORS["white"]
    thickness: int = 1
    closed: bool = False
    fill: tuple | None = None                # if set, the polygon is filled

    def __post_init__(self):
        self.points = np.atleast_2d(np.asarray(self.points, dtype=np.float64))


def line(a, b, color=COLORS["white"], thickness: int = 1, n: int = 24) -> Polyline:
    """A straight world-space segment subdivided into ``n`` pieces."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return Polyline(a[None, :] * (1 - t) + b[None, :] * t, color, thickness)


def polygon(corners, color=COLORS["white"], thickness: int = 1,
            n: int = 16, fill=None) -> Polyline:
    """A closed polygon whose every edge is subdivided."""
    corners = np.asarray(corners, dtype=np.float64)
    pts = []
    for i in range(len(corners)):
        a, b = corners[i], corners[(i + 1) % len(corners)]
        t = np.linspace(0.0, 1.0, n, endpoint=False)[:, None]
        pts.append(a[None, :] * (1 - t) + b[None, :] * t)
    return Polyline(np.vstack(pts), color, thickness, closed=True, fill=fill)


# --------------------------------------------------------------------------
# Primitives
# --------------------------------------------------------------------------
def axes(length: float = 1.0, origin=(0, 0, 0), thickness: int = 2) -> list[Polyline]:
    """RGB = XYZ world axes at ``origin``."""
    o = np.asarray(origin, float)
    return [
        line(o, o + [length, 0, 0], COLORS["red"], thickness),
        line(o, o + [0, length, 0], COLORS["green"], thickness),
        line(o, o + [0, 0, length], COLORS["blue"], thickness),
    ]


def ground_grid(half_extent: float = 6.0, step: float = 1.0, y: float = 0.0,
                color=COLORS["dim"], accent=COLORS["grey"]) -> list[Polyline]:
    """A grid on the world XZ plane (``y`` is down in our convention)."""
    out = []
    ticks = np.arange(-half_extent, half_extent + 1e-9, step)
    for t in ticks:
        c = accent if abs(t) < 1e-9 else color
        out.append(line([t, y, -half_extent], [t, y, half_extent], c, 1, n=48))
        out.append(line([-half_extent, y, t], [half_extent, y, t], c, 1, n=48))
    return out


def cube(center=(0, 0, 0), size: float = 1.0, color=COLORS["yellow"],
         thickness: int = 2) -> list[Polyline]:
    """Wireframe cube -- the classic "are my straight lines straight?" probe."""
    c, h = np.asarray(center, float), size / 2.0
    s = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
                  [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], float) * h + c
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    return [line(s[a], s[b], color, thickness) for a, b in edges]


def checkerboard(rows: int = 6, cols: int = 9, square: float = 0.4,
                 origin=(0, 0, 0), normal: str = "z",
                 dark=(35, 35, 35), light=(225, 225, 225)) -> list[Polyline]:
    """A filled checkerboard plate -- the pattern every calibration uses.

    ``normal`` selects the plane the board lies in: ``"z"`` faces the camera,
    ``"y"`` lies flat on the ground.
    """
    o = np.asarray(origin, float)
    out = []
    for r in range(rows):
        for c in range(cols):
            u0, u1 = c * square, (c + 1) * square
            v0, v1 = r * square, (r + 1) * square
            quad_uv = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
            if normal == "z":
                q = [o + [u, v, 0] for u, v in quad_uv]
            else:  # lying on the ground plane
                q = [o + [u, 0, v] for u, v in quad_uv]
            col = dark if (r + c) % 2 == 0 else light
            out.append(polygon(q, color=col, n=10, fill=col))
    return out


def frustum(K: np.ndarray, width: int, height: int, near: float = 0.15,
            far: float = 1.4, pose_R: np.ndarray | None = None,
            pose_t: np.ndarray | None = None,
            color=COLORS["cyan"], thickness: int = 2) -> list[Polyline]:
    """The viewing frustum implied by ``K`` -- drawn in world coordinates.

    This is where K becomes physical: raise ``fx`` and the pyramid narrows,
    move ``cx`` and the pyramid *shears* off-axis.  The corners are the four
    image corners back-projected as ideal pinhole rays (distortion is ignored
    here on purpose -- the frustum is the geometry, D is the lens on top of it).
    """
    from .intrinsics import pixel_to_normalized
    corners_px = np.array([[0, 0], [width, 0], [width, height], [0, height]], float)
    xy = pixel_to_normalized(corners_px, K)
    rays = np.hstack([xy, np.ones((4, 1))])

    def to_world(pts_cam):
        if pose_R is None:
            return pts_cam
        # p_cam = R p_world + t   ->   p_world = R^T (p_cam - t)
        return (pts_cam - np.asarray(pose_t, float).reshape(1, 3)) @ np.asarray(pose_R, float)

    near_c, far_c = rays * near, rays * far
    origin_c = np.zeros((1, 3))
    near_w, far_w, org_w = to_world(near_c), to_world(far_c), to_world(origin_c)

    out = [polygon(far_w, color, thickness, n=12),
           polygon(near_w, COLORS["dim"], 1, n=8)]
    for i in range(4):
        out.append(line(org_w[0], far_w[i], color, thickness))
    # A short "up" marker so the image orientation is unambiguous.
    top_mid = (far_w[0] + far_w[1]) / 2.0
    out.append(line(top_mid, top_mid + (top_mid - (far_w[2] + far_w[3]) / 2.0) * 0.12,
                    COLORS["magenta"], thickness))
    return out


def camera_gizmo(pose_R=None, pose_t=None, scale: float = 0.22) -> list[Polyline]:
    """A little box + axis triad drawn at the camera's own position."""
    def to_world(p):
        if pose_R is None:
            return p
        return (p - np.asarray(pose_t, float).reshape(1, 3)) @ np.asarray(pose_R, float)

    body = np.array([[-1, -0.7, -1.6], [1, -0.7, -1.6], [1, 0.7, -1.6], [-1, 0.7, -1.6],
                     [-1, -0.7, 0], [1, -0.7, 0], [1, 0.7, 0], [-1, 0.7, 0]]) * scale
    body_w = to_world(body)
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
             (0, 4), (1, 5), (2, 6), (3, 7)]
    out = [line(body_w[a], body_w[b], COLORS["white"], 1, n=2) for a, b in edges]
    triad = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]]) * scale * 2.2
    tw = to_world(triad)
    for i, col in enumerate([COLORS["red"], COLORS["green"], COLORS["blue"]]):
        out.append(line(tw[0], tw[i + 1], col, 2, n=2))
    return out


def sphere_wire(center=(0, 0, 0), radius: float = 0.6, rings: int = 5,
                color=COLORS["magenta"], thickness: int = 1) -> list[Polyline]:
    """Latitude/longitude wireframe sphere -- curved lines for contrast."""
    c = np.asarray(center, float)
    out = []
    for i in range(1, rings):
        phi = np.pi * i / rings
        t = np.linspace(0, 2 * np.pi, 64)
        pts = np.stack([radius * np.sin(phi) * np.cos(t),
                        radius * np.cos(phi) * np.ones_like(t),
                        radius * np.sin(phi) * np.sin(t)], axis=1) + c
        out.append(Polyline(pts, color, thickness, closed=True))
    for j in range(rings):
        th = np.pi * j / rings
        t = np.linspace(0, 2 * np.pi, 64)
        pts = np.stack([radius * np.sin(t) * np.cos(th),
                        radius * np.cos(t),
                        radius * np.sin(t) * np.sin(th)], axis=1) + c
        out.append(Polyline(pts, color, thickness, closed=True))
    return out


def default_scene(grid: bool = True) -> list[Polyline]:
    """The shared demo scene: a ground grid, two cubes, a sphere and a board.

    Deliberately built from long straight edges (the cubes, the grid) *and*
    genuinely curved ones (the sphere), so you can tell real curvature apart
    from curvature the lens invented.  Everything sits between Z = 2 m and
    Z = 6 m in front of the world origin.
    """
    s: list[Polyline] = []
    if grid:
        s += ground_grid(half_extent=7.0, step=0.5, y=1.3)
    s += axes(0.9)
    s += cube(center=(-1.35, 0.45, 3.8), size=1.4, color=COLORS["yellow"])
    s += cube(center=(1.55, 0.65, 5.4), size=1.2, color=COLORS["orange"])
    s += sphere_wire(center=(0.2, 0.1, 2.6), radius=0.5)
    s += checkerboard(rows=5, cols=7, square=0.34, origin=(-1.2, -1.65, 4.9))
    return s
