"""A ~150-line software renderer, written so that K and D are the only optics.

There is no OpenGL and no shading here.  Every pixel drawn goes through
:func:`camintrinsics.intrinsics.project_points`, so whatever you see on screen
is *literally* the intrinsic model doing its job.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np

from .intrinsics import project_points, split_K
from .scene import COLORS, Polyline

__all__ = ["Pose", "look_at", "orbit_pose", "render", "draw_crosshair",
           "draw_text_block", "hstack_labeled", "save_image"]


@dataclass
class Pose:
    """A rigid world -> camera transform: ``p_cam = R @ p_world + t``."""
    R: np.ndarray
    t: np.ndarray

    def apply(self, points_world: np.ndarray) -> np.ndarray:
        P = np.atleast_2d(np.asarray(points_world, dtype=np.float64))
        return P @ self.R.T + self.t.reshape(1, 3)

    @property
    def center(self) -> np.ndarray:
        """Camera centre in world coordinates: ``C = -R^T t``."""
        return -self.R.T @ self.t.reshape(3)


def look_at(eye, target, up=(0, -1, 0)) -> Pose:
    """Build a Pose that puts the camera at ``eye`` looking at ``target``.

    OpenCV convention: +Z forward (into the scene), +X right, +Y **down** --
    hence the default ``up=(0,-1,0)`` for a world whose +Y also points down.
    """
    eye, target, up = (np.asarray(v, dtype=np.float64) for v in (eye, target, up))
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(-up, z)          # 'up' points -Y, so -up is the world's +Y-ish
    nx = np.linalg.norm(x)
    if nx < 1e-9:                 # degenerate: looking straight along 'up'
        x = np.cross(np.array([1.0, 0.0, 0.0]), z)
        nx = np.linalg.norm(x)
    x /= nx
    y = np.cross(z, x)
    R = np.stack([x, y, z], axis=0)
    return Pose(R, -R @ eye)


def orbit_pose(target, distance: float, yaw_deg: float, pitch_deg: float) -> Pose:
    """Camera orbiting ``target`` -- the mouse-free way to fly around a scene."""
    yaw, pitch = np.radians(yaw_deg), np.radians(pitch_deg)
    target = np.asarray(target, dtype=np.float64)
    offset = np.array([distance * np.cos(pitch) * np.sin(yaw),
                       -distance * np.sin(pitch),
                       -distance * np.cos(pitch) * np.cos(yaw)])
    return look_at(target + offset, target)


# --------------------------------------------------------------------------
def _clip_near(pts_cam: np.ndarray, near: float) -> list[np.ndarray]:
    """Split a camera-frame polyline into runs in front of the near plane.

    Without this step, a point just behind the camera projects to a wild
    coordinate and you get long streaks shooting across the image -- the
    single most common bug in hand-written projective renderers.
    """
    z = pts_cam[:, 2]
    runs, cur = [], []
    for i in range(len(pts_cam)):
        if z[i] >= near:
            cur.append(pts_cam[i])
        else:
            if cur:
                # extend the run up to the near plane before cutting it
                a, b = cur[-1], pts_cam[i]
                s = (near - a[2]) / (b[2] - a[2])
                cur.append(a + s * (b - a))
                runs.append(np.array(cur))
                cur = []
        if z[i] < near and i + 1 < len(pts_cam) and z[i + 1] >= near:
            a, b = pts_cam[i], pts_cam[i + 1]
            s = (near - a[2]) / (b[2] - a[2])
            cur.append(a + s * (b - a))
    if cur:
        runs.append(np.array(cur))
    return [r for r in runs if len(r) >= 2]


def render(img: np.ndarray, polylines: list[Polyline], pose: Pose,
           K: np.ndarray, D=None, near: float = 0.05,
           clip_pixels: float = 6000.0) -> np.ndarray:
    """Draw a scene into ``img`` through the camera described by ``pose, K, D``."""
    h, w = img.shape[:2]
    for pl in polylines:
        pts_cam = pose.apply(pl.points)
        if pl.closed and len(pts_cam) > 2:
            pts_cam = np.vstack([pts_cam, pts_cam[:1]])
        for run in _clip_near(pts_cam, near):
            uv = project_points(run, K, D)
            if not np.all(np.isfinite(uv)):
                continue
            if np.abs(uv).max() > clip_pixels:
                continue
            poly = np.round(uv).astype(np.int32)
            if pl.fill is not None and len(poly) >= 3:
                cv2.fillPoly(img, [poly], pl.fill, lineType=cv2.LINE_AA)
            else:
                cv2.polylines(img, [poly], False, pl.color, pl.thickness,
                              lineType=cv2.LINE_AA)
    return img


# --------------------------------------------------------------------------
# Small drawing helpers shared by the apps
# --------------------------------------------------------------------------
def draw_crosshair(img: np.ndarray, K: np.ndarray, size: int = 14) -> None:
    """Mark the principal point (cx, cy) and the geometric image centre.

    Seeing these two markers drift apart is the fastest way to understand that
    ``(cx, cy)`` is *not* "the middle of the image" -- it is where the optical
    axis happens to pierce the sensor.
    """
    h, w = img.shape[:2]
    fx, fy, cx, cy, _ = split_K(K)
    gx, gy = int(round(w / 2)), int(round(h / 2))
    cv2.drawMarker(img, (gx, gy), COLORS["dim"], cv2.MARKER_CROSS, size, 1)
    px, py = int(round(cx)), int(round(cy))
    cv2.drawMarker(img, (px, py), COLORS["red"], cv2.MARKER_CROSS, size + 6, 2)
    cv2.circle(img, (px, py), size, COLORS["red"], 1, cv2.LINE_AA)


def draw_text_block(img: np.ndarray, lines, org=(10, 22), scale: float = 0.45,
                    color=COLORS["white"], line_h: int = 17,
                    bg: bool = True) -> None:
    """Draw a left-aligned multi-line HUD with a readable dark backdrop."""
    x, y = org
    if bg and lines:
        wmax = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
                   for t in lines)
        overlay = img.copy()
        cv2.rectangle(overlay, (x - 6, y - 16),
                      (x + wmax + 8, y + line_h * len(lines) - 6), (18, 18, 18), -1)
        cv2.addWeighted(overlay, 0.62, img, 0.38, 0, img)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (x, y + i * line_h), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, 1, cv2.LINE_AA)


def hstack_labeled(images, labels, pad: int = 8, label_h: int = 26,
                   bg=(24, 24, 24)) -> np.ndarray:
    """Stack images side by side, each with a caption bar above it."""
    hs = max(im.shape[0] for im in images)
    tiles = []
    for im, lab in zip(images, labels):
        tile = np.full((hs + label_h, im.shape[1], 3), bg, np.uint8)
        tile[label_h:label_h + im.shape[0], :im.shape[1]] = im
        cv2.putText(tile, lab, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    COLORS["white"], 1, cv2.LINE_AA)
        tiles.append(tile)
    out = np.full((hs + label_h, sum(t.shape[1] for t in tiles) + pad * (len(tiles) - 1), 3),
                  bg, np.uint8)
    x = 0
    for t in tiles:
        out[:, x:x + t.shape[1]] = t
        x += t.shape[1] + pad
    return out


def save_image(path: str, img: np.ndarray) -> bool:
    """Write an image, reporting honestly.

    ``cv2.imwrite`` returns False for a path it cannot write and raises for an
    extension it does not recognise.  Either way the caller must not go on to
    print "wrote ..." -- a tool that lies about what it did is worse than one
    that fails.
    """
    directory = os.path.dirname(os.path.abspath(path))
    try:
        os.makedirs(directory, exist_ok=True)
        if cv2.imwrite(path, img):
            print(f"wrote {path}  ({img.shape[1]}x{img.shape[0]})")
            return True
        print(f"could not write {path} -- is the directory writable?",
              file=sys.stderr)
    except (OSError, cv2.error) as exc:
        print(f"could not write {path} -- {exc}", file=sys.stderr)
    return False
