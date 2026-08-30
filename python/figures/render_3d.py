#!/usr/bin/env python3
"""Render the 3D figure: the frustum K describes, beside the image it produces.

    uv run python/figures/render_3d.py --out data/generated/app3d.png

This is a *renderer*, not an app. Interactive exploration lives in the web
viewer (https://cyhunblr.github.io/learn_camera_intrinsics/).
Every pixel here still goes through the projection function by hand -- no
OpenGL -- so a bent line is D bending it.
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from camintrinsics import make_K                                 # noqa: E402
from camintrinsics.renderer import (Pose, draw_crosshair,         # noqa: E402
                                    draw_text_block, hstack_labeled,
                                    orbit_pose, render, save_image)
from camintrinsics.scene import (camera_gizmo, default_scene,     # noqa: E402
                                 frustum)
from camintrinsics.presets import (PRESETS, kd_hud_lines,         # noqa: E402
                                   preset_model)

CAM_W, CAM_H = 520, 390          # the resolution of the camera we are studying
WORLD_W, WORLD_H = 640, 480      # the third-person viewport
TARGET = np.array([0.0, 0.0, 4.0])       # what the studied camera looks at
VIEW_TARGET = np.array([0.0, 0.0, 2.0])  # what the third-person view orbits

CAPTION = [
    "fx,fy -> frustum width (FOV).  cx,cy -> the frustum shears off the blue optical axis.",
    "k1,k2,k3 bend lines radially; p1,p2 tilt the whole image (decentred lens).",
]


def compose(K, D, cam_pose: Pose, view_pose: Pose, use_D: bool,
            show_grid: bool, preset_name: str, caption: bool = True) -> np.ndarray:
    scene = default_scene(grid=show_grid)

    # --- left: third-person view, always an ideal pinhole so it stays readable
    world = np.full((WORLD_H, WORLD_W, 3), (22, 22, 26), np.uint8)
    K_view = make_K(WORLD_W * 0.9, WORLD_W * 0.9, WORLD_W / 2, WORLD_H / 2)
    overlay = (frustum(K, CAM_W, CAM_H, near=0.10, far=2.0,
                       pose_R=cam_pose.R, pose_t=cam_pose.t)
               + camera_gizmo(cam_pose.R, cam_pose.t))
    render(world, scene + overlay, view_pose, K_view, None)

    # --- right: the camera's own image, K and D doing all the work
    cam = np.full((CAM_H, CAM_W, 3), (22, 22, 26), np.uint8)
    render(cam, scene, cam_pose, K, D if use_D else None)
    draw_crosshair(cam, K)
    cv2.rectangle(cam, (0, 0), (CAM_W - 1, CAM_H - 1), (70, 70, 80), 1)

    top = hstack_labeled(
        [world, cam],
        ["world view  -  cyan = the frustum K describes",
         f"camera view  {CAM_W}x{CAM_H}  -  distortion {'ON' if use_D else 'OFF'}"],
    )

    hud_w = top.shape[1]
    hud = np.full((150, hud_w, 3), (18, 18, 22), np.uint8)
    left = kd_hud_lines(K, D, CAM_W, CAM_H)
    C = cam_pose.center
    right = [f"preset: {preset_name}",
             f"camera centre  ({C[0]:+.2f}, {C[1]:+.2f}, {C[2]:+.2f})",
             "the blue axis is the optical axis;",
             "the frustum only stays centred on it",
             f"while cx = {CAM_W / 2:.0f} and cy = {CAM_H / 2:.0f}.",
             "distortion is applied AFTER the",
             "perspective divide, BEFORE K."]
    draw_text_block(hud, left, org=(14, 24), scale=0.44, line_h=19, bg=False)
    draw_text_block(hud, right, org=(hud_w // 2 + 20, 24), scale=0.44,
                    line_h=19, bg=False)

    parts = [top, hud]
    if caption:
        bar = np.full((48, hud_w, 3), (14, 14, 18), np.uint8)
        draw_text_block(bar, CAPTION, org=(14, 22), scale=0.44, line_h=19, bg=False)
        parts.append(bar)
    return np.vstack(parts)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default="data/generated/app3d.png")
    ap.add_argument("--preset", choices=list(PRESETS), default="webcam (mild barrel)")
    ap.add_argument("--no-distortion", action="store_true")
    args = ap.parse_args()

    K, D = preset_model(args.preset, CAM_W, CAM_H)
    frame = compose(K, D,
                    orbit_pose(TARGET, 2.6, 0.0, 6.0),
                    orbit_pose(VIEW_TARGET, 8.0, 38.0, 22.0),
                    not args.no_distortion, True, args.preset)

    return 0 if save_image(args.out, frame) else 1


if __name__ == "__main__":
    raise SystemExit(main())
