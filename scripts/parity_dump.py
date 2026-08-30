#!/usr/bin/env python3
"""Print reference values for scripts/check_parity.sh. See that script."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "python"))
from camintrinsics import (fov_deg, make_D, make_K, max_distorted_radius,   # noqa: E402
                           optimal_new_camera_matrix, pixel_to_normalized,
                           project_points, undistort_normalized)

def f(v):                       # one formatting rule for all three languages
    return "nan" if v != v else f"{v:.9f}"

for line in sys.stdin:
    if not line.strip():
        continue
    fx, fy, cx, cy, k1, k2, p1, p2, k3, w, h = (float(t) for t in line.split())
    K, D, w, h = make_K(fx, fy, cx, cy), make_D(k1, k2, p1, p2, k3), int(w), int(h)
    print("case", line.strip())
    for P in ([0.9, -0.4, 3.0], [-1.2, 0.7, 5.0], [2.0, 1.5, 2.2]):
        u, v = project_points([P], K, D)[0]
        print("  project", f(u), f(v))
    print("  fov", *(f(v) for v in fov_deg(K, w, h)))
    print("  rmax", f(max_distorted_radius(D)))
    for u, v in ([0, 0], [w / 2, 0], [w, h], [w * 1.5, h * 1.5]):
        x, y = undistort_normalized(pixel_to_normalized([[u, v]], K), D)[0]
        print("  undistort", f(x), f(y))
    for a in (0.0, 0.5, 1.0):
        try:
            Kn, roi = optimal_new_camera_matrix(K, D, w, h, a)
        except ValueError:
            print("  newK unavailable")
            continue
        print("  newK", f(Kn[0, 0]), f(Kn[1, 1]), f(Kn[0, 2]), f(Kn[1, 2]), *roi)
