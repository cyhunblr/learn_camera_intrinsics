"""learn_camera_intrinsics -- a hands-on tour of the K matrix and the D vector.

Quick start::

    from camintrinsics import make_K, make_D, project_points
    K = make_K(fx=800, fy=800, cx=640, cy=360)
    D = make_D(k1=-0.28, k2=0.09)
    project_points([[0.5, 0.2, 4.0]], K, D)
"""

from .intrinsics import (K_from_fov, backproject_pixels, crop_K,
                         distort_normalized, distortion_profile, flip_D,
                         flip_K, fov_deg, is_invertible_over_image, make_D,
                         make_K, max_distorted_radius, max_valid_radius,
                         normalized_to_pixel, optimal_new_camera_matrix,
                         pixel_to_normalized, project_points, scale_K,
                         split_K, undistort_normalized)

__version__ = "1.0.0"

__all__ = [
    "make_K", "make_D", "split_K", "K_from_fov", "fov_deg",
    "scale_K", "crop_K", "flip_K", "flip_D",
    "pixel_to_normalized", "normalized_to_pixel",
    "distort_normalized", "undistort_normalized",
    "project_points", "backproject_pixels",
    "distortion_profile", "max_valid_radius", "max_distorted_radius",
    "is_invertible_over_image", "optimal_new_camera_matrix",
]
