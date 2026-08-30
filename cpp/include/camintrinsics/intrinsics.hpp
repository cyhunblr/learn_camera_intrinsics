// Camera intrinsics from scratch: the K matrix and the D vector.
//
// This is the C++ twin of python/camintrinsics/intrinsics.py. The function
// names, the maths and the results are identical, so you can read one and
// use the other. Everything is written with plain arithmetic rather than
// OpenCV calls, so the model is visible; the tests then prove it agrees with
// OpenCV to machine precision.
//
// Conventions
//   * right-handed frames, camera looks down +Z, +X right, +Y down (OpenCV)
//   * xy = normalized image coordinates, uv = pixels
#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace ci {

using Mat33 = cv::Matx33d;
/// OpenCV distortion vector, in OpenCV's own order: k1, k2, p1, p2, k3.
using Dist = cv::Vec<double, 5>;

struct KParams {
  double fx, fy, cx, cy, skew;
};

// --- building and inspecting K -------------------------------------------

/// K = [fx s cx; 0 fy cy; 0 0 1]. Focal lengths and principal point in pixels.
Mat33 makeK(double fx, double fy, double cx, double cy, double skew = 0.0);

/// Assemble [k1, k2, p1, p2, k3]. Mind the order: the tangential terms p1, p2
/// sit *between* k2 and k3. Getting it wrong is the classic calibration bug.
Dist makeD(double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0,
           double k3 = 0.0);

KParams splitK(const Mat33& K);

// --- field of view --------------------------------------------------------

/// (horizontal, vertical, diagonal) FOV in degrees for an ideal pinhole.
/// Computed as two asymmetric half-angles per axis, because cx is generally
/// not width/2.
cv::Vec3d fovDeg(const Mat33& K, int width, int height);

/// Centred, square-pixel K from a horizontal field of view.
Mat33 KFromFov(double hfovDeg, int width, int height);

// --- what image operations do to K ---------------------------------------

/// K after resizing the image. Every entry of the first two rows scales,
/// including cx, cy and the skew.
Mat33 scaleK(const Mat33& K, double sx, double sy);

/// K after cropping with the new origin at (x0, y0). Focal lengths are
/// untouched: cropping changes the field of view, not the lens.
Mat33 cropK(const Mat33& K, double x0, double y0);

/// K after mirroring. Pair with flipD -- a mirror is only exact if the
/// tangential coefficients are mirrored too.
Mat33 flipK(const Mat33& K, int width, int height, bool horizontal = true,
            bool vertical = false);

/// D after mirroring: p2 -> -p2 for a horizontal flip, p1 -> -p1 vertical.
/// The radial terms are even functions and survive untouched.
Dist flipD(const Dist& D, bool horizontal = true, bool vertical = false);

// --- K as a coordinate change --------------------------------------------

cv::Point2d normalizedToPixel(const cv::Point2d& xy, const Mat33& K);
cv::Point2d pixelToNormalized(const cv::Point2d& uv, const Mat33& K);
std::vector<cv::Point2d> normalizedToPixel(const std::vector<cv::Point2d>& xy,
                                           const Mat33& K);
std::vector<cv::Point2d> pixelToNormalized(const std::vector<cv::Point2d>& uv,
                                           const Mat33& K);

// --- the D vector ---------------------------------------------------------

/// Forward lens model in normalized coordinates (OpenCV plumb-bob):
///   radial = 1 + k1 r^2 + k2 r^4 + k3 r^6
///   x' = x*radial + 2 p1 x y + p2 (r^2 + 2 x^2)
///   y' = y*radial + p1 (r^2 + 2 y^2) + 2 p2 x y
cv::Point2d distortNormalized(const cv::Point2d& xy, const Dist& D);
std::vector<cv::Point2d> distortNormalized(const std::vector<cv::Point2d>& xy,
                                           const Dist& D);

/// Inverse of distortNormalized, or NaN if it cannot be done.
///
/// OpenCV uses a fixed-point iteration, which stops contracting near the
/// corners of a strong wide-angle lens and silently oscillates. This uses
/// Newton's method on the same 2x2 system: quadratic convergence.
///
/// Newton is fast but not magic. A point past the fold radius has no solution,
/// and a Newton step can converge to the polynomial's *other* root -- for
/// k1 = -0.5, undistorting r' = 0.82 gives x = -1.72, which satisfies the
/// equation exactly and is physically meaningless. Both are rejected, by
/// radius up front and by residual afterwards, and rejected points come back
/// as NaN -- the same convention projectPoints uses for points behind the
/// camera.
///
/// `rLimit` is maxDistortedRadius(D); pass it when you already have it (it
/// costs a 2000-sample scan), or leave it negative to have it computed.
cv::Point2d undistortNormalized(const cv::Point2d& xyd, const Dist& D,
                                double rLimit = -1.0);
std::vector<cv::Point2d> undistortNormalized(const std::vector<cv::Point2d>& xyd,
                                             const Dist& D);

/// Radial-only curve r' vs r, sampled n times over [0, rMax].
void distortionProfile(const Dist& D, double rMax, int n,
                       std::vector<double>& r, std::vector<double>& rd);

/// Radius at which r'(r) stops increasing; past it the model folds over and
/// undistortion is no longer unique. Returns rMax if it never folds.
double maxValidRadius(const Dist& D, double rMax = 3.0, int n = 2000);

/// The largest r' the model can produce. Any point beyond it cannot be
/// undistorted at all -- cv::undistort will silently return garbage.
double maxDistortedRadius(const Dist& D, double rMax = 3.0, int n = 2000);

/// Can all four image corners be undistorted?
bool isInvertibleOverImage(const Mat33& K, const Dist& D, int width, int height);

/// The K of the undistorted image, and the region of it holding real data.
///
/// Same algorithm as cv::getOptimalNewCameraMatrix, written out here because
/// OpenCV's version undistorts its sampling grid with the default five
/// fixed-point iterations, which do not converge on a strong lens -- 13% in fx
/// for this repository's "action cam" preset. Uses undistortNormalized, so the
/// figures, the Python half and the web viewer all agree.
///
/// Throws std::domain_error if some image pixel lies past the model's fold
/// radius, so there is no undistorted image to describe.
Mat33 optimalNewCameraMatrix(const Mat33& K, const Dist& D, int width,
                             int height, double alpha, cv::Rect* validRoi = nullptr);

// --- the full forward model ----------------------------------------------

/// Project camera-frame points to pixels: divide, distort, then apply K.
/// Points with Z <= 0 are behind the camera; `valid` is set false for them and
/// their pixel is left as quiet NaN.
std::vector<cv::Point2d> projectPoints(const std::vector<cv::Point3d>& pointsCam,
                                       const Mat33& K, const Dist& D,
                                       std::vector<bool>* valid = nullptr);

cv::Point2d projectPoint(const cv::Point3d& pointCam, const Mat33& K,
                         const Dist& D, bool* valid = nullptr);

/// Pixel -> a point on its viewing ray at the given depth.
cv::Point3d backprojectPixel(const cv::Point2d& uv, const Mat33& K,
                             const Dist& D, double depth = 1.0);

}  // namespace ci
