// Synthetic 2D test charts plus the two image warps that D controls.
//
// The charts are *ideal pinhole* images: what a perfect, undistorted camera
// with intrinsics K_ideal would record. Feeding one through distortImage()
// produces what a real lens would have delivered.
#pragma once

#include <opencv2/core.hpp>
#include <vector>

#include "camintrinsics/intrinsics.hpp"

namespace ci {

cv::Mat gridImage(int width, int height, int step = 40);
cv::Mat checkerboardImage(int width, int height, int squares = 10,
                          int margin = 40);
/// Concentric circles plus radial spokes. Radial distortion moves points along
/// the spokes and leaves their angle alone, so this chart separates the radial
/// terms from the tangential ones.
cv::Mat radialTarget(int width, int height, int rings = 10, int spokes = 24);
cv::Mat photoLike(int width, int height);

/// Render what a lens with (K, D) would have recorded from an ideal image.
///
/// The map is built backwards, destination to source: output pixel -> K^-1 ->
/// undistort -> K_ideal -> sample. Forward-mapping would leave holes.
/// Output pixels past the distortion model's fold point are left at the border
/// colour -- no ideal point maps there, so any value would be a fabrication.
cv::Mat distortImage(const cv::Mat& ideal, const Mat33& K, const Dist& D,
                     const Mat33& KIdeal);

/// Straighten an image. alpha = 0 crops until every output pixel is real;
/// alpha = 1 keeps every input pixel and gains black curved borders. Either
/// way K changes -- that is what KNew and roi are for.
cv::Mat undistortImage(const cv::Mat& distorted, const Mat33& K, const Dist& D,
                       double alpha, Mat33* KNew, cv::Rect* roi);

/// Per-pixel displacement introduced by D: delta = distorted - ideal, in px.
void distortionField(const Mat33& K, const Dist& D, int width, int height,
                     int step, std::vector<cv::Point2d>& uv,
                     std::vector<cv::Point2d>& delta);

}  // namespace ci
