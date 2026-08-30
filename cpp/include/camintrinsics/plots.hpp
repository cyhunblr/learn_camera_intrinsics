// Small OpenCV-drawn diagnostic plots -- no plotting library, so they can live
// inside a live interactive window at 60 fps.
#pragma once

#include <opencv2/core.hpp>

#include "camintrinsics/intrinsics.hpp"

namespace ci {

/// Plot r' against r: the whole radial model on one curve. The grey diagonal
/// is "no distortion"; below it is barrel, above it is pincushion. Where the
/// curve stops rising, the model has folded and undistortion is not unique.
cv::Mat plotDistortionProfile(const Dist& D, int width = 380, int height = 260,
                              double rMax = 1.2);

/// Arrows showing where D pushes each pixel, colour-coded by magnitude. Long
/// arrows in the corners and none at the centre is pure radial distortion; a
/// field that does not vanish at the principal point means p1/p2 are at work.
cv::Mat plotDistortionField(const Mat33& K, const Dist& D, int width,
                            int height, int step = 46, double gain = 1.0);

}  // namespace ci
