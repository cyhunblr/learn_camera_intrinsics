// A ~150-line software renderer, written so that K and D are the only optics.
// There is no OpenGL here: every pixel drawn goes through ci::projectPoints,
// so whatever appears on screen is literally the intrinsic model doing its job.
#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "camintrinsics/intrinsics.hpp"
#include "camintrinsics/scene.hpp"
#include "camintrinsics/util.hpp"

namespace ci {

/// A rigid world -> camera transform: p_cam = R * p_world + t.
struct Pose {
  cv::Matx33d R = cv::Matx33d::eye();
  cv::Vec3d t{0, 0, 0};

  cv::Point3d apply(const cv::Point3d& pWorld) const;
  /// Camera centre in world coordinates: C = -R^T t.
  cv::Point3d center() const;
};

/// Camera at `eye` looking at `target`. OpenCV convention: +Z forward, +Y down,
/// hence the default up vector pointing along -Y.
Pose lookAt(const cv::Point3d& eye, const cv::Point3d& target,
            const cv::Point3d& up = {0, -1, 0});

/// Camera orbiting `target` -- the mouse-free way to fly around a scene.
Pose orbitPose(const cv::Point3d& target, double distance, double yawDeg,
               double pitchDeg);

/// Draw a scene into `img` through the camera described by (pose, K, D).
void render(cv::Mat& img, const std::vector<Polyline>& polylines,
            const Pose& pose, const Mat33& K, const Dist& D,
            double nearZ = 0.05, double clipPixels = 6000.0);

/// Mark the principal point (cx, cy) and the geometric image centre. Watching
/// them drift apart is the fastest way to learn that (cx, cy) is not "the
/// middle of the image".
void drawCrosshair(cv::Mat& img, const Mat33& K, int size = 14);

/// Left-aligned multi-line HUD with an optional dark backdrop.
void drawTextBlock(cv::Mat& img, const std::vector<std::string>& lines,
                   cv::Point org = {10, 22}, double scale = 0.45,
                   const cv::Scalar& color = colors::kWhite, int lineH = 17,
                   bool bg = true);

/// Stack images side by side, each with a caption bar above it.
cv::Mat hstackLabeled(const std::vector<cv::Mat>& images,
                      const std::vector<std::string>& labels, int pad = 8,
                      int labelH = 26);

/// Stack images vertically, left-aligned, padding the narrow ones.
cv::Mat vstack(const std::vector<cv::Mat>& images,
               const cv::Scalar& bg = cv::Scalar(24, 24, 28));

}  // namespace ci
