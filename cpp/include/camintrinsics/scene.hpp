// A tiny 3D scene description: everything is a subdivided 3D polyline.
//
// Why subdivide? A straight world line stays straight under a pinhole
// projection, but lens distortion bends it. Store only the two endpoints and
// you would draw a straight segment between two correctly-curved endpoints --
// hiding the very effect the demo exists to show.
#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace ci {

struct Polyline {
  std::vector<cv::Point3d> pts;  ///< world frame
  cv::Scalar color{235, 235, 235};
  int thickness = 1;
  bool closed = false;
  bool filled = false;
};

namespace colors {
const cv::Scalar kWhite(235, 235, 235);   // BGR throughout, because OpenCV
const cv::Scalar kGrey(120, 120, 120);
const cv::Scalar kDim(70, 70, 70);
const cv::Scalar kRed(60, 60, 235);
const cv::Scalar kGreen(90, 210, 90);
const cv::Scalar kBlue(235, 160, 60);
const cv::Scalar kYellow(60, 210, 235);
const cv::Scalar kCyan(220, 220, 60);
const cv::Scalar kMagenta(200, 60, 200);
const cv::Scalar kOrange(40, 140, 250);
}  // namespace colors

/// A straight segment subdivided into n pieces so distortion can bend it.
Polyline line(const cv::Point3d& a, const cv::Point3d& b,
              const cv::Scalar& color = colors::kWhite, int thickness = 1,
              int n = 24);

/// A closed polygon whose every edge is subdivided.
Polyline polygon(const std::vector<cv::Point3d>& corners,
                 const cv::Scalar& color = colors::kWhite, int thickness = 1,
                 int n = 16, bool filled = false);

void appendAxes(std::vector<Polyline>& out, double length = 1.0,
                const cv::Point3d& origin = {0, 0, 0}, int thickness = 2);
void appendGroundGrid(std::vector<Polyline>& out, double halfExtent = 7.0,
                      double step = 0.5, double y = 1.3);
void appendCube(std::vector<Polyline>& out, const cv::Point3d& center,
                double size, const cv::Scalar& color = colors::kYellow,
                int thickness = 2);
void appendCheckerboard(std::vector<Polyline>& out, int rows, int cols,
                        double square, const cv::Point3d& origin);
void appendSphereWire(std::vector<Polyline>& out, const cv::Point3d& center,
                      double radius, int rings = 5,
                      const cv::Scalar& color = colors::kMagenta);

/// The viewing frustum implied by K, drawn in world coordinates. Raise fx and
/// the pyramid narrows; slide cx and it shears off the optical axis.
void appendFrustum(std::vector<Polyline>& out, const cv::Matx33d& K, int width,
                   int height, double nearZ, double farZ,
                   const cv::Matx33d& poseR, const cv::Vec3d& poseT,
                   const cv::Scalar& color = colors::kCyan, int thickness = 2);

/// A small box plus an axis triad drawn at the camera's own position.
void appendCameraGizmo(std::vector<Polyline>& out, const cv::Matx33d& poseR,
                       const cv::Vec3d& poseT, double scale = 0.22);

/// The shared demo scene: ground grid, two cubes, a sphere and a board,
/// all between Z = 2 m and Z = 6 m. Identical to the Python default_scene().
std::vector<Polyline> defaultScene(bool grid = true);

}  // namespace ci
