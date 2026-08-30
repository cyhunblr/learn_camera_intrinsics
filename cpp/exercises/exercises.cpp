// Your turn. Replace each `throw NotImplemented{}` with real code, rebuild,
// and run ./build/bin/check_exercises. The specification for each function is
// in exercises.hpp, and the theory is in docs/.
#include "exercises.hpp"

#include <cmath>

namespace exlab {
namespace {

cv::Matx33d build_K(double fx, double fy, double cx, double cy, double skew) {
  (void)fx; (void)fy; (void)cx; (void)cy; (void)skew;
  throw NotImplemented{};
}

std::vector<cv::Point2d> project_pinhole(
    const std::vector<cv::Point3d>& pointsCam, const cv::Matx33d& K) {
  (void)pointsCam; (void)K;
  throw NotImplemented{};
}

std::vector<cv::Point2d> distort(const std::vector<cv::Point2d>& xy, double k1,
                                 double k2, double p1, double p2, double k3) {
  (void)xy; (void)k1; (void)k2; (void)p1; (void)p2; (void)k3;
  throw NotImplemented{};
}

cv::Vec2d fov_degrees(const cv::Matx33d& K, int width, int height) {
  (void)K; (void)width; (void)height;
  throw NotImplemented{};
}

cv::Matx33d K_after_resize(const cv::Matx33d& K, double sx, double sy) {
  (void)K; (void)sx; (void)sy;
  throw NotImplemented{};
}

cv::Matx33d K_after_crop(const cv::Matx33d& K, double x0, double y0) {
  (void)K; (void)x0; (void)y0;
  throw NotImplemented{};
}

std::vector<cv::Point2d> undistort_point(const std::vector<cv::Point2d>& xyd,
                                         double k1, double k2, double p1,
                                         double p2, double k3, int iters) {
  (void)xyd; (void)k1; (void)k2; (void)p1; (void)p2; (void)k3; (void)iters;
  throw NotImplemented{};
}

cv::Matx33d K_from_hfov(double hfovDeg, int width, int height) {
  (void)hfovDeg; (void)width; (void)height;
  throw NotImplemented{};
}

std::string classify_distortion(double k1, double k2, double k3, double r) {
  (void)k1; (void)k2; (void)k3; (void)r;
  throw NotImplemented{};
}

cv::Matx33d pipeline_K(const cv::Matx33d& K, double cropX0, double cropY0,
                       double scale) {
  (void)K; (void)cropX0; (void)cropY0; (void)scale;
  throw NotImplemented{};
}

const Impl kImpl = {"exercises",         build_K,         project_pinhole,
                    distort,             fov_degrees,     K_after_resize,
                    K_after_crop,        undistort_point, K_from_hfov,
                    classify_distortion, pipeline_K};

}  // namespace

const Impl& exercises() { return kImpl; }

}  // namespace exlab
