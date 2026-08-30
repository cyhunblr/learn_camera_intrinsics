// Worked solutions. Read these after you have attempted exercises.cpp -- and
// then compare approaches rather than just diffing: several of these have more
// than one reasonable shape.
#include "exercises.hpp"

#include <cmath>
#include <limits>

namespace exlab {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDeg = 180.0 / CV_PI;

cv::Matx33d build_K(double fx, double fy, double cx, double cy, double skew) {
  return cv::Matx33d(fx, skew, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
}

std::vector<cv::Point2d> project_pinhole(
    const std::vector<cv::Point3d>& pointsCam, const cv::Matx33d& K) {
  std::vector<cv::Point2d> out;
  out.reserve(pointsCam.size());
  for (const cv::Point3d& p : pointsCam) {
    if (p.z <= 0.0) {
      out.emplace_back(kNaN, kNaN);
      continue;
    }
    // Perspective divide, then K. No matrix inverse, no 4x4: the closed form
    // is both clearer and faster.
    const double x = p.x / p.z, y = p.y / p.z;
    out.emplace_back(K(0, 0) * x + K(0, 1) * y + K(0, 2), K(1, 1) * y + K(1, 2));
  }
  return out;
}

std::vector<cv::Point2d> distort(const std::vector<cv::Point2d>& xy, double k1,
                                 double k2, double p1, double p2, double k3) {
  std::vector<cv::Point2d> out;
  out.reserve(xy.size());
  for (const cv::Point2d& q : xy) {
    const double x = q.x, y = q.y;
    const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    out.emplace_back(x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
                     y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y);
  }
  return out;
}

cv::Vec2d fov_degrees(const cv::Matx33d& K, int width, int height) {
  const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
  // Two half-angles per axis, because cx is not necessarily width/2.
  return {kDeg * (std::atan2(cx, fx) + std::atan2(width - cx, fx)),
          kDeg * (std::atan2(cy, fy) + std::atan2(height - cy, fy))};
}

cv::Matx33d K_after_resize(const cv::Matx33d& K, double sx, double sy) {
  cv::Matx33d out = K;
  for (int c = 0; c < 3; ++c) {
    out(0, c) *= sx;  // fx, skew and cx are all horizontal pixel lengths
    out(1, c) *= sy;  // fy and cy are vertical pixel lengths
  }
  return out;
}

cv::Matx33d K_after_crop(const cv::Matx33d& K, double x0, double y0) {
  cv::Matx33d out = K;
  out(0, 2) -= x0;  // only the origin moved; the lens is unchanged
  out(1, 2) -= y0;
  return out;
}

std::vector<cv::Point2d> undistort_point(const std::vector<cv::Point2d>& xyd,
                                         double k1, double k2, double p1,
                                         double p2, double k3, int iters) {
  std::vector<cv::Point2d> out;
  out.reserve(xyd.size());
  for (const cv::Point2d& q : xyd) {
    double x = q.x, y = q.y;
    for (int i = 0; i < iters; ++i) {
      const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
      const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
      const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
      x = (q.x - dx) / radial;
      y = (q.y - dy) / radial;
    }
    out.emplace_back(x, y);
  }
  return out;
}

cv::Matx33d K_from_hfov(double hfovDeg, int width, int height) {
  const double fx = (width / 2.0) / std::tan(hfovDeg / kDeg / 2.0);
  return build_K(fx, fx, width / 2.0, height / 2.0, 0.0);
}

std::string classify_distortion(double k1, double k2, double k3, double r) {
  const double r2 = r * r;
  const double rd = r * (1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
  if (rd < r - 1e-9) return "barrel";
  if (rd > r + 1e-9) return "pincushion";
  return "none";
}

cv::Matx33d pipeline_K(const cv::Matx33d& K, double cropX0, double cropY0,
                       double scale) {
  // A pixel meets the crop first (coordinates shift), then the resize
  // (everything scales). So crop, then scale -- not the other way round.
  return K_after_resize(K_after_crop(K, cropX0, cropY0), scale, scale);
}

const Impl kImpl = {"solutions",         build_K,         project_pinhole,
                    distort,             fov_degrees,     K_after_resize,
                    K_after_crop,        undistort_point, K_from_hfov,
                    classify_distortion, pipeline_K};

}  // namespace

const Impl& solutions() { return kImpl; }

}  // namespace exlab
