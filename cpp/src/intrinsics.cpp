#include "camintrinsics/intrinsics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ci {
namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kDeg = 180.0 / CV_PI;
}  // namespace

Mat33 makeK(double fx, double fy, double cx, double cy, double skew) {
  return Mat33(fx, skew, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
}

Dist makeD(double k1, double k2, double p1, double p2, double k3) {
  return Dist(k1, k2, p1, p2, k3);
}

KParams splitK(const Mat33& K) {
  return KParams{K(0, 0), K(1, 1), K(0, 2), K(1, 2), K(0, 1)};
}

cv::Vec3d fovDeg(const Mat33& K, int width, int height) {
  const KParams p = splitK(K);
  const double hfov =
      kDeg * (std::atan2(p.cx, p.fx) + std::atan2(width - p.cx, p.fx));
  const double vfov =
      kDeg * (std::atan2(p.cy, p.fy) + std::atan2(height - p.cy, p.fy));
  // Diagonal comes from the corner with the largest normalized radius.
  double rmax = 0.0;
  for (const cv::Point2d& c : {cv::Point2d(0, 0), cv::Point2d(width, 0),
                               cv::Point2d(0, height),
                               cv::Point2d(width, height)}) {
    const double x = (c.x - p.cx) / p.fx, y = (c.y - p.cy) / p.fy;
    rmax = std::max(rmax, std::hypot(x, y));
  }
  return cv::Vec3d(hfov, vfov, 2.0 * kDeg * std::atan(rmax));
}

Mat33 KFromFov(double hfovDeg, int width, int height) {
  const double fx = (width / 2.0) / std::tan(hfovDeg / kDeg / 2.0);
  return makeK(fx, fx, width / 2.0, height / 2.0);
}

Mat33 scaleK(const Mat33& K, double sx, double sy) {
  Mat33 out = K;
  for (int c = 0; c < 3; ++c) {
    out(0, c) *= sx;
    out(1, c) *= sy;
  }
  return out;
}

Mat33 cropK(const Mat33& K, double x0, double y0) {
  Mat33 out = K;
  out(0, 2) -= x0;
  out(1, 2) -= y0;
  return out;
}

Mat33 flipK(const Mat33& K, int width, int height, bool horizontal,
            bool vertical) {
  Mat33 out = K;
  if (horizontal) {
    out(0, 2) = (width - 1) - out(0, 2);
    out(0, 1) = -out(0, 1);
  }
  if (vertical) out(1, 2) = (height - 1) - out(1, 2);
  return out;
}

Dist flipD(const Dist& D, bool horizontal, bool vertical) {
  Dist out = D;
  if (horizontal) out[3] = -out[3];  // p2
  if (vertical) out[2] = -out[2];    // p1
  return out;
}

cv::Point2d normalizedToPixel(const cv::Point2d& xy, const Mat33& K) {
  const KParams p = splitK(K);
  return {p.fx * xy.x + p.skew * xy.y + p.cx, p.fy * xy.y + p.cy};
}

cv::Point2d pixelToNormalized(const cv::Point2d& uv, const Mat33& K) {
  // Closed-form K^-1: solve the second row first, then substitute.
  const KParams p = splitK(K);
  const double y = (uv.y - p.cy) / p.fy;
  return {(uv.x - p.cx - p.skew * y) / p.fx, y};
}

std::vector<cv::Point2d> normalizedToPixel(const std::vector<cv::Point2d>& xy,
                                           const Mat33& K) {
  std::vector<cv::Point2d> out;
  out.reserve(xy.size());
  for (const auto& p : xy) out.push_back(normalizedToPixel(p, K));
  return out;
}

std::vector<cv::Point2d> pixelToNormalized(const std::vector<cv::Point2d>& uv,
                                           const Mat33& K) {
  std::vector<cv::Point2d> out;
  out.reserve(uv.size());
  for (const auto& p : uv) out.push_back(pixelToNormalized(p, K));
  return out;
}

cv::Point2d distortNormalized(const cv::Point2d& xy, const Dist& D) {
  const double k1 = D[0], k2 = D[1], p1 = D[2], p2 = D[3], k3 = D[4];
  const double x = xy.x, y = xy.y;
  const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
  const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
  return {x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x),
          y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y};
}

std::vector<cv::Point2d> distortNormalized(const std::vector<cv::Point2d>& xy,
                                           const Dist& D) {
  std::vector<cv::Point2d> out;
  out.reserve(xy.size());
  for (const auto& p : xy) out.push_back(distortNormalized(p, D));
  return out;
}

cv::Point2d undistortNormalized(const cv::Point2d& xyd, const Dist& D,
                                double rLimit) {
  constexpr int kIters = 30;
  constexpr double kStepTol = 1e-12;
  constexpr double kResidualTol = 1e-9;

  if (rLimit < 0.0) rLimit = maxDistortedRadius(D);
  if (std::hypot(xyd.x, xyd.y) > rLimit) return {kNaN, kNaN};  // no solution

  const double k1 = D[0], k2 = D[1], p1 = D[2], p2 = D[3], k3 = D[4];
  double x = xyd.x, y = xyd.y;
  for (int i = 0; i < kIters; ++i) {
    const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    const double drad = k1 + 2.0 * k2 * r2 + 3.0 * k3 * r4;  // d(radial)/d(r^2)

    const double fx = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double fy = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

    const double j00 = radial + 2.0 * x * x * drad + 2.0 * p1 * y + 6.0 * p2 * x;
    const double j01 = 2.0 * x * y * drad + 2.0 * p1 * x + 2.0 * p2 * y;
    const double j11 = radial + 2.0 * y * y * drad + 6.0 * p1 * y + 2.0 * p2 * x;

    const double rx = xyd.x - fx, ry = xyd.y - fy;
    const double det = j00 * j11 - j01 * j01;   // the Jacobian is symmetric
    // A near-singular Jacobian means Newton has no usable step. Take none
    // rather than a clamped one: clamping |det| keeps the magnitude at ~1/eps
    // and, for a negative det, points the step the wrong way.
    if (std::abs(det) <= 1e-12) break;
    const double dx = (j11 * rx - j01 * ry) / det;
    const double dy = (-j01 * rx + j00 * ry) / det;
    x += dx;
    y += dy;
    if (std::abs(dx) < kStepTol && std::abs(dy) < kStepTol) break;
  }

  // Believe the answer only if it solves the equation we were given.
  const cv::Point2d back = distortNormalized({x, y}, D);
  if (std::hypot(back.x - xyd.x, back.y - xyd.y) > kResidualTol)
    return {kNaN, kNaN};
  return {x, y};
}

std::vector<cv::Point2d> undistortNormalized(const std::vector<cv::Point2d>& xyd,
                                             const Dist& D) {
  const double rLimit = maxDistortedRadius(D);   // scan once, not per point
  std::vector<cv::Point2d> out;
  out.reserve(xyd.size());
  for (const auto& p : xyd) out.push_back(undistortNormalized(p, D, rLimit));
  return out;
}

void distortionProfile(const Dist& D, double rMax, int n,
                       std::vector<double>& r, std::vector<double>& rd) {
  r.resize(n);
  rd.resize(n);
  for (int i = 0; i < n; ++i) {
    r[i] = rMax * i / static_cast<double>(n - 1);
    rd[i] = distortNormalized({r[i], 0.0}, D).x;
  }
}

double maxValidRadius(const Dist& D, double rMax, int n) {
  std::vector<double> r, rd;
  distortionProfile(D, rMax, n, r, rd);
  for (int i = 0; i + 1 < n; ++i)
    if (rd[i + 1] <= rd[i]) return r[i];
  return rMax;
}

double maxDistortedRadius(const Dist& D, double rMax, int n) {
  return distortNormalized({maxValidRadius(D, rMax, n), 0.0}, D).x;
}

bool isInvertibleOverImage(const Mat33& K, const Dist& D, int width,
                           int height) {
  double rmax = 0.0;
  for (const cv::Point2d& c : {cv::Point2d(0, 0), cv::Point2d(width, 0),
                               cv::Point2d(0, height),
                               cv::Point2d(width, height)}) {
    const cv::Point2d xy = pixelToNormalized(c, K);
    rmax = std::max(rmax, std::hypot(xy.x, xy.y));
  }
  return rmax <= maxDistortedRadius(D);
}

Mat33 optimalNewCameraMatrix(const Mat33& K, const Dist& D, int width,
                             int height, double alpha, cv::Rect* validRoi) {
  constexpr int kN = 9;
  const double rLimit = maxDistortedRadius(D);
  double oL = 1e30, oR = -1e30, oT = 1e30, oB = -1e30;
  double iL = -1e30, iR = 1e30, iT = -1e30, iB = 1e30;
  for (int j = 0; j < kN; ++j) {
    for (int i = 0; i < kN; ++i) {
      const double u = i * (width - 1.0) / (kN - 1);
      const double v = j * (height - 1.0) / (kN - 1);
      const cv::Point2d p =
          undistortNormalized(pixelToNormalized({u, v}, K), D, rLimit);
      if (!std::isfinite(p.x) || !std::isfinite(p.y))
        throw std::domain_error(
            "this lens cannot be undistorted over the whole image: some pixels "
            "lie past the model's fold radius (see maxDistortedRadius)");
      oL = std::min(oL, p.x); oR = std::max(oR, p.x);
      oT = std::min(oT, p.y); oB = std::max(oB, p.y);
      if (i == 0) iL = std::max(iL, p.x);
      if (i == kN - 1) iR = std::min(iR, p.x);
      if (j == 0) iT = std::max(iT, p.y);
      if (j == kN - 1) iB = std::min(iB, p.y);
    }
  }
  const auto mix = [alpha](double in, double out) {
    return in * (1.0 - alpha) + out * alpha;
  };
  const double x0 = mix(iL, oL), y0 = mix(iT, oT);
  const double x1 = mix(iR, oR), y1 = mix(iB, oB);
  const double fx = (width - 1.0) / std::max(x1 - x0, 1e-9);
  const double fy = (height - 1.0) / std::max(y1 - y0, 1e-9);
  const Mat33 KNew = makeK(fx, fy, -fx * x0, -fy * y0);
  if (validRoi) {
    // The valid region is the inner rectangle, in the new pixel grid.
    const int rx0 = std::max(0, int(std::ceil(fx * iL + KNew(0, 2))));
    const int ry0 = std::max(0, int(std::ceil(fy * iT + KNew(1, 2))));
    const int rx1 = std::min(width, int(std::floor(fx * iR + KNew(0, 2))));
    const int ry1 = std::min(height, int(std::floor(fy * iB + KNew(1, 2))));
    *validRoi = cv::Rect(rx0, ry0, std::max(0, rx1 - rx0), std::max(0, ry1 - ry0));
  }
  return KNew;
}

cv::Point2d projectPoint(const cv::Point3d& pointCam, const Mat33& K,
                         const Dist& D, bool* valid) {
  if (pointCam.z <= 1e-9) {
    if (valid) *valid = false;
    return {kNaN, kNaN};
  }
  if (valid) *valid = true;
  const cv::Point2d xy(pointCam.x / pointCam.z, pointCam.y / pointCam.z);
  return normalizedToPixel(distortNormalized(xy, D), K);
}

std::vector<cv::Point2d> projectPoints(const std::vector<cv::Point3d>& pointsCam,
                                       const Mat33& K, const Dist& D,
                                       std::vector<bool>* valid) {
  std::vector<cv::Point2d> out;
  out.reserve(pointsCam.size());
  if (valid) valid->assign(pointsCam.size(), false);
  for (size_t i = 0; i < pointsCam.size(); ++i) {
    bool ok = false;
    out.push_back(projectPoint(pointsCam[i], K, D, &ok));
    if (valid) (*valid)[i] = ok;
  }
  return out;
}

cv::Point3d backprojectPixel(const cv::Point2d& uv, const Mat33& K,
                             const Dist& D, double depth) {
  const cv::Point2d xy = undistortNormalized(pixelToNormalized(uv, K), D);
  return {xy.x * depth, xy.y * depth, depth};
}

}  // namespace ci
