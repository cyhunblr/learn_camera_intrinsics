#include "camintrinsics/scene.hpp"

#include <cmath>

#include "camintrinsics/intrinsics.hpp"

namespace ci {
namespace {

cv::Point3d lerp(const cv::Point3d& a, const cv::Point3d& b, double t) {
  return a * (1.0 - t) + b * t;
}

/// p_cam = R p_world + t, so p_world = R^T (p_cam - t).
cv::Point3d camToWorld(const cv::Point3d& p, const cv::Matx33d& R,
                       const cv::Vec3d& t) {
  const cv::Vec3d d(p.x - t[0], p.y - t[1], p.z - t[2]);
  const cv::Vec3d w = R.t() * d;
  return {w[0], w[1], w[2]};
}

}  // namespace

Polyline line(const cv::Point3d& a, const cv::Point3d& b,
              const cv::Scalar& color, int thickness, int n) {
  Polyline pl;
  pl.color = color;
  pl.thickness = thickness;
  pl.pts.reserve(n + 1);
  for (int i = 0; i <= n; ++i) pl.pts.push_back(lerp(a, b, i / double(n)));
  return pl;
}

Polyline polygon(const std::vector<cv::Point3d>& corners,
                 const cv::Scalar& color, int thickness, int n, bool filled) {
  Polyline pl;
  pl.color = color;
  pl.thickness = thickness;
  pl.closed = true;
  pl.filled = filled;
  for (size_t i = 0; i < corners.size(); ++i) {
    const cv::Point3d& a = corners[i];
    const cv::Point3d& b = corners[(i + 1) % corners.size()];
    for (int k = 0; k < n; ++k) pl.pts.push_back(lerp(a, b, k / double(n)));
  }
  return pl;
}

void appendAxes(std::vector<Polyline>& out, double length,
                const cv::Point3d& o, int thickness) {
  out.push_back(line(o, o + cv::Point3d(length, 0, 0), colors::kRed, thickness));
  out.push_back(line(o, o + cv::Point3d(0, length, 0), colors::kGreen, thickness));
  out.push_back(line(o, o + cv::Point3d(0, 0, length), colors::kBlue, thickness));
}

void appendGroundGrid(std::vector<Polyline>& out, double halfExtent,
                      double step, double y) {
  for (double t = -halfExtent; t <= halfExtent + 1e-9; t += step) {
    const cv::Scalar c = std::abs(t) < 1e-9 ? colors::kGrey : colors::kDim;
    out.push_back(line({t, y, -halfExtent}, {t, y, halfExtent}, c, 1, 48));
    out.push_back(line({-halfExtent, y, t}, {halfExtent, y, t}, c, 1, 48));
  }
}

void appendCube(std::vector<Polyline>& out, const cv::Point3d& c, double size,
                const cv::Scalar& color, int thickness) {
  const double h = size / 2.0;
  const cv::Point3d v[8] = {
      c + cv::Point3d(-h, -h, -h), c + cv::Point3d(h, -h, -h),
      c + cv::Point3d(h, h, -h),   c + cv::Point3d(-h, h, -h),
      c + cv::Point3d(-h, -h, h),  c + cv::Point3d(h, -h, h),
      c + cv::Point3d(h, h, h),    c + cv::Point3d(-h, h, h)};
  const int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                        {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto& pair : e)
    out.push_back(line(v[pair[0]], v[pair[1]], color, thickness));
}

void appendCheckerboard(std::vector<Polyline>& out, int rows, int cols,
                        double square, const cv::Point3d& o) {
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const double u0 = c * square, u1 = (c + 1) * square;
      const double v0 = r * square, v1 = (r + 1) * square;
      const std::vector<cv::Point3d> quad = {
          o + cv::Point3d(u0, v0, 0), o + cv::Point3d(u1, v0, 0),
          o + cv::Point3d(u1, v1, 0), o + cv::Point3d(u0, v1, 0)};
      const cv::Scalar col = ((r + c) % 2 == 0) ? cv::Scalar(35, 35, 35)
                                                : cv::Scalar(225, 225, 225);
      out.push_back(polygon(quad, col, 1, 10, /*filled=*/true));
    }
  }
}

void appendSphereWire(std::vector<Polyline>& out, const cv::Point3d& c,
                      double radius, int rings, const cv::Scalar& color) {
  const int kSeg = 64;
  for (int i = 1; i < rings; ++i) {
    const double phi = CV_PI * i / rings;
    Polyline pl;
    pl.color = color;
    pl.closed = true;
    for (int k = 0; k < kSeg; ++k) {
      const double t = 2 * CV_PI * k / kSeg;
      pl.pts.push_back(c + cv::Point3d(radius * std::sin(phi) * std::cos(t),
                                       radius * std::cos(phi),
                                       radius * std::sin(phi) * std::sin(t)));
    }
    out.push_back(pl);
  }
  for (int j = 0; j < rings; ++j) {
    const double th = CV_PI * j / rings;
    Polyline pl;
    pl.color = color;
    pl.closed = true;
    for (int k = 0; k < kSeg; ++k) {
      const double t = 2 * CV_PI * k / kSeg;
      pl.pts.push_back(c + cv::Point3d(radius * std::sin(t) * std::cos(th),
                                       radius * std::cos(t),
                                       radius * std::sin(t) * std::sin(th)));
    }
    out.push_back(pl);
  }
}

void appendFrustum(std::vector<Polyline>& out, const cv::Matx33d& K, int width,
                   int height, double nearZ, double farZ,
                   const cv::Matx33d& R, const cv::Vec3d& t,
                   const cv::Scalar& color, int thickness) {
  // The four image corners, back-projected as ideal pinhole rays. Distortion
  // is deliberately ignored: the frustum is geometry, D is the lens on top.
  const cv::Point2d cornersPx[4] = {{0, 0}, {double(width), 0},
                                    {double(width), double(height)},
                                    {0, double(height)}};
  std::vector<cv::Point3d> nearW, farW;
  for (const auto& c : cornersPx) {
    const cv::Point2d xy = pixelToNormalized(c, K);
    nearW.push_back(camToWorld({xy.x * nearZ, xy.y * nearZ, nearZ}, R, t));
    farW.push_back(camToWorld({xy.x * farZ, xy.y * farZ, farZ}, R, t));
  }
  const cv::Point3d org = camToWorld({0, 0, 0}, R, t);
  out.push_back(polygon(farW, color, thickness, 12));
  out.push_back(polygon(nearW, colors::kDim, 1, 8));
  for (int i = 0; i < 4; ++i) out.push_back(line(org, farW[i], color, thickness));
  // A short "up" marker so the image orientation is unambiguous.
  const cv::Point3d topMid = (farW[0] + farW[1]) * 0.5;
  const cv::Point3d botMid = (farW[2] + farW[3]) * 0.5;
  out.push_back(line(topMid, topMid + (topMid - botMid) * 0.12,
                     colors::kMagenta, thickness));
}

void appendCameraGizmo(std::vector<Polyline>& out, const cv::Matx33d& R,
                       const cv::Vec3d& t, double s) {
  const cv::Point3d body[8] = {
      {-s, -0.7 * s, -1.6 * s}, {s, -0.7 * s, -1.6 * s},
      {s, 0.7 * s, -1.6 * s},   {-s, 0.7 * s, -1.6 * s},
      {-s, -0.7 * s, 0},        {s, -0.7 * s, 0},
      {s, 0.7 * s, 0},          {-s, 0.7 * s, 0}};
  const int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6},
                        {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto& pair : e)
    out.push_back(line(camToWorld(body[pair[0]], R, t),
                       camToWorld(body[pair[1]], R, t), colors::kWhite, 1, 2));
  const double a = s * 2.2;
  const cv::Point3d o = camToWorld({0, 0, 0}, R, t);
  const cv::Scalar triadCols[3] = {colors::kRed, colors::kGreen, colors::kBlue};
  const cv::Point3d dirs[3] = {{a, 0, 0}, {0, a, 0}, {0, 0, a}};
  for (int i = 0; i < 3; ++i)
    out.push_back(line(o, camToWorld(dirs[i], R, t), triadCols[i], 2, 2));
}

std::vector<Polyline> defaultScene(bool grid) {
  std::vector<Polyline> s;
  if (grid) appendGroundGrid(s, 7.0, 0.5, 1.3);
  appendAxes(s, 0.9);
  appendCube(s, {-1.35, 0.45, 3.8}, 1.4, colors::kYellow);
  appendCube(s, {1.55, 0.65, 5.4}, 1.2, colors::kOrange);
  appendSphereWire(s, {0.2, 0.1, 2.6}, 0.5);
  appendCheckerboard(s, 5, 7, 0.34, {-1.2, -1.65, 4.9});
  return s;
}

}  // namespace ci
