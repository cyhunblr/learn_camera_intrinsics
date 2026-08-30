#include "camintrinsics/renderer.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>

namespace ci {

cv::Point3d Pose::apply(const cv::Point3d& p) const {
  const cv::Vec3d v = R * cv::Vec3d(p.x, p.y, p.z) + t;
  return {v[0], v[1], v[2]};
}

cv::Point3d Pose::center() const {
  const cv::Vec3d c = -(R.t() * t);
  return {c[0], c[1], c[2]};
}

Pose lookAt(const cv::Point3d& eye, const cv::Point3d& target,
            const cv::Point3d& up) {
  cv::Vec3d z(target.x - eye.x, target.y - eye.y, target.z - eye.z);
  z /= cv::norm(z);
  const cv::Vec3d upv(-up.x, -up.y, -up.z);  // 'up' points -Y in our world
  cv::Vec3d x = upv.cross(z);
  if (cv::norm(x) < 1e-9) x = cv::Vec3d(1, 0, 0).cross(z);  // degenerate
  x /= cv::norm(x);
  const cv::Vec3d y = z.cross(x);
  Pose pose;
  pose.R = cv::Matx33d(x[0], x[1], x[2], y[0], y[1], y[2], z[0], z[1], z[2]);
  pose.t = -(pose.R * cv::Vec3d(eye.x, eye.y, eye.z));
  return pose;
}

Pose orbitPose(const cv::Point3d& target, double distance, double yawDeg,
               double pitchDeg) {
  const double yaw = yawDeg * CV_PI / 180.0, pitch = pitchDeg * CV_PI / 180.0;
  const cv::Point3d eye(target.x + distance * std::cos(pitch) * std::sin(yaw),
                        target.y - distance * std::sin(pitch),
                        target.z - distance * std::cos(pitch) * std::cos(yaw));
  return lookAt(eye, target);
}

namespace {

/// Split a camera-frame polyline into runs in front of the near plane.
/// Skip this and a point just behind the camera projects to a wild coordinate,
/// producing the long streaks that plague hand-written projective renderers.
std::vector<std::vector<cv::Point3d>> clipNear(
    const std::vector<cv::Point3d>& pts, double nearZ) {
  std::vector<std::vector<cv::Point3d>> runs;
  std::vector<cv::Point3d> cur;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (pts[i].z >= nearZ) {
      cur.push_back(pts[i]);
    } else {
      if (!cur.empty()) {
        const cv::Point3d& a = cur.back();
        const double s = (nearZ - a.z) / (pts[i].z - a.z);
        cur.push_back(a + (pts[i] - a) * s);
        runs.push_back(cur);
        cur.clear();
      }
      if (i + 1 < pts.size() && pts[i + 1].z >= nearZ) {
        const double s = (nearZ - pts[i].z) / (pts[i + 1].z - pts[i].z);
        cur.push_back(pts[i] + (pts[i + 1] - pts[i]) * s);
      }
    }
  }
  if (!cur.empty()) runs.push_back(cur);
  runs.erase(std::remove_if(runs.begin(), runs.end(),
                            [](const std::vector<cv::Point3d>& r) {
                              return r.size() < 2;
                            }),
             runs.end());
  return runs;
}

}  // namespace

void render(cv::Mat& img, const std::vector<Polyline>& polylines,
            const Pose& pose, const Mat33& K, const Dist& D, double nearZ,
            double clipPixels) {
  for (const Polyline& pl : polylines) {
    std::vector<cv::Point3d> cam;
    cam.reserve(pl.pts.size() + 1);
    for (const cv::Point3d& p : pl.pts) cam.push_back(pose.apply(p));
    if (pl.closed && cam.size() > 2) cam.push_back(cam.front());

    for (const auto& run : clipNear(cam, nearZ)) {
      const std::vector<cv::Point2d> uv = projectPoints(run, K, D, nullptr);
      std::vector<cv::Point> poly;
      poly.reserve(uv.size());
      bool ok = true;
      for (const cv::Point2d& p : uv) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
            std::abs(p.x) > clipPixels || std::abs(p.y) > clipPixels) {
          ok = false;
          break;
        }
        poly.emplace_back(cvRound(p.x), cvRound(p.y));
      }
      if (!ok || poly.size() < 2) continue;
      if (pl.filled && poly.size() >= 3)
        cv::fillPoly(img, std::vector<std::vector<cv::Point>>{poly}, pl.color,
                     cv::LINE_AA);
      else
        cv::polylines(img, poly, false, pl.color, pl.thickness, cv::LINE_AA);
    }
  }
}

void drawCrosshair(cv::Mat& img, const Mat33& K, int size) {
  const KParams p = splitK(K);
  cv::drawMarker(img, {img.cols / 2, img.rows / 2}, colors::kDim,
                 cv::MARKER_CROSS, size, 1);
  const cv::Point pp(cvRound(p.cx), cvRound(p.cy));
  cv::drawMarker(img, pp, colors::kRed, cv::MARKER_CROSS, size + 6, 2);
  cv::circle(img, pp, size, colors::kRed, 1, cv::LINE_AA);
}

void drawTextBlock(cv::Mat& img, const std::vector<std::string>& lines,
                   cv::Point org, double scale, const cv::Scalar& color,
                   int lineH, bool bg) {
  if (lines.empty()) return;
  if (bg) {
    int wmax = 0, baseline = 0;
    for (const auto& t : lines)
      wmax = std::max(wmax, cv::getTextSize(t, cv::FONT_HERSHEY_SIMPLEX, scale,
                                            1, &baseline).width);
    cv::Mat overlay = img.clone();
    cv::rectangle(overlay, cv::Point(org.x - 6, org.y - 16),
                  cv::Point(org.x + wmax + 8,
                            org.y + lineH * int(lines.size()) - 6),
                  cv::Scalar(18, 18, 18), -1);
    cv::addWeighted(overlay, 0.62, img, 0.38, 0, img);
  }
  for (size_t i = 0; i < lines.size(); ++i)
    cv::putText(img, lines[i], {org.x, org.y + int(i) * lineH},
                cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
}

cv::Mat hstackLabeled(const std::vector<cv::Mat>& images,
                      const std::vector<std::string>& labels, int pad,
                      int labelH) {
  const cv::Scalar bg(24, 24, 28);
  int hs = 0, total = 0;
  for (const auto& im : images) hs = std::max(hs, im.rows);
  for (const auto& im : images) total += im.cols;
  total += pad * (int(images.size()) - 1);

  cv::Mat out(hs + labelH, total, CV_8UC3, bg);
  int x = 0;
  for (size_t i = 0; i < images.size(); ++i) {
    images[i].copyTo(out(cv::Rect(x, labelH, images[i].cols, images[i].rows)));
    cv::putText(out, i < labels.size() ? labels[i] : "", {x + 8, 18},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, colors::kWhite, 1, cv::LINE_AA);
    x += images[i].cols + pad;
  }
  return out;
}

cv::Mat vstack(const std::vector<cv::Mat>& images, const cv::Scalar& bg) {
  int w = 0, h = 0;
  for (const auto& im : images) {
    w = std::max(w, im.cols);
    h += im.rows;
  }
  cv::Mat out(h, w, CV_8UC3, bg);
  int y = 0;
  for (const auto& im : images) {
    im.copyTo(out(cv::Rect(0, y, im.cols, im.rows)));
    y += im.rows;
  }
  return out;
}

}  // namespace ci
