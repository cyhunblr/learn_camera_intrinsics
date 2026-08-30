#include "camintrinsics/plots.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include "camintrinsics/patterns.hpp"
#include "camintrinsics/renderer.hpp"
#include "camintrinsics/scene.hpp"

namespace ci {

cv::Mat plotDistortionProfile(const Dist& D, int width, int height,
                              double rMax) {
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(24, 24, 28));
  const int m = 38, x0 = m, y0 = height - m, x1 = width - 12, y1 = 12;
  std::vector<double> r, rd;
  distortionProfile(D, rMax, 400, r, rd);
  double yMax = rMax;
  for (double v : rd) yMax = std::max(yMax, v * 1.05);
  yMax = std::max(yMax, 1e-6);

  auto toPx = [&](double rx, double ry) {
    return cv::Point(int(x0 + (x1 - x0) * rx / rMax),
                     int(y0 + (y1 - y0) * ry / yMax));
  };
  for (int i = 0; i <= 6; ++i) {
    const double g = rMax * i / 6.0;
    cv::line(img, toPx(g, 0), toPx(g, yMax), cv::Scalar(44, 44, 48), 1);
    cv::line(img, toPx(0, yMax * i / 6.0), toPx(rMax, yMax * i / 6.0),
             cv::Scalar(44, 44, 48), 1);
  }
  cv::line(img, toPx(0, 0), toPx(rMax, std::min(rMax, yMax)),
           cv::Scalar(110, 110, 110), 1, cv::LINE_AA);  // identity
  std::vector<cv::Point> curve;
  curve.reserve(r.size());
  for (size_t i = 0; i < r.size(); ++i) curve.push_back(toPx(r[i], rd[i]));
  cv::polylines(img, curve, false, colors::kOrange, 2, cv::LINE_AA);

  cv::line(img, {x0, y0}, {x1, y0}, cv::Scalar(150, 150, 150), 1);
  cv::line(img, {x0, y0}, {x0, y1}, cv::Scalar(150, 150, 150), 1);
  cv::putText(img, "r (normalized)", {x0 + 4, height - 12},
              cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(170, 170, 170), 1,
              cv::LINE_AA);
  cv::putText(img, "r'", {8, y1 + 14}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
              cv::Scalar(170, 170, 170), 1, cv::LINE_AA);
  const char* tag = rd.back() < r.back() ? "barrel"
                    : rd.back() > r.back() ? "pincushion" : "none";
  cv::putText(img, std::string("radial: ") + tag, {x0 + 4, y1 + 16},
              cv::FONT_HERSHEY_SIMPLEX, 0.45, colors::kOrange, 1, cv::LINE_AA);
  return img;
}

cv::Mat plotDistortionField(const Mat33& K, const Dist& D, int width,
                            int height, int step, double gain) {
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(24, 24, 28));
  std::vector<cv::Point2d> uv, delta;
  distortionField(K, D, width, height, step, uv, delta);
  double mmax = 1e-6;
  for (const auto& d : delta) mmax = std::max(mmax, cv::norm(d));
  for (size_t i = 0; i < uv.size(); ++i) {
    cv::Mat lut(1, 1, CV_8UC1,
                cv::Scalar(255.0 * cv::norm(delta[i]) / mmax));
    cv::Mat rgb;
    cv::applyColorMap(lut, rgb, cv::COLORMAP_TURBO);
    const cv::Vec3b c = rgb.at<cv::Vec3b>(0, 0);
    cv::arrowedLine(img, cv::Point(int(uv[i].x), int(uv[i].y)),
                    cv::Point(int(uv[i].x + delta[i].x * gain),
                              int(uv[i].y + delta[i].y * gain)),
                    cv::Scalar(c[0], c[1], c[2]), 1, cv::LINE_AA, 0, 0.3);
  }
  const KParams p = splitK(K);
  cv::drawMarker(img, {int(p.cx), int(p.cy)}, colors::kWhite, cv::MARKER_CROSS,
                 16, 1);
  cv::putText(img, fmt("max shift %.1f px", mmax), {10, height - 12},
              cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(210, 210, 210), 1,
              cv::LINE_AA);
  return img;
}

}  // namespace ci
