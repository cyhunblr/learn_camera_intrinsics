#include "camintrinsics/patterns.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>

namespace ci {
namespace {
const cv::Scalar kBg(26, 26, 30);
}

cv::Mat gridImage(int width, int height, int step) {
  cv::Mat img(height, width, CV_8UC3, kBg);
  const cv::Scalar color(200, 200, 200), accent(60, 170, 250);
  for (int x = 0; x <= width; x += step) {
    const bool major = (x / step) % 5 == 0;
    cv::line(img, {x, 0}, {x, height}, major ? accent : color, major ? 2 : 1,
             cv::LINE_AA);
  }
  for (int y = 0; y <= height; y += step) {
    const bool major = (y / step) % 5 == 0;
    cv::line(img, {0, y}, {width, y}, major ? accent : color, major ? 2 : 1,
             cv::LINE_AA);
  }
  cv::rectangle(img, {1, 1}, {width - 2, height - 2}, cv::Scalar(90, 230, 90), 2);
  return img;
}

cv::Mat checkerboardImage(int width, int height, int squares, int margin) {
  cv::Mat img(height, width, CV_8UC3, kBg);
  const int side = std::min(width - 2 * margin, height - 2 * margin) / squares;
  const int x0 = (width - side * squares) / 2, y0 = (height - side * squares) / 2;
  for (int r = 0; r < squares; ++r)
    for (int c = 0; c < squares; ++c)
      if ((r + c) % 2 != 0)
        cv::rectangle(img, {x0 + c * side, y0 + r * side},
                      {x0 + (c + 1) * side, y0 + (r + 1) * side},
                      cv::Scalar(235, 235, 235), -1);
  cv::rectangle(img, {x0, y0}, {x0 + side * squares, y0 + side * squares},
                cv::Scalar(60, 170, 250), 2);
  return img;
}

cv::Mat radialTarget(int width, int height, int rings, int spokes) {
  cv::Mat img(height, width, CV_8UC3, kBg);
  const double cx = width / 2.0, cy = height / 2.0;
  const double rmax = std::hypot(cx, cy);
  for (int i = 1; i <= rings; ++i)
    cv::circle(img, {int(cx), int(cy)}, int(rmax * i / rings),
               cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
  for (int k = 0; k < spokes; ++k) {
    const double a = 2 * CV_PI * k / spokes;
    cv::line(img, {int(cx), int(cy)},
             {int(cx + rmax * std::cos(a)), int(cy + rmax * std::sin(a))},
             cv::Scalar(110, 110, 110), 1, cv::LINE_AA);
  }
  cv::drawMarker(img, {int(cx), int(cy)}, cv::Scalar(60, 60, 235),
                 cv::MARKER_CROSS, 20, 2);
  return img;
}

cv::Mat photoLike(int width, int height) {
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(60, 45, 35));
  const int horizon = int(height * 0.45);
  cv::rectangle(img, {0, 0}, {width, horizon}, cv::Scalar(150, 110, 70), -1);
  for (int i = -4; i <= 4; ++i)
    cv::line(img, {width / 2, horizon}, {width / 2 + i * width / 9, height},
             cv::Scalar(230, 230, 230), 3, cv::LINE_AA);
  const int bx[4] = {40, 200, 520, 690}, bw[4] = {120, 90, 140, 90};
  const double bh[4] = {0.30, 0.22, 0.34, 0.20};
  for (int b = 0; b < 4; ++b) {
    const int top = int(horizon - height * bh[b]);
    cv::rectangle(img, {bx[b], top}, {bx[b] + bw[b], horizon},
                  cv::Scalar(95, 95, 105), -1);
    cv::rectangle(img, {bx[b], top}, {bx[b] + bw[b], horizon},
                  cv::Scalar(40, 40, 45), 2);
    for (int wy = top + 12; wy < horizon - 10; wy += 26)
      for (int wx = bx[b] + 10; wx < bx[b] + bw[b] - 14; wx += 24)
        cv::rectangle(img, {wx, wy}, {wx + 12, wy + 14},
                      cv::Scalar(70, 190, 230), -1);
  }
  cv::line(img, {0, horizon}, {width, horizon}, cv::Scalar(230, 230, 230), 2,
           cv::LINE_AA);
  return img;
}

cv::Mat distortImage(const cv::Mat& ideal, const Mat33& K, const Dist& D,
                     const Mat33& KIdeal) {
  const int ow = ideal.cols, oh = ideal.rows;
  const double rLimit = maxDistortedRadius(D);
  cv::Mat mx(oh, ow, CV_32FC1), my(oh, ow, CV_32FC1);
  for (int v = 0; v < oh; ++v) {
    float* rowx = mx.ptr<float>(v);
    float* rowy = my.ptr<float>(v);
    for (int u = 0; u < ow; ++u) {
      const cv::Point2d xyd = pixelToNormalized({double(u), double(v)}, K);
      const cv::Point2d xy = undistortNormalized(xyd, D, rLimit);
      if (!std::isfinite(xy.x) || !std::isfinite(xy.y)) {
        rowx[u] = -1.0f;      // no ideal point maps here; remap -> borderValue
        rowy[u] = -1.0f;
        continue;
      }
      const cv::Point2d src = normalizedToPixel(xy, KIdeal);
      rowx[u] = float(src.x);
      rowy[u] = float(src.y);
    }
  }
  cv::Mat out;
  cv::remap(ideal, out, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
            cv::Scalar(12, 12, 14));
  return out;
}

cv::Mat undistortImage(const cv::Mat& distorted, const Mat33& K, const Dist& D,
                       double alpha, Mat33* KNew, cv::Rect* roi) {
  cv::Rect r;
  const Mat33 Kn = optimalNewCameraMatrix(K, D, distorted.cols, distorted.rows,
                                          alpha, &r);
  cv::Mat out;
  // cv::undistort itself is exact -- it maps destination to source with the
  // *forward* model. Only the new camera matrix above needed replacing.
  cv::undistort(distorted, out, cv::Mat(K), cv::Mat(D), cv::Mat(Kn));
  if (KNew) *KNew = Kn;
  if (roi) *roi = r;
  return out;
}

void distortionField(const Mat33& K, const Dist& D, int width, int height,
                     int step, std::vector<cv::Point2d>& uv,
                     std::vector<cv::Point2d>& delta) {
  uv.clear();
  delta.clear();
  for (int v = step / 2; v < height; v += step) {
    for (int u = step / 2; u < width; u += step) {
      const cv::Point2d p(u, v);
      const cv::Point2d xy = pixelToNormalized(p, K);  // treat as an ideal ray
      const cv::Point2d pd = normalizedToPixel(distortNormalized(xy, D), K);
      uv.push_back(p);
      delta.push_back(pd - p);
    }
  }
}

}  // namespace ci
