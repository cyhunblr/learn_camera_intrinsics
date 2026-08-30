// Render the 2D figure: a chart, what K and D do to it, and it straightened.
//
//   ./build/bin/render_2d [--out out.png] [--preset N] [--chart N] [--alpha A]
//
// This is a *renderer*, not an app. Interactive exploration lives in the web
// viewer (https://cyhunblr.github.io/learn_camera_intrinsics/),
// which does the same maths in JavaScript. This exists
// so the figures can be regenerated, and as an exact twin of the Python half.
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "camintrinsics/patterns.hpp"
#include "camintrinsics/plots.hpp"
#include "camintrinsics/presets.hpp"
#include "camintrinsics/renderer.hpp"
#include "camintrinsics/util.hpp"

namespace {

constexpr int W = 440, H = 330;

const std::vector<std::string> kCaption = {
    "fx,fy zoom the image.  cx,cy slide it.  k1<0 barrel, k1>0 pincushion.",
    "p1,p2 are tangential: they tilt the pattern instead of squeezing it."};

cv::Mat makeChart(int index) {
  switch (index) {
    case 1: return ci::checkerboardImage(W, H, 8, 18);
    case 2: return ci::radialTarget(W, H, 8, 24);
    case 3: return ci::photoLike(W, H);
    default: return ci::gridImage(W, H, 28);
  }
}

cv::Mat compose(const cv::Mat& chart, const ci::Mat33& K, const ci::Dist& D,
                double alpha, const std::string& presetName, bool caption) {
  const ci::Mat33 KIdeal = ci::makeK(W / 2.0, W / 2.0, W / 2.0, H / 2.0);
  const cv::Mat cam = ci::distortImage(chart, K, D, KIdeal);
  ci::Mat33 KNew;
  cv::Rect roi;
  const cv::Mat und = ci::undistortImage(cam, K, D, alpha, &KNew, &roi);

  cv::Mat camAnnot = cam.clone();
  ci::drawCrosshair(camAnnot, K);

  cv::Mat undAnnot = und.clone();
  if (roi.width > 0 && roi.height > 0)
    cv::rectangle(undAnnot, roi, cv::Scalar(90, 230, 90), 1);

  const cv::Mat top = ci::hstackLabeled(
      {chart, camAnnot, undAnnot},
      {"1) ideal pinhole  (the ground truth)",
       "2) camera view  =  K and D applied",
       ci::fmt("3) undistorted  alpha=%.1f  (green = valid ROI)", alpha)});

  cv::Mat hud(H, W, CV_8UC3, cv::Scalar(24, 24, 28));
  std::vector<std::string> lines = ci::kdHudLines(K, D, W, H);
  lines.push_back("");
  lines.push_back("preset: " + presetName);
  lines.push_back(ci::fmt("K_new: fx %6.1f  fy %6.1f", KNew(0, 0), KNew(1, 1)));
  lines.push_back(ci::fmt("       cx %6.1f  cy %6.1f", KNew(0, 2), KNew(1, 2)));
  lines.push_back(ci::fmt("valid ROI: %dx%d of %dx%d", roi.width, roi.height, W, H));
  ci::drawTextBlock(hud, lines, {12, 26}, 0.42, ci::colors::kWhite, 19, false);

  const cv::Mat bottom = ci::hstackLabeled(
      {ci::plotDistortionProfile(D, W, H), ci::plotDistortionField(K, D, W, H, 34),
       hud},
      {"4) radial profile  r' vs r", "5) displacement field (D only)",
       "6) the numbers"});

  const int width = std::max(top.cols, bottom.cols);
  cv::Mat pad(10, width, CV_8UC3, cv::Scalar(24, 24, 28));
  std::vector<cv::Mat> parts = {top, pad, bottom};
  if (caption) {
    cv::Mat bar(48, width, CV_8UC3, cv::Scalar(16, 16, 20));
    ci::drawTextBlock(bar, kCaption, {14, 22}, 0.44, ci::colors::kWhite, 19, false);
    parts.push_back(bar);
  }
  return ci::vstack(parts);
}

}  // namespace

int main(int argc, char** argv) {
  std::string out = "data/generated/app2d.png";
  int presetIdx = 1, chartIdx = 0;
  double alpha = 0.0;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
    else if (!std::strcmp(argv[i], "--preset") && i + 1 < argc) presetIdx = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--chart") && i + 1 < argc) chartIdx = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--alpha") && i + 1 < argc) alpha = std::atof(argv[++i]);
  }
  const auto& presets = ci::presets();
  presetIdx = std::max(0, std::min<int>(presetIdx, int(presets.size()) - 1));

  ci::Mat33 K;
  ci::Dist D;
  ci::presetModel(presets[presetIdx], W, H, &K, &D);
  const cv::Mat frame = compose(makeChart(chartIdx), K, D, alpha,
                                presets[presetIdx].name, true);
  return ci::writeImage(out, frame) ? 0 : 1;
}
