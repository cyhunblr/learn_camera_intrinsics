// Example 3 - reading a D vector: what each coefficient looks like.
//
//   ./build/bin/03_distortion_anatomy [out.png]
//
// Six lenses, one chart, one figure. Each column changes exactly one
// coefficient so you can learn the visual signature of k1, k2, k3, p1 and p2
// well enough to guess a D vector by eye.
#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <string>
#include <vector>

#include "camintrinsics/patterns.hpp"
#include "camintrinsics/plots.hpp"
#include "camintrinsics/renderer.hpp"
#include "camintrinsics/util.hpp"

namespace {
constexpr int W = 300, H = 240;
struct Case { const char* name; ci::Dist D; };
}  // namespace

int main(int argc, char** argv) {
  const std::vector<Case> cases = {
      {"no distortion", ci::makeD()},
      {"k1 = -0.35  barrel", ci::makeD(-0.35)},
      {"k1 = +0.35  pincushion", ci::makeD(0.35)},
      {"k1=-0.4 k2=+0.25  moustache", ci::makeD(-0.40, 0.25)},
      {"p1 = +0.05  tangential", ci::makeD(0, 0, 0.05)},
      {"p2 = +0.05  tangential", ci::makeD(0, 0, 0, 0.05)}};

  const cv::Mat chart = ci::gridImage(W, H, 24);
  const ci::Mat33 K = ci::makeK(W * 0.55, W * 0.55, W / 2.0, H / 2.0);
  const ci::Mat33 KIdeal = ci::makeK(W / 2.0, W / 2.0, W / 2.0, H / 2.0);

  std::printf("r' sampled along the +x axis (so p1, whose term is 2*p1*x*y,\n"
              "contributes nothing there -- tangential effects are not "
              "radial):\n\n");
  std::printf("%-32s %9s %9s %9s %11s\n", "lens", "r=0.3", "r=0.6", "r=1.0",
              "folds at r");
  std::printf("%s\n", std::string(74, '-').c_str());
  std::vector<cv::Mat> images;
  std::vector<std::string> labels;
  for (const Case& c : cases) {
    std::printf("%-32s", c.name);
    for (double r : {0.3, 0.6, 1.0})
      std::printf(" %9.4f", ci::distortNormalized({r, 0.0}, c.D).x);
    std::printf(" %11.2f\n", ci::maxValidRadius(c.D));
    images.push_back(ci::distortImage(chart, K, c.D, KIdeal));
    labels.push_back(c.name);
  }

  std::printf("\nhow to read a D vector by eye\n");
  std::printf("  k1 < 0            barrel: the frame edges bow outward, "
              "corners pull in.\n");
  std::printf("  k1 > 0            pincushion: edges bow inward.\n");
  std::printf("  k1 < 0, k2 > 0    'moustache': barrel near the centre, "
              "pincushion at\n                    the edge. Very common in "
              "wide zooms; a single k1\n                    cannot model it, "
              "which is why k2 exists.\n");
  std::printf("  p1, p2 != 0       the lens is not centred on the sensor. "
              "Real values are\n                    tiny (1e-4 .. 1e-3); "
              "anything near 0.01 means your\n                    calibration "
              "is fitting noise.\n");
  std::printf("  k3                only matters for very wide lenses; on a "
              "normal lens it\n                    is poorly constrained and "
              "often best fixed to 0.\n");

  const ci::Dist fold = ci::makeD(-0.5);
  std::printf("\n  invertibility: undistortion only exists below the radius "
              "where\n  r'(r) stops increasing. For k1 = -0.50 the curve peaks "
              "at\n  r = %.3f, so no point with r' above %.3f can ever\n  be "
              "undistorted -- cv::undistort returns silent garbage there.\n",
              ci::maxValidRadius(fold), ci::maxDistortedRadius(fold));

  if (argc > 1) {
    std::vector<cv::Mat> rows;
    for (size_t i = 0; i < cases.size(); i += 3) {
      std::vector<cv::Mat> strip(images.begin() + i, images.begin() + i + 3);
      std::vector<std::string> stripLabels(labels.begin() + i,
                                           labels.begin() + i + 3);
      rows.push_back(ci::hstackLabeled(strip, stripLabels));
      std::vector<cv::Mat> plots;
      for (size_t j = i; j < i + 3; ++j) {
        cv::Mat p;
        cv::resize(ci::plotDistortionProfile(cases[j].D, 300, 200), p, {W, 190});
        plots.push_back(p);
      }
      rows.push_back(ci::hstackLabeled(plots, {"r' vs r", "r' vs r", "r' vs r"}));
    }
    if (!ci::writeImage(argv[1], ci::vstack(rows))) return 1;
  } else {
    std::printf("\n(pass an output path to write the figure)\n");
  }
  return 0;
}
