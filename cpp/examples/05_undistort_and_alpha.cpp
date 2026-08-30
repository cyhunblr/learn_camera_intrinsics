// Example 5 - undistortion, the alpha knob, and the K that comes out of it.
//
//   ./build/bin/05_undistort_and_alpha [out.png]
//
// Undistorting is not a filter you apply and forget. It produces a *different
// camera*: a new K, a new field of view, and a region of the output that
// contains no real data. Using the original K on an undistorted image is one of
// the most common bugs in production perception code.
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "camintrinsics/patterns.hpp"
#include "camintrinsics/renderer.hpp"
#include "camintrinsics/util.hpp"

namespace {
constexpr int W = 480, H = 360;
const ci::Mat33 K = ci::makeK(W * 0.52, W * 0.52, W / 2.0 - 6, H / 2.0 + 4);
const ci::Dist D = ci::makeD(-0.34, 0.11, 0.0008, -0.0006);
}  // namespace

int main(int argc, char** argv) {
  const cv::Mat ideal = ci::gridImage(W, H, 30);
  const ci::Mat33 KIdeal = ci::makeK(W / 2.0, W / 2.0, W / 2.0, H / 2.0);
  const cv::Mat raw = ci::distortImage(ideal, K, D, KIdeal);

  std::vector<cv::Mat> tiles = {raw};
  std::vector<std::string> labels = {"what the camera records"};

  auto alphaTable = [&](const ci::Dist& Dc, const char* title, bool collect) {
    std::printf("\n%s\n", title);
    std::printf("  original:  fx %.1f  cx %.1f  cy %.1f   hFOV %.1f deg\n",
                K(0, 0), K(0, 2), K(1, 2), ci::fovDeg(K, W, H)[0]);
    std::printf("  %6s %9s %9s %9s %9s %7s %12s %7s\n", "alpha", "fx_new",
                "fy_new", "cx_new", "cy_new", "hFOV", "valid ROI", "kept");
    std::printf("  %s\n", std::string(76, '-').c_str());
    for (double alpha : {0.0, 0.5, 1.0}) {
      ci::Mat33 KNew;
      cv::Rect roi;
      const cv::Mat out = ci::undistortImage(raw, K, Dc, alpha, &KNew, &roi);
      const double kept = 100.0 * roi.width * roi.height / (W * H);
      std::printf("  %6.1f %9.1f %9.1f %9.1f %9.1f %7.1f %12s %6.1f%%\n", alpha,
                  KNew(0, 0), KNew(1, 1), KNew(0, 2), KNew(1, 2),
                  ci::fovDeg(KNew, W, H)[0],
                  ci::fmt("%dx%d", roi.width, roi.height).c_str(), kept);
      if (collect) {
        cv::Mat vis = out.clone();
        if (roi.width > 0) cv::rectangle(vis, roi, cv::Scalar(90, 230, 90), 2);
        tiles.push_back(vis);
        labels.push_back(ci::fmt("undistorted, alpha = %.1f", alpha));
      }
    }
  };

  alphaTable(D, "BARREL lens  (k1 = -0.34)", true);
  alphaTable(ci::makeD(0.30, -0.05), "PINCUSHION lens  (k1 = +0.30), same K",
             false);

  std::printf("\nreading those tables\n");
  std::printf("  alpha = 0  the output holds only real pixels: OpenCV inscribes "
              "the\n             largest rectangle inside the undistorted image "
              "outline.\n             The reported ROI is then the whole frame.\n");
  std::printf("  alpha = 1  every input pixel is kept: OpenCV takes the "
              "bounding\n             rectangle instead. Black curved borders "
              "appear and the\n             valid ROI shrinks. Outside it, "
              "pixels are invented.\n");
  std::printf("\n  Which way fx_new moves is decided by the distortion, not by "
              "alpha:\n    barrel     the periphery was squeezed, so "
              "undistorting spreads it\n               out -- fx_new DROPS and "
              "the FOV grows, at every alpha.\n    pincushion the periphery was "
              "stretched, so undistorting pulls it\n               back in -- "
              "fx_new RISES and the FOV shrinks.\n  alpha only decides how much "
              "of that new image you keep.\n");

  // --- numerical proof -----------------------------------------------------
  std::printf("\n%s\nnumerical proof: one 3D point, three ways of getting its "
              "pixel\n%s\n", std::string(70, '=').c_str(),
              std::string(70, '=').c_str());
  const std::vector<cv::Point3d> P = {{0.8, -0.35, 4.0}};
  ci::Mat33 KNew;
  cv::Rect roi;
  ci::undistortImage(raw, K, D, 0.0, &KNew, &roi);
  const cv::Point2d rawUV = ci::projectPoints(P, K, D)[0];
  const cv::Point2d good = ci::projectPoints(P, KNew, ci::makeD())[0];
  const cv::Point2d bad = ci::projectPoints(P, K, ci::makeD())[0];
  std::printf("  in the raw (distorted) image, using K and D : (%7.2f, %7.2f)"
              "   <- correct\n", rawUV.x, rawUV.y);
  std::printf("  in the undistorted image, using K_new       : (%7.2f, %7.2f)"
              "   <- correct\n", good.x, good.y);
  std::printf("  in the undistorted image, using the old K   : (%7.2f, %7.2f)"
              "   <- WRONG\n", bad.x, bad.y);
  std::printf("  the bug is worth %.1f px here, and it grows toward the image "
              "corners.\n", cv::norm(good - bad));
  std::printf("\n  Rule of thumb: an undistorted image has D = [0,0,0,0,0] and "
              "K_new.\n  Carry both together, or do not undistort at all and "
              "keep projecting\n  with (K, D) -- which is usually faster "
              "anyway.\n");

  if (argc > 1) {
    const cv::Mat fig = ci::vstack(
        {ci::hstackLabeled({tiles[0], tiles[1]}, {labels[0], labels[1]}),
         ci::hstackLabeled({tiles[2], tiles[3]}, {labels[2], labels[3]})});
    if (!ci::writeImage(argv[1], fig)) return 1;
  }
  return 0;
}
