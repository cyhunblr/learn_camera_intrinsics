// Example 2 - what each entry of K actually does.
//
//   ./build/bin/02_k_matrix_anatomy [out.png]
//
// Renders the same 3D scene four times, changing exactly one thing each time.
// The point to internalise: K contains no rotation and no translation. It
// cannot move the camera. It only decides how the already-projected ray lands
// on the sensor grid.

#include <cstdio>
#include <string>
#include <vector>

#include "camintrinsics/renderer.hpp"
#include "camintrinsics/util.hpp"
#include "camintrinsics/scene.hpp"

namespace {
constexpr int W = 420, H = 320;
constexpr double kBaseF = 320.0;

cv::Mat shot(const ci::Mat33& K) {
  cv::Mat img(H, W, CV_8UC3, cv::Scalar(24, 24, 28));
  ci::render(img, ci::defaultScene(), ci::orbitPose({0, 0, 4.0}, 4.4, 14.0, 10.0),
             K, ci::makeD());
  ci::drawCrosshair(img, K);
  const cv::Vec3d f = ci::fovDeg(K, W, H);
  ci::drawTextBlock(img,
                    {ci::fmt("fx %.0f  fy %.0f", K(0, 0), K(1, 1)),
                     ci::fmt("cx %.0f  cy %.0f  skew %.0f", K(0, 2), K(1, 2),
                             K(0, 1)),
                     ci::fmt("FOV %.1f x %.1f deg", f[0], f[1])},
                    {10, 20}, 0.42, ci::colors::kWhite, 16);
  return img;
}
}  // namespace

int main(int argc, char** argv) {
  struct Variant { ci::Mat33 K; const char* caption; };
  const std::vector<Variant> variants = {
      {ci::makeK(kBaseF, kBaseF, W / 2.0, H / 2.0), "baseline"},
      {ci::makeK(kBaseF * 2, kBaseF * 2, W / 2.0, H / 2.0),
       "fx,fy x2  ->  zoom in, FOV halves"},
      {ci::makeK(kBaseF, kBaseF * 1.7, W / 2.0, H / 2.0),
       "fy x1.7  ->  non-square pixels"},
      {ci::makeK(kBaseF, kBaseF, W / 2.0 - 120, H / 2.0 + 60),
       "cx,cy moved -> image shifts, camera does not"}};

  std::printf("%-46s %7s %7s %7s\n", "variant", "hFOV", "vFOV", "dFOV");
  for (const Variant& v : variants) {
    const cv::Vec3d f = ci::fovDeg(v.K, W, H);
    std::printf("%-46s %7.2f %7.2f %7.2f\n", v.caption, f[0], f[1], f[2]);
  }

  std::printf("\nthings worth noticing\n");
  std::printf("  * fx and fy are focal lengths measured IN PIXELS, so they "
              "change\n    when you resize the image even though the lens did "
              "not change.\n");
  std::printf("  * fx != fy only means the pixels are not square (or someone\n"
              "    stretched the image). Modern sensors give fx/fy within ~1%%.\n");
  std::printf("  * moving cx,cy slides the image; it does NOT rotate the "
              "camera.\n    A rotation changes what is visible at infinity, a "
              "cx shift does not.\n");
  std::printf("  * skew is the shear between the sensor axes. It is 0 for "
              "every\n    digital sensor you will meet, and cv::projectPoints "
              "ignores it\n    entirely -- see docs/course/02_the_K_matrix.md.\n");

  if (argc > 1) {
    const cv::Mat row1 = ci::hstackLabeled({shot(variants[0].K), shot(variants[1].K)},
                                           {variants[0].caption, variants[1].caption});
    const cv::Mat row2 = ci::hstackLabeled({shot(variants[2].K), shot(variants[3].K)},
                                           {variants[2].caption, variants[3].caption});
    if (!ci::writeImage(argv[1], ci::vstack({row1, row2}))) return 1;
  } else {
    std::printf("\n(pass an output path to write the figure)\n");
  }
  return 0;
}
