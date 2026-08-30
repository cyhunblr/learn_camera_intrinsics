// Example 4 - the bookkeeping trap: resizing, cropping and ROIs.
//
//   ./build/bin/04_resize_crop_roi
//
// Calibration gives you a K that belongs to *one specific image size*. The
// moment a preprocessing step resizes, crops, pads or letterboxes the frame,
// that K is wrong -- and nothing crashes. Your reprojection is just quietly off
// by a few pixels, which is exactly enough to ruin a 3D detection.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "camintrinsics/intrinsics.hpp"

namespace {

constexpr int W = 1920, H = 1080;
const ci::Mat33 K = ci::makeK(1450.0, 1452.0, 962.4, 531.7);
const ci::Dist D = ci::makeD(-0.21, 0.06, 0.0004, -0.0002);
const std::vector<cv::Point3d> kPts = {
    {0.0, 0.0, 5.0}, {1.8, -1.1, 6.5}, {-2.4, 0.9, 4.0}};

double maxDeviation(const std::vector<cv::Point2d>& a,
                    const std::vector<cv::Point2d>& b) {
  double err = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    err = std::max(err, std::max(std::abs(a[i].x - b[i].x),
                                 std::abs(a[i].y - b[i].y)));
  return err;
}

void report(const char* name, const ci::Mat33& K2,
            const std::vector<cv::Point2d>& expect) {
  const auto got = ci::projectPoints(kPts, K2, D);
  const double err = maxDeviation(got, expect);
  const ci::KParams p = ci::splitK(K2);
  std::printf("\n%s %s\n", err < 1e-6 ? "OK  " : "FAIL", name);
  std::printf("     K -> fx %8.2f  fy %8.2f  cx %8.2f  cy %8.2f\n", p.fx, p.fy,
              p.cx, p.cy);
  std::printf("     max deviation from the expected physical location: %.3e px\n",
              err);
}

void rule(const char* title) {
  std::printf("\n%s\n%s\n%s\n", std::string(74, '=').c_str(), title,
              std::string(74, '=').c_str());
}

}  // namespace

int main() {
  std::printf("calibrated at %dx%d:  fx %.1f  fy %.1f  cx %.1f  cy %.1f\n\n", W,
              H, K(0, 0), K(1, 1), K(0, 2), K(1, 2));
  const auto base = ci::projectPoints(kPts, K, D);
  for (size_t i = 0; i < kPts.size(); ++i)
    std::printf("   world (%.1f, %.1f, %.1f) -> pixel (%8.2f, %8.2f)\n",
                kPts[i].x, kPts[i].y, kPts[i].z, base[i].x, base[i].y);

  // --- 1. resize ----------------------------------------------------------
  rule("1) resize to half.  Everything in the first two rows of K scales.");
  std::vector<cv::Point2d> half;
  for (const auto& p : base) half.emplace_back(p.x * 0.5, p.y * 0.5);
  report("half resolution, K scaled correctly", ci::scaleK(K, 0.5, 0.5), half);

  std::printf("\n   the wrong way -- scale the focal lengths but forget cx, cy:\n");
  ci::Mat33 KBad = K;
  KBad(0, 0) *= 0.5;
  KBad(1, 1) *= 0.5;
  std::printf("     error: up to %.1f px at half resolution\n",
              maxDeviation(ci::projectPoints(kPts, KBad, D), half));
  std::printf("     -> a silent, constant offset. It survives every downstream "
              "stage,\n        and it looks exactly like a small extrinsic "
              "calibration error.\n");

  std::printf("\n   non-uniform resize (1920x1080 -> 640x480 without keeping "
              "the aspect):\n");
  const double sx = 640.0 / W, sy = 480.0 / H;
  std::vector<cv::Point2d> stretched;
  for (const auto& p : base) stretched.emplace_back(p.x * sx, p.y * sy);
  const ci::Mat33 KStretch = ci::scaleK(K, sx, sy);
  report("letterbox-free stretch", KStretch, stretched);
  std::printf("     fx/fy is now %.3f: the stretch faked non-square pixels.\n",
              KStretch(0, 0) / KStretch(1, 1));

  // --- 2. crop ------------------------------------------------------------
  rule("2) crop.  Focal lengths do NOT change -- the lens did not change.");
  const double x0 = 320, y0 = 180;
  std::vector<cv::Point2d> cropped;
  for (const auto& p : base) cropped.emplace_back(p.x - x0, p.y - y0);
  report("crop starting at (320, 180)", ci::cropK(K, x0, y0), cropped);
  std::printf("     Cropping narrows the field of view without touching fx or "
              "fy.\n     Digital zoom = crop + resize, so it does change fx: "
              "crop then scale.\n");
  const ci::Mat33 KZoom = ci::scaleK(ci::cropK(K, x0, y0), W / (W - 2 * x0),
                                     W / (W - 2 * x0));
  std::printf("\n     2x digital zoom back to %d wide: fx %.1f (was %.1f)\n", W,
              KZoom(0, 0), K(0, 0));

  // --- 3. padding ---------------------------------------------------------
  rule("3) padding and letterboxing (the classic ML-preprocessing bug)");
  const double padL = 64, padT = 40;
  std::vector<cv::Point2d> padded;
  for (const auto& p : base) padded.emplace_back(p.x + padL, p.y + padT);
  report("pad 64 left, 40 top", ci::cropK(K, -padL, -padT), padded);
  std::printf("     A letterbox resize is 'scale, then pad'. Apply scaleK "
              "first,\n     then cropK with negative offsets, in that order.\n");

  // --- 4. flip ------------------------------------------------------------
  rule("4) horizontal flip (data augmentation that changes the camera model)");
  const ci::Mat33 KFlip = ci::flipK(K, W, H, true, false);
  const ci::Dist DFlip = ci::flipD(D, true, false);
  std::vector<cv::Point3d> mirrored;
  for (const auto& p : kPts) mirrored.emplace_back(-p.x, p.y, p.z);
  std::vector<cv::Point2d> expect;
  for (const auto& p : base) expect.emplace_back((W - 1) - p.x, p.y);
  std::printf("     cx %.1f -> %.1f      p2 %+.4f -> %+.4f\n", K(0, 2),
              KFlip(0, 2), D[3], DFlip[3]);
  std::printf("     K and D both mirrored    : %.3e px  <- exact\n",
              maxDeviation(ci::projectPoints(mirrored, KFlip, DFlip), expect));
  std::printf("     K mirrored, D left as-is : %.3e px  <- silent bias\n",
              maxDeviation(ci::projectPoints(mirrored, KFlip, D), expect));
  std::printf("     The radial terms are even functions, so they survive a "
              "mirror. The\n     tangential ones are not: a horizontal flip "
              "needs p2 -> -p2.\n     Mirroring also flips handedness: if you "
              "flip images for augmentation\n     you must mirror the "
              "extrinsics too, or your poses become left-handed.\n");

  // --- 5. what does not change -------------------------------------------
  rule("5) what does NOT change: D");
  std::printf("     D lives in normalized coordinates, before K is applied, so "
              "resizing\n     and cropping leave every coefficient untouched. "
              "If someone hands you\n     'the D for 640x480', they are "
              "confused -- D has no resolution.\n\n     (Undistorting, however,"
              " DOES change K -- see example 05.)\n");
  return 0;
}
