// Render the 3D figure: the frustum K describes, beside the image it produces.
//
//   ./build/bin/render_3d [--out out.png] [--preset N] [--no-distortion]
//
// This is a *renderer*, not an app. Interactive exploration lives in the web
// viewer (https://cyhunblr.github.io/learn_camera_intrinsics/).
// Every pixel here still goes through the projection
// function by hand -- no OpenGL -- so a bent line is D bending it.
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "camintrinsics/presets.hpp"
#include "camintrinsics/renderer.hpp"
#include "camintrinsics/util.hpp"
#include "camintrinsics/scene.hpp"

namespace {

constexpr int CAM_W = 520, CAM_H = 390;      // the camera we are studying
constexpr int WORLD_W = 640, WORLD_H = 480;  // the third-person viewport
const cv::Point3d kTarget(0.0, 0.0, 4.0);
const cv::Point3d kViewTarget(0.0, 0.0, 2.0);

const std::vector<std::string> kCaption = {
    "fx,fy -> frustum width (FOV).  cx,cy -> the frustum shears off the blue optical axis.",
    "k1,k2,k3 bend lines radially; p1,p2 tilt the whole image (decentred lens)."};

cv::Mat compose(const ci::Mat33& K, const ci::Dist& D, const ci::Pose& camPose,
                const ci::Pose& viewPose, bool useD, bool showGrid,
                const std::string& presetName, bool caption) {
  const std::vector<ci::Polyline> scene = ci::defaultScene(showGrid);

  // Left: third-person view, always an ideal pinhole so it stays readable.
  cv::Mat world(WORLD_H, WORLD_W, CV_8UC3, cv::Scalar(22, 22, 26));
  const ci::Mat33 KView = ci::makeK(WORLD_W * 0.9, WORLD_W * 0.9,
                                    WORLD_W / 2.0, WORLD_H / 2.0);
  std::vector<ci::Polyline> overlay = scene;
  ci::appendFrustum(overlay, K, CAM_W, CAM_H, 0.10, 2.0, camPose.R, camPose.t);
  ci::appendCameraGizmo(overlay, camPose.R, camPose.t);
  ci::render(world, overlay, viewPose, KView, ci::makeD());

  // Right: the camera's own image, K and D doing all the work.
  cv::Mat cam(CAM_H, CAM_W, CV_8UC3, cv::Scalar(22, 22, 26));
  ci::render(cam, scene, camPose, K, useD ? D : ci::makeD());
  ci::drawCrosshair(cam, K);
  cv::rectangle(cam, {0, 0}, {CAM_W - 1, CAM_H - 1}, cv::Scalar(70, 70, 80), 1);

  const cv::Mat top = ci::hstackLabeled(
      {world, cam},
      {"world view  -  cyan = the frustum K describes",
       ci::fmt("camera view  %dx%d  -  distortion %s", CAM_W, CAM_H,
               useD ? "ON" : "OFF")});

  cv::Mat hud(150, top.cols, CV_8UC3, cv::Scalar(18, 18, 22));
  ci::drawTextBlock(hud, ci::kdHudLines(K, D, CAM_W, CAM_H), {14, 24}, 0.44,
                    ci::colors::kWhite, 19, false);
  const cv::Point3d C = camPose.center();
  const std::vector<std::string> right = {
      "preset: " + presetName,
      ci::fmt("camera centre  (%+.2f, %+.2f, %+.2f)", C.x, C.y, C.z),
      "the blue axis is the optical axis;",
      "the frustum only stays centred on it",
      ci::fmt("while cx = %d and cy = %d.", CAM_W / 2, CAM_H / 2),
      "distortion is applied AFTER the",
      "perspective divide, BEFORE K."};
  ci::drawTextBlock(hud, right, {top.cols / 2 + 20, 24}, 0.44,
                    ci::colors::kWhite, 19, false);

  std::vector<cv::Mat> parts = {top, hud};
  if (caption) {
    cv::Mat bar(48, top.cols, CV_8UC3, cv::Scalar(14, 14, 18));
    ci::drawTextBlock(bar, kCaption, {14, 22}, 0.44, ci::colors::kWhite, 19, false);
    parts.push_back(bar);
  }
  return ci::vstack(parts);
}

}  // namespace

int main(int argc, char** argv) {
  std::string out = "data/generated/app3d.png";
  int presetIdx = 1;
  bool useD = true;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
    else if (!std::strcmp(argv[i], "--preset") && i + 1 < argc) presetIdx = std::atoi(argv[++i]);
    else if (!std::strcmp(argv[i], "--no-distortion")) useD = false;
  }
  const auto& presets = ci::presets();
  presetIdx = std::max(0, std::min<int>(presetIdx, int(presets.size()) - 1));

  ci::Mat33 K;
  ci::Dist D;
  ci::presetModel(presets[presetIdx], CAM_W, CAM_H, &K, &D);
  const cv::Mat frame = compose(K, D, ci::orbitPose(kTarget, 2.6, 0.0, 6.0),
                                ci::orbitPose(kViewTarget, 8.0, 38.0, 22.0),
                                useD, true, presets[presetIdx].name, true);
  return ci::writeImage(out, frame) ? 0 : 1;
}
