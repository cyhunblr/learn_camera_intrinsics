#include "camintrinsics/presets.hpp"

#include "camintrinsics/util.hpp"

namespace ci {

const std::vector<Preset>& presets() {
  static const std::vector<Preset> kPresets = {
      {"ideal pinhole", 1.00, 0.00, 0.00, 0.0, 0.0, 0.0},
      {"webcam (mild barrel)", 0.85, -0.18, 0.05, 0.001, -0.001, 0.0},
      {"action cam (strong barrel)", 0.45, -0.35, 0.12, 0.0, 0.0, 0.0},
      {"tele (pincushion)", 1.80, 0.22, -0.06, 0.0, 0.0, 0.0},
      {"decentred lens", 0.90, -0.12, 0.02, 0.012, 0.009, 0.0},
  };
  return kPresets;
}

void presetModel(const Preset& p, int width, int height, Mat33* K, Dist* D) {
  *K = makeK(p.f * width, p.f * width, width / 2.0, height / 2.0);
  *D = makeD(p.k1, p.k2, p.p1, p.p2, p.k3);
}

std::vector<std::string> kdHudLines(const Mat33& K, const Dist& D, int width,
                                    int height) {
  const KParams p = splitK(K);
  const cv::Vec3d f = fovDeg(K, width, height);
  std::vector<std::string> lines = {
      fmt("K = [ %7.1f %6.2f %7.1f ]", p.fx, p.skew, p.cx),
      fmt("    [ %7.1f %6.1f %7.1f ]", 0.0, p.fy, p.cy),
      fmt("    [ %7.1f %6.1f %7.1f ]", 0.0, 0.0, 1.0),
      fmt("D  k1 %+.3f   k2 %+.3f   k3 %+.3f", D[0], D[1], D[4]),
      fmt("   p1 %+.4f  p2 %+.4f", D[2], D[3]),
      fmt("FOV  h %5.1f  v %5.1f  d %5.1f deg", f[0], f[1], f[2]),
      fmt("fx/fy %5.3f   principal offset (%+.0f, %+.0f) px", p.fx / p.fy,
          p.cx - width / 2.0, p.cy - height / 2.0),
  };
  if (!isInvertibleOverImage(K, D, width, height)) {
    lines.push_back(fmt("!! corners exceed r'max=%.2f:", maxDistortedRadius(D)));
    lines.push_back("   undistort is undefined there");
  }
  return lines;
}

}  // namespace ci
