// Print reference values for scripts/check_parity.sh. See that script.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "camintrinsics/intrinsics.hpp"

namespace {
// One formatting rule shared with the Python and JavaScript dumpers.
void f(double v) { std::isnan(v) ? std::printf("nan") : std::printf("%.9f", v); }
}  // namespace

int main() {
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.find_first_not_of(" \t") == std::string::npos) continue;
    std::istringstream is(line);
    double fx, fy, cx, cy, k1, k2, p1, p2, k3, w, h;
    is >> fx >> fy >> cx >> cy >> k1 >> k2 >> p1 >> p2 >> k3 >> w >> h;
    const auto K = ci::makeK(fx, fy, cx, cy);
    const auto D = ci::makeD(k1, k2, p1, p2, k3);
    const int W = int(w), H = int(h);
    std::printf("case %s\n", line.c_str());
    for (const cv::Point3d& P : {cv::Point3d(0.9, -0.4, 3.0),
                                 cv::Point3d(-1.2, 0.7, 5.0),
                                 cv::Point3d(2.0, 1.5, 2.2)}) {
      const cv::Point2d q = ci::projectPoint(P, K, D);
      std::printf("  project "); f(q.x); std::printf(" "); f(q.y); std::printf("\n");
    }
    const cv::Vec3d v = ci::fovDeg(K, W, H);
    std::printf("  fov "); f(v[0]); std::printf(" "); f(v[1]); std::printf(" "); f(v[2]); std::printf("\n");
    std::printf("  rmax "); f(ci::maxDistortedRadius(D)); std::printf("\n");
    for (const cv::Point2d& uv : {cv::Point2d(0, 0), cv::Point2d(w / 2, 0),
                                  cv::Point2d(w, h), cv::Point2d(w * 1.5, h * 1.5)}) {
      const cv::Point2d g = ci::undistortNormalized(ci::pixelToNormalized(uv, K), D);
      std::printf("  undistort "); f(g.x); std::printf(" "); f(g.y); std::printf("\n");
    }
    for (double a : {0.0, 0.5, 1.0}) {
      cv::Rect r;
      ci::Mat33 Kn;
      try {
        Kn = ci::optimalNewCameraMatrix(K, D, W, H, a, &r);
      } catch (const std::domain_error&) {
        std::printf("  newK unavailable\n");
        continue;
      }
      std::printf("  newK "); f(Kn(0, 0)); std::printf(" "); f(Kn(1, 1));
      std::printf(" "); f(Kn(0, 2)); std::printf(" "); f(Kn(1, 2));
      std::printf(" %d %d %d %d\n", r.x, r.y, r.width, r.height);
    }
  }
  return 0;
}
