// Example 1 - the pinhole projection, one arithmetic step at a time.
//
//   ./build/bin/01_pinhole_projection
//
// Take one 3D point in the camera frame and walk it all the way to a pixel,
// printing every intermediate value. Then check the answer against
// cv::projectPoints. If you understand this file you understand the forward
// camera model; everything else in the repo is detail on top of it.
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

#include "camintrinsics/intrinsics.hpp"

int main() {
  const ci::Mat33 K = ci::makeK(800.0, 800.0, 640.0, 360.0);
  const ci::Dist D = ci::makeD(-0.28, 0.09, 0.001, -0.0015, 0.0);
  const cv::Point3d P(0.9, -0.4, 3.0);  // 3 m ahead, to the right and up

  std::printf("K = [%7.1f %6.2f %7.1f; 0 %6.1f %7.1f; 0 0 1]\n", K(0, 0),
              K(0, 1), K(0, 2), K(1, 1), K(1, 2));
  std::printf("D = [%.3f %.3f %.4f %.4f %.3f]\n", D[0], D[1], D[2], D[3], D[4]);
  std::printf("P_cam = (%.2f, %.2f, %.2f)\n\n", P.x, P.y, P.z);

  // --- step 1: perspective divide -----------------------------------------
  const double x = P.x / P.z, y = P.y / P.z;
  std::printf("1) perspective divide   x = X/Z, y = Y/Z\n");
  std::printf("   x = %+.3f / %.3f = %+.6f\n", P.x, P.z, x);
  std::printf("   y = %+.3f / %.3f = %+.6f\n", P.y, P.z, y);
  std::printf("   -> 'normalized image coordinates': the ray, with the depth\n"
              "      thrown away. Every point on this ray gives the same (x, y).\n\n");

  // --- step 2: lens distortion --------------------------------------------
  const double r2 = x * x + y * y;
  const double radial = 1 + D[0] * r2 + D[1] * r2 * r2 + D[4] * r2 * r2 * r2;
  const cv::Point2d xd = ci::distortNormalized({x, y}, D);
  std::printf("2) distortion (still unitless, still resolution independent)\n");
  std::printf("   r^2 = %.6f   radial gain = %.6f\n", r2, radial);
  std::printf("   tangential dx = %+.6f\n",
              2 * D[2] * x * y + D[3] * (r2 + 2 * x * x));
  std::printf("   x' = %+.6f   y' = %+.6f\n", xd.x, xd.y);
  std::printf("   the lens moved the point by %.6f in normalized units\n\n",
              std::hypot(xd.x - x, xd.y - y));

  // --- step 3: intrinsics --------------------------------------------------
  const cv::Point2d uv = ci::normalizedToPixel(xd, K);
  std::printf("3) intrinsics: scale by the focal length, shift by the "
              "principal point\n");
  std::printf("   u = fx*x' + s*y' + cx = %.1f*%+.6f + %.1f = %8.3f\n", K(0, 0),
              xd.x, K(0, 2), uv.x);
  std::printf("   v = fy*y' + cy        = %.1f*%+.6f + %.1f = %8.3f\n\n",
              K(1, 1), xd.y, K(1, 2), uv.y);

  // --- cross-check ---------------------------------------------------------
  std::vector<cv::Point2d> ocv;
  cv::projectPoints(std::vector<cv::Point3d>{P}, cv::Vec3d(0, 0, 0),
                    cv::Vec3d(0, 0, 0), cv::Mat(K), cv::Mat(D), ocv);
  std::printf("cross-check\n");
  std::printf("   this repo          : (%.6f, %.6f)\n", uv.x, uv.y);
  std::printf("   cv::projectPoints  : (%.6f, %.6f)\n", ocv[0].x, ocv[0].y);
  std::printf("   difference         : %.2e px\n\n",
              std::max(std::abs(uv.x - ocv[0].x), std::abs(uv.y - ocv[0].y)));

  // --- the lesson about depth ---------------------------------------------
  std::printf("depth is not recoverable from one pixel:\n");
  for (double d : {1.0, 3.0, 10.0, 100.0}) {
    const cv::Point3d q(P.x / P.z * d, P.y / P.z * d, d);
    const cv::Point2d p = ci::projectPoint(q, K, D);
    std::printf("   the same ray at Z = %6.1f m -> pixel (%.3f, %.3f)\n", d, p.x,
                p.y);
  }
  std::printf("\n   Identical pixels. A single camera measures direction, not "
              "distance.\n");
  std::printf("   That is why you need stereo, a known plane, motion, or a "
              "depth sensor.\n\n");

  // --- what breaks ---------------------------------------------------------
  const cv::Point3d behind(0.9, -0.4, -3.0);
  const cv::Point2d mine = ci::projectPoint(behind, K, D);
  std::vector<cv::Point2d> lie;
  cv::projectPoints(std::vector<cv::Point3d>{behind}, cv::Vec3d(0, 0, 0),
                    cv::Vec3d(0, 0, 0), cv::Mat(K), cv::Mat(D), lie);
  std::printf("points behind the camera (Z <= 0) have no projection:\n");
  std::printf("   ci::projectPoint  -> (%.1f, %.1f)  (NaN, on purpose)\n", mine.x,
              mine.y);
  std::printf("   cv::projectPoints -> (%.2f, %.2f)  (a plausible-looking lie: "
              "the 'mirror' point)\n", lie[0].x, lie[0].y);
  std::printf("   Always clip on Z before you trust a projection.\n");
  return 0;
}
