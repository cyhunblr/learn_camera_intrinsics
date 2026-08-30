// Exercises - the C++ half of python/exercises/.
//
//   cd cpp && cmake -B build && cmake --build build -j
//   ./build/bin/check_exercises              # scorecard
//   ./build/bin/check_exercises 3            # just exercise 3
//   ./build/bin/check_exercises --solutions  # verify the reference answers
//
// Rules of the game
//   * Write the maths yourself. Do not call cv::projectPoints, cv::undistort
//     or anything from namespace ci -- that would just be copying the answer.
//   * cv::Matx33d, cv::Point2d and std::vector are fine, of course.
//   * Worked answers live in solutions.cpp. Look after you have tried.
//
// Conventions (identical to the rest of the repo)
//   * camera looks down +Z, +X right, +Y down
//   * xy = normalized image coordinates, uv = pixels
#pragma once

#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace exlab {

/// Thrown by every unfinished stub so the checker can tell "not done yet"
/// apart from "done and wrong".
struct NotImplemented : std::logic_error {
  NotImplemented() : std::logic_error("not implemented yet") {}
};

/// One complete set of answers: the student's, or the reference.
struct Impl {
  const char* name;

  /// 1. Assemble K = [fx s cx; 0 fy cy; 0 0 1].
  cv::Matx33d (*build_K)(double fx, double fy, double cx, double cy,
                         double skew);

  /// 2. Project camera-frame points to pixels, no distortion. Points with
  ///    Z <= 0 are behind the camera and must come back as NaN.
  std::vector<cv::Point2d> (*project_pinhole)(
      const std::vector<cv::Point3d>& pointsCam, const cv::Matx33d& K);

  /// 3. Apply the OpenCV plumb-bob model in normalized coordinates:
  ///      radial = 1 + k1 r^2 + k2 r^4 + k3 r^6
  ///      x' = x*radial + 2 p1 x y + p2 (r^2 + 2 x^2)
  ///      y' = y*radial + p1 (r^2 + 2 y^2) + 2 p2 x y
  std::vector<cv::Point2d> (*distort)(const std::vector<cv::Point2d>& xy,
                                      double k1, double k2, double p1,
                                      double p2, double k3);

  /// 4. (hfov, vfov) in degrees. Careful: cx is not necessarily width/2, so
  ///    each axis is the sum of two different half-angles.
  cv::Vec2d (*fov_degrees)(const cv::Matx33d& K, int width, int height);

  /// 5. K of the same camera after the image is resized by (sx, sy).
  cv::Matx33d (*K_after_resize)(const cv::Matx33d& K, double sx, double sy);

  /// 6. K after cropping so the new image starts at pixel (x0, y0).
  cv::Matx33d (*K_after_crop)(const cv::Matx33d& K, double x0, double y0);

  /// 7. Invert exercise 3. No closed form exists: iterate from x = xd,
  ///    removing the tangential part and dividing out the radial gain,
  ///    recomputing r^2 from the current estimate each pass.
  std::vector<cv::Point2d> (*undistort_point)(
      const std::vector<cv::Point2d>& xyd, double k1, double k2, double p1,
      double p2, double k3, int iters);

  /// 8. Centred, square-pixel K from a horizontal field of view.
  cv::Matx33d (*K_from_hfov)(double hfovDeg, int width, int height);

  /// 9. "barrel", "pincushion" or "none" at radius r, tolerance 1e-9. The
  ///    answer may depend on r: a moustache lens is both.
  std::string (*classify_distortion)(double k1, double k2, double k3, double r);

  /// 10. A frame is cropped, then the crop is resized. Given the K calibrated
  ///     for the full image, return the K valid for the final one.
  cv::Matx33d (*pipeline_K)(const cv::Matx33d& K, double cropX0, double cropY0,
                            double scale);
};

const Impl& exercises();  ///< your implementations (exercises.cpp)
const Impl& solutions();  ///< the reference answers (solutions.cpp)

}  // namespace exlab
