// The reference checks. Ground truth comes from OpenCV wherever OpenCV has an
// equivalent, so passing here means passing against the real thing.
#include "checks.hpp"

#include <opencv2/calib3d.hpp>

#include <cmath>
#include <sstream>

#include "camintrinsics/util.hpp"

namespace exlab {
namespace {

const cv::Matx33d kKRef(812.0, 0.0, 639.1, 0.0, 809.5, 361.4, 0.0, 0.0, 1.0);
const cv::Vec<double, 5> kDRef(-0.27, 0.10, 0.0012, -0.0009, 0.004);

std::vector<cv::Point3d> pts3() {
  cv::RNG rng(20260829);
  std::vector<cv::Point3d> out;
  for (int i = 0; i < 40; ++i)
    out.emplace_back(rng.uniform(-2.5, 2.5), rng.uniform(-2.0, 2.0),
                     rng.uniform(1.5, 12.0));
  return out;
}

std::vector<cv::Point2d> xyRef() {
  cv::RNG rng(12345);
  std::vector<cv::Point2d> out;
  for (int i = 0; i < 60; ++i)
    out.emplace_back(rng.uniform(-0.8, 0.8), rng.uniform(-0.6, 0.6));
  return out;
}

void require(bool ok, const std::string& msg) {
  if (!ok) throw CheckFailed(msg);
}

void close(double got, double want, double tol, const std::string& what) {
  const double err = std::abs(got - want);
  require(err <= tol, ci::fmt("%s: got %.6f, expected %.6f (off by %.3e, "
                              "tolerance %.1e)",
                              what.c_str(), got, want, err, tol));
}

void closeMat(const cv::Matx33d& got, const cv::Matx33d& want, double tol,
              const std::string& what) {
  double err = 0.0;
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) err = std::max(err, std::abs(got(r, c) - want(r, c)));
  require(err <= tol, ci::fmt("%s: matrix off by %.3e (tolerance %.1e)",
                              what.c_str(), err, tol));
}

void closePts(const std::vector<cv::Point2d>& got,
              const std::vector<cv::Point2d>& want, double tol,
              const std::string& what) {
  require(got.size() == want.size(),
          ci::fmt("%s: expected %zu points, got %zu", what.c_str(), want.size(),
                  got.size()));
  double err = 0.0;
  for (size_t i = 0; i < got.size(); ++i)
    err = std::max(err, std::max(std::abs(got[i].x - want[i].x),
                                 std::abs(got[i].y - want[i].y)));
  require(err <= tol, ci::fmt("%s: off by %.3e (tolerance %.1e)", what.c_str(),
                              err, tol));
}

/// OpenCV ground truth for projecting camera-frame points.
std::vector<cv::Point2d> ocvProject(const std::vector<cv::Point3d>& pts,
                                    const cv::Matx33d& K,
                                    const cv::Vec<double, 5>& D) {
  std::vector<cv::Point2d> out;
  cv::projectPoints(pts, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), cv::Mat(K),
                    cv::Mat(D), out);
  return out;
}

// -------------------------------------------------------------------------
void check01(const Impl& m) {
  closeMat(m.build_K(812.0, 809.5, 639.1, 361.4, 0.0), kKRef, 1e-12, "K");
  const cv::Matx33d Ks = m.build_K(800, 800, 320, 240, 1.5);
  close(Ks(0, 1), 1.5, 1e-12, "skew belongs at K(0,1)");
  close(Ks(1, 0), 0.0, 1e-12, "K(1,0) must stay 0");
}

void check02(const Impl& m) {
  const auto pts = pts3();
  closePts(m.project_pinhole(pts, kKRef),
           ocvProject(pts, kKRef, cv::Vec<double, 5>::all(0)), 1e-9,
           "pinhole projection");
  const std::vector<cv::Point3d> behind = {{1, 1, -2}, {0, 0, 0}};
  for (const cv::Point2d& p : m.project_pinhole(behind, kKRef))
    require(std::isnan(p.x) && std::isnan(p.y),
            "points with Z <= 0 must project to NaN");
}

void check03(const Impl& m) {
  const auto xy = xyRef();
  std::vector<cv::Point3d> rays;
  for (const auto& q : xy) rays.emplace_back(q.x, q.y, 1.0);
  // With K = I, cv::projectPoints returns distorted normalized coordinates.
  closePts(m.distort(xy, kDRef[0], kDRef[1], kDRef[2], kDRef[3], kDRef[4]),
           ocvProject(rays, cv::Matx33d::eye(), kDRef), 1e-9, "distortion");
}

void check04(const Impl& m) {
  const cv::Matx33d cases[2] = {
      kKRef, cv::Matx33d(400.0, 0, 100.0, 0, 400.0, 300.0, 0, 0, 1.0)};
  const int w[2] = {1280, 640}, h[2] = {720, 480};
  for (int i = 0; i < 2; ++i) {
    const cv::Vec2d got = m.fov_degrees(cases[i], w[i], h[i]);
    const double fx = cases[i](0, 0), fy = cases[i](1, 1);
    const double cx = cases[i](0, 2), cy = cases[i](1, 2);
    const double kDegC = 180.0 / CV_PI;
    close(got[0], kDegC * (std::atan2(cx, fx) + std::atan2(w[i] - cx, fx)), 1e-9,
          "horizontal FOV (did you assume cx = width/2?)");
    close(got[1], kDegC * (std::atan2(cy, fy) + std::atan2(h[i] - cy, fy)), 1e-9,
          "vertical FOV");
  }
}

void check05(const Impl& m) {
  cv::Matx33d want = kKRef;
  for (int c = 0; c < 3; ++c) {
    want(0, c) *= 0.5;
    want(1, c) *= 0.25;
  }
  const cv::Matx33d got = m.K_after_resize(kKRef, 0.5, 0.25);
  closeMat(got, want, 1e-12, "K after resize (did you forget cx and cy?)");

  // Behavioural check: the same 3D point must land on the same physical spot.
  const auto pts = pts3();
  const auto a = ocvProject(pts, kKRef, cv::Vec<double, 5>::all(0));
  const auto b = ocvProject(pts, got, cv::Vec<double, 5>::all(0));
  std::vector<cv::Point2d> expect;
  for (const auto& p : a) expect.emplace_back(p.x * 0.5, p.y * 0.25);
  closePts(b, expect, 1e-9, "resized projection");
}

void check06(const Impl& m) {
  cv::Matx33d want = kKRef;
  want(0, 2) -= 120;
  want(1, 2) -= 64;
  const cv::Matx33d got = m.K_after_crop(kKRef, 120, 64);
  closeMat(got, want, 1e-12, "K after crop");
  close(got(0, 0), kKRef(0, 0), 1e-12,
        "cropping must not change fx -- the lens did not change");
}

void check07(const Impl& m) {
  const auto xy = xyRef();
  std::vector<cv::Point3d> rays;
  for (const auto& q : xy) rays.emplace_back(q.x, q.y, 1.0);
  const auto xyd = ocvProject(rays, cv::Matx33d::eye(), kDRef);
  closePts(m.undistort_point(xyd, kDRef[0], kDRef[1], kDRef[2], kDRef[3],
                             kDRef[4], 20),
           xy, 1e-8, "undistortion (does your loop recompute r^2 each pass?)");
}

void check08(const Impl& m) {
  const cv::Matx33d K = m.K_from_hfov(90.0, 1280, 720);
  close(K(0, 0), 640.0, 1e-9, "fx for 90 deg hFOV on 1280 wide");
  close(K(0, 2), 640.0, 1e-9, "cx");
  close(K(1, 2), 360.0, 1e-9, "cy");
  const cv::Matx33d K2 = m.K_from_hfov(60.0, 800, 600);
  const double want = 400.0 / std::tan(30.0 * CV_PI / 180.0);
  close(K2(0, 0), want, 1e-9, "fx for 60 deg hFOV");
  close(K2(1, 1), want, 1e-9, "fy must equal fx for square pixels");
}

void check09(const Impl& m) {
  struct Case {
    double k1, k2, k3, r;
    const char* want;
  };
  const Case cases[] = {{-0.3, 0.0, 0.0, 1.0, "barrel"},
                        {0.3, 0.0, 0.0, 1.0, "pincushion"},
                        {0.0, 0.0, 0.0, 1.0, "none"},
                        {-0.4, 0.25, 0.0, 0.5, "barrel"},
                        {-0.4, 0.25, 0.0, 1.4, "pincushion"}};
  for (const Case& c : cases) {
    const std::string got = m.classify_distortion(c.k1, c.k2, c.k3, c.r);
    require(got == c.want,
            ci::fmt("k1=%.2f k2=%.2f r=%.2f: expected \"%s\", got \"%s\"", c.k1,
                    c.k2, c.r, c.want, got.c_str()));
  }
}

void check10(const Impl& m) {
  cv::Matx33d want = kKRef;
  want(0, 2) -= 200;
  want(1, 2) -= 100;
  for (int c = 0; c < 3; ++c) {
    want(0, c) *= 0.5;
    want(1, c) *= 0.5;
  }
  const cv::Matx33d got = m.pipeline_K(kKRef, 200, 100, 0.5);
  closeMat(got, want, 1e-12,
           "crop-then-resize K (did you apply the resize before the crop?)");

  cv::Matx33d swapped = kKRef;
  for (int c = 0; c < 3; ++c) {
    swapped(0, c) *= 0.5;
    swapped(1, c) *= 0.5;
  }
  swapped(0, 2) -= 200;
  require(std::abs(got(0, 2) - swapped(0, 2)) > 1e-6,
          "that is the resize-then-crop answer; the crop happens first here");
}

const std::vector<Check> kChecks = {
    {1, "build K", check01},
    {2, "pinhole projection", check02},
    {3, "apply distortion", check03},
    {4, "field of view", check04},
    {5, "K after resize", check05},
    {6, "K after crop", check06},
    {7, "undistort a point", check07},
    {8, "K from FOV", check08},
    {9, "classify distortion", check09},
    {10, "crop-then-resize pipeline", check10},
};

}  // namespace

const std::vector<Check>& checks() { return kChecks; }

}  // namespace exlab
