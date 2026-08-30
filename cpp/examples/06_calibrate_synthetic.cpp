// Example 6 - calibrate a camera whose true K and D you already know.
//
//   ./build/bin/06_calibrate_synthetic
//
// Real calibration has no ground truth: you get numbers and a reprojection
// error, and you have to decide whether to trust them. Here we synthesise the
// whole experiment, so we can measure how far the recovered K and D actually
// are from the truth -- and watch that error change with noise, with the number
// of views, and with how the boards were held.
//
// The third section is the important one: it reproduces the single most common
// real-world calibration mistake and shows that the reprojection error does
// *not* warn you about it.
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "camintrinsics/intrinsics.hpp"
#include "camintrinsics/util.hpp"

namespace {

constexpr int W = 1280, H = 960;
const ci::Mat33 kKTrue = ci::makeK(980.0, 982.0, 641.5, 478.2);
const ci::Dist kDTrue = ci::makeD(-0.24, 0.08, 0.0006, -0.0004, 0.0);
constexpr int kRows = 6, kCols = 9;
constexpr double kSquare = 0.025;

std::vector<cv::Point3f> boardPoints() {
  std::vector<cv::Point3f> p;
  for (int r = 0; r < kRows; ++r)
    for (int c = 0; c < kCols; ++c)
      p.emplace_back(float(c * kSquare), float(r * kSquare), 0.0f);
  return p;
}

struct Views {
  std::vector<std::vector<cv::Point3f>> obj;
  std::vector<std::vector<cv::Point2f>> img;
};

/// Synthesise n observations of the board through the true camera.
Views makeViews(int n, cv::RNG& rng, double tiltDeg = 32.0, double zLo = 0.35,
                double zHi = 0.75, double noisePx = 0.0) {
  const std::vector<cv::Point3f> objp = boardPoints();
  cv::Point3d centroid(0, 0, 0);
  for (const auto& p : objp) centroid += cv::Point3d(p.x, p.y, p.z);
  centroid *= 1.0 / objp.size();

  Views v;
  for (int i = 0; i < n; ++i) {
    cv::Vec3d axis(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0),
                   rng.uniform(-1.0, 1.0));
    axis /= cv::norm(axis);
    const cv::Vec3d rvec = axis * (rng.uniform(0.0, tiltDeg) * CV_PI / 180.0);
    cv::Matx33d R;
    cv::Rodrigues(rvec, R);
    const cv::Point3d wanted(rng.uniform(-0.06, 0.06), rng.uniform(-0.05, 0.05),
                             rng.uniform(zLo, zHi));
    const cv::Vec3d rc = R * cv::Vec3d(centroid.x, centroid.y, centroid.z);
    const cv::Vec3d tvec(wanted.x - rc[0], wanted.y - rc[1], wanted.z - rc[2]);

    std::vector<cv::Point2f> uv;
    cv::projectPoints(objp, rvec, tvec, cv::Mat(kKTrue), cv::Mat(kDTrue), uv);
    bool visible = true;
    for (const auto& p : uv)
      if (p.x < 8 || p.y < 8 || p.x > W - 8 || p.y > H - 8) visible = false;
    if (!visible) continue;  // the board must be fully inside the frame
    if (noisePx > 0)
      for (auto& p : uv) {
        p.x += float(rng.gaussian(noisePx));
        p.y += float(rng.gaussian(noisePx));
      }
    v.obj.push_back(objp);
    v.img.push_back(uv);
  }
  return v;
}

struct Result {
  double rms;
  cv::Mat K, D;
};

Result run(const Views& v, int flags = 0) {
  Result r;
  std::vector<cv::Mat> rvecs, tvecs;
  r.rms = cv::calibrateCamera(v.obj, v.img, {W, H}, r.K, r.D, rvecs, tvecs,
                              flags);
  return r;
}

const char* kHeader =
    "%-34s %4s %8s %9s %9s %8s %8s %9s\n";

void header() {
  std::printf(kHeader, "experiment", "#img", "RMS px", "d fx", "d fy", "d cx",
              "d cy", "d k1");
  std::printf("%s\n", std::string(96, '-').c_str());
}

void report(const std::string& name, const Result& r, size_t n) {
  const double* K = r.K.ptr<double>();
  const double* D = r.D.ptr<double>();
  std::printf("%-34s %4zu %8.4f %+9.2f %+9.2f %+8.2f %+8.2f %+9.4f\n",
              name.c_str(), n, r.rms, K[0] - kKTrue(0, 0), K[4] - kKTrue(1, 1),
              K[2] - kKTrue(0, 2), K[5] - kKTrue(1, 2), D[0] - kDTrue[0]);
}

}  // namespace

int main() {
  std::printf("ground truth\n  fx %.1f  fy %.1f  cx %.1f  cy %.1f\n",
              kKTrue(0, 0), kKTrue(1, 1), kKTrue(0, 2), kKTrue(1, 2));
  std::printf("  D  [%.4f %.4f %.4f %.4f %.4f]\n\n", kDTrue[0], kDTrue[1],
              kDTrue[2], kDTrue[3], kDTrue[4]);
  std::printf("errors below are (recovered - true)\n\n");
  header();

  // 1. how many views do you need? -----------------------------------------
  for (int n : {3, 6, 12, 25}) {
    cv::RNG rng(7);
    const Views v = makeViews(n, rng, 32.0, 0.35, 0.75, 0.25);
    report(ci::fmt("%d well-tilted views, 0.25 px noise", n), run(v),
           v.obj.size());
  }
  std::printf("\n");

  // 2. how much does corner noise hurt? ------------------------------------
  for (double noise : {0.0, 0.1, 0.5, 1.5}) {
    cv::RNG rng(11);
    const Views v = makeViews(20, rng, 32.0, 0.35, 0.75, noise);
    report(ci::fmt("20 views, corner noise %.2f px", noise), run(v),
           v.obj.size());
  }
  std::printf("\n");

  // 3. the mistake everybody makes -----------------------------------------
  cv::RNG rngFlat(3);
  const Views flat = makeViews(20, rngFlat, 2.0, 0.50, 0.55, 0.25);
  const Result rFlat = run(flat);
  report("20 views, board held FLAT", rFlat, flat.obj.size());

  cv::RNG rngTilt(3);
  const Views tilt = makeViews(20, rngTilt, 35.0, 0.30, 0.80, 0.25);
  const Result rTilt = run(tilt);
  report("20 views, board TILTED + depth", rTilt, tilt.obj.size());

  const double fxFlat = rFlat.K.ptr<double>()[0];
  const double fxTilt = rTilt.K.ptr<double>()[0];
  std::printf("\n%s\nwhat to take away\n%s\n", std::string(78, '=').c_str(),
              std::string(78, '=').c_str());
  std::printf("  The flat-board run reports RMS = %.3f px, which looks "
              "excellent,\n  yet fx is off by %+.1f px -- %.0f%% wrong. Note "
              "that\n  cx and cy came out fine: the degeneracy is specifically "
              "between focal\n  length and board distance, which a flat board "
              "cannot separate.\n",
              rFlat.rms, fxFlat - kKTrue(0, 0),
              100.0 * std::abs(fxFlat / kKTrue(0, 0) - 1.0));
  std::printf("  The tilted run reports a similar RMS = %.3f px and recovers fx "
              "to %+.1f px.\n\n", rTilt.rms, fxTilt - kKTrue(0, 0));
  std::printf("  Reprojection error measures how well the model fits the data "
              "you gave it.\n  It cannot tell you the data was uninformative. "
              "With every board at the\n  same distance and angle, focal length "
              "and board distance trade off against\n  each other almost "
              "perfectly -- the optimiser is free to pick the wrong\n  "
              "combination and still fit every corner.\n\n");
  std::printf("  Practical checklist\n"
              "    * tilt the board 30-45 deg in several directions, not just "
              "flat on\n"
              "    * vary the distance so the board fills 1/3 to 3/4 of the "
              "frame\n"
              "    * push the board into all four corners: that is where D is "
              "measured\n"
              "    * 15-25 good views beats 60 sloppy ones\n"
              "    * fix k3 (CALIB_FIX_K3) unless the lens is genuinely very "
              "wide\n"
              "    * sanity-check the result: cx, cy within a few %% of the "
              "image centre,\n      fx/fy within ~1%%, and fx consistent with "
              "the lens spec and sensor\n");

  // 4. is a free k3 worth it? ----------------------------------------------
  std::printf("\n%s\nbonus: is a free k3 worth it on a lens whose true k3 is "
              "0?\n%s\n", std::string(78, '=').c_str(),
              std::string(78, '=').c_str());
  std::printf("One calibration cannot answer that -- the difference hides in "
              "the spread\nacross repeats. So run the same experiment 12 times "
              "with different noise\nand different board poses, and look at the "
              "variance.\n\n");
  std::printf("%-14s %8s %9s %8s %9s %8s %9s %8s\n", "", "RMS px", "fx bias",
              "fx std", "k1 bias", "k1 std", "k3 mean", "k3 std");
  std::printf("%s\n", std::string(78, '-').c_str());
  const int flagsList[2] = {0, cv::CALIB_FIX_K3};
  const char* flagNames[2] = {"free k1,k2,k3", "k3 fixed to 0"};
  for (int f = 0; f < 2; ++f) {
    std::vector<double> fxs, k1s, k3s, rmss;
    for (int seed = 0; seed < 12; ++seed) {
      cv::RNG rng(seed + 1);
      const Views v = makeViews(15, rng, 32.0, 0.35, 0.75, 0.4);
      const Result r = run(v, flagsList[f]);
      rmss.push_back(r.rms);
      fxs.push_back(r.K.ptr<double>()[0]);
      k1s.push_back(r.D.ptr<double>()[0]);
      k3s.push_back(r.D.ptr<double>()[4]);
    }
    auto mean = [](const std::vector<double>& v) {
      double s = 0;
      for (double x : v) s += x;
      return s / v.size();
    };
    auto stdev = [&](const std::vector<double>& v) {
      const double m = mean(v);
      double s = 0;
      for (double x : v) s += (x - m) * (x - m);
      return std::sqrt(s / v.size());
    };
    std::printf("%-14s %8.4f %+9.2f %8.2f %+9.4f %8.4f %+9.4f %8.4f\n",
                flagNames[f], mean(rmss), mean(fxs) - kKTrue(0, 0), stdev(fxs),
                mean(k1s) - kDTrue[0], stdev(k1s), mean(k3s), stdev(k3s));
  }
  std::printf("\n  The reprojection error is identical to four decimals, and fx "
              "barely\n  cares. But look at k3: its true value is 0, and the "
              "free fit scatters it\n  across a range far wider than the "
              "coefficient itself -- it is almost\n  unconstrained by this "
              "data. That noise does not stay contained: it leaks\n  into k1.\n");
  std::printf("\n  So the argument for CALIB_FIX_K3 is not 'lower error'. It is "
              "that an\n  unconstrained parameter buys you nothing and "
              "destabilises the ones you\n  actually use.\n");
  return 0;
}
