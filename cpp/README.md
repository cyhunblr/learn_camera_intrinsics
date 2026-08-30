# C++ side

Requires OpenCV 4 (`core imgproc calib3d imgcodecs` — no `highgui`) and a C++17
compiler. Tested with OpenCV 4.2 on GCC 9.

The C++ half uses the system OpenCV via CMake's `find_package` and has nothing
to do with `uv` — that manages the Python side only.

```bash
cd cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Everything lands in `cpp/build/bin/`.

On Debian/Ubuntu the dependency is `sudo apt install libopencv-dev cmake g++`.

## Interactive exploration

Live exploration lives in the browser:
**<https://cyhunblr.github.io/learn_camera_intrinsics/>**.
Nothing here opens a window.

## Figure renderers

```bash
./build/bin/render_2d --out out.png [--preset N] [--chart N] [--alpha A]
./build/bin/render_3d --out out.png [--preset N] [--no-distortion]
```

They draw the same figures as their Python twins from the same maths. The
geometry is identical to the last bit; the pixels are not quite, because
OpenCV's built-in Hershey font rasterises slightly differently between major
versions, so the HUD text lands a pixel or two apart.

## Examples

```bash
./build/bin/01_pinhole_projection
./build/bin/02_k_matrix_anatomy    out.png     # figure path is optional
./build/bin/03_distortion_anatomy  out.png
./build/bin/04_resize_crop_roi
./build/bin/05_undistort_and_alpha out.png
./build/bin/06_calibrate_synthetic
```

Examples 1–5 print the same numbers as their Python twins, to the last decimal.
Example 6 uses OpenCV's RNG rather than NumPy's, so the individual figures
differ while every conclusion is identical.

## Exercises

Implement the ten functions in `cpp/exercises/exercises.cpp` — the
specifications are in `exercises.hpp` — then:

```bash
cmake --build build -j && ./build/bin/check_exercises
./build/bin/check_exercises 3 7           # just those two
./build/bin/check_exercises --solutions   # verify the reference answers
ctest --test-dir build                    # the solutions as a regression test
```

## The library

`cpp/include/camintrinsics/` mirrors the Python package function for function.

```cpp
#include "camintrinsics/intrinsics.hpp"

const ci::Mat33 K = ci::makeK(800, 800, 640, 360);
const ci::Dist  D = ci::makeD(-0.28, 0.09);
const cv::Point2d uv = ci::projectPoint({0.5, 0.2, 4.0}, K, D);
const cv::Vec3d fov = ci::fovDeg(K, 1280, 720);
const ci::Mat33 Khalf = ci::scaleK(K, 0.5, 0.5);
```

| header | contents |
| --- | --- |
| `intrinsics.hpp` | `K`/`D` maths from scratch, verified against OpenCV |
| `scene.hpp` | 3D primitives as subdivided polylines |
| `renderer.hpp` | the software renderer, `Pose`, `lookAt`, `orbitPose` |
| `patterns.hpp` | test charts and the two image warps |
| `plots.hpp` | the radial-profile and displacement-field plots |
| `presets.hpp` | the named lens presets and the model summary text |
| `util.hpp` | `ci::fmt` and `ci::writeImage` (honest image writing) |
