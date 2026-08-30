# Documentation

Everything written for this repository, in one map. If you are here to learn,
start with [the path](#the-path) and read it in order.

The material is split by *what you need at that moment*:

| Folder | For | Read it when |
| --- | --- | --- |
| [`course/`](course/) | Understanding | You want to know how something works and why |
| [`reference/`](reference/) | Facts | You already understand it and need the formula |
| [`practice/`](practice/) | Doing | You want to prove to yourself that you know it |

---

## Start here

This repository teaches one thing properly: **how a camera turns a 3D point into
a pixel**, and what the two objects that control it — the intrinsic matrix `K`
and the distortion vector `D` — actually do.

It is deliberately narrow. There is no epipolar geometry, no stereo, no SLAM.
Those all sit on top of this, and they are hard to learn while you are still
unsure whether `cx` moves the camera or the image.

### How the repository is meant to be used

Each chapter follows the same rhythm:

1. **read** the theory,
2. **run** the matching example and watch the numbers,
3. **play** with the [viewer](https://cyhunblr.github.io/learn_camera_intrinsics/)
   until the parameter feels physical,
4. **implement** the matching exercise from scratch,
5. **check** yourself with the quiz.

You do not need to pick a language. Python and C++ contain the same library,
the same six examples, the same figure renderers and the same ten exercises,
and they produce identical numbers. Use whichever you think in; skim the other
to see what changes and what does not.

---

## The path

| # | Chapter | Example | Exercises | Quiz topic |
| --- | --------- | --------- | ----------- | ------------ |
| 1 | [The pinhole model](course/01_pinhole_and_projection.md) | `01_pinhole_projection` | 2 | `pinhole` |
| 2 | [The K matrix](course/02_the_K_matrix.md) | `02_k_matrix_anatomy` | 1, 4, 8 | `K` |
| 3 | [The D vector](course/03_the_D_vector.md) | `03_distortion_anatomy` | 3, 9 | `D` |
| 4 | [Resize, crop and ROI](course/04_resize_crop_and_roi.md) | `04_resize_crop_roi` | 5, 6, 10 | `resize` |
| 5 | [Undistortion](course/05_undistortion.md) | `05_undistort_and_alpha` | 7 | `undistort` |
| 6 | [Calibration in practice](course/06_calibration_in_practice.md) | `06_calibrate_synthetic` | — | `calibration` |

## Reference

* [Cheat sheet](reference/cheatsheet.md) — the whole pipeline, both objects and
  every transformation rule on one printable page.
* [Verification](reference/verification.md) — what this repository checks about
  its own numbers, and how to re-run those checks yourself.

## Practice

* [Exercises](practice/exercises.md) — ten functions to implement from scratch,
  in Python or C++, with an automatic scorecard.
* [Quiz](practice/quiz.md) — 28 questions with folded answers. The same bank
  also runs interactively in the terminal.

## Setting up

* [Python side](../python/README.md) — `pipx`, `uv`, and how to run everything.
* [C++ side](../cpp/README.md) — OpenCV, CMake, and the same programs.
* [Contributing](../CONTRIBUTING.md) — commit conventions and the pre-push checks.

---

## The viewer

One interactive viewer runs alongside the whole path: **[Intrinsics Bench](https://cyhunblr.github.io/learn_camera_intrinsics/)**,
a single page you open in a browser. Nothing to install.

* **3D scene** — a world with the frustum your `K` describes drawn into it,
  beside the image that camera produces. Drag either view to move it.
* **2D chart** — a test chart through the same camera three times: with `D`
  switched off, with `D` on, and undistorted again. Below both: the radial
  curve `r′(r)` and the per-pixel displacement field.

Every number in it is live and physical — `fx` in pixels, `k1` unitless, the
`K` matrix typeset at the bottom — and the maths is the same port you are
reading about here.

---

## Conventions used everywhere

* Right-handed frames. The camera looks down **+Z**, **+X** is right, **+Y** is
  **down**. This is the OpenCV convention, not the OpenGL one (where the camera
  looks down −Z and +Y is up). Mixing them up flips your image vertically and
  costs an afternoon.
* Pixel coordinates `(u, v)`: `u` right, `v` down, origin at the centre of the
  top-left pixel.
* **Normalized image coordinates** `(x, y)` are what you get after the
  perspective divide and before `K`. They are unitless and independent of image
  resolution. Almost every confusing thing about intrinsics becomes obvious once
  you keep track of which space you are in.
* `K` is intrinsics only. Where the camera *is* lives in the extrinsics
  `[R | t]`, which this repository deliberately keeps out of the way.

## Notation

$$
\mathbf{K} = \begin{bmatrix} f_x & s & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}
\qquad
\mathbf{D} = \begin{bmatrix} k_1 & k_2 & p_1 & p_2 & k_3 \end{bmatrix}
$$

Note the ordering of `D`: the two **tangential** coefficients `p1, p2` sit
between the radial `k2` and `k3`. That is OpenCV's ordering, it is not
alphabetical, it is not grouped by type, and getting it wrong is the single most
common camera bug in the wild.
