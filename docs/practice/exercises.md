# Exercises

Ten functions to implement from scratch. They are the same ten in both
languages, checked by the same expectations, so pick whichever you think in.

| Python | C++ |
| --- | --- |
| [`python/exercises/exercises.py`](../../python/exercises/exercises.py) | [`cpp/exercises/exercises.cpp`](../../cpp/exercises/exercises.cpp) |

## The rules

* **Use NumPy (or plain C++).** Do not call OpenCV — writing the maths yourself
  is the entire point.
* **Do not import `camintrinsics`** either; that is just copying the answer.
* Every function has a worked answer in `solutions.py` / `solutions.cpp`. Look
  after you have tried, and compare approaches rather than diffing text.

Conventions are the repository's own: camera down **+Z**, `xy` is normalized,
`uv` is pixels, point arrays are `(N, 2)` or `(N, 3)`.

## The list

| # | Function | Does | Chapter |
| --- | --- | --- | --- |
| 1 | `ex01_build_K` | Assemble `K` from `fx, fy, cx, cy, skew` | [2](../course/02_the_K_matrix.md) |
| 2 | `ex02_project_pinhole` | Project 3D points to pixels; `Z <= 0` must give `NaN` | [1](../course/01_pinhole_and_projection.md) |
| 3 | `ex03_distort` | Apply the plumb-bob model in normalized coordinates | [3](../course/03_the_D_vector.md) |
| 4 | `ex04_fov_degrees` | Horizontal and vertical field of view from `K` | [2](../course/02_the_K_matrix.md) |
| 5 | `ex05_K_after_resize` | Update `K` for a resized image | [4](../course/04_resize_crop_and_roi.md) |
| 6 | `ex06_K_after_crop` | Update `K` for a cropped image | [4](../course/04_resize_crop_and_roi.md) |
| 7 | `ex07_undistort_point` | Invert exercise 3 — the hard one | [5](../course/05_undistortion.md) |
| 8 | `ex08_K_from_hfov` | Build a centred, square-pixel `K` from a field of view | [2](../course/02_the_K_matrix.md) |
| 9 | `ex09_classify_distortion` | Barrel, pincushion or none at a given radius | [3](../course/03_the_D_vector.md) |
| 10 | `ex10_pipeline_K` | Crop *then* resize — the one that bites in production | [4](../course/04_resize_crop_and_roi.md) |

Exercise 7 is where people learn something they did not expect: the obvious
fixed-point iteration does not always converge. Chapter 5 explains why, and
[the verification notes](../reference/verification.md) record what this
repository does instead.

## Checking your work

### Python

```bash
uv run python/exercises/check.py               # scorecard for all ten
uv run python/exercises/check.py 3 7           # only exercises 3 and 7
uv run python/exercises/check.py --solutions   # verify the reference answers
uv run pytest                                  # the same checks under pytest
```

### C++

```bash
./cpp/build/bin/check_exercises                # scorecard for all ten
./cpp/build/bin/check_exercises 3 7            # only exercises 3 and 7
./cpp/build/bin/check_exercises --solutions    # verify the reference answers
ctest --test-dir cpp/build                     # the same, as a test
```

An exercise you have not written yet is reported as `not implemented yet`, not
as a failure, so the scorecard is readable from the very first run.

When you have finished, take the [quiz](quiz.md).
