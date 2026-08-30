# Verification

A teaching repository that gets the maths wrong teaches the wrong thing. This
page records what is actually checked, how, and what the checks measured — so
you can re-run them rather than take the claims on faith.

## Agreement with OpenCV

`project_points` is checked against `cv2.projectPoints` over 100,000 random
points, at a wide-angle `K` with all five distortion coefficients non-zero:

```text
max |ours - cv2.projectPoints|  =  3.7e-09 px
```

That is the accumulated difference in operation order between two float64
implementations of the same formula, not a modelling difference. For the
exercises the tolerance is set at `1e-9`, which the reference solutions meet.

## Agreement between the three ports

Python, C++ and the viewer's JavaScript are three independent implementations.
They are diffed **byte for byte**, not compared to a tolerance:

```bash
./scripts/check_parity.sh
```

```text
  python == js
  python == cpp
all three ports agree
```

Five camera presets are dumped through projection, field of view, fold radius,
undistortion (including points outside the image) and `optimal_new_camera_matrix`
at three `alpha` values, and the three text dumps must be identical. This runs
in CI on every push. It earned its place on its first run by catching a real
divergence: Python raised where the other two silently returned `K`.

## Undistortion

Undistortion has no closed form, so it is solved numerically — and the naive
solver is wrong in a way that is easy to miss.

* **The fixed-point iteration used by many implementations does not always
  converge.** At the corner of a modest wide-angle lens it oscillates
  indefinitely between two radii, and stopping after a fixed number of passes
  leaves an error of about 26 px while reporting nothing.
* This repository uses **Newton's method** with the analytic 2×2 Jacobian
  instead.
* Some distorted points have **no solution at all** — beyond the fold radius the
  distortion curve stops increasing, so nothing maps there. Those points return
  `NaN`. A plausible-looking wrong answer is worse than an admitted failure, and
  the solver checks its own residual before accepting a root.

Chapter [5, Undistortion](../course/05_undistortion.md) explains the geometry;
`max_valid_radius` and `max_distorted_radius` expose the boundary in code, and
the viewer draws it.

## What CI runs

| Job | Checks |
| --- | --- |
| `python` | Exercise solutions under pytest, every example runs, figures regenerate |
| `cpp` | Builds, `ctest`, every example runs, figure renderers run |
| `parity` | `scripts/check_parity.sh` across all three ports |
| `commits` | Commit messages against [Conventional Commits](../../CONTRIBUTING.md) |

## Running everything yourself

```bash
uv run pytest                     # Python exercise solutions
ctest --test-dir cpp/build        # C++ exercise solutions
./scripts/check_parity.sh         # three-port parity
./scripts/generate_figures.sh     # regenerate every figure in data/generated/
```

The parity script needs `uv`, `node`, and a built `cpp/build`.
