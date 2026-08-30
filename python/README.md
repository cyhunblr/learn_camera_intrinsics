# Python side

## Setup

Dependencies are managed with [uv](https://docs.astral.sh/uv/); uv itself is
installed with [pipx](https://pipx.pypa.io/), which keeps it in its own isolated
environment rather than in a project or in your system Python.

```bash
pipx install uv          # once per machine
uv sync                  # creates .venv and installs everything
```

### Installing pipx and uv from scratch

**1. pipx.** pipx is itself a Python application, so installing it is the one
place `pip` may still be needed. Prefer your system package manager, which
avoids that — and which is mandatory on newer distributions, where
[PEP 668](https://peps.python.org/pep-0668/) blocks `pip install` into the
system Python.

| system | command |
| --- | --- |
| Debian / Ubuntu | `sudo apt install pipx` |
| Fedora / RHEL | `sudo dnf install pipx` |
| Arch | `sudo pacman -S python-pipx` |
| macOS | `brew install pipx` |
| Windows | `scoop install pipx`, or `py -m pip install --user pipx` |
| anything else with pip | `python3 -m pip install --user pipx` |

Then, once, so that `~/.local/bin` is on your `PATH`:

```bash
pipx ensurepath          # then open a new shell
```

**2. uv.**

```bash
pipx install uv
uv --version             # 0.12.7 at the time of writing
```

Two things that bite people on Debian and Ubuntu:

* If you install pipx **with pip** rather than apt, you also need
  `sudo apt install python3-venv` — pipx builds a virtual environment per tool,
  and Debian ships that ability in a separate package. `python3 -m venv --help`
  succeeds even when it is missing, so the failure only shows up later.
* Ubuntu 20.04 ships a very old pipx (0.12.3.1, from 2020). It still installs
  the current uv correctly — verified — so there is no need to chase a newer one.

### Installing uv without pipx

uv is a self-contained Rust binary that does not need Python at all, so it also
ships a standalone installer. This is the route to use if you have no pipx, no
pip, or no Python whatsoever — `uv` can then install Python for you.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh                             # Linux, macOS
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"    # Windows
brew install uv                                                             # macOS
```

The script installs to `~/.local/bin` and appends a `PATH` line to your shell
profile. Two environment variables control that if you would rather it did not:

```bash
UV_INSTALL_DIR=/opt/uv INSTALLER_NO_MODIFY_PATH=1 \
    sh -c "$(curl -LsSf https://astral.sh/uv/install.sh)"
```

If it warns that `uv` is *shadowed by other commands in your PATH*, you already
have a uv somewhere else — check with `command -v uv`.

### Keeping uv up to date

Which command works depends on how uv was installed, and uv will tell you off
if you pick the wrong one:

| installed via | upgrade with |
| --- | --- |
| pipx | `pipx upgrade uv` |
| standalone installer | `uv self update` |
| Homebrew | `brew upgrade uv` |

`uv self update` on a pipx-managed uv fails with *"Self-update is only available
for uv binaries installed via the standalone installation scripts."* — that is
expected, not a broken install.

### You do not need a matching Python either

If uv cannot find an interpreter that satisfies `requires-python`, it downloads
one. On a machine whose only Python was 3.8, `uv sync` here fetched 3.14 and
built the environment against it. `uv python list` shows what it has; adding
`--python 3.11` to `uv sync` pins a specific version.

Everything below is run from the **repository root**. Prefix each command with
`uv run` and the environment is checked and activated for you:

```bash
uv run python/examples/01_pinhole_projection.py
uv run pytest
```

<details><summary>Running the repository without uv at all</summary>

The package is a standard PEP 621 project, so `pip install -e .` works too, and
every script also inserts `python/` into `sys.path` itself — so
`python3 python/examples/01_pinhole_projection.py` runs with nothing installed
but `numpy` and `opencv-python`. The uv path is the supported one; these are
escape hatches.

</details>

## Interactive exploration

Live exploration lives in the browser, not here:
**<https://cyhunblr.github.io/learn_camera_intrinsics/>**. It is one file
(`web/index.html`) that ports this package's maths to JavaScript and gives it
real controls.

## Figure renderers

These regenerate the figures used by the README and docs. They draw, they write
a PNG, they exit — there is no window and no interaction.

```bash
uv run python/figures/render_2d.py --out data/generated/app2d.png
uv run python/figures/render_3d.py --out data/generated/app3d.png
```

Both accept `--preset` (see `camintrinsics.presets.PRESETS`); `render_2d.py`
also takes `--chart` and `--alpha`, and `render_3d.py` takes `--no-distortion`.
`./scripts/generate_figures.sh` runs the whole set.

## Examples

Each one prints a narrated walkthrough; those that draw a figure take
`--save out.png`.

```bash
uv run python/examples/01_pinhole_projection.py
uv run python/examples/02_k_matrix_anatomy.py    --save data/generated/ex02.png
uv run python/examples/03_distortion_anatomy.py  --save data/generated/ex03.png
uv run python/examples/04_resize_crop_roi.py
uv run python/examples/05_undistort_and_alpha.py --save data/generated/ex05.png
uv run python/examples/06_calibrate_synthetic.py
```

## Exercises

Implement the ten functions in `python/exercises/exercises.py`, then:

```bash
uv run python/exercises/check.py            # scorecard
uv run python/exercises/check.py 3 7        # just those two
uv run python/exercises/check.py --solutions   # verify the reference answers
uv run pytest -v                            # the same checks under pytest
```

Ground truth comes from OpenCV wherever OpenCV has an equivalent, so passing
means passing against the real thing.

## Quiz

```bash
uv run python/quiz/run_quiz.py                 # 10 random questions
uv run python/quiz/run_quiz.py -n 28           # the whole bank
uv run python/quiz/run_quiz.py --topic D       # one topic
uv run python/quiz/run_quiz.py --study         # every question with its answer
```

## The library

`python/camintrinsics/` is a small, dependency-light package you can import
directly:

```python
import sys; sys.path.insert(0, "python")
from camintrinsics import make_K, make_D, project_points, fov_deg, scale_K

K = make_K(fx=800, fy=800, cx=640, cy=360)
D = make_D(k1=-0.28, k2=0.09)
project_points([[0.5, 0.2, 4.0]], K, D)     # -> array([[739.50, 399.80]])
fov_deg(K, 1280, 720)                        # -> (77.3, 48.5, 85.1)
scale_K(K, 0.5)                              # K for the half-size image
```

| module | contents |
| --- | --- |
| `intrinsics.py` | `K`/`D` maths from scratch, verified against OpenCV |
| `scene.py` | 3D primitives as subdivided polylines |
| `renderer.py` | the software renderer, `Pose`, `look_at`, `orbit_pose`, `save_image` |
| `patterns.py` | test charts and the two image warps |
| `plots.py` | the radial-profile and displacement-field plots |
| `presets.py` | the named lens presets and the model summary text |
