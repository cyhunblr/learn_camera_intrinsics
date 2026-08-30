# 6. Calibration in practice

> Run alongside: `06_calibrate_synthetic` — it calibrates a camera whose true
> `K` and `D` we know, so every error in this chapter is measured, not asserted.

## What calibration actually solves

You show the camera a rigid pattern whose geometry you know — usually a
checkerboard with squares of a measured size. For each view, the unknowns are
the board's pose $(\mathbf{R}_i, \mathbf{t}_i)$; shared across all views are `K`
and `D`. The optimiser minimises the total **reprojection error**: the distance
in pixels between each detected corner and where the current model says it
should be.

<!-- markdownlint-disable-next-line MD013 -->
$$ \min_{\mathbf{K},\mathbf{D},\{\mathbf{R}_i,\mathbf{t}_i\}} \sum_{i}\sum_{j} \left\| \mathbf{u}_{ij} - \pi(\mathbf{K}, \mathbf{D}, \mathbf{R}_i, \mathbf{t}_i, \mathbf{X}_j) \right\|^2 $$

`cv2.calibrateCamera` returns the RMS of that residual.

## The trap: a low RMS proves almost nothing

This is the most important thing in this repository, so here is the measurement
rather than the claim. Twenty views, 0.25 px of corner noise, identical in every
respect except how the board was held:

| experiment | RMS | error in `fx` |
| --- | --- | --- |
| board held **flat**, all at the same distance | 0.341 px | **+171.7 px (17% wrong)** |
| board **tilted** 35°, distance varied | 0.340 px | +1.3 px |

Same reprojection error. One result is excellent and one is useless.

The reason is a **degeneracy**. If every board is fronto-parallel at roughly the
same depth, then "a slightly bigger board slightly further away" and "a slightly
smaller board slightly closer" produce nearly identical images. Focal length and
board distance trade off against each other almost perfectly, and the optimiser
is free to pick the wrong combination and still fit every corner to a fraction
of a pixel.

Notice which parameters survived: `cx` and `cy` came out fine. The degeneracy is
specifically between focal length and depth. **Tilting the board is what breaks
it** — a tilted board's perspective foreshortening depends on focal length in a
way that its size does not.

> Reprojection error measures how well the model fits the data you supplied.
> It cannot tell you the data was uninformative.

## A checklist that actually works

* **Tilt the board 30–45° in several directions.** Not just left-right: tilt it
  about both axes, and rotate it in-plane too. This is the single highest-value
  habit.
* **Vary the distance** so the board fills roughly ⅓ to ¾ of the frame across
  the set.
* **Push the board into all four corners** of the image. `D` is measured at
  large radius; if no corner data exists, `k1` and `k2` are guesses.
* **15–25 good views beat 60 sloppy ones.** More bad views do not fix a
  degeneracy, they just make the optimiser more confident about the wrong
  answer.
* **Fix `k3`** with `cv2.CALIB_FIX_K3` unless the lens is genuinely very wide
  (see below).
* **Measure the square size properly.** A 1% error in the square size is a 1%
  error in every distance you ever compute. Print on rigid board, not paper on a
  clipboard; measure across many squares and divide.
* **Keep the board flat.** A warped print puts a systematic error into `D`,
  which is usually where inflated `p1`/`p2` values come from.

## Sanity checks on the result

Before you trust a calibration, check that:

* `cx`, `cy` are within a few percent of the image centre. A principal point
  20% off-centre means the optimiser wandered.
* `fx/fy` is within about 1% of 1.0 (unless you knowingly resized non-uniformly).
* $F = f_x \cdot p_x$ matches the lens on the camera.
* `p1`, `p2` are of order 1e-4 to 1e-3, not 1e-2.
* Recalibrating with a different set of images gives you the same numbers. This
  is the check that catches degeneracy, and almost nobody does it.

## The k3 question

Should you let `k3` float? One calibration cannot tell you — the answer is in
the *spread* across repeats. Twelve independent calibrations, 15 views each,
0.4 px noise, on a lens whose true `k3` is exactly 0:

| | RMS | `fx` std | `k1` std | recovered `k3` |
| --- | --- | --- | --- | --- |
| free `k1, k2, k3` | 0.5433 | 7.81 | 0.0197 | mean +0.33, **std 1.47** |
| `k3` fixed to 0 | 0.5435 | 7.94 | **0.0122** | — |

The reprojection error is identical to four decimals. But `k3` scatters across a
range hundreds of times wider than the coefficient it is estimating — it is
essentially unconstrained by this data — and that noise does not stay contained.
It leaks into `k1`, whose spread grows by about 60%.

So the argument for `CALIB_FIX_K3` is not "lower error". It is that an
unconstrained parameter buys you nothing and destabilises the ones you actually
use. Add `k3` only when the lens is wide enough to need it, and then check that
it comes out repeatable across recalibrations.

The same reasoning applies to `CALIB_ZERO_TANGENT_DIST` on a lens you have
reason to believe is well centred, and to `CALIB_FIX_ASPECT_RATIO` when you know
the pixels are square.

## Other patterns

Checkerboards are the default because corner detection is sub-pixel accurate and
robust. Two alternatives worth knowing:

* **ChArUco** (`cv2.aruco`) — a checkerboard with ArUco markers in the white
  squares, so the board does not need to be fully visible. Much easier to get
  corner coverage with, which directly addresses the checklist above.
* **Circle grids** (`cv2.findCirclesGrid`) — centroids are accurate but shift
  slightly under perspective, since the centroid of a projected circle is not
  the projection of its centre. Asymmetric grids resolve orientation ambiguity.

## When the pinhole model is not enough

Past roughly 120° of field of view the plumb-bob model starts to struggle: the
radial polynomial needs increasingly large coefficients and eventually folds
(see [chapter 3](03_the_D_vector.md)). At that point switch to `cv2.fisheye`,
which models $\theta_d$ as a polynomial in the *incidence angle* $\theta$ rather
than in $r$, and stays well-behaved out to and past 180°.

## Check yourself

* Quiz: `uv run python/quiz/run_quiz.py --topic calibration`

**Next:** [Cheat sheet](../reference/cheatsheet.md)
