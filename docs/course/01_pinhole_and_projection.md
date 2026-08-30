# 1. The pinhole model

> Run alongside: `uv run python/examples/01_pinhole_projection.py`
> or `./cpp/build/bin/01_pinhole_projection`

## The whole model in one line

A camera takes a point in 3D and returns a pixel. Between those two things there
are exactly three steps:

```text
 (X, Y, Z)  --- perspective divide --->  (x, y)  --- distortion D --->  (x', y')  --- intrinsics K --->  (u, v)
  camera frame                        normalized                     normalized                        pixels
  metres                              unitless                       unitless                          pixels
```

Everything else in this repository is a detail of one of those three arrows.

## Step 1: the perspective divide

Put a pinhole at the origin and a sensor one unit in front of it. A ray from the
world point $(X, Y, Z)$ through the pinhole meets that plane at

$$ x = \frac{X}{Z}, \qquad y = \frac{Y}{Z} $$

This is the only place perspective happens. Everything after it is a
two-dimensional coordinate change.

The pair $(x, y)$ is called **normalized image coordinates**. Note what has been
thrown away: the depth. $(1, 0, 4)$ and $(2, 0, 8)$ both give $x = 0.25$.

```text
        Z
        ^          . P = (X, Y, Z)
        |        .
        |      .
        |    .
        |  .            all points on this ray
   -----O------------   give the same (x, y)
     pinhole
```

**A single camera measures direction, not distance.** That one sentence explains
why monocular depth estimation is hard, why you need stereo or motion or a known
ground plane, and why a projection function has an inverse only if you supply
extra information.

Try it: example 1 projects the same ray at 1 m, 3 m, 10 m and 100 m and prints
four identical pixels.

## Step 2: distortion

A real lens is not a pinhole. It bends rays slightly, more so the further they
are from the optical axis. That correction is applied here, in normalized
coordinates, and is the subject of [chapter 3](03_the_D_vector.md).

The important structural fact: distortion happens **after** the divide and
**before** `K`. It is therefore unitless and completely independent of image
resolution — resizing an image does not change `D`.

## Step 3: intrinsics

Normalized coordinates are still abstract. `K` maps them onto the actual grid of
photosites:

$$ u = f_x x' + s\,y' + c_x, \qquad v = f_y y' + c_y $$

which is [chapter 2](02_the_K_matrix.md).

## Where it breaks: points behind the camera

The divide by $Z$ is happy to accept a negative $Z$. A point 3 m *behind* the
camera produces a perfectly finite-looking pixel — the mirror image of where it
would be if it were in front. Nothing warns you.

```python
project_points([[0.9, -0.4, -3.0]], K, D)   # this repo -> [nan nan]
cv2.projectPoints(...)                       # OpenCV    -> (406.58, 463.77)
```

Both behaviours are defensible; the point is that OpenCV's answer is a
plausible-looking lie. **Always clip on $Z > 0$ before you trust a projection.**
Forgetting this is why LiDAR points from behind a vehicle sometimes appear
painted onto the camera image.

## The full matrix form

You will often see the model written as a single matrix product:

<!-- markdownlint-disable-next-line MD013 -->
$$ \lambda \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \mathbf{K}\,[\,\mathbf{R} \mid \mathbf{t}\,] \begin{bmatrix} X_w \\ Y_w \\ Z_w \\ 1 \end{bmatrix} $$

Two things to notice. First, $[\mathbf{R} \mid \mathbf{t}]$ — the extrinsics —
takes the point from the world frame to the camera frame; this repository
assumes you have already done that and works in the camera frame throughout.
Second, that compact form **cannot express distortion**, because distortion is
not a linear map. Any time you see a single 3×4 projection matrix $P = K[R|t]$,
distortion has either been removed beforehand or ignored.

## Check yourself

* Exercise 2 (`project_pinhole`) — implement the divide plus `K`, including
  the NaN behaviour for $Z \le 0$.
* Quiz: `uv run python/quiz/run_quiz.py --topic pinhole`

**Next:** [The K matrix](02_the_K_matrix.md)
