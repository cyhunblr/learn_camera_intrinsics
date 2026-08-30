# Cheat sheet

One page. Print it, or keep it open while you debug.

## The pipeline

```text
(X, Y, Z)  --divide-->  (x, y)  --distort D-->  (x', y')  --K-->  (u, v)
 camera frame          normalized              normalized        pixels
 metres                unitless                unitless          pixels
```

```text
x  = X/Z                       y  = Y/Z
r2 = x² + y²
rad = 1 + k1·r2 + k2·r2² + k3·r2³
x' = x·rad + 2·p1·x·y + p2·(r2 + 2x²)
y' = y·rad + p1·(r2 + 2y²) + 2·p2·x·y
u  = fx·x' + s·y' + cx         v  = fy·y' + cy
```

## The two objects

```text
      | fx  s  cx |
  K = |  0 fy  cy |          D = [k1, k2, p1, p2, k3]
      |  0  0   1 |                       ^^^^^^
                                       tangential, between k2 and k3
```

| symbol | is | units | typical |
| --- | --- | --- | --- |
| `fx`, `fy` | focal length ÷ pixel pitch | **pixels** | 0.5–2 × image width |
| `cx`, `cy` | principal point | pixels | ≈ image centre, off by a few px |
| `s` | sensor axis shear | pixels | **0**; OpenCV ignores it |
| `k1` | 1st radial | — | −0.5 … +0.3 |
| `k2` | 2nd radial | — | −0.4 … +0.4 |
| `p1`, `p2` | tangential | — | **1e-4 … 1e-3** |
| `k3` | 3rd radial | — | ≈ 0 unless very wide |

## Conversions

```text
hFOV  = atan(cx/fx) + atan((W-cx)/fx)          # exact, asymmetric
fx    = (W/2) / tan(hFOV/2)                    # centred
F_mm  = fx · pixel_pitch_mm                    # sanity check vs the lens
90° hFOV  <=>  fx = W/2
```

## Image operations

| operation | `fx`, `fy` | `cx`, `cy` | `D` |
| --- | --- | --- | --- |
| resize × (sx, sy) | × sx, × sy | × sx, × sy | unchanged |
| crop at (x0, y0) | unchanged | − x0, − y0 | unchanged |
| pad by (l, t) | unchanged | + l, + t | unchanged |
| flip horizontally | unchanged | (W−1) − cx | `p2` → −`p2` |
| undistort | **changes** | **changes** | **→ 0** |

Order matters: apply them to `K` in the order the pixels meet them.
Letterbox = scale first, then pad.

## Distortion at a glance

| what you see | coefficients |
| --- | --- |
| straight lines bow **outward** | `k1 < 0` (barrel) |
| straight lines bow **inward** | `k1 > 0` (pincushion) |
| barrel in the middle, pincushion at the edge | `k1 < 0`, `k2 > 0` (moustache) |
| the image is lopsided, not symmetric | `p1`, `p2` ≠ 0 |
| smeared "petals" after undistorting | the model folded — see `max_valid_radius` |

## Red flags in a calibration result

- `fx/fy` off 1.0 by more than ~1% → the image was resized non-uniformly
- `cx`, `cy` more than a few % from centre → degenerate or failed fit
- `|p1|` or `|p2|` near 1e-2 → fitting noise, not a lens
- low RMS but the numbers move on recalibration → **degenerate pose set**
- `F = fx · pitch` disagrees with the lens spec → wrong image size somewhere

## Quick recipes

```python
# project 3D -> pixels, distortion included, no image warp needed
uv = project_points(points_cam, K, D)

# undistort a handful of detections instead of the whole image
xy = cv2.undistortPoints(uv.reshape(-1,1,2), K, D).reshape(-1,2)

# undistort an image and remember that K changed
K_new, roi = cv2.getOptimalNewCameraMatrix(K, D, (w,h), alpha, (w,h))
img_u = cv2.undistort(img, K, D, None, K_new)   # now use (K_new, zeros)

# K for a resized+cropped pipeline (crop happens first)
K_final = scale_K(crop_K(K, x0, y0), s)
```

## Rules of thumb

1. `K` cannot rotate or move the camera. `D` cannot either.
2. If straight lines are curved, no `K` will fix it — that is `D`.
3. `D` has no resolution. `K` has exactly one.
4. Clip on `Z > 0` before trusting any projection.
5. Prefer projecting with `(K, D)` over undistorting the whole frame.
6. A low reprojection error is necessary, not sufficient. Tilt the board.
