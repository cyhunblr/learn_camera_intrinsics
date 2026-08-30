# 4. Resize, crop and ROI

> Run alongside: `04_resize_crop_roi` — it proves every rule below numerically.

Calibration gives you a `K` that belongs to **one specific image size**. The
moment a preprocessing step resizes, crops, pads or letterboxes the frame, that
`K` is wrong — and nothing crashes. Your reprojection is quietly off by a few
pixels, which is exactly enough to ruin a 3D detection and exactly little enough
to be mistaken for an extrinsic calibration error.

The rule that generates all the others:

> **Apply the operations to `K` in the same order the pixels meet them.**

## Resize

<!-- markdownlint-disable-next-line MD013 -->
$$ \mathbf{K}' = \begin{bmatrix} s_x & 0 & 0 \\ 0 & s_y & 0 \\ 0 & 0 & 1 \end{bmatrix} \mathbf{K} $$

which in practice means: **multiply the entire first row by $s_x$ and the entire
second row by $s_y$** — including `cx`, `cy` and the skew.

```python
K_half = scale_K(K, 0.5)     # fx, fy, cx, cy and s all halve
```

The classic bug is scaling `fx`, `fy` and forgetting `cx`, `cy`:

```text
fx *= 0.5;  fy *= 0.5        # WRONG
-> a constant offset of up to ~480 px at half resolution on a 1920-wide image
```

It is silent, it is constant, it survives every downstream stage, and it looks
exactly like a small extrinsic error. Example 4 measures it.

A **non-uniform** resize (1920×1080 → 640×640, say) applies different $s_x$ and
$s_y$, which makes `fx/fy` ≠ 1. You have not changed the sensor; you have faked
non-square pixels. That is legitimate as long as `K` records it.

## Crop

<!-- markdownlint-disable-next-line MD013 -->
$$ \mathbf{K}' = \begin{bmatrix} 1 & 0 & -x_0 \\ 0 & 1 & -y_0 \\ 0 & 0 & 1 \end{bmatrix} \mathbf{K} $$

Only `cx` and `cy` move. **`fx` and `fy` do not change** — the lens did not
change, you just kept less of its image circle. Cropping narrows the field of
view at constant focal length.

```python
K_crop = crop_K(K, x0=320, y0=180)
```

## Padding and letterboxing

Padding is a crop with a negative origin:

```python
K_pad = crop_K(K, -pad_left, -pad_top)
```

A letterbox resize (the standard ML preprocessing step) is *scale, then pad*, so:

```python
K_final = crop_K(scale_K(K, s, s), -pad_left, -pad_top)
```

Get the order backwards and the padding is scaled too — a plausible-looking `K`
that is wrong by exactly the padding times the scale factor.

## Digital zoom

Digital zoom is *crop, then resize back up*, so it **does** change `fx`:

```python
K_zoom = scale_K(crop_K(K, x0, y0), W / (W - 2*x0))
```

This is why an optical and a digital 2× zoom have the same `K` but very
different images: the optical one gathered more light on the same pixels.

## Horizontal flip

Mirroring for data augmentation is not free.

$$ c_x \to (W-1) - c_x, \qquad s \to -s, \qquad p_2 \to -p_2 $$

The radial terms are even functions of $x$ and $y$, so they survive a mirror
untouched. The tangential terms are not: substituting $x \to -x$ into the model
shows that the mirrored lens is described by $p_2 \to -p_2$ (and $p_1 \to -p_1$
for a vertical flip). Example 4 measures what you lose by forgetting: a
systematic sub-pixel bias that no downstream recalibration will explain.

Mirroring also flips handedness. If you flip images for augmentation you must
mirror the extrinsics too, or your recovered poses come out left-handed.

## What never changes

**`D`.** It lives in normalized coordinates, before `K`. Resizing, cropping,
padding and flipping (apart from the tangential sign above) leave every
coefficient alone.

## Summary table

| Operation | `fx`, `fy` | `cx`, `cy` | skew | `D` |
| --- | --- | --- | --- | --- |
| resize by $(s_x, s_y)$ | × $s_x$, × $s_y$ | × $s_x$, × $s_y$ | × $s_x$ | unchanged |
| crop at $(x_0, y_0)$ | unchanged | − $x_0$, − $y_0$ | unchanged | unchanged |
| pad by $(l, t)$ | unchanged | + $l$, + $t$ | unchanged | unchanged |
| horizontal flip | unchanged | $c_x \to (W{-}1) - c_x$ | negated | $p_2$ negated |
| undistort | **changes** | **changes** | 0 | **becomes 0** |

That last row is [chapter 5](05_undistortion.md).

## Check yourself

* Exercise 5 (`K_after_resize`), 6 (`K_after_crop`), 10 (`pipeline_K`).
* Quiz: `uv run python/quiz/run_quiz.py --topic resize`

**Next:** [Undistortion](05_undistortion.md)
