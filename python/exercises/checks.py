"""The reference checks used by both check.py and test_exercises.py.

Each check receives the module under test (``exercises`` or ``solutions``) and
raises AssertionError with a message that says what went wrong, not just that
something did.  Ground truth comes from OpenCV wherever OpenCV has an
equivalent, so passing here means passing against the real thing.
"""

import numpy as np

try:
    import cv2
except ImportError:                                    # pragma: no cover
    cv2 = None

RNG = np.random.RandomState(20260829)
K_REF = np.array([[812.0, 0.0, 639.1], [0.0, 809.5, 361.4], [0.0, 0.0, 1.0]])
D_REF = (-0.27, 0.10, 0.0012, -0.0009, 0.004)
PTS3 = np.column_stack([RNG.uniform(-2.5, 2.5, 40),
                        RNG.uniform(-2.0, 2.0, 40),
                        RNG.uniform(1.5, 12.0, 40)])
XY = np.column_stack([RNG.uniform(-0.8, 0.8, 60), RNG.uniform(-0.6, 0.6, 60)])


def _close(got, want, tol, what):
    got = np.asarray(got, dtype=np.float64)
    want = np.asarray(want, dtype=np.float64)
    assert got.shape == want.shape, \
        f"{what}: expected shape {want.shape}, got {got.shape}"
    err = np.nanmax(np.abs(got - want))
    assert err <= tol, f"{what}: off by {err:.3e} (tolerance {tol:.1e})"


def check01(m):
    K = m.ex01_build_K(812.0, 809.5, 639.1, 361.4)
    _close(K, K_REF, 1e-12, "K")
    assert np.asarray(K).dtype == np.float64, "K must be float64"
    Ks = m.ex01_build_K(800, 800, 320, 240, skew=1.5)
    assert abs(Ks[0, 1] - 1.5) < 1e-12, "skew belongs at K[0, 1]"
    assert abs(Ks[1, 0]) < 1e-12, "K[1, 0] must stay 0"


def check02(m):
    uv = m.ex02_project_pinhole(PTS3, K_REF)
    want = cv2.projectPoints(PTS3, np.zeros(3), np.zeros(3), K_REF,
                             np.zeros(5))[0].reshape(-1, 2)
    _close(uv, want, 1e-9, "pinhole projection")
    behind = np.array([[1.0, 1.0, -2.0], [0.0, 0.0, 0.0]])
    got = np.asarray(m.ex02_project_pinhole(behind, K_REF))
    assert np.all(np.isnan(got)), \
        "points with Z <= 0 must project to NaN, got " + str(got)


def check03(m):
    got = m.ex03_distort(XY, *D_REF)
    pts3 = np.column_stack([XY, np.ones(len(XY))])
    want_px = cv2.projectPoints(pts3, np.zeros(3), np.zeros(3),
                                np.eye(3), np.array(D_REF))[0].reshape(-1, 2)
    _close(got, want_px, 1e-9, "distortion")


def check04(m):
    for K, w, h in ((K_REF, 1280, 720),
                    (np.array([[400.0, 0, 100.0], [0, 400.0, 300.0], [0, 0, 1.0]]),
                     640, 480)):
        hf, vf = m.ex04_fov_degrees(K, w, h)
        want_h = np.degrees(np.arctan2(K[0, 2], K[0, 0])
                            + np.arctan2(w - K[0, 2], K[0, 0]))
        want_v = np.degrees(np.arctan2(K[1, 2], K[1, 1])
                            + np.arctan2(h - K[1, 2], K[1, 1]))
        _close([hf, vf], [want_h, want_v], 1e-9,
               f"FOV for cx={K[0, 2]} (did you assume cx = width/2?)")


def check05(m):
    K2 = m.ex05_K_after_resize(K_REF, 0.5, 0.25)
    want = K_REF.copy()
    want[0, :] *= 0.5
    want[1, :] *= 0.25
    _close(K2, want, 1e-12, "K after resize")
    # behavioural check: the same 3D point must land on the same physical spot
    uv = cv2.projectPoints(PTS3, np.zeros(3), np.zeros(3), K_REF,
                           np.zeros(5))[0].reshape(-1, 2)
    uv2 = cv2.projectPoints(PTS3, np.zeros(3), np.zeros(3), np.asarray(K2),
                            np.zeros(5))[0].reshape(-1, 2)
    _close(uv2, uv * [0.5, 0.25], 1e-9,
           "resized projection (did you forget to scale cx and cy?)")


def check06(m):
    K2 = np.asarray(m.ex06_K_after_crop(K_REF, 120, 64))
    want = K_REF.copy()
    want[0, 2] -= 120
    want[1, 2] -= 64
    _close(K2, want, 1e-12, "K after crop")
    assert abs(K2[0, 0] - K_REF[0, 0]) < 1e-12, \
        "cropping must not change fx -- the lens did not change"


def check07(m):
    xyd = np.asarray(cv2.projectPoints(
        np.column_stack([XY, np.ones(len(XY))]), np.zeros(3), np.zeros(3),
        np.eye(3), np.array(D_REF))[0]).reshape(-1, 2)
    got = m.ex07_undistort_point(xyd, *D_REF)
    _close(got, XY, 1e-8, "undistortion (does your loop recompute r^2 each pass?)")


def check08(m):
    K = np.asarray(m.ex08_K_from_hfov(90.0, 1280, 720))
    _close([K[0, 0], K[0, 2], K[1, 2]], [640.0, 640.0, 360.0], 1e-9,
           "K from 90 deg hFOV on 1280x720")
    K2 = np.asarray(m.ex08_K_from_hfov(60.0, 800, 600))
    want = 400.0 / np.tan(np.radians(30.0))
    _close([K2[0, 0], K2[1, 1]], [want, want], 1e-9, "K from 60 deg hFOV")


def check09(m):
    cases = [((-0.3, 0.0, 0.0), 1.0, "barrel"),
             ((0.3, 0.0, 0.0), 1.0, "pincushion"),
             ((0.0, 0.0, 0.0), 1.0, "none"),
             ((-0.4, 0.25, 0.0), 0.5, "barrel"),
             ((-0.4, 0.25, 0.0), 1.4, "pincushion")]
    for (k1, k2, k3), r, want in cases:
        got = m.ex09_classify_distortion(k1, k2, k3, r)
        assert got == want, \
            f"k1={k1}, k2={k2}, r={r}: expected {want!r}, got {got!r}"


def check10(m):
    K2 = np.asarray(m.ex10_pipeline_K(K_REF, 200, 100, 0.5))
    want = K_REF.copy()
    want[0, 2] -= 200
    want[1, 2] -= 100
    want[0, :] *= 0.5
    want[1, :] *= 0.5
    _close(K2, want, 1e-12,
           "crop-then-resize K (did you apply the resize before the crop?)")
    swapped = K_REF.copy()
    swapped[0, :] *= 0.5
    swapped[1, :] *= 0.5
    swapped[0, 2] -= 200
    swapped[1, 2] -= 100
    assert abs(K2[0, 2] - swapped[0, 2]) > 1e-6, \
        "that is the resize-then-crop answer; the crop happens first here"


CHECKS = [
    (1, "build K", check01),
    (2, "pinhole projection", check02),
    (3, "apply distortion", check03),
    (4, "field of view", check04),
    (5, "K after resize", check05),
    (6, "K after crop", check06),
    (7, "undistort a point", check07),
    (8, "K from FOV", check08),
    (9, "classify distortion", check09),
    (10, "crop-then-resize pipeline", check10),
]
