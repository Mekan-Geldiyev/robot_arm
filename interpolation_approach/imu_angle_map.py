"""
Boxing Robot Arm - Wrist IMU Angle -> Servo Target Mapping
------------------------------------------------------------
Phase 2 of the wrist-IMU chapter (see CLAUDE.md's "Wrist IMU punch
detector" section) - phase 1 (hook vs uppercut vs straight-punch type
classification, imu_punch_classify_test.ino) is confirmed working on real
throws. This module takes that classifier's numeric `angleDeg` output
(the angle between a punch's own acceleration vector and the wrist's
"up" reference - 0 means roughly UP the body, ~90+ means roughly ACROSS
it) and interpolates it into a real (pan, tilt, yaw, elbow) target on
THIS robot, instead of only snapping to one of two fixed canned poses.

Pure/stateless, no serial/hardware here - same spirit as
punch_angle_270.py's punch_animation_frame(). See imu_angle_test.py for
the standalone dry-run that actually reads live IMU serial output through
this function.

==================== GROUND-TRUTH METHODOLOGY ====================
Anchors live in interpolation_implement/imu_angle_calibration.json, same
"measure several real samples, store the result, don't hardcode a guess"
pattern as interpolation_implement/calibration_data_270.json - each
anchor pairs a measured angleDeg (the MEDIAN of a real labeled-throw
batch, see imu_results_2026-10-29.md) with a positions_270.POSITIONS key.
Position VALUES are resolved through POSITIONS at runtime rather than
copied into the JSON, so this can't drift out of sync with the single
source of truth there the way punch_angle_270.PUNCHES already avoids that
for the canned-animation side.

Only 2 anchors exist right now (UPPERCUT, HOOK) since those are the only
two types confirmed on the IMU side so far (see
feedback_punch_animation_workflow / imu-punch-detector memory - hook and
uppercut classification is confirmed good, this is the next step).
OVERHAND/HIGH_OVERHAND are NOT included - no real throws of those types
have been logged with the IMU running, so there's no measured angleDeg to
anchor them to. The interpolation below is written to generalize to 3+
anchors without changes, so adding them later (once real data exists) is
just a JSON edit, not a code change.

A straight "punch" (jab/cross, low rotation) has no mapping at all -
there's no PUNCHES-style canned position for it on this build either
(track_interpolation_270.py's own vision-side wiring has the same gap:
"anything that's not a hook defaults to Uppercut"). angle_to_target()
only accepts "hook"/"uppercut" - callers must check type themselves.
====================================================================
"""

import json
import os
import sys

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

from positions_270 import POSITIONS  # noqa: E402

_CALIBRATION_PATH = os.path.join(
    _PARENT_DIR, "interpolation_implement", "imu_angle_calibration.json"
)


def _load_anchors():
    """Returns a list of (angle_deg, (pan, tilt, yaw, elbow)) tuples,
    sorted ascending by angle_deg - resolved through POSITIONS, not the
    JSON's own position keys directly, so a typo'd/renamed POSITIONS key
    fails loudly here at import time rather than silently elsewhere."""
    with open(_CALIBRATION_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
    anchors = []
    for position_key, entry in data["angle_anchors"].items():
        anchors.append((entry["angle_deg"], POSITIONS[position_key]))
    anchors.sort(key=lambda pair: pair[0])
    return anchors


_ANCHORS = _load_anchors()


def angle_to_target(angle_deg):
    """Interpolates a live IMU angleDeg reading into a (pan, tilt, yaw,
    elbow) target, by linearly blending between whichever two anchors
    bracket it. Clamps to the outermost anchors' angle_deg rather than
    extrapolating past them - a punch angrier/milder than anything in the
    real ground-truth data gets the nearest real anchor's position
    verbatim instead of overshooting into an unverified pose.

    Requires at least 2 anchors (guaranteed by the committed calibration
    file - UPPERCUT and HOOK). With exactly 2, this is a plain 2-point
    lerp; with 3+ (once OVERHAND/HIGH_OVERHAND are added), it finds and
    interpolates within whichever consecutive pair brackets angle_deg,
    so no code change is needed when that data arrives."""
    if angle_deg <= _ANCHORS[0][0]:
        return _ANCHORS[0][1]
    if angle_deg >= _ANCHORS[-1][0]:
        return _ANCHORS[-1][1]

    for (lo_angle, lo_pos), (hi_angle, hi_pos) in zip(_ANCHORS, _ANCHORS[1:]):
        if lo_angle <= angle_deg <= hi_angle:
            t = (angle_deg - lo_angle) / (hi_angle - lo_angle)
            return tuple(
                lo_pos[i] + (hi_pos[i] - lo_pos[i]) * t for i in range(4)
            )

    raise AssertionError("angle_deg within anchor range but no bracketing pair found")


if __name__ == "__main__":
    print(f"Loaded {len(_ANCHORS)} anchors from {_CALIBRATION_PATH}:")
    for angle_deg, pos in _ANCHORS:
        print(f"  angle={angle_deg:5.1f} -> (pan,tilt,yaw,elbow)={tuple(round(v, 1) for v in pos)}")

    print("\nSmoke test - interpolated targets at a spread of angles:")
    for test_angle in (0, 20, 41.7, 50, 60, 73.6, 90, 110):
        pan, tilt, yaw, elbow = angle_to_target(test_angle)
        print(f"  angle={test_angle:5.1f} -> pan={pan:6.1f} tilt={tilt:6.1f} yaw={yaw:6.1f} elbow={elbow:6.1f}")
