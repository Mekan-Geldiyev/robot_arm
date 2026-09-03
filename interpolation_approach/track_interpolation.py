"""
Boxing Robot Arm - Interpolation-Based Motion Mirroring
---------------------------------------------------------
Alternative to track.py's per-channel formula + MIN/MAX/INVERT calibration.
Instead of mapping each of pan/tilt/yaw/elbow independently through its own
hand-tuned range, this looks up the nearest real, hand-verified calibration
poses (calibration_data.json) in raw MediaPipe-angle space and blends their
known-good servo outputs by inverse distance. The servo output for any live
pose is always a weighted average of REAL calibrated poses, so it can never
overshoot past a value that's already been confirmed safe/correct.

This file is purely additive - track.py is untouched, so switching back to
the old approach is just running `python track.py` again from the parent
folder.

Reuses geometry (angle_3pt, elevation_angle, elbow_ik_angle, yaw_angle),
OneEuroFilter, side-selection, and the tuned filter/threshold constants
directly from track.py rather than copy-pasting them, so the two approaches
stay in sync on everything except the final raw->servo mapping step.

Run (from anywhere):   python track_interpolation.py
Quit:                  press 'q' with the webcam window focused
"""

import json
import math
import os
import sys
import time

import cv2
import mediapipe as mp
import serial

# track.py lives one directory up (robot_arm/) - add it to sys.path so it
# can be imported as a plain module. Importing it only defines functions/
# classes/constants (main() is guarded by __name__ == "__main__"), so this
# has no side effects like opening the camera or the serial port.
_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import track  # noqa: E402  (import after sys.path setup, intentional)
import hook_animation_test  # noqa: E402
from punch_classifier import PunchClassifier, LABEL_COLORS  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

# calibration_data.json currently lives in interpolation_implement/ (a
# sibling folder to this one, not inside it) - pointed here rather than
# moved, since the file already existed there before this script did.
CALIBRATION_DATA_PATH = os.path.join(_PARENT_DIR, "interpolation_implement", "calibration_data.json")

# How many nearest calibration positions to blend between. 3 matches "2-3
# nearest positions" from the spec; K automatically shrinks if fewer than
# this many valid calibration positions exist at all.
NEIGHBOR_COUNT = 3

# Inverse-distance-weighting power: weight = 1 / distance^POWER. Lowered
# 2.0 -> 1.0 after "insanely finnicky" hardware feedback: with only 11
# sparse calibration points, POWER=2.0 let whichever single point was
# nearest dominate almost completely, so a small wobble in the live
# reading that swapped which point was nearest caused a big, sudden jump
# in servo output (a real discontinuity - the top-K neighbor SET changes
# even though IDW itself is smooth within a fixed set). POWER=1.0 spreads
# weight more evenly across the 3 neighbors, so swapping which one is
# closest changes the blend gradually instead of snapping between two
# almost-single-point results.
IDW_POWER = 1.0

# Second-stage smoothing applied to the INTERPOLATED SERVO OUTPUT, on top
# of the raw-signal filtering already inherited from track.py. The two
# solve different problems: track.py's filters smooth the noisy raw
# pan/tilt/yaw/elbow signal *before* it ever reaches the interpolator, but
# they can't smooth the discontinuity that happens *inside* interpolation
# when the nearest-neighbor set changes - that jump only exists after
# interpolation, so it needs its own filter afterward.
#
# MIN_CUTOFF stays low on purpose - that's what governs smoothing while
# nearly still, and it's untouched by the BETA change below, so holding a
# pose should feel exactly as stable as before.
#
# BETA raised 0.015 -> 0.05 (2026-08-25): BETA is specifically the "how
# fast does filtering loosen up as speed increases" knob (see the
# OneEuroFilter docstring/comments in track.py) - the low value was making
# even genuine fast motion crawl, not just damping neighbor-set jumps. The
# jump-damping job is now split across SNAP_DISTANCE (bypasses this filter
# entirely on a confident single-point match) and IDW_POWER=1.0 (spreads
# weight more evenly so a neighbor swap moves the output less in the first
# place) - so there's more room to let BETA react to real speed than there
# was when this filter was the only jump-damping mechanism. If flicker
# comes back on ambiguous (non-snapped) poses specifically, lower this
# before touching MIN_CUTOFF.
OUTPUT_MIN_CUTOFF = 0.2
OUTPUT_BETA = 0.05

# Flags a frame as "low confidence" when even the SINGLE nearest calibration
# point is this far away (in the same normalized distance units as
# InterpolationMapper.interpolate()'s "d=" log output - roughly "fraction of
# that channel's calibration-set range", summed in quadrature across all 4
# channels). This doesn't change the servo output at all, it's purely a
# signal for deciding what to calibrate next: if this keeps firing for a
# pose you care about, that region has no real coverage yet and needs a new
# calibration_data.json entry, not more interpolation tuning. Starting
# guess based on distances observed during testing - tighten/loosen once
# you've watched it fire (or not) across a range of real poses.
LOW_CONFIDENCE_DISTANCE = 0.6

# The opposite end of the same idea: when the nearest calibration point is
# THIS close (same normalized units as LOW_CONFIDENCE_DISTANCE), stop
# blending in the 2nd/3rd neighbors at all and just use that single point's
# servo values directly. Blending in a distant neighbor at low weight still
# drags the output away from a calibration point you're basically standing
# on - e.g. at d=0.07 from a point, a 3rd neighbor at d=0.65 can still pull
# ~8% of the result toward itself. "Warm" poses should just BE that
# position, not a 92/8 blend leaning toward it.
SNAP_DISTANCE = 0.15

# Required gap between the nearest and 2nd-nearest point before a snap is
# allowed to fire - see the long comment in InterpolationMapper.interpolate()
# for why this exists (closely-packed points like the HOOK_* arc can make
# SNAP_DISTANCE alone ambiguous). Starting guess - if snapping still feels
# flickery on closely-spaced points, raise this; if legitimate snaps stop
# firing on well-separated points, lower it.
SNAP_MARGIN = 0.1

# ============================================================


class CalibrationLoader:
    """Loads and validates calibration_data.json. Each entry maps a named,
    physically-verified pose to a 4-value raw MediaPipe reading
    (pan, tilt, yaw, elbow - the same "raw:" numbers track.py prints) and
    the 4 servo angles that were confirmed correct for that exact pose."""

    REQUIRED_KEYS = ("mediapipe", "servo")

    def __init__(self, path):
        self.path = path
        self.positions = []  # list of (name, mediapipe_vec, servo_vec) tuples
        self._load()

    def _load(self):
        with open(self.path, "r") as f:
            data = json.load(f)

        raw_positions = data.get("calibration_positions", {})
        for name, entry in raw_positions.items():
            if not all(key in entry for key in self.REQUIRED_KEYS):
                print(f"WARNING: skipping '{name}' - missing 'mediapipe' or 'servo' key")
                continue

            mp_vec = entry["mediapipe"]
            servo_vec = entry["servo"]

            if len(mp_vec) != 4 or len(servo_vec) != 4:
                print(
                    f"WARNING: skipping '{name}' - expected 4 values each, "
                    f"got mediapipe={len(mp_vec)} servo={len(servo_vec)}"
                )
                continue

            if not all(0 <= v <= 180 for v in servo_vec):
                print(f"WARNING: skipping '{name}' - servo value outside 0-180: {servo_vec}")
                continue

            self.positions.append((
                name,
                tuple(float(v) for v in mp_vec),
                tuple(float(v) for v in servo_vec),
            ))

        if len(self.positions) < 2:
            raise ValueError(
                f"Need at least 2 valid calibration positions to interpolate between, "
                f"found {len(self.positions)} in {self.path}"
            )

        print(f"Loaded {len(self.positions)} calibration positions from {self.path}")


class InterpolationMapper:
    """Finds the nearest calibration positions to a live raw reading
    (Euclidean distance in raw pan/tilt/yaw/elbow space) and blends their
    servo outputs by inverse distance (Shepard's method) - the closer a
    calibration position is to what's happening right now, the more it
    counts. Because the weights are non-negative and always sum to 1, the
    result is always a weighted average of REAL calibrated servo values -
    it can't extrapolate past any of them into an unverified/unsafe angle,
    unlike a naive per-channel formula fit."""

    def __init__(self, positions, k=NEIGHBOR_COUNT, power=IDW_POWER):
        self.positions = positions
        self.k = min(k, len(positions))
        self.power = power

        # Per-channel scale for distance normalization - without this, raw
        # Euclidean distance lets whichever channel happens to span the
        # widest range in degrees dominate "closeness" almost entirely.
        # Checked against the actual calibration set: elbow spans ~137
        # degrees and tilt ~119, but pan only ~65 and yaw ~61 - unnormalized,
        # a pose that's badly wrong on pan/yaw but close on tilt/elbow could
        # still be picked as the nearest neighbor, which is exactly the
        # "works for some poses, not others" symptom reported after testing.
        # Dividing each channel's difference by its own observed range
        # before squaring makes all 4 channels contribute comparably.
        cols = list(zip(*(mp_vec for _, mp_vec, _ in positions))) if positions else []
        self.scale = tuple(max(max(c) - min(c), 1.0) for c in cols) if cols else (1.0,) * 4

    def interpolate(self, query):
        """query: (pan, tilt, yaw, elbow) raw values.
        Returns ((pan, tilt, yaw, elbow) servo floats, [(name, weight, distance), ...])."""
        scored = []
        for name, mp_vec, servo_vec in self.positions:
            dist = math.sqrt(sum(
                ((q - m) / s) ** 2 for q, m, s in zip(query, mp_vec, self.scale)
            ))
            scored.append((dist, name, servo_vec))
        scored.sort(key=lambda item: item[0])
        nearest = scored[: self.k]

        # Sitting (near) exactly on a calibrated point, or just close enough
        # (SNAP_DISTANCE) to call it that position - use it directly rather
        # than blending in farther neighbors (or dividing by a near-zero
        # distance).
        #
        # BUT only when that's unambiguous: also require a real margin over
        # the 2nd-nearest point (SNAP_MARGIN), not just being close in an
        # absolute sense. SNAP_DISTANCE alone worked fine when all 11 points
        # were spread far apart - close to one meant far from the rest. Once
        # points are deliberately packed close together to trace a smooth
        # path (the HOOK_* arc), being close to one of them often means
        # being fairly close to its neighbor too, and ordinary jitter can
        # flip which one "wins" from frame to frame. Since a snap skips
        # output smoothing entirely (that's the point - trust a confident
        # match instantly), flipping between two snap-worthy neighbors
        # produced a full unsmoothed jump every time - the exact "mixes the
        # values a lot" jitter reported testing the hook. Requiring a
        # margin means an ambiguous frame falls through to normal blending
        # instead, which already smooths fine on its own.
        if len(nearest) < 2:
            margin_ok = True
        else:
            margin_ok = nearest[1][0] - nearest[0][0] >= SNAP_MARGIN

        if nearest[0][0] < SNAP_DISTANCE and margin_ok:
            dist, name, servo_vec = nearest[0]
            return servo_vec, [(name, 1.0, dist)]

        weights = [1.0 / (dist ** self.power) for dist, _, _ in nearest]
        total = sum(weights)
        weights = [w / total for w in weights]

        blended = [0.0, 0.0, 0.0, 0.0]
        used = []
        for (dist, name, servo_vec), w in zip(nearest, weights):
            for i in range(4):
                blended[i] += w * servo_vec[i]
            used.append((name, w, dist))

        return tuple(blended), used


def apply_output_filter(filt, t, value, snapped):
    """Runs value through the OneEuroFilter, EXCEPT when snapped=True (we
    landed on a single calibration point via SNAP_DISTANCE, not a blend) -
    in that case the value is already a confident, verified match, not
    something that needs smoothing, so it's passed straight through. The
    filter's internal memory is force-set to that value/time instead of
    left stale, so if the next frame un-snaps back into a blend, smoothing
    resumes from the real last position instead of jumping from wherever
    the filter's history happened to be when the snap started. Without
    this, a perfect snap to a calibrated pose (e.g. HAND_OUT_CENTER's
    yaw=0) would still crawl toward the target over many frames instead of
    just being it - the filter can't tell "confident snap" apart from
    "noisy blend" on its own, since both just look like "a value changed."
    """
    if snapped:
        filt.x_prev = value
        filt.dx_prev = 0.0
        filt.t_prev = t
        return value
    return filt(t, value)


def apply_safety_limits(servo_vec):
    """Hardware safety clamps that apply no matter which software approach
    computed the numbers - these are facts about the physical build, not
    about track.py's specific formulas. Currently just the yaw floor (see
    track.YAW_SAFE_FLOOR) - the hand-glued yaw horn mechanically strains
    below this value. IDW interpolation already can't produce an output
    outside any single calibrated servo value's range, but the floor is
    kept here too as a second, independent guard."""
    pan, tilt, yaw, elbow = servo_vec
    yaw = max(yaw, track.YAW_SAFE_FLOOR)
    return (
        track.clamp(pan, 0, 180),
        track.clamp(tilt, 0, 180),
        track.clamp(yaw, 0, 180),
        track.clamp(elbow, 0, 180),
    )


def main():
    loader = CalibrationLoader(CALIBRATION_DATA_PATH)
    mapper = InterpolationMapper(loader.positions)

    ser = None
    try:
        ser = serial.Serial(track.SERIAL_PORT, track.BAUD_RATE, timeout=1, write_timeout=1)
        time.sleep(2)  # give the Arduino time to reset after the port opens
        print(f"Connected to Arduino on {track.SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"WARNING: could not open {track.SERIAL_PORT} ({e})")
        print("Continuing without serial - tracking/display only.")

    track.ensure_model_downloaded()

    BaseOptions = mp.tasks.BaseOptions
    PoseLandmarker = mp.tasks.vision.PoseLandmarker
    PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode
    drawing_utils = mp.tasks.vision.drawing_utils
    POSE_CONNECTIONS = mp.tasks.vision.PoseLandmarksConnections.POSE_LANDMARKS

    landmarker = PoseLandmarker.create_from_options(
        PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=track.MODEL_PATH),
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
    )

    cap = cv2.VideoCapture(track.CAMERA_INDEX)
    if not cap.isOpened():
        print("ERROR: could not open webcam")
        return

    last_sent = {"pan": None, "tilt": None, "yaw": None, "elbow": None}
    last_send_time = 0.0

    pan_filter = track.OneEuroFilter(track.PAN_MIN_CUTOFF, track.PAN_BETA, track.D_CUTOFF)
    tilt_filter = track.OneEuroFilter(track.TILT_MIN_CUTOFF, track.TILT_BETA, track.D_CUTOFF)
    yaw_filter = track.OneEuroFilter(track.YAW_MIN_CUTOFF, track.YAW_BETA, track.D_CUTOFF)
    elbow_filter = track.OneEuroFilter(track.ELBOW_MIN_CUTOFF, track.ELBOW_BETA, track.D_CUTOFF)

    # Post-interpolation output filters - see OUTPUT_MIN_CUTOFF/BETA above.
    out_pan_filter = track.OneEuroFilter(OUTPUT_MIN_CUTOFF, OUTPUT_BETA, track.D_CUTOFF)
    out_tilt_filter = track.OneEuroFilter(OUTPUT_MIN_CUTOFF, OUTPUT_BETA, track.D_CUTOFF)
    out_yaw_filter = track.OneEuroFilter(OUTPUT_MIN_CUTOFF, OUTPUT_BETA, track.D_CUTOFF)
    out_elbow_filter = track.OneEuroFilter(OUTPUT_MIN_CUTOFF, OUTPUT_BETA, track.D_CUTOFF)

    elbow_l1 = None
    elbow_l2 = None
    last_smooth_yaw = 0.0
    last_smooth_elbow = 150.0  # a neutral-ish bent-guard guess until the elbow's first seen
    tracked_side = track.SIDE_A
    start_time = time.monotonic()

    # Punch classification (hook / uppercut / unclassified punch) - see
    # punch_classifier.py. Only "hook" triggers the scripted animation
    # (hook_animation_test.py) right now, since that's the only animation
    # that exists - uppercut/punch are classified and shown on screen for
    # visibility, but don't yet DO anything different from normal live
    # tracking. While hook_start_time is not None, yaw/elbow are driven by
    # the scripted curve instead of live interpolation; pan/tilt keep
    # coming from the normal live pipeline the entire time either way, so
    # only the two channels proven unstable during a real hook (yaw, elbow
    # - see the HOOK_* notes in calibration_data.json) get overridden.
    punch_classifier = PunchClassifier()
    hook_start_time = None
    punch_flash_label = ""
    punch_flash_until = 0.0

    print(f"Interpolating between {len(loader.positions)} calibration positions "
          f"(nearest {mapper.k}, IDW power {IDW_POWER}). Punch classification active. Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame_time = time.monotonic()
        frame = cv2.flip(frame, 1)
        if track.CROP_RIGHT_FRAC < 1.0:
            frame = frame[:, : int(frame.shape[1] * track.CROP_RIGHT_FRAC)]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((frame_time - start_time) * 1000)
        results = landmarker.detect_for_video(mp_image, timestamp_ms)

        if not results.pose_landmarks:
            cv2.putText(frame, "No pose detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("Boxing Robot Arm - Interpolation Mode", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        lm = results.pose_landmarks[0]
        drawing_utils.draw_landmarks(frame, lm, POSE_CONNECTIONS)

        shoulder_a = lm[track.SIDE_A["shoulder"]]
        shoulder_b = lm[track.SIDE_B["shoulder"]]
        if tracked_side is track.SIDE_A:
            if shoulder_b.x + track.SIDE_SWITCH_MARGIN < shoulder_a.x:
                tracked_side = track.SIDE_B
        else:
            if shoulder_a.x + track.SIDE_SWITCH_MARGIN < shoulder_b.x:
                tracked_side = track.SIDE_A

        shoulder = lm[tracked_side["shoulder"]]
        elbow = lm[tracked_side["elbow"]]
        wrist = lm[tracked_side["wrist"]]
        hip = lm[tracked_side["hip"]]
        other_shoulder = shoulder_b if tracked_side is track.SIDE_A else shoulder_a

        low_confidence = (
            shoulder.visibility < track.VISIBILITY_MIN
            or wrist.visibility < track.VISIBILITY_MIN
            or hip.visibility < track.VISIBILITY_MIN
        )
        elbow_visible = elbow.visibility >= track.VISIBILITY_MIN

        if low_confidence:
            cv2.putText(frame, "Low confidence - holding last position", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
            cv2.imshow("Boxing Robot Arm - Interpolation Mode", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            continue

        raw_pan = track.angle_3pt(hip, shoulder, wrist, keys=("x", "z"))
        raw_tilt = track.elevation_angle(shoulder, wrist)

        if elbow_visible:
            l1_frame = track.landmark_distance(shoulder, elbow, keys=("x", "y", "z"))
            l2_frame = track.landmark_distance(elbow, wrist, keys=("x", "y", "z"))
            if elbow_l1 is None:
                elbow_l1, elbow_l2 = l1_frame, l2_frame
            else:
                elbow_l1 += track.ELBOW_LENGTH_ALPHA * (l1_frame - elbow_l1)
                elbow_l2 += track.ELBOW_LENGTH_ALPHA * (l2_frame - elbow_l2)

        wrist_shoulder_dist = track.landmark_distance(shoulder, wrist, keys=("x", "y", "z"))
        raw_elbow = (
            track.elbow_ik_angle(elbow_l1, elbow_l2, wrist_shoulder_dist)
            if elbow_l1 is not None else None
        )

        raw_yaw = track.yaw_angle(shoulder, elbow, wrist, hip) if elbow_visible else None
        if raw_yaw is not None and raw_elbow is not None:
            fade = (track.YAW_ELBOW_FADE_START - raw_elbow) / (track.YAW_ELBOW_FADE_START - track.YAW_ELBOW_FADE_END)
            raw_yaw *= track.clamp(fade, 0.0, 1.0)

        # Keep feeding the classifier real samples even while an animation
        # is playing (the camera's still watching your real arm) - just
        # don't let a detection START a second animation on top of one
        # already running.
        punch_result = punch_classifier.update(shoulder, wrist, other_shoulder, raw_yaw, frame_time)
        if punch_result:
            punch_flash_label = punch_result["type"].upper()
            punch_flash_until = frame_time + 1.0
            yaw_str = f"{punch_result['yaw']:.1f}" if punch_result["yaw"] is not None else "n/a"
            print(f"\n>>> {punch_flash_label} (speed={punch_result['speed']:.2f} "
                  f"travel={punch_result['travel']:.2f} yaw={yaw_str})")
            if punch_result["type"] == "hook" and hook_start_time is None:
                hook_start_time = frame_time
                print(">>> Playing hook animation, pan/tilt still live\n")

        smooth_pan = pan_filter(frame_time, raw_pan)
        smooth_tilt = tilt_filter(frame_time, raw_tilt)
        smooth_yaw = yaw_filter(frame_time, raw_yaw) if raw_yaw is not None else last_smooth_yaw
        smooth_elbow = elbow_filter(frame_time, raw_elbow) if raw_elbow is not None else last_smooth_elbow
        last_smooth_yaw = smooth_yaw
        last_smooth_elbow = smooth_elbow

        query = (smooth_pan, smooth_tilt, smooth_yaw, smooth_elbow)
        servo_vec, used = mapper.interpolate(query)
        pan_f, tilt_f, yaw_f, elbow_f = apply_safety_limits(servo_vec)

        # If a hook animation is playing, yaw/elbow come from the scripted
        # curve instead of live interpolation - pan/tilt (already computed
        # above from live tracking) are untouched either way. This is the
        # hybrid: the two channels proven unstable during a real hook get a
        # verified animation, everything else keeps mirroring you live.
        animating = hook_start_time is not None
        if animating:
            elapsed_ms = (frame_time - hook_start_time) * 1000.0
            anim_yaw, anim_elbow, finished = hook_animation_test.hook_animation_frame(elapsed_ms)
            _, _, yaw_f, elbow_f = apply_safety_limits((pan_f, tilt_f, anim_yaw, anim_elbow))
            if finished:
                hook_start_time = None
                animating = False
                print(">>> Hook animation complete - resuming live yaw/elbow tracking")

        # Smooth the interpolated output itself (see OUTPUT_MIN_CUTOFF/BETA
        # above) - this is what damps the jump when the nearest-neighbor
        # set changes, which raw-signal smoothing alone can't reach. BUT
        # skip it when snapped (len(used)==1, see SNAP_DISTANCE) - that
        # value is already a confident, verified calibration match, not
        # noise to smooth away - and skip it on yaw/elbow while animating,
        # for the same reason: a scripted curve isn't noise either, and
        # smoothing it would just add lag on top of carefully-tuned timing
        # (see apply_output_filter()).
        snapped = len(used) == 1
        pan_f = apply_output_filter(out_pan_filter, frame_time, pan_f, snapped)
        tilt_f = apply_output_filter(out_tilt_filter, frame_time, tilt_f, snapped)
        yaw_f = apply_output_filter(out_yaw_filter, frame_time, yaw_f, snapped or animating)
        elbow_f = apply_output_filter(out_elbow_filter, frame_time, elbow_f, snapped or animating)

        pan, tilt, yaw, elbow_out = int(pan_f), int(tilt_f), int(yaw_f), int(elbow_f)

        side_label = "A" if tracked_side is track.SIDE_A else "B"
        neighbors_str = " ".join(f"{name}(w={w:.2f},d={d:.2f})" for name, w, d in used)
        nearest_dist = used[0][2] if used else float("inf")
        low_confidence = nearest_dist > LOW_CONFIDENCE_DISTANCE
        confidence_flag = " [LOW CONFIDENCE - no nearby calibration point]" if low_confidence else ""
        print(
            f"[side {side_label}] raw=[{smooth_pan:6.1f},{smooth_tilt:6.1f},{smooth_yaw:6.1f},{smooth_elbow:6.1f}] "
            f"-> {neighbors_str} -> servo: pan={pan:3d} tilt={tilt:3d} yaw={yaw:3d} elbow={elbow_out:3d}{confidence_flag}"
        )

        current = {"pan": pan, "tilt": tilt, "yaw": yaw, "elbow": elbow_out}
        changed = {ch: last_sent[ch] is None or abs(current[ch] - last_sent[ch]) >= track.DEADBAND_DEG
                   for ch in current}
        send_vals = {ch: current[ch] if changed[ch] else last_sent[ch] for ch in current}

        time_ok = (frame_time - last_send_time) * 1000 >= track.SEND_INTERVAL_MS
        if ser and time_ok and any(changed.values()):
            line = f"{send_vals['pan']},{send_vals['tilt']},{send_vals['yaw']},{send_vals['elbow']}\n"
            try:
                ser.write(line.encode())
                last_sent = dict(send_vals)
                last_send_time = frame_time
            except serial.SerialException as e:
                print(f"WARNING: serial write failed ({e}) - is the Arduino still connected?")

        nearest_name = used[0][0] if used else "n/a"
        cv2.putText(
            frame,
            f"[{side_label}] pan:{pan} tilt:{tilt} yaw:{yaw} elbow:{elbow_out}  near:{nearest_name}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
        )
        if low_confidence:
            cv2.putText(
                frame, "LOW CONFIDENCE - no nearby calibration point", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2,
            )
        if animating:
            cv2.putText(
                frame, "HOOK ANIMATION PLAYING (yaw/elbow scripted)", (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
            )

        # Punch classification banner - same color/size treatment as
        # punch_classifier.py's standalone test, so it reads the same way
        # here: a full-width color banner is hard to miss mid-swing,
        # unlike small corner text. Briefly covers the [side A] status line
        # above while it's up (~1s) - acceptable, the banner is the more
        # important thing to see in that moment.
        if frame_time < punch_flash_until:
            color = LABEL_COLORS.get(punch_flash_label, LABEL_COLORS["PUNCH"])
            frame_h, frame_w = frame.shape[:2]
            (text_w, text_h), baseline = cv2.getTextSize(
                punch_flash_label, cv2.FONT_HERSHEY_SIMPLEX, 1.8, 4
            )
            banner_h = text_h + baseline + 30
            cv2.rectangle(frame, (0, 0), (frame_w, banner_h), color, -1)
            text_x = (frame_w - text_w) // 2
            text_y = banner_h - baseline - 15
            text_color = (255, 255, 255) if color != (0, 165, 255) else (0, 0, 0)
            cv2.putText(frame, punch_flash_label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.8, text_color, 4)

        cv2.imshow("Boxing Robot Arm - Interpolation Mode", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    if ser:
        ser.close()


if __name__ == "__main__":
    main()
