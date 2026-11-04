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
import types

import cv2

# See the matching comment in track.py - Windows Smart App Control blocks
# matplotlib's native DLL, which mediapipe's own package init pulls in
# transitively even though this project never uses it. Stubbed out here too
# since this file imports mediapipe directly, before it imports track.
if "matplotlib" not in sys.modules:
    _stub_matplotlib = types.ModuleType("matplotlib")
    _stub_pyplot = types.ModuleType("matplotlib.pyplot")
    _stub_matplotlib.pyplot = _stub_pyplot
    sys.modules["matplotlib"] = _stub_matplotlib
    sys.modules["matplotlib.pyplot"] = _stub_pyplot

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
import punch_angle_270  # noqa: E402
import voice_punch_270  # noqa: E402
import imu_punch_listener  # noqa: E402
from punch_classifier import PunchClassifier, LABEL_COLORS  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

# calibration_data_270.json currently lives in interpolation_implement/ (a
# sibling folder to this one, not inside it) - pointed here rather than
# moved, since the file already existed there before this script did.
# Named _270 specifically (distinct from calibration_data.json, which
# track_interpolation.py/track.py still use for the 180-degree hardware)
# so neither build's calibration data can get silently overwritten by the
# other now that both live in this one repo.
CALIBRATION_DATA_PATH = os.path.join(_PARENT_DIR, "interpolation_implement", "calibration_data_270.json")

# Whether punch detection/recognition is active, which detector (wrist
# IMU vs vision) gets to trigger it, and whether live mirroring runs at
# all are now asked at startup (see prompt_startup_config()) instead of
# being fixed constants here - lets one script cover pure mirroring, pure
# punch-reaction, or both, without a code edit each session.

# Overrides track.TILT_BETA (0.03) for this build only - NOT changed in
# track.py itself, since that file is shared with the 180-degree build and
# this is specifically about a 270-degree symptom (see CLAUDE.md's
# HAND_OUT_SIDE -> HAND_DOWN lag note). BETA controls how fast filtering
# loosens up as speed increases - 0.03 is the same value pan/yaw use,
# untuned for tilt's specific case of a large (77 degree), fast swing.
# track.ELBOW_BETA was raised to 0.10 for an analogous "needs to react
# faster" reason (different root cause - noise amplification, not a slow
# large swing - but the same parameter). First guess, not yet confirmed -
# watch the true_raw_tilt vs smoothed gap in the per-frame log during a
# real HAND_OUT_SIDE -> HAND_DOWN motion; tune further from what that
# actually shows rather than assuming this value is already right.
TILT_BETA_270 = 0.08

# This build's real servo range - must match ANGLE_MAX in robot_arm_270.ino
# (270, not the old 180-degree hardware's range). Used both to validate
# calibration_data.json entries aren't out of range and to clamp live
# output - a leftover hardcoded 180 here would silently corrupt or reject
# any value above 180 (e.g. HAND_DOWN's servo tilt=220), which is exactly
# the kind of bug that looks like "nothing's wrong" until a specific
# calibration point goes missing or a joint stops reaching its real range.
SERVO_ANGLE_MAX = 270

# Yaw's real neutral on this build is 70 (down=inward/toward the body=
# crash risk, up=outward=safe - see positions_270.py's header). This floor
# is a first-guess placeholder, same conservative value as
# hook_animation_270.py's untested STRIKE_END_YAW, NOT a confirmed safe
# limit yet - lower it only after testing each new value standalone, the
# same process the old hardware's floor went through (40 -> 30 -> 20).
YAW_SAFE_FLOOR_270 = 30

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

# How long the same point must stay the confident (SNAP_DISTANCE +
# SNAP_MARGIN) nearest match before actually locking to it - see the hold-
# time comment in InterpolationMapper.interpolate(). Lowered from 2.0
# (2026-10-31) - felt too sluggish holding HAND_DOWN in real testing
# (nowhere near locked in after several seconds of holding the pose
# still). 0.3 is still well above a single frame (ordinary jitter can't
# accidentally satisfy it) but far more responsive - lower it if locking
# feels sluggish, but raising SNAP_DISTANCE/SNAP_MARGIN instead is
# probably the wrong fix for flicker now that this exists (that's the
# mistake this is specifically replacing).
#
# If a pose STILL never locks in even after this change, that's a
# different problem this constant can't fix - it means that pose is never
# becoming the confident nearest match in the first place (a distance/
# calibration-data issue, not a hold-time one). HAND_DOWN specifically has
# a known note in calibration_data_270.json: its raw elbow reading (115)
# doesn't match what a fully-straight hanging arm should read, a
# MediaPipe z-depth limitation, not something hold-time tuning can fix -
# watch the per-frame "-> {name}" console output while holding the pose
# to tell which situation this actually is.
HOLD_TIME_SEC = 0.3

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

            if not all(0 <= v <= SERVO_ANGLE_MAX for v in servo_vec):
                print(f"WARNING: skipping '{name}' - servo value outside 0-{SERVO_ANGLE_MAX}: {servo_vec}")
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

        # Which calibration point has been the confident (within
        # SNAP_DISTANCE + SNAP_MARGIN) nearest match, and since when - see
        # HOLD_TIME_SEC's use in interpolate() below.
        self._hold_name = None
        self._hold_since = None

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

    def interpolate(self, query, now):
        """query: (pan, tilt, yaw, elbow) raw values. now: time.monotonic()
        timestamp for this frame, used for the hold-time requirement below.
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

        confident = nearest[0][0] < SNAP_DISTANCE and margin_ok
        candidate_name = nearest[0][1] if confident else None

        # Require the SAME point to stay the confident match for
        # HOLD_TIME_SEC before actually locking to it, not just being
        # close enough for one frame. Pure distance-based snapping (the
        # old behavior) re-decides fresh every frame, so a raw reading
        # jittering right at the SNAP_DISTANCE/SNAP_MARGIN boundary could
        # flicker in and out of "snapped" even while genuinely holding
        # still - reported as "not very good" after live testing. This
        # doesn't change WHAT triggers a snap, only requires it to be
        # true continuously for a while first.
        if candidate_name != self._hold_name:
            self._hold_name = candidate_name
            self._hold_since = now if confident else None

        if confident and (now - self._hold_since) >= HOLD_TIME_SEC:
            dist, name, servo_vec = nearest[0]
            return servo_vec, [(name, 1.0, dist)]

        # max(dist, epsilon) guards against a divide-by-zero when a point
        # is at (or extremely near) zero distance but hasn't been held
        # long enough yet to actually snap - previously unreachable
        # because a confident match always returned immediately, before
        # HOLD_TIME_SEC existed to delay that return.
        weights = [1.0 / (max(dist, 1e-6) ** self.power) for dist, _, _ in nearest]
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
    about track.py's specific formulas.

    NOT using track.YAW_SAFE_FLOOR (20) here - that value is specific to
    the OLD 180-degree hardware's horn mounting/neutral and means nothing
    on this build, where yaw's real neutral is 70 (see positions_270.py).
    YAW_SAFE_FLOOR_270 below is a first-guess placeholder matching
    hook_animation_270.py's conservative, untested strike depth - replace
    it with a real confirmed floor once that's been tested standalone,
    the same way the old hardware's floor was only lowered after each
    value was separately verified safe."""
    pan, tilt, yaw, elbow = servo_vec
    yaw = max(yaw, YAW_SAFE_FLOOR_270)
    return (
        track.clamp(pan, 0, SERVO_ANGLE_MAX),
        track.clamp(tilt, 0, SERVO_ANGLE_MAX),
        track.clamp(yaw, 0, SERVO_ANGLE_MAX),
        track.clamp(elbow, 0, SERVO_ANGLE_MAX),
    )


def prompt_startup_config():
    """Asks a few questions at launch instead of requiring code edits each
    session (the old PUNCH_DETECTION_ENABLED constant, or physically
    disconnecting the IMU, to change behavior) - lets the same script run
    as pure mirroring, pure punch-reaction, or both. Punch TYPE routing is
    no longer a session choice (there used to be an "mpu or vision?"
    question here) - see the hook-vision/uppercut-IMU split in the main
    loop below, confirmed by a real paired-agreement test across 55 real
    throws (imu_vision_agreement_results_2026-11-03.md): vision calls
    hooks right 87% of the time but uppercuts only 22%; the wrist IMU is
    the mirror image, 95% on uppercuts and only 30% on hooks. Each type
    always goes to whichever detector is actually good at it - wearing
    the brace only controls whether the IMU half is available at all."""
    def ask_yes_no(question, default_yes=True):
        suffix = "[Y/n]" if default_yes else "[y/N]"
        while True:
            answer = input(f"{question} {suffix}: ").strip().lower()
            if answer == "":
                return default_yes
            if answer in ("y", "yes"):
                return True
            if answer in ("n", "no"):
                return False
            print("Please answer y or n.")

    print("=== Boxing Robot Arm - Session Setup ===")
    wearing_brace = ask_yes_no("Are you wearing the wrist brace with the MPU-6050?")
    mirror_enabled = ask_yes_no("Enable live mirroring (arm follows your real arm)?")
    punch_enabled = ask_yes_no("Enable punch recognition (arm plays hook/uppercut animations)?")

    if punch_enabled:
        if wearing_brace:
            print("Punch routing: HOOK from vision, UPPERCUT from the wrist IMU.")
        else:
            print("No brace - punch routing: vision only (hook AND uppercut - IMU unavailable).")

    print(f"=== wearing_brace={wearing_brace}  mirror={mirror_enabled}  punch={punch_enabled} ===\n")
    return {
        "wearing_brace": wearing_brace,
        "mirror_enabled": mirror_enabled,
        "punch_enabled": punch_enabled,
    }


def main():
    config = prompt_startup_config()

    loader = CalibrationLoader(CALIBRATION_DATA_PATH)
    mapper = InterpolationMapper(loader.positions)

    ser = None
    try:
        ser = serial.Serial(track.SERIAL_PORT, track.BAUD_RATE, timeout=1, write_timeout=1)
        print(f"Connected to Arduino on {track.SERIAL_PORT} - waiting for it to finish booting...")
        # robot_arm_270.ino now blocks for ~4s doing wrist-IMU calibration
        # before it ever reaches loop() (see that file's header) - a blind
        # sleep(2) isn't long enough anymore and commands sent too early
        # would just be dropped before the Arduino is reading them. Wait
        # for its own "imu calibrated" readiness line instead, bounded so
        # this doesn't hang forever if a different/older sketch is loaded.
        deadline = time.monotonic() + 10.0
        ready = False
        while time.monotonic() < deadline:
            line = ser.readline().decode(errors="replace").strip()
            if "imu calibrated" in line:
                print(f"Arduino ready: {line}")
                ready = True
                break
        if not ready:
            print("WARNING: did not see 'imu calibrated' within 10s - continuing anyway "
                  "(a different/older sketch may be loaded, or the wrist wasn't held still).")
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
    tilt_filter = track.OneEuroFilter(track.TILT_MIN_CUTOFF, TILT_BETA_270, track.D_CUTOFF)  # see TILT_BETA_270 above
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
    # punch_classifier.py. ONE animation path now (unlike the 180-degree
    # build, which kept automatic-hook and voice-cued as two separate
    # mechanisms) - every trigger, voice or automatic, plays the same full
    # punch_angle_270.punch_animation_frame() (pan/tilt/yaw/elbow fully
    # scripted, except pan is deliberately discarded below and kept live -
    # see the comment where it's applied). Priority each time a strike is
    # detected:
    #   1. A pending voice/text-logged tilt (voice_punch_270.py) - wins if
    #      present, maps to whichever of the 4 PUNCHES it snapped to.
    #   2. No pending tilt: fall back to the classifier's own geometric
    #      guess - "hook" plays PUNCHES["4"] (Hook), "uppercut" plays
    #      PUNCHES["1"] (Uppercut). Unclassified "punch" does nothing
    #      special, same as before.
    punch_classifier = PunchClassifier()

    punch_anim_start_time = None
    punch_anim_start_pos = None   # (pan, tilt, yaw, elbow) - wherever live tracking
                                   # actually was the instant the strike was detected,
                                   # NOT a fixed/hardcoded start - forcing a fixed start
                                   # here was tried and reverted on the 180-degree build
                                   # (the "cocks back to load the hook" bug, see
                                   # hook_animation_test.py's STRIKE_START_YAW history) -
                                   # don't reintroduce that mistake.
    punch_anim_target_pos = None  # (pan, tilt, yaw, elbow) - the resolved punch's full position
    punch_anim_pan_swing = 0.0    # degrees - PAN_SWING_BY_KEY.get(key, 0.0), 0 for everything but Uppercut
    punch_anim_hold_ms = punch_angle_270.HOLD_DURATION_MS  # HOLD_DURATION_BY_KEY.get(key, default), longer for Uppercut

    # Voice/text-cued tilt selection (voice_punch_270.py) - say/type a
    # number before throwing instead of relying on the classifier's guess.
    # Runs in its own thread; only consumed (pop_next_tilt()) right when
    # punch_classifier's speed/travel gate actually fires below, never
    # every frame, so an unrelated later motion can't accidentally consume
    # a stale logged tilt.
    voice_listener = voice_punch_270.create_listener()  # voice_punch_270.INPUT_MODE picks voice vs. typed
    voice_listener.start()

    # Wrist IMU punch trigger (imu_punch_listener.py) - reads off the SAME
    # serial connection as the servo commands above (the IMU logic now
    # lives inside robot_arm_270.ino itself, not a second Arduino). Unlike
    # voice, this can fire a punch animation ON ITS OWN, without waiting
    # for the vision gate - see the trigger-priority block below and that
    # module's own docstring for why (speed is the entire point of this
    # sensor).
    imu_listener = (
        imu_punch_listener.IMUPunchListener(ser)
        if (ser and config["wearing_brace"]) else None
    )
    if imu_listener:
        imu_listener.start()

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
        # already running. Both detectors keep running/updating their own
        # internal state regardless of which one actually gets to trigger
        # below (preserves cooldown/history continuity either way).
        punch_result = (
            punch_classifier.update(shoulder, wrist, other_shoulder, raw_yaw, raw_elbow, frame_time)
            if config["punch_enabled"] else None
        )
        imu_punch_cmd = (
            imu_listener.pop_next_punch()
            if (imu_listener and config["punch_enabled"]) else None
        )
        start_punch_this_frame = False
        punch_target_pos = None  # (pan, tilt, yaw, elbow) resolved below if a punch fires this frame
        punch_target_pan_swing = 0.0
        punch_target_hold_ms = punch_angle_270.HOLD_DURATION_MS

        # Hook-vision / uppercut-IMU split, confirmed by a real paired
        # agreement test across 55 real throws
        # (imu_vision_agreement_results_2026-11-03.md): vision calls
        # hooks right 87% of the time but uppercuts only 22% (it mostly
        # reads a real uppercut as "hook" or an unclassified "punch");
        # the wrist IMU is the mirror image - 95% on uppercuts, only 30%
        # on hooks. So each type is ALWAYS routed to whichever detector
        # is actually good at it, instead of picking one source for the
        # whole session the way this used to work. A naive "both must
        # agree" rule was tried on the same data and scored WORSE than
        # either signal alone - don't reintroduce that.
        if imu_punch_cmd and imu_punch_cmd["type"] in ("uppercut", "punch") and punch_anim_start_time is None:
            # Wrist IMU called uppercut (or the low-rotation "punch"
            # fallback - same PUNCHES["1"] target either way, matching
            # imu_punch_listener.TYPE_TO_KEY). Takes priority and can
            # start an animation ON ITS OWN, without waiting for
            # punch_classifier's speed/travel gate to also agree - the
            # whole point of the IMU chapter (see
            # imu_punch_listener.py's docstring). An IMU "hook" call is
            # deliberately NOT trusted here (only 30% accuracy) - it
            # falls through and lets vision handle that frame instead.
            resolved_key = imu_punch_cmd["key"]
            effective_label, resolved_target = punch_angle_270.PUNCHES[resolved_key]
            resolved_pan_swing = punch_angle_270.PAN_SWING_BY_KEY.get(resolved_key, 0.0)
            resolved_hold_ms = punch_angle_270.HOLD_DURATION_BY_KEY.get(resolved_key, punch_angle_270.HOLD_DURATION_MS)

            punch_flash_label = f"IMU {effective_label or imu_punch_cmd['type']}".upper()
            punch_flash_until = frame_time + 1.0
            print(f"\n>>> {punch_flash_label} (from wrist IMU)")

            start_punch_this_frame = True
            punch_target_pos = resolved_target
            punch_target_pan_swing = resolved_pan_swing
            punch_target_hold_ms = resolved_hold_ms
        elif punch_result and (punch_result["type"] == "hook" or not config["wearing_brace"]):
            # Vision fired. Trusted outright when it specifically calls
            # "hook" (87% accuracy - what vision is actually good at).
            # When it instead calls "uppercut"/unclassified "punch", that
            # fallback is only trusted if there's no wrist IMU this
            # session (no brace) - with the brace on, the IMU branch
            # above already owns that type, and vision's 22%-accurate
            # uppercut guess is deliberately ignored here rather than
            # risking a wrong animation or a double-trigger race against
            # the IMU. A pending voice/text tilt still wins over either,
            # consumed only here (not every frame) so an unrelated later
            # motion can't burn a stale logged tilt (see
            # PENDING_EXPIRY_SEC in voice_punch_270.py).
            voice_tilt_cmd = voice_listener.pop_next_tilt()
            if voice_tilt_cmd:
                resolved_key = voice_tilt_cmd["key"]
            elif punch_result["type"] == "hook":
                resolved_key = "4"
            else:
                # No-brace fallback only (see the elif condition above) -
                # punch_classifier.py can't tell a jab apart from an
                # uppercut (both read as a fast, roughly-vertical motion
                # with no strong yaw signal - a real geometric ambiguity,
                # same category as the wide-hook-vs-reach limit documented
                # for the 180-degree build), and there's no jab animation
                # yet, so anything that's not a hook defaults to Uppercut.
                resolved_key = "1"
            effective_label, resolved_target = punch_angle_270.PUNCHES[resolved_key]
            # PAN_SWING_BY_KEY is Uppercut-only (confirmed in
            # punch_angle_270.py's own standalone test before being wired
            # here) - every other key is 0, meaning pan keeps getting
            # discarded/kept live below exactly as before.
            resolved_pan_swing = punch_angle_270.PAN_SWING_BY_KEY.get(resolved_key, 0.0)
            resolved_hold_ms = punch_angle_270.HOLD_DURATION_BY_KEY.get(resolved_key, punch_angle_270.HOLD_DURATION_MS)

            punch_flash_label = (effective_label or punch_result["type"]).upper()
            punch_flash_until = frame_time + 1.0
            yaw_str = f"{punch_result['yaw']:.1f}" if punch_result["yaw"] is not None else "n/a"
            print(f"\n>>> {punch_flash_label} (speed={punch_result['speed']:.2f} "
                  f"travel={punch_result['travel']:.2f} yaw={yaw_str})")

            if punch_anim_start_time is None:
                # Don't grab the live start position here - this frame's
                # pan_f/tilt_f/yaw_f/elbow_f haven't been computed yet
                # (that happens below). Just flag it.
                start_punch_this_frame = True
                punch_target_pos = resolved_target
                punch_target_pan_swing = resolved_pan_swing
                punch_target_hold_ms = resolved_hold_ms

        smooth_pan = pan_filter(frame_time, raw_pan)
        smooth_tilt = tilt_filter(frame_time, raw_tilt)
        smooth_yaw = yaw_filter(frame_time, raw_yaw) if raw_yaw is not None else last_smooth_yaw
        smooth_elbow = elbow_filter(frame_time, raw_elbow) if raw_elbow is not None else last_smooth_elbow
        last_smooth_yaw = smooth_yaw
        last_smooth_elbow = smooth_elbow

        query = (smooth_pan, smooth_tilt, smooth_yaw, smooth_elbow)
        servo_vec, used = mapper.interpolate(query, frame_time)
        pan_f, tilt_f, yaw_f, elbow_f = apply_safety_limits(servo_vec)

        if start_punch_this_frame:
            punch_anim_start_time = frame_time
            punch_anim_start_pos = (pan_f, tilt_f, yaw_f, elbow_f)  # real live values, not a fixed start
            punch_anim_target_pos = punch_target_pos
            punch_anim_pan_swing = punch_target_pan_swing
            punch_anim_hold_ms = punch_target_hold_ms
            print(f">>> Playing {punch_flash_label} animation, "
                  f"{'pan swings too' if punch_anim_pan_swing else 'pan still live'}\n")

        # tilt/yaw/elbow always come from the scripted curve
        # (punch_angle_270.punch_animation_frame - reposition, then
        # strike). pan only comes from it too when punch_anim_pan_swing is
        # non-zero (Uppercut, confirmed good in punch_angle_270.py's own
        # standalone test before being wired here) - for every other
        # punch the animated pan is discarded and pan_f (already computed
        # above from live tracking) is kept instead, same as before this
        # existed. Same cooldown re-arming as the 180-degree build, for
        # the same double-dip reason (see that build's CLAUDE.md).
        punch_animating = punch_anim_start_time is not None
        if punch_animating:
            elapsed_ms = (frame_time - punch_anim_start_time) * 1000.0
            anim_pan, anim_tilt, anim_yaw, anim_elbow, finished = punch_angle_270.punch_animation_frame(
                elapsed_ms, start_pos=punch_anim_start_pos, target_pos=punch_anim_target_pos,
                pan_swing_deg=punch_anim_pan_swing, hold_duration_ms=punch_anim_hold_ms,
            )
            used_pan = anim_pan if punch_anim_pan_swing else pan_f
            pan_f, tilt_f, yaw_f, elbow_f = apply_safety_limits((used_pan, anim_tilt, anim_yaw, anim_elbow))
            if finished:
                punch_anim_start_time = None
                punch_animating = False
                punch_anim_pan_swing = 0.0
                punch_anim_hold_ms = punch_angle_270.HOLD_DURATION_MS
                # Restart the classifier's cooldown from NOW, not from the
                # original detection instant - otherwise a real arm
                # retraction right after the animation (which often
                # outlasts the cooldown) reads as a second punch. Same
                # "double-dip" fix as the 180-degree build.
                punch_classifier.last_punch_at = frame_time
                print(">>> Punch animation complete - resuming live tracking")

        # Smooth the interpolated output itself (see OUTPUT_MIN_CUTOFF/BETA
        # above) - this is what damps the jump when the nearest-neighbor
        # set changes, which raw-signal smoothing alone can't reach. BUT
        # skip it when snapped (len(used)==1, see SNAP_DISTANCE) - that
        # value is already a confident, verified calibration match, not
        # noise to smooth away - and skip it on tilt/yaw/elbow while the
        # punch animation owns them, for the same reason: it isn't noise
        # either, and smoothing it would just add lag on top of carefully-
        # tuned timing (see apply_output_filter()). pan only needs the
        # same bypass while it's ACTUALLY being scripted (Uppercut's swing
        # - punch_anim_pan_swing nonzero) - for every other punch pan stays
        # live and gets normal smoothing like any other live motion.
        snapped = len(used) == 1
        pan_scripted = punch_animating and bool(punch_anim_pan_swing)
        pan_f = apply_output_filter(out_pan_filter, frame_time, pan_f, snapped or pan_scripted)
        tilt_f = apply_output_filter(out_tilt_filter, frame_time, tilt_f, snapped or punch_animating)
        yaw_f = apply_output_filter(out_yaw_filter, frame_time, yaw_f, snapped or punch_animating)
        elbow_f = apply_output_filter(out_elbow_filter, frame_time, elbow_f, snapped or punch_animating)

        pan, tilt, yaw, elbow_out = int(pan_f), int(tilt_f), int(yaw_f), int(elbow_f)

        side_label = "A" if tracked_side is track.SIDE_A else "B"
        neighbors_str = " ".join(f"{name}(w={w:.2f},d={d:.2f})" for name, w, d in used)
        nearest_dist = used[0][2] if used else float("inf")
        low_confidence = nearest_dist > LOW_CONFIDENCE_DISTANCE
        confidence_flag = " [LOW CONFIDENCE - no nearby calibration point]" if low_confidence else ""
        # NOTE: the old "raw=" label here was wrong - those were already
        # SMOOTHED (post-filter) values, not the true raw MediaPipe
        # reading, which made it impossible to tell filter lag apart from
        # anything else just by reading this line. raw_tilt (true,
        # pre-filter) is added specifically to diagnose the HAND_OUT_SIDE
        # -> HAND_DOWN lag (see CLAUDE.md) - if raw_tilt swings fast but
        # smooth_tilt trails far behind it for a couple seconds, that's
        # the OneEuroFilter lagging, not the interpolation/blend stage.
        print(
            f"[side {side_label}] true_raw_tilt={raw_tilt:6.1f}  "
            f"smoothed=[{smooth_pan:6.1f},{smooth_tilt:6.1f},{smooth_yaw:6.1f},{smooth_elbow:6.1f}] "
            f"-> {neighbors_str} -> servo: pan={pan:3d} tilt={tilt:3d} yaw={yaw:3d} elbow={elbow_out:3d}{confidence_flag}"
        )

        current = {"pan": pan, "tilt": tilt, "yaw": yaw, "elbow": elbow_out}
        changed = {ch: last_sent[ch] is None or abs(current[ch] - last_sent[ch]) >= track.DEADBAND_DEG
                   for ch in current}
        send_vals = {ch: current[ch] if changed[ch] else last_sent[ch] for ch in current}

        time_ok = (frame_time - last_send_time) * 1000 >= track.SEND_INTERVAL_MS
        # When mirroring is off (config["mirror_enabled"]==False), only a
        # punch animation is allowed to move the arm - continuous live-
        # tracking values are computed above regardless (cheap, and
        # vision-based punch detection needs them anyway) but never SENT
        # unless an animation is actually playing, so the arm just holds
        # still between punches instead of also following your real arm.
        mirror_ok = config["mirror_enabled"] or punch_animating
        if ser and time_ok and any(changed.values()) and mirror_ok:
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
        if punch_animating:
            cv2.putText(
                frame, "PUNCH ANIMATION PLAYING (tilt/yaw/elbow scripted)", (10, 80),
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

    voice_listener.stop()
    if imu_listener:
        imu_listener.stop()
    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    if ser:
        ser.close()


if __name__ == "__main__":
    main()
