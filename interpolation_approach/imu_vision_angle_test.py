"""
Boxing Robot Arm - Fused Vision+IMU Continuous Punch Angle Display
--------------------------------------------------------------------
Camera + wrist IMU running together, like imu_vision_agreement_test.py,
but a different goal: instead of grading IMU-vs-vision TYPE agreement,
this reports a CONTINUOUS angle for each thrown punch - a step toward the
project's long-term goal of driving the arm continuously instead of
snapping between 4 canned positions (UPPERCUT/HIGH_OVERHAND/OVERHAND/
HOOK - see positions_270.py). Servo commands are OPT-IN (see "Drive the
real arm" prompt below) - display/log-only by default, same safety
property as imu_vision_agreement_test.py when declined. Run this INSTEAD
OF track_interpolation_270.py (same serial port, only one process can
hold it open). Does NOT touch robot_arm_270.ino, track_interpolation_270.py,
punch_classifier.py, track.py, positions_270.py, or punch_angle_270.py -
pure read-only reuse of all of them.

ARM-DRIVING MODE (2026-11-05): when enabled, mirrors ONLY punches, not
continuous live tracking - the arm sits at positions_270.POSITIONS
["REST_GUARD"] and does nothing until a real punch fires, then plays
punch_angle_270.punch_animation_frame() to the resolved target and eases
back to rest after. Vision's event drives a HOOK-FAMILY animation with a
CONTINUOUS tilt (not a fixed lookup) computed from the measured angle_deg
via HOOK_FAMILY_TILT_ANCHORS - real, hand-jogged tilt values the user
measured directly on the hardware via robot_arm_270.ino's serial jog
commands (res/tilt_ACTUAL_angles.png). A reading outside that anchor
range is SUPPRESSED, not clamped-and-fired - real test data showed
recovery/retraction motions reading near -101 degrees, well outside the
family's range, and firing the arm on those would look like it's reacting
to non-punch motion.

TRIGGER PRIORITY (fixed 2026-11-06, tightened same day after a second
live test - see UPPERCUT_CONFIRM_WINDOW_SEC and
VISION_OVERRIDE_MAX_ANGLE_DEG's comments for the full story): vision is
the priority signal for hook, but only an override threshold's worth of
confidently hook-shaped reading can cancel an ALREADY-PENDING IMU
uppercut call - vision's full hook-family range still triggers normally
on its own otherwise. IMU is the ONLY thing that can ever trigger
uppercut; its call arms a short confirm window rather than firing
instantly, so a genuinely misread real hook (IMU is only ~30% accurate on
hooks) still has a brief chance to be corrected, without that same window
letting ambiguous, diagonal-reading real uppercuts get stolen by vision
too easily (an earlier, looser version of this fix did exactly that).
Vision never triggers uppercut itself.

WHY VISION, NOT THE WRIST IMU, SUPPLIES THE ANGLE: a prior attempt
(imu_punch_signed_angle_test.ino, see CLAUDE.md's "Wrist IMU punch
detector" chapter) computed a continuous SIGNED angle from the wrist IMU
ALONE - accelerometer direction at impact vs. an adaptive "up" reference.
Four structurally different attempts all failed the same way: real hooks
collapsed toward the uppercut reading unless thrown in an exaggerated,
atypical way. Root cause: a 3-axis accelerometer alone has only ONE real
physical reference (gravity) - the second axis needed to resolve a
SIGNED angle has no physical measurement to anchor to, so it's
contaminated by terminal wrist-roll/pronation at impact. Formally
shelved - do not re-derive a signed angle from IMU accel-direction alone.
Vision never had this problem (it measures real body-frame position, not
single-accelerometer direction), so it's the angle source here. The IMU's
role is narrowed to what it's already confirmed good at: fast/precise
punch TIMING (vision lags more and can miss very fast throws) and its own
already-trusted binary hook/uppercut call - logged as a cross-check only,
never compared numerically to vision's angle (different coordinate
systems entirely: the firmware's angleDeg is unsigned, accel-vs-refUp;
vision's angle_deg here is signed, atan2-based).

ANGLE CONVENTION: punch_classifier.PunchClassifier.update() already
computes dx, dy at the exact instant its speed/travel gate fires - net
wrist displacement over the detection window, shoulder-width normalized,
in UNFLIPPED image coords (image y grows downward, unlike
track.elevation_angle()'s own dy, which IS flipped). This file converts
that into a signed angle via:

    angle_deg = degrees(atan2(-dy, dx))

matching elevation_angle()'s up-positive convention (a rise makes dy<0,
so -dy>0, so atan2 gives +90). Unlike elevation_angle() (which uses
abs(dx), clamped to (-90,+90] since it can't tell left/right reach
apart), this uses SIGNED dx directly -> full (-180,+180] range:
0=rightward, 90=up, +-180=leftward, -90=down. NOT yet verified against
real throws - the first live session IS the verification, not just a
demo. See the sanity table below and in the project plan.

NOT CALIBRATED against the existing discrete PUNCH_TILTS/POSITIONS
values on purpose (a deliberate scope choice, not an oversight) - this
is a raw geometric measurement tool. Mapping it onto the existing punch
family's servo scale, and eventually wiring it into live control, are
both explicitly separate future steps.

Usage:
    python imu_vision_angle_test.py
Throw real punches (hook, uppercut, overhand, anything) - each detected
punch prints its angle and draws a line from your shoulder in that
direction, labeled with the degree value, held ~1s. 'q' to quit (prints a
final summary).

Sanity check while testing (intuition check, NOT a calibration target):
    Hook toward frame-right   -> angle near    0 deg -> line points right
    Hook toward frame-left    -> angle near +-180 deg -> line points left
    Uppercut (wrist rises)    -> angle near  +90 deg -> line points up
    Wrist drops               -> angle near  -90 deg -> line points down
If hooks and uppercuts cluster near the SAME angle regardless of type,
that's a bug (stale/frozen dx,dy), not a repeat of the IMU's information
limit - vision doesn't share that sensing ceiling.
"""

import csv
import math
import os
import sys
import threading
import time

import cv2
import mediapipe as mp
import serial

_PARENT_DIR = os.path.dirname(os.path.abspath(__file__))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)
_ROOT_DIR = os.path.dirname(_PARENT_DIR)
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

import track  # noqa: E402
from punch_classifier import PunchClassifier, LABEL_COLORS  # noqa: E402
from positions_270 import POSITIONS  # noqa: E402
import punch_angle_270  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

# Same pairing constants as imu_vision_agreement_test.py, unchanged - see
# that file's own comments for why these specific values (IMU reacts well
# under a second, vision needs several frames plus inference time; 1.5s
# comfortably covers both without risking pairing two separate throws).
PAIR_WINDOW_SEC = 1.5
FINALIZE_TIMEOUT_SEC = 2.0

# How long the shoulder-anchored angle line + label stays on screen after
# a detected punch - same value as FLASH_DURATION_SEC elsewhere in this
# codebase (punch_classifier.py, track_interpolation_270.py), so this
# tool's timing feels consistent with everything else already built.
ANGLE_DISPLAY_HOLD_SEC = 1.0
LINE_LENGTH_PX = 150
LINE_THICKNESS_PX = 3

LOG_PATH = os.path.join(_PARENT_DIR, "imu_vision_angle_log.csv")
CSV_FIELDS = [
    "timestamp", "paired",
    "vision_angle_deg", "vision_type", "vision_dx", "vision_dy",
    "imu_type", "imu_angle_deg", "imu_elapsed_ms",
]

# Continuous hook-family tilt anchors - real, hand-jogged servo tilt values
# matched to rough vision angle_deg directions, confirmed by the user
# directly on the hardware via robot_arm_270.ino's serial jog commands
# (tilt:<value>) - see res/tilt_ACTUAL_angles.png (2026-11-05). (vision
# angle_deg, tilt), sorted descending by angle. Pan/yaw/elbow for every
# point on this curve come from positions_270.POSITIONS["HOOK"] - only
# tilt varies, same as HOOK/OVERHAND/HIGH_OVERHAND already do there.
# Straight uppercut is deliberately NOT on this curve - it stays routed
# through the IMU's own fixed POSITIONS["UPPERCUT"] target (95% accurate
# there vs vision's 22% - see CLAUDE.md's "Wrist IMU punch detector"
# chapter), never through this continuous vision-angle mapping.
HOOK_FAMILY_TILT_ANCHORS = [
    (45.0, 67),
    (0.0, 120),
    (-45.0, 187),
    (-90.0, 240),
]

# This build's real servo range and yaw crash-risk floor - duplicated
# locally rather than imported, same reasoning as
# track_interpolation_270.py's own SERVO_ANGLE_MAX/YAW_SAFE_FLOOR_270 (NOT
# pulled from track.py, which is shared with the different 180-degree
# hardware and has different safe values).
SERVO_ANGLE_MAX = 270
YAW_SAFE_FLOOR_270 = 30

# Plain reposition-only ease back to REST_GUARD between punches -
# deliberately NOT punch_angle_270.punch_animation_frame() (that function
# always ends in a yaw strike-sweep, which "returning to rest" must never
# do). Same speed-constant spirit as that file's own reposition phase.
RETURN_TO_REST_SPEED_DEG_PER_SEC = 300
RETURN_TO_REST_MIN_DURATION_MS = 60

# How long an IMU "uppercut"/"punch" call waits, unactuated, before it's
# trusted - added 2026-11-06 after a live test showed hook/uppercut
# confusion: IMU is only ~30% accurate on real hooks and tends to call a
# real hook "uppercut" instead (CLAUDE.md's 2026-11-03 agreement-test
# numbers), and IMU typically fires well before vision's own slower,
# window-based detection catches up - so triggering on the IMU call
# immediately gave it an unfair, uncontested head start over vision even
# on throws that were actually hooks. This window gives vision a real
# chance to override it (see the trigger-priority block in main() and
# VISION_OVERRIDE_MAX_ANGLE_DEG below) before committing. Lowered from an
# initial 0.35 after the SAME live test showed a new problem the other
# way - real uppercuts started getting missed, vision stealing them as
# "hook" before IMU's window elapsed. 0.15 is a tighter, still-untested
# guess paired with the new VISION_OVERRIDE_MAX_ANGLE_DEG restriction
# below (the real fix for that problem - this alone wouldn't have been
# enough) - tune further after watching it live.
UPPERCUT_CONFIRM_WINDOW_SEC = 0.15

# How confidently "hook" a vision reading has to be to override an
# ALREADY-PENDING IMU uppercut candidate specifically - added 2026-11-06.
# angle_to_tilt()'s full HOOK_FAMILY_TILT_ANCHORS range (-90 to +45) is
# right for vision's OWN standalone triggering (no IMU candidate
# pending), but real test data (imu_vision_angle_results_2026-11-05.md)
# showed genuine uppercuts - especially diagonal ones - commonly read
# 33-48 degrees, well inside that full range, which let them wrongly
# cancel a correct, already-pending IMU uppercut call. Real hooks in that
# same data clustered tightly -5 to +17 degrees. 20 sits comfortably
# above the validated hook cluster and comfortably below the diagonal-
# uppercut cluster - only readings this confidently "hook-shaped" are
# allowed to veto IMU; anything above it defers to the pending IMU
# candidate instead (still logged, just not treated as an override).
VISION_OVERRIDE_MAX_ANGLE_DEG = 20.0

# ============================================================


class ImuSerialReader:
    """Background-thread reader for 'PUNCH,<type>,<angleDeg>,<elapsedMs>,...'
    lines - the SAME line format robot_arm_270.ino already prints for the
    LIVE binary classifier (not touched by this file). Queues every event
    (imu_vision_agreement_test.py's pattern) rather than a single pending
    slot (imu_punch_listener.IMUPunchListener's pattern), since every
    event needs to reach the CSV, not just the latest."""

    def __init__(self, ser):
        self._ser = ser
        self._events = []
        self._lock = threading.Lock()
        self._thread = None
        self._running = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def drain(self):
        """Returns and clears all events queued since the last call -
        each is {"type", "angle_deg", "elapsed_ms", "t"} (t =
        time.monotonic() when the line was received)."""
        with self._lock:
            events, self._events = self._events, []
        return events

    def _listen_loop(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line.startswith("PUNCH,"):
                continue
            fields = line.split(",")
            if len(fields) < 4:
                continue
            try:
                angle_deg = float(fields[2])
                elapsed_ms = int(fields[3])
            except ValueError:
                continue
            with self._lock:
                self._events.append({
                    "type": fields[1], "angle_deg": angle_deg,
                    "elapsed_ms": elapsed_ms, "t": time.monotonic(),
                })
            print(f"[imu] {fields[1]} angleDeg={angle_deg:.1f} elapsed={elapsed_ms}ms")


def _append_row(path, row, write_header):
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def _angle_endpoint(shoulder_px, angle_deg, length_px):
    """Pixel-space endpoint of a line from shoulder_px, length_px long, in
    direction angle_deg - same up-positive/signed convention as
    track.elevation_angle() and this file's own vision angle_deg (0=right,
    90=up, +-180=left, -90=down). Pixel y grows DOWNWARD, opposite that
    convention, so y is negated here - the exact inverse of the flip
    elevation_angle() itself applies going from image coords to its own
    up-positive output."""
    rad = math.radians(angle_deg)
    end_x = shoulder_px[0] + length_px * math.cos(rad)
    end_y = shoulder_px[1] - length_px * math.sin(rad)
    return (int(round(end_x)), int(round(end_y)))


def draw_punch_angle(frame, shoulder_px, angle_deg, color, type_label):
    start_px = (int(round(shoulder_px[0])), int(round(shoulder_px[1])))
    end_px = _angle_endpoint(shoulder_px, angle_deg, LINE_LENGTH_PX)
    cv2.line(frame, start_px, end_px, color, LINE_THICKNESS_PX)
    cv2.circle(frame, end_px, 5, color, -1)
    label = f"{type_label.upper()} {angle_deg:+.0f} deg"
    cv2.putText(frame, label, (end_px[0] + 10, end_px[1]),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)


def angle_to_tilt(angle_deg):
    """Bracket-and-linearly-interpolate angle_deg (vision's atan2-based
    reading) between the 2 nearest HOOK_FAMILY_TILT_ANCHORS points - same
    style as interpolation_approach/imu_angle_map.py's angle_to_target().
    Returns None if angle_deg falls outside the anchor range - deliberately
    SUPPRESSED, not clamped-and-fired (see HOOK_FAMILY_TILT_ANCHORS's own
    comment for why)."""
    hi_angle, _ = HOOK_FAMILY_TILT_ANCHORS[0]
    lo_angle, _ = HOOK_FAMILY_TILT_ANCHORS[-1]
    if angle_deg > hi_angle or angle_deg < lo_angle:
        return None
    for (a1, t1), (a2, t2) in zip(HOOK_FAMILY_TILT_ANCHORS, HOOK_FAMILY_TILT_ANCHORS[1:]):
        if a2 <= angle_deg <= a1:
            frac = (a1 - angle_deg) / (a1 - a2)
            return t1 + (t2 - t1) * frac
    return None  # unreachable given the range check above


def apply_safety_limits(pan, tilt, yaw, elbow):
    """Same hardware safety clamps track_interpolation_270.py applies to
    every live servo command - duplicated locally (see
    SERVO_ANGLE_MAX/YAW_SAFE_FLOOR_270 above) rather than imported, since
    that file is the continuous-mirroring live loop and this one
    deliberately isn't."""
    yaw = max(yaw, YAW_SAFE_FLOOR_270)
    return (
        track.clamp(pan, 0, SERVO_ANGLE_MAX),
        track.clamp(tilt, 0, SERVO_ANGLE_MAX),
        track.clamp(yaw, 0, SERVO_ANGLE_MAX),
        track.clamp(elbow, 0, SERVO_ANGLE_MAX),
    )


def ease_to_rest_frame(elapsed_ms, start_pos):
    """Plain 4-channel ease from start_pos to REST_GUARD - NOT
    punch_angle_270.punch_animation_frame(), which always ends in a yaw
    strike-sweep that "returning to rest" must never do. Pure/stateless,
    same calling convention as punch_animation_frame() otherwise. Returns
    (pan, tilt, yaw, elbow, finished)."""
    target_pos = POSITIONS["REST_GUARD"]
    start_pan, start_tilt, start_yaw, start_elbow = start_pos
    tgt_pan, tgt_tilt, tgt_yaw, tgt_elbow = target_pos

    distance = max(
        abs(tgt_pan - start_pan), abs(tgt_tilt - start_tilt),
        abs(tgt_yaw - start_yaw), abs(tgt_elbow - start_elbow),
    )
    duration_ms = max(distance / RETURN_TO_REST_SPEED_DEG_PER_SEC * 1000.0, RETURN_TO_REST_MIN_DURATION_MS)
    if elapsed_ms >= duration_ms:
        return tgt_pan, tgt_tilt, tgt_yaw, tgt_elbow, True

    eased = punch_angle_270.ease_in_out_cubic(elapsed_ms / duration_ms)
    pan = start_pan + (tgt_pan - start_pan) * eased
    tilt = start_tilt + (tgt_tilt - start_tilt) * eased
    yaw = start_yaw + (tgt_yaw - start_yaw) * eased
    elbow = start_elbow + (tgt_elbow - start_elbow) * eased
    return pan, tilt, yaw, elbow, False


def prompt_drive_arm():
    """Same lightweight startup-prompt pattern as
    track_interpolation_270.py's prompt_startup_config() - asked once per
    session instead of a fixed constant, so display-only testing and real
    arm-driving don't need a code edit to switch between. Defaults to NO
    (display-only) on a bare Enter - driving the real arm is an explicit
    opt-in, never an accidental one."""
    answer = input("Drive the real arm this session? (sends real servo commands) [y/N]: ").strip().lower()
    return answer in ("y", "yes")


def main():
    drive_arm = prompt_drive_arm()

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

    ser = serial.Serial(track.SERIAL_PORT, track.BAUD_RATE, timeout=1, write_timeout=1)
    print(f"Connected to Arduino on {track.SERIAL_PORT} - waiting for it to finish booting...")
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

    imu_reader = ImuSerialReader(ser)
    imu_reader.start()

    arm_current_pos = POSITIONS["REST_GUARD"]  # tracks wherever the arm actually is right now
    arm_phase = "rest"  # "rest" | "punching" | "returning"
    arm_anim_start_time = None
    arm_anim_start_pos = None
    arm_anim_target_pos = None
    arm_anim_pan_swing = 0.0
    arm_anim_hold_ms = punch_angle_270.HOLD_DURATION_MS
    imu_uppercut_pending = None  # {"t": frame_time} - an armed-but-not-yet-committed IMU uppercut call, see UPPERCUT_CONFIRM_WINDOW_SEC
    if drive_arm:
        punch_angle_270.go_ready(ser, *arm_current_pos)
        print(f"[arm] LIVE - servo commands will be sent. Starting at REST_GUARD {arm_current_pos}")
    else:
        print("[arm] display-only - no servo commands will be sent")

    classifier = PunchClassifier()
    elbow_l1 = None
    elbow_l2 = None
    tracked_side = track.SIDE_A
    start_time = time.monotonic()

    active_display = None  # {"angle_deg", "type", "detection_side", "until"} - drives ONLY the on-screen line/label
    pending_vision = None  # {"type", "angle_deg", "dx", "dy", "t"}          - drives ONLY CSV pairing bookkeeping
    pending_imu = None     # {"type", "angle_deg", "elapsed_ms", "t"}        - drives ONLY CSV pairing bookkeeping

    counts = {"yes": 0, "vision_only": 0, "imu_only": 0}
    write_header = not os.path.exists(LOG_PATH)

    def finalize(vision_event, imu_event):
        nonlocal write_header
        if vision_event and imu_event:
            paired = "yes"
        elif vision_event:
            paired = "vision_only"
        else:
            paired = "imu_only"
        counts[paired] += 1

        row = {
            "timestamp": f"{time.time():.3f}",
            "paired": paired,
            "vision_angle_deg": f"{vision_event['angle_deg']:.1f}" if vision_event else "",
            "vision_type": vision_event["type"] if vision_event else "",
            "vision_dx": f"{vision_event['dx']:.3f}" if vision_event else "",
            "vision_dy": f"{vision_event['dy']:.3f}" if vision_event else "",
            "imu_type": imu_event["type"] if imu_event else "",
            "imu_angle_deg": f"{imu_event['angle_deg']:.1f}" if imu_event else "",
            "imu_elapsed_ms": imu_event["elapsed_ms"] if imu_event else "",
        }
        _append_row(LOG_PATH, row, write_header)
        write_header = False

        if vision_event:
            imu_note = f", imu_type={imu_event['type']}" if imu_event else ""
            print(f">>> angle={vision_event['angle_deg']:+.1f} deg  type={vision_event['type']}  "
                  f"(paired={paired}{imu_note})")
        else:
            print(f"[pair] IMU-only event, no angle available: type={imu_event['type']}")

    print("\nThrow real punches - angle shown after each one. 'q' to quit.\n")

    try:
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

            result = None       # this frame's vision classifier result, if any - used below by the arm-trigger block too
            angle_deg = None
            if results.pose_landmarks:
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

                other_side = track.SIDE_B if tracked_side is track.SIDE_A else track.SIDE_A
                shoulder = lm[tracked_side["shoulder"]]
                elbow = lm[tracked_side["elbow"]]
                wrist = lm[tracked_side["wrist"]]
                other_shoulder = lm[other_side["shoulder"]]

                elbow_visible = elbow.visibility >= track.VISIBILITY_MIN
                raw_yaw = None
                raw_elbow = None
                if elbow_visible:
                    hip = lm[tracked_side["hip"]]
                    l1_frame = track.landmark_distance(shoulder, elbow, keys=("x", "y", "z"))
                    l2_frame = track.landmark_distance(elbow, wrist, keys=("x", "y", "z"))
                    if elbow_l1 is None:
                        elbow_l1, elbow_l2 = l1_frame, l2_frame
                    else:
                        elbow_l1 += track.ELBOW_LENGTH_ALPHA * (l1_frame - elbow_l1)
                        elbow_l2 += track.ELBOW_LENGTH_ALPHA * (l2_frame - elbow_l2)
                    wrist_shoulder_dist = track.landmark_distance(shoulder, wrist, keys=("x", "y", "z"))
                    raw_elbow = track.elbow_ik_angle(elbow_l1, elbow_l2, wrist_shoulder_dist)
                    raw_yaw = track.yaw_angle(shoulder, elbow, wrist, hip)
                    if raw_yaw is not None and raw_elbow is not None:
                        fade = (track.YAW_ELBOW_FADE_START - raw_elbow) / (track.YAW_ELBOW_FADE_START - track.YAW_ELBOW_FADE_END)
                        raw_yaw *= track.clamp(fade, 0.0, 1.0)

                result = classifier.update(shoulder, wrist, other_shoulder, raw_yaw, raw_elbow, frame_time)
                if result:
                    angle_deg = math.degrees(math.atan2(-result["dy"], result["dx"]))
                    print(f"[vision] {result['type']} angle={angle_deg:+.1f} "
                          f"dx={result['dx']:+.2f} dy={result['dy']:+.2f} "
                          f"speed={result['speed']:.2f} travel={result['travel']:.2f}")

                    # On-screen label only - "punch" (the classifier's own
                    # unclassified fallback, neither hook nor uppercut
                    # confidently called) always displays as "hook" instead,
                    # per the user's request: there are only two REAL labels
                    # on screen now (hook/uppercut), everything vision fires
                    # that isn't an IMU-driven uppercut is shown as a hook at
                    # whatever angle it measured. CSV logging (pending_vision
                    # below) keeps the TRUE classifier type unchanged - still
                    # useful for diagnosing the classifier later.
                    display_type = "hook" if result["type"] == "punch" else result["type"]
                    active_display = {
                        "angle_deg": angle_deg,
                        "type": display_type,
                        "detection_side": tracked_side,
                        "until": frame_time + ANGLE_DISPLAY_HOLD_SEC,
                    }

                    if pending_vision:
                        # A new vision event arrived before the previous one
                        # ever paired/timed out - finalize the OLD one as a
                        # one-sided event now rather than silently losing it.
                        finalize(pending_vision, None)
                    pending_vision = {"type": result["type"], "angle_deg": angle_deg,
                                       "dx": result["dx"], "dy": result["dy"], "t": frame_time}

                # Re-index by the SIDE snapshotted at detection time, not
                # the loop's own `shoulder` (which tracks whichever side is
                # CURRENTLY selected and can flip mid-hold) - keeps the
                # line anchored to the arm that actually threw the punch.
                # Live position each frame (not a frozen pixel spot) so the
                # line stays visually attached as the user moves/recovers.
                if active_display and frame_time < active_display["until"]:
                    anchor = lm[active_display["detection_side"]["shoulder"]]
                    shoulder_px = (anchor.x * frame.shape[1], anchor.y * frame.shape[0])
                    color = LABEL_COLORS.get(active_display["type"].upper(), LABEL_COLORS["PUNCH"])
                    draw_punch_angle(frame, shoulder_px, active_display["angle_deg"], color, active_display["type"])
            else:
                cv2.putText(frame, "No pose detected", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Vision+IMU Punch Angle Test", frame)

            imu_events = imu_reader.drain()  # captured once, reused below for both CSV pairing and the arm trigger

            imu_fired_uppercut = any(e["type"] in ("uppercut", "punch") for e in imu_events)
            for event in imu_events:
                if pending_imu:
                    finalize(None, pending_imu)
                pending_imu = event

            # --- Arm-driving trigger priority -------------------------------
            # FIXED 2026-11-06 after a live test confused hook/uppercut: the
            # old rule let an IMU uppercut/punch call trigger immediately,
            # unconditionally - but IMU is only ~30% accurate on real hooks
            # (CLAUDE.md's 2026-11-03 agreement-test numbers) and typically
            # fires well BEFORE vision's own slower, window-based detection
            # catches up, so a real hook commonly got stolen by an IMU
            # misfire before vision ever got a say. New rule: vision is TOP
            # PRIORITY for hook - an in-range hook-family reading wins
            # immediately and outright, no exceptions (its continuous angle
            # is already validated clean on real throws, far more
            # trustworthy here than vision's own discrete type label ever
            # was). IMU is the ONLY thing that can ever trigger uppercut
            # (vision never routes to POSITIONS["UPPERCUT"] - an
            # out-of-range angle_to_tilt() reading is always suppressed,
            # never redirected there) - but an IMU uppercut call no longer
            # fires instantly either: it arms a short confirm window
            # (UPPERCUT_CONFIRM_WINDOW_SEC) instead, giving vision a real
            # chance to override it with a CONFIDENTLY hook-shaped reading
            # (angle_deg <= VISION_OVERRIDE_MAX_ANGLE_DEG - tighter than
            # the full hook-family range, see that constant's comment for
            # why the full range is too permissive specifically for this
            # override decision) before it's actually committed. When
            # there's no pending IMU candidate, vision triggers on the
            # full hook-family range as normal - this restriction only
            # applies to overriding IMU. Never interrupts an in-progress
            # STRIKE ("punching"), but CAN interrupt an in-progress
            # return-to-rest - same "start from wherever the arm really
            # is" principle already enforced elsewhere in this codebase (a
            # fixed/frozen start position was tried once on a different
            # file and reverted - see punch_angle_270.py's
            # STRIKE_START_YAW history note).
            if drive_arm and arm_phase != "punching":
                # Vision first - an in-range hook-family reading wins
                # outright UNLESS an IMU uppercut is already pending and
                # this reading isn't confidently hook-shaped enough to
                # override it (see VISION_OVERRIDE_MAX_ANGLE_DEG) - in
                # that ambiguous case, defer to IMU's own pending commit
                # instead of risking a false override on a real uppercut.
                if result and angle_deg is not None:
                    tilt = angle_to_tilt(angle_deg)
                    if tilt is not None:
                        can_trigger = (imu_uppercut_pending is None
                                       or angle_deg <= VISION_OVERRIDE_MAX_ANGLE_DEG)
                        if can_trigger:
                            hook_pan, _, hook_yaw, hook_elbow = POSITIONS["HOOK"]
                            print(f">>> [arm] HOOK-FAMILY tilt={tilt:.0f} (vision {angle_deg:+.0f} deg) "
                                  f"- starting from {tuple(round(v) for v in arm_current_pos)}")
                            arm_phase = "punching"
                            arm_anim_start_time = frame_time
                            arm_anim_start_pos = arm_current_pos
                            arm_anim_target_pos = (hook_pan, tilt, hook_yaw, hook_elbow)
                            arm_anim_pan_swing = 0.0
                            arm_anim_hold_ms = punch_angle_270.HOLD_DURATION_MS
                            imu_uppercut_pending = None
                        else:
                            print(f"[arm] vision angle {angle_deg:+.1f} deg too close to uppercut "
                                  f"territory to override pending IMU call - deferring to IMU")
                    else:
                        lo_angle, hi_angle = HOOK_FAMILY_TILT_ANCHORS[-1][0], HOOK_FAMILY_TILT_ANCHORS[0][0]
                        print(f"[arm] suppressed - angle {angle_deg:+.1f} outside "
                              f"{lo_angle:.0f} to {hi_angle:.0f} range, not treated as a hook")

                # IMU - arms (or refreshes) a pending uppercut candidate,
                # but only commits it below once the confirm window has
                # elapsed with no vision override. Skipped entirely if
                # vision just won this exact frame (arm_phase is already
                # "punching" by this point).
                if arm_phase != "punching" and imu_fired_uppercut:
                    imu_uppercut_pending = {"t": frame_time}

                if (arm_phase != "punching" and imu_uppercut_pending is not None
                        and frame_time - imu_uppercut_pending["t"] >= UPPERCUT_CONFIRM_WINDOW_SEC):
                    print(f">>> [arm] UPPERCUT (IMU, confirmed) - starting from {tuple(round(v) for v in arm_current_pos)}")
                    arm_phase = "punching"
                    arm_anim_start_time = frame_time
                    arm_anim_start_pos = arm_current_pos
                    arm_anim_target_pos = POSITIONS["UPPERCUT"]
                    arm_anim_pan_swing = punch_angle_270.PAN_SWING_BY_KEY.get("1", 0.0)
                    arm_anim_hold_ms = punch_angle_270.HOLD_DURATION_BY_KEY.get("1", punch_angle_270.HOLD_DURATION_MS)
                    imu_uppercut_pending = None

            # --- Arm animation stepping (every frame, independent of
            # whether anything NEW was detected this frame) ------------------
            if drive_arm and arm_phase == "punching":
                elapsed_ms = (frame_time - arm_anim_start_time) * 1000.0
                pan, tilt, yaw, elbow, finished = punch_angle_270.punch_animation_frame(
                    elapsed_ms, start_pos=arm_anim_start_pos, target_pos=arm_anim_target_pos,
                    pan_swing_deg=arm_anim_pan_swing, hold_duration_ms=arm_anim_hold_ms,
                )
                pan, tilt, yaw, elbow = apply_safety_limits(pan, tilt, yaw, elbow)
                ser.write(f"{round(pan)},{round(tilt)},{round(yaw)},{round(elbow)}\n".encode())
                arm_current_pos = (pan, tilt, yaw, elbow)
                if finished:
                    print(">>> [arm] punch complete - returning to rest")
                    arm_phase = "returning"
                    arm_anim_start_time = frame_time
                    arm_anim_start_pos = arm_current_pos
                    # Re-arm the vision gate's cooldown from NOW, not the
                    # original detection instant - same "double-dip" fix
                    # track_interpolation_270.py already uses, otherwise a
                    # real arm retraction right after this animation reads
                    # as a second punch.
                    classifier.last_punch_at = frame_time
            elif drive_arm and arm_phase == "returning":
                elapsed_ms = (frame_time - arm_anim_start_time) * 1000.0
                pan, tilt, yaw, elbow, finished = ease_to_rest_frame(elapsed_ms, arm_anim_start_pos)
                pan, tilt, yaw, elbow = apply_safety_limits(pan, tilt, yaw, elbow)
                ser.write(f"{round(pan)},{round(tilt)},{round(yaw)},{round(elbow)}\n".encode())
                arm_current_pos = (pan, tilt, yaw, elbow)
                if finished:
                    arm_phase = "rest"

            # Pair whatever's pending - same throw if both arrived close
            # together; if both are present but far apart, the older one
            # belongs to a different throw, so finalize it alone and let
            # the newer one keep waiting for its own match; if only one
            # side has shown up at all, give up waiting after
            # FINALIZE_TIMEOUT_SEC and log it alone (vision-only is still
            # a complete, usable angle reading; imu-only never has one).
            if pending_imu and pending_vision:
                if abs(pending_imu["t"] - pending_vision["t"]) <= PAIR_WINDOW_SEC:
                    finalize(pending_vision, pending_imu)
                    pending_imu = None
                    pending_vision = None
                elif pending_imu["t"] < pending_vision["t"]:
                    finalize(None, pending_imu)
                    pending_imu = None
                else:
                    finalize(pending_vision, None)
                    pending_vision = None
            elif pending_imu and (frame_time - pending_imu["t"] > FINALIZE_TIMEOUT_SEC):
                finalize(None, pending_imu)
                pending_imu = None
            elif pending_vision and (frame_time - pending_vision["t"] > FINALIZE_TIMEOUT_SEC):
                finalize(pending_vision, None)
                pending_vision = None

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        imu_reader.stop()
        ser.close()
        cap.release()
        cv2.destroyAllWindows()
        landmarker.close()

        print("\n==================== FINAL SUMMARY ====================")
        print(f"Paired (vision+imu):  {counts['yes']}")
        print(f"Vision-only (still a valid angle): {counts['vision_only']}")
        print(f"IMU-only (no angle, cross-check-only miss): {counts['imu_only']}")
        print(f"Full log: {LOG_PATH}")


if __name__ == "__main__":
    main()
