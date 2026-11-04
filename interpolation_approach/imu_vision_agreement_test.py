"""
Boxing Robot Arm - IMU vs Vision Punch-Type Agreement Test
------------------------------------------------------------
Combine-the-two-working-classifiers task, goal 1: log every thrown
punch's ground-truth label next to what the wrist IMU said
(robot_arm_270.ino's already-live binary classifier - angleDeg vs
adaptive refUp, UNCHANGED here) and what the vision classifier said
(punch_classifier.PunchClassifier, UNCHANGED here), so accuracy of each
signal - and of the simplest possible combined rule - can be measured
from real data instead of guessed. Does NOT add a new classifier and does
NOT touch robot_arm_270.ino, track_interpolation_270.py, or
punch_classifier.py - this is a read-only comparison harness. Run this
INSTEAD OF track_interpolation_270.py (same serial port, only one process
can hold it open) - it never writes servo commands, so the arm just sits
at boot-center the whole time, which is fine for this test.

Camera/MediaPipe/side-tracking setup below is copied from
punch_classifier.py's own _standalone_test(), unchanged - this file adds
the serial/IMU half and the labeling/pairing/logging on top of that
already-proven scaffolding, not a rewrite of it.

Ground-truth label is KEYBOARD, not spoken voice - same sticky-mode UX
punch_dataset.py already uses for its own hand-labeled capture tool
(press once, stays active until you press the other key), not a per-
throw prompt. Faster and more robust than real speech recognition for
this kind of back-to-back-throws data collection; swap in Vosk later via
voice_punch_270.py's pattern if real voice turns out to matter here.

Usage:
    python imu_vision_agreement_test.py
Keys:
    h = label upcoming throws HOOK   (sticky - stays active)
    u = label upcoming throws UPPERCUT (sticky - stays active)
    q = quit (prints final accuracy summary, closes everything)
Throw at least 10 hooks and 10 uppercuts while labeled correctly - each
throw is logged automatically as soon as the IMU and/or vision side
report something, no per-throw keypress needed beyond picking the mode.
"""

import csv
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
from punch_classifier import PunchClassifier  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

# How close together (seconds) an IMU event and a vision event need to be
# to count as the SAME physical throw. Generous on purpose - the IMU
# reacts in well under a second (its own capture window maxes out at
# CAPTURE_MAX_MS=400ms), vision needs several frames
# (punch_classifier.HOOK_WINDOW_SEC=0.12s) PLUS camera/MediaPipe inference
# time on top - 1.5s comfortably covers both without risking pairing two
# genuinely separate throws thrown in quick succession.
PAIR_WINDOW_SEC = 1.5

# If only ONE side (IMU or vision) has reported anything for this long,
# stop waiting for the other and log the throw as a one-sided miss rather
# than holding it forever - a real miss is exactly the data this test is
# supposed to surface, not something to hide by waiting indefinitely.
FINALIZE_TIMEOUT_SEC = 2.0

LOG_PATH = os.path.join(_PARENT_DIR, "imu_vision_agreement_log.csv")
DISAGREEMENT_LOG_PATH = os.path.join(_PARENT_DIR, "imu_vision_disagreements.csv")

CSV_FIELDS = ["timestamp", "voice_label", "imu_class", "imu_angle_deg", "vision_class", "agree"]

LABEL_KEYS = {ord("h"): "hook", ord("u"): "uppercut"}

# ============================================================


class ImuSerialReader:
    """Background-thread reader for 'PUNCH,<type>,<angleDeg>,...' lines -
    the SAME line format robot_arm_270.ino already prints for the LIVE
    binary classifier (not touched by this file). Queues every event
    rather than exposing a single pending slot
    (imu_punch_listener.IMUPunchListener's pattern) since this test needs
    to see and log EVERY event, not just the latest."""

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
        each is {"type", "angle_deg", "t"} (t = time.monotonic() when the
        line was received)."""
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
            if len(fields) < 3:
                continue
            try:
                angle_deg = float(fields[2])
            except ValueError:
                continue
            with self._lock:
                self._events.append({"type": fields[1], "angle_deg": angle_deg, "t": time.monotonic()})
            print(f"[imu] {fields[1]} angle={angle_deg:.1f}")


def _append_row(path, row, write_header):
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def _pct(correct, total):
    return f"{100.0 * correct / total:.0f}%" if total else "n/a"


def main():
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

    classifier = PunchClassifier()
    elbow_l1 = None
    elbow_l2 = None
    tracked_side = track.SIDE_A
    start_time = time.monotonic()
    flash_label = ""
    flash_until = 0.0

    current_label = None
    pending_vision = None  # {"type", "t"}
    pending_imu = None     # {"type", "angle_deg", "t"}

    counts = {
        "imu_correct": 0, "imu_total": 0,
        "vision_correct": 0, "vision_total": 0,
        "combined_correct": 0, "combined_total": 0,
    }

    write_header = not os.path.exists(LOG_PATH)
    write_header_dis = not os.path.exists(DISAGREEMENT_LOG_PATH)

    def finalize(label, imu_event, vision_event):
        nonlocal write_header, write_header_dis
        imu_class = imu_event["type"] if imu_event else None
        imu_angle = f"{imu_event['angle_deg']:.1f}" if imu_event else ""
        vision_class = vision_event["type"] if vision_event else None

        if imu_class is not None and vision_class is not None:
            agree = "yes" if imu_class == vision_class else "no"
        else:
            agree = "n/a"  # one-sided miss - not a disagreement, a dropout

        row = {
            "timestamp": f"{time.time():.3f}",
            "voice_label": label,
            "imu_class": imu_class or "",
            "imu_angle_deg": imu_angle,
            "vision_class": vision_class or "",
            "agree": agree,
        }
        _append_row(LOG_PATH, row, write_header)
        write_header = False

        if agree != "yes":
            _append_row(DISAGREEMENT_LOG_PATH, row, write_header_dis)
            write_header_dis = False

        if imu_class is not None:
            counts["imu_total"] += 1
            if imu_class == label:
                counts["imu_correct"] += 1
        if vision_class is not None:
            counts["vision_total"] += 1
            if vision_class == label:
                counts["vision_correct"] += 1

        # Simplest possible combined rule for this first pass - "only
        # trust it when both signals agree with each other AND that
        # agreed answer is right." Deliberately NOT a new classifier,
        # just a mechanical agreement check - don't build a smarter rule
        # until this data shows a real gap worth building one for.
        counts["combined_total"] += 1
        if imu_class is not None and vision_class is not None and imu_class == vision_class == label:
            counts["combined_correct"] += 1

        print(
            f">>> [{label}] imu={imu_class or '-'} vision={vision_class or '-'} agree={agree}  |  "
            f"running acc: imu={_pct(counts['imu_correct'], counts['imu_total'])}"
            f"({counts['imu_total']}) vision={_pct(counts['vision_correct'], counts['vision_total'])}"
            f"({counts['vision_total']}) combined={_pct(counts['combined_correct'], counts['combined_total'])}"
            f"({counts['combined_total']})"
        )

    print("\nh = label HOOK   u = label UPPERCUT   q = quit\n"
          "Pick a label, then throw real punches - logged automatically.\n")

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
                    flash_label = result["type"].upper()
                    flash_until = frame_time + 1.0
                    print(f"[vision] {result['type']} speed={result['speed']:.2f} travel={result['travel']:.2f}")
                    if current_label and pending_vision:
                        # A new vision event arrived before the previous
                        # one was ever paired/timed out - finalize the OLD
                        # one as a one-sided miss now rather than silently
                        # overwriting and losing it.
                        finalize(current_label, None, pending_vision)
                    pending_vision = {"type": result["type"], "t": frame_time}

                if frame_time < flash_until:
                    cv2.putText(frame, flash_label, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

            mode_str = current_label.upper() if current_label else "NONE - press h or u"
            cv2.putText(frame, f"Label mode: {mode_str}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("IMU vs Vision Agreement Test", frame)

            for event in imu_reader.drain():
                if current_label and pending_imu:
                    finalize(current_label, pending_imu, None)
                pending_imu = event

            # Pair whatever's pending - same throw if both arrived close
            # together; if both are present but far apart, the older one
            # belongs to a different throw, so finalize it alone and let
            # the newer one keep waiting for its own match; if only one
            # side has shown up at all, give up waiting after
            # FINALIZE_TIMEOUT_SEC and log it as a one-sided miss.
            if current_label and pending_imu and pending_vision:
                if abs(pending_imu["t"] - pending_vision["t"]) <= PAIR_WINDOW_SEC:
                    finalize(current_label, pending_imu, pending_vision)
                    pending_imu = None
                    pending_vision = None
                elif pending_imu["t"] < pending_vision["t"]:
                    finalize(current_label, pending_imu, None)
                    pending_imu = None
                else:
                    finalize(current_label, None, pending_vision)
                    pending_vision = None
            elif current_label and pending_imu and (frame_time - pending_imu["t"] > FINALIZE_TIMEOUT_SEC):
                finalize(current_label, pending_imu, None)
                pending_imu = None
            elif current_label and pending_vision and (frame_time - pending_vision["t"] > FINALIZE_TIMEOUT_SEC):
                finalize(current_label, None, pending_vision)
                pending_vision = None

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key in LABEL_KEYS:
                current_label = LABEL_KEYS[key]
                print(f"\n=== Label mode: {current_label.upper()} ===\n")
    finally:
        imu_reader.stop()
        ser.close()
        cap.release()
        cv2.destroyAllWindows()
        landmarker.close()

        print("\n==================== FINAL SUMMARY ====================")
        print(f"IMU alone:      {_pct(counts['imu_correct'], counts['imu_total'])} "
              f"({counts['imu_correct']}/{counts['imu_total']})")
        print(f"Vision alone:   {_pct(counts['vision_correct'], counts['vision_total'])} "
              f"({counts['vision_correct']}/{counts['vision_total']})")
        print(f"Combined (must agree): {_pct(counts['combined_correct'], counts['combined_total'])} "
              f"({counts['combined_correct']}/{counts['combined_total']})")
        print(f"Full log: {LOG_PATH}")
        print(f"Disagreements/misses: {DISAGREEMENT_LOG_PATH}")


if __name__ == "__main__":
    main()
