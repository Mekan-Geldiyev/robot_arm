"""
Boxing Robot Arm - Hook Detector
------------------------------------
Standalone module (import HookDetector elsewhere) with a built-in camera
test mode for tuning it in isolation before wiring into
track_interpolation.py.

Two conditions, both borrowed/adapted from studying EFIGHT's punch
detector (C:\\Users\\Mekan\\Desktop\\EFIGHT\\src\\game\\punchDetector.js),
plus one condition that's new here:

  1. SPEED  - the wrist is moving fast right now, relative to shoulder
              width (so it's scale/distance independent, same idea as
              EFIGHT's PUNCH_SPEED).
  2. TRAVEL - the wrist has covered real ground over a short look-back
              window, not just one noisy frame (rejects jitter, same idea
              as EFIGHT's PUNCH_TRAVEL/PUNCH_WINDOW_MS).
  3. YAW    - NEW, not in EFIGHT (MoveNet's 2D keypoints can't give them
              this signal at all). raw_yaw is this project's elbow
              hinge-plane rotation - it's near zero for a straight-arm
              jab/uppercut and only grows large for a hook (see
              YAW_ELBOW_FADE in track.py). This is what actually tells a
              hook apart from any other fast punch - EFIGHT never needed
              to, since it only cares THAT a punch landed, not what kind.

IMPORTANT: feed this RAW (unfiltered) landmark positions and the RAW
(pre-OneEuroFilter) yaw value, not the smoothed ones - the whole point is
catching a fast transient, and smoothing exists specifically to blunt fast
transients. In track_interpolation.py's main loop, that's the shoulder/
wrist landmarks straight from `lm[...]`, and `raw_yaw` right after the
YAW_ELBOW_FADE multiply, before it reaches yaw_filter().

Standalone test (no Arduino, camera only):
    python hook_detector.py
    Prints "HOOK DETECTED" with speed/travel/yaw whenever it fires.
"""

import math
import os
import sys
import time

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import track  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

# Starting points adapted from EFIGHT's constants.js (PUNCH_SPEED=0.006
# shoulder-widths/ms, PUNCH_TRAVEL=0.4, PUNCH_WINDOW_MS=120,
# PUNCH_COOLDOWN_MS=350) - converted to seconds since this codebase uses
# time.monotonic() in seconds, not performance.now() in ms. The
# shoulder-widths unit itself should carry over reasonably well (it's
# real-world punch speed relative to body size, not tied to MoveNet vs
# MediaPipe specifically) but expect to re-tune against this camera/model's
# actual frame rate and jitter characteristics - these are a starting
# guess, not a verified value the way the HOOK_* calibration points are.
HOOK_SPEED_THRESHOLD = 6.0     # shoulder-widths/sec, instantaneous
HOOK_TRAVEL_THRESHOLD = 0.4    # shoulder-widths covered within the window
HOOK_WINDOW_SEC = 0.12         # look-back window for committed travel
HOOK_HISTORY_SEC = 0.2         # how much history to keep at all
HOOK_COOLDOWN_SEC = 0.35       # minimum gap between fires

# Raised 20 -> 60 (2026-08-26) from a real labeled dataset of 42 hooks +
# 43 uppercuts (punch_dataset.py, see punch_classifier.py's git history for
# the analysis). Turns out yaw isn't purely an elbow signal in practice -
# rotating your torso during an uppercut (very natural to do) also
# produces real, non-trivial yaw, and it clustered at 21-55 in the data,
# overlapping the low end of real hooks (which ranged ~21-131, mostly
# 70+). 60 cleanly separates all of that batch's body-rotation false
# positives from all but the 3 weakest real hooks (which fall back to
# unclassified "punch" instead of a confidently WRONG "hook" - a safer
# failure). Not a perfect boundary (some overlap is real, not a tuning
# artifact) - re-check against fresh data if hooks/uppercuts feel
# misclassified again after further testing.
HOOK_YAW_THRESHOLD = 60.0

# ============================================================


class HookDetector:
    """Tracks ONE wrist (whichever side track_interpolation.py is
    currently following) relative to its own shoulder, in shoulder-widths,
    the same normalization EFIGHT uses. Call update() once per frame."""

    def __init__(self):
        self.hist = []  # list of {"x", "y", "t"} - wrist relative to shoulder
        self.last_hook_at = 0.0

    def update(self, shoulder, wrist, other_shoulder, raw_yaw, now):
        """
        shoulder, wrist: raw landmarks of the currently-tracked arm.
        other_shoulder: the OTHER side's raw shoulder landmark, purely to
            measure real shoulder width (distance-independent scale) - not
            used for side-selection here, that's already been decided by
            the caller.
        raw_yaw: the current frame's yaw AFTER the YAW_ELBOW_FADE multiply,
            BEFORE yaw_filter() smoothing. May be None if the elbow wasn't
            visible this frame.
        now: time.monotonic() timestamp for this frame.

        Returns None, or a dict {"speed", "travel", "yaw"} on the frame a
        hook is detected.
        """
        shoulder_width = math.hypot(shoulder.x - other_shoulder.x, shoulder.y - other_shoulder.y)
        sw = shoulder_width if shoulder_width > 1e-6 else 1.0

        rx = (wrist.x - shoulder.x) / sw
        ry = (wrist.y - shoulder.y) / sw

        self.hist.append({"x": rx, "y": ry, "t": now})
        while self.hist and now - self.hist[0]["t"] > HOOK_HISTORY_SEC:
            self.hist.pop(0)

        if len(self.hist) < 2:
            return None

        prev = self.hist[-2]
        dt = now - prev["t"]
        inst_speed = math.hypot(rx - prev["x"], ry - prev["y"]) / dt if dt > 0 else 0.0

        travel = 0.0
        for sample in reversed(self.hist):
            if now - sample["t"] > HOOK_WINDOW_SEC:
                break
            d = math.hypot(rx - sample["x"], ry - sample["y"])
            if d > travel:
                travel = d

        off_cooldown = (now - self.last_hook_at) > HOOK_COOLDOWN_SEC
        yaw_ok = raw_yaw is not None and abs(raw_yaw) > HOOK_YAW_THRESHOLD

        if inst_speed > HOOK_SPEED_THRESHOLD and travel > HOOK_TRAVEL_THRESHOLD and yaw_ok and off_cooldown:
            self.last_hook_at = now
            return {"speed": inst_speed, "travel": travel, "yaw": raw_yaw}

        return None


def _standalone_test():
    """Camera-only test harness, no Arduino - prints when a hook fires so
    the thresholds above can be tuned before integrating into
    track_interpolation.py."""
    import cv2
    import mediapipe as mp

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

    detector = HookDetector()
    elbow_l1 = None
    elbow_l2 = None
    tracked_side = track.SIDE_A
    start_time = time.monotonic()
    flash_until = 0.0

    print("No Arduino needed - camera/detection only. Throw a hook. 'q' to quit.")

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

            result = detector.update(shoulder, wrist, other_shoulder, raw_yaw, frame_time)
            if result:
                flash_until = frame_time + 0.4
                print(f"HOOK DETECTED  speed={result['speed']:.2f}  travel={result['travel']:.2f}  yaw={result['yaw']:.1f}")

            color = (0, 0, 255) if frame_time < flash_until else (0, 255, 0)
            cv2.putText(frame, f"yaw={raw_yaw:.1f}" if raw_yaw is not None else "yaw=n/a",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if frame_time < flash_until:
                cv2.putText(frame, "HOOK!", (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        else:
            cv2.putText(frame, "No pose detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Hook Detector Test", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()


if __name__ == "__main__":
    _standalone_test()
