"""
Boxing Robot Arm - Punch Classifier (hook vs uppercut, for now)
-------------------------------------------------------------------
Camera-only, no Arduino, no hardware needed - pure detection R&D.
Standalone module (import PunchClassifier elsewhere) with a built-in
camera test mode.

hook_detector.py already answers "is this a hook" (fast committed motion +
large raw_yaw). This file is broader: it answers "is this a punch, and if
so, which KIND" - hook vs uppercut today, jab planned for later once
depth/z is brought into the picture (a jab thrown at the camera barely
moves in the image plane, which is exactly the case EFIGHT's own detector
was built to handle by not caring about direction at all - see
punchDetector.js. Telling a jab apart from "not punching" needs that same
z-aware treatment, which is why it's deferred rather than guessed at now).

THE APPROACH - two independent signals, checked in priority order:

  1. YAW (this project's own signal, not something EFIGHT has) - the
     elbow hinge-plane rotation. Large |raw_yaw| is a strong, physically
     direct sign of a hook (see YAW_ELBOW_FADE in track.py + all the
     HOOK_* calibration notes - a hook is specifically the motion that
     swings this plane out). Checked FIRST because it's the most reliable
     signal available - trust it over the geometric heuristic below when
     it's present.

  2. MOTION DIRECTION (EFIGHT-style wrist-relative-to-shoulder tracking,
     see hook_detector.py's docstring for the full lineage) - once a fast,
     committed punch is confirmed (speed + travel, same gate as
     hook_detector.py), look at which axis DOMINATES the wrist's
     displacement over the detection window:
        mostly horizontal   -> hook-shaped motion
        mostly rising (up)  -> uppercut-shaped motion
     This is the fallback for when yaw doesn't clearly say "hook" (e.g. a
     hook thrown with the elbow not fully committed, or genuinely an
     uppercut, where yaw should be small anyway - see calibration notes).

IMPORTANT: same as hook_detector.py - feed this RAW (unfiltered) landmark
positions and RAW (pre-OneEuroFilter) yaw, not smoothed values. Fast
transients are exactly what smoothing exists to blunt.

Standalone test (no Arduino, camera only):
    python punch_classifier.py
    Prints "HOOK" / "UPPERCUT" / "PUNCH (unclassified)" whenever something
    fires, with the signals that led to that call.
"""

import math
import os
import sys
import time

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import track  # noqa: E402
from hook_detector import (  # noqa: E402
    HOOK_SPEED_THRESHOLD,
    HOOK_TRAVEL_THRESHOLD,
    HOOK_WINDOW_SEC,
    HOOK_HISTORY_SEC,
    HOOK_COOLDOWN_SEC,
    HOOK_YAW_THRESHOLD,
)

# ==================== TUNABLE SETTINGS ====================

# The punch-happened gate (speed/travel/window/cooldown) is shared with
# hook_detector.py's constants above, unchanged - "is something decisive
# happening" doesn't depend on which punch it turns out to be.

# How much one axis of wrist displacement needs to dominate the other
# before calling the direction "clearly horizontal" or "clearly vertical" -
# guards against calling a diagonal motion confidently one or the other.
DIRECTION_DOMINANCE_RATIO = 1.3

# --- Standalone test display only (doesn't affect classification) ---
FLASH_DURATION_SEC = 1.0    # how long the color banner stays up after a hit
FLASH_FONT_SCALE = 1.8
FLASH_FONT_THICKNESS = 4
LABEL_COLORS = {            # BGR, since that's what cv2 wants
    "HOOK": (0, 0, 255),        # red
    "UPPERCUT": (255, 0, 0),    # blue
    "PUNCH": (0, 165, 255),     # orange - unclassified
}

# ============================================================


class PunchClassifier:
    """Tracks ONE wrist (whichever side is currently tracked) relative to
    its own shoulder, in shoulder-widths - same normalization as
    hook_detector.HookDetector. Call update() once per frame."""

    def __init__(self):
        self.hist = []  # {"x", "y", "tilt", "t"} - wrist relative to shoulder
        self.last_punch_at = 0.0

    def update(self, shoulder, wrist, other_shoulder, raw_yaw, now):
        """
        shoulder, wrist: raw landmarks of the currently-tracked arm.
        other_shoulder: the OTHER side's raw shoulder landmark, purely for
            real shoulder-width scale (see hook_detector.py).
        raw_yaw: current frame's yaw AFTER YAW_ELBOW_FADE, BEFORE
            yaw_filter() smoothing. May be None if elbow wasn't visible.
        now: time.monotonic() timestamp for this frame.

        Returns None, or a dict on the frame a punch is classified:
            {"type": "hook" | "uppercut" | "punch",
             "speed", "travel", "dx", "dy", "dtilt", "yaw"}
        "punch" means the speed/travel gate fired but neither the yaw nor
        the direction heuristic confidently called it hook or uppercut
        (could be a jab, or an ambiguous diagonal motion).
        """
        shoulder_width = math.hypot(shoulder.x - other_shoulder.x, shoulder.y - other_shoulder.y)
        sw = shoulder_width if shoulder_width > 1e-6 else 1.0

        rx = (wrist.x - shoulder.x) / sw
        ry = (wrist.y - shoulder.y) / sw  # image y grows downward
        raw_tilt = track.elevation_angle(shoulder, wrist)

        self.hist.append({"x": rx, "y": ry, "tilt": raw_tilt, "t": now})
        while self.hist and now - self.hist[0]["t"] > HOOK_HISTORY_SEC:
            self.hist.pop(0)

        if len(self.hist) < 2:
            return None

        prev = self.hist[-2]
        dt = now - prev["t"]
        inst_speed = math.hypot(rx - prev["x"], ry - prev["y"]) / dt if dt > 0 else 0.0

        # Displacement + travel over the window, measured from the OLDEST
        # sample still in the window to now - this is what gives a
        # direction (dx, dy), not just a scalar distance like
        # hook_detector.py's "furthest single sample" travel.
        window_start = None
        travel = 0.0
        for sample in reversed(self.hist):
            if now - sample["t"] > HOOK_WINDOW_SEC:
                break
            window_start = sample
            d = math.hypot(rx - sample["x"], ry - sample["y"])
            if d > travel:
                travel = d

        off_cooldown = (now - self.last_punch_at) > HOOK_COOLDOWN_SEC

        if not (inst_speed > HOOK_SPEED_THRESHOLD and travel > HOOK_TRAVEL_THRESHOLD and off_cooldown):
            return None

        self.last_punch_at = now

        dx = rx - window_start["x"]
        dy = ry - window_start["y"]
        dtilt = raw_tilt - window_start["tilt"]

        punch_type = self._classify(dx, dy, dtilt, raw_yaw)
        return {
            "type": punch_type, "speed": inst_speed, "travel": travel,
            "dx": dx, "dy": dy, "dtilt": dtilt, "yaw": raw_yaw,
        }

    @staticmethod
    def _classify(dx, dy, dtilt, raw_yaw):
        # 1. Yaw first - our own physically-direct signal, trust it over
        # the 2D motion heuristic when it's clearly present.
        if raw_yaw is not None and abs(raw_yaw) > HOOK_YAW_THRESHOLD:
            return "hook"

        # 2. No strong yaw - fall back to which axis dominates the wrist's
        # actual 2D travel. Rising = ry decreasing (image y grows
        # downward). dtilt (elevation_angle's own rate of change) was
        # tried here as an uppercut-specific confirmation and dropped: it
        # saturates near +-90 degrees once the wrist is nearly directly
        # above/below the shoulder (atan2 asymptote) - exactly a real
        # uppercut's geometry, which made dtilt read as ~0 for a fast,
        # nearly-vertical rise in testing. The raw shoulder-widths
        # displacement (rising/horizontal) doesn't have that problem.
        #
        # BUG FIXED HERE (found via real labeled data in punch_dataset.py):
        # the hook fallback used to compare against signed `rising`
        # (negative on a net-downward move, e.g. the window catching a
        # punch's recovery snap-back instead of the strike) - any positive
        # `horizontal` beats a negative number, so a downward motion
        # defaulted to "hook" every time by accident, regardless of how
        # small the actual sideways movement was. Now compares against
        # abs(dy) instead, so a downward (not sideways) motion correctly
        # falls through to "punch" (unclassified) instead of a confident
        # wrong answer.
        horizontal = abs(dx)
        vertical = abs(dy)
        rising = -dy
        if rising > 0 and rising > horizontal * DIRECTION_DOMINANCE_RATIO:
            return "uppercut"
        if horizontal > vertical * DIRECTION_DOMINANCE_RATIO:
            return "hook"

        return "punch"


def _standalone_test():
    """Camera-only test harness - throw hooks and uppercuts, watch which
    one it calls and why, before this gets wired into anything else."""
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

    classifier = PunchClassifier()
    elbow_l1 = None
    elbow_l2 = None
    tracked_side = track.SIDE_A
    start_time = time.monotonic()
    flash_label = ""
    flash_until = 0.0

    print("No Arduino needed - camera/detection only. Throw hooks and uppercuts. 'q' to quit.")

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

            result = classifier.update(shoulder, wrist, other_shoulder, raw_yaw, frame_time)
            if result:
                flash_label = result["type"].upper()
                flash_until = frame_time + FLASH_DURATION_SEC
                print(
                    f"{flash_label}  speed={result['speed']:.2f} travel={result['travel']:.2f} "
                    f"dx={result['dx']:+.2f} dy={result['dy']:+.2f} dtilt={result['dtilt']:+.1f} "
                    f"yaw={result['yaw']}"
                )

            yaw_str = f"{raw_yaw:.1f}" if raw_yaw is not None else "n/a"
            cv2.putText(frame, f"yaw={yaw_str}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            if frame_time < flash_until:
                color = LABEL_COLORS.get(flash_label, LABEL_COLORS["PUNCH"])
                frame_h, frame_w = frame.shape[:2]
                (text_w, text_h), baseline = cv2.getTextSize(
                    flash_label, cv2.FONT_HERSHEY_SIMPLEX, FLASH_FONT_SCALE, FLASH_FONT_THICKNESS
                )
                banner_h = text_h + baseline + 30
                # Solid color banner across the top, label centered in it -
                # much harder to miss than colored text alone, especially
                # while you're mid-swing and not looking closely at the
                # small yaw readout.
                cv2.rectangle(frame, (0, 0), (frame_w, banner_h), color, -1)
                text_x = (frame_w - text_w) // 2
                text_y = banner_h - baseline - 15
                text_color = (255, 255, 255) if color != (0, 165, 255) else (0, 0, 0)
                cv2.putText(frame, flash_label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            FLASH_FONT_SCALE, text_color, FLASH_FONT_THICKNESS)
        else:
            cv2.putText(frame, "No pose detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Punch Classifier Test", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()


if __name__ == "__main__":
    _standalone_test()
