"""
Boxing Robot Arm - Calibration Point Capture Helper
-----------------------------------------------------
Standalone tool for building calibration_data.json entries with multiple
samples instead of one noisy snapshot. No Arduino/serial involved at all -
this only ever reads the camera and prints numbers.

Hold a pose, press SPACE - it grabs the next BURST_SIZE confident readings
(not necessarily consecutive frames; "No pose"/low-confidence frames don't
count toward the burst) and prints the mean and standard deviation per
channel, plus a ready-to-paste "mediapipe": [...] array.

Why average instead of a single reading: a single frame can be a fluke -
e.g. BOXING_JAB's raw elbow read 50 one time and 170 the next for what was
meant to be the same pose. Averaging several samples smooths out ordinary
frame-to-frame jitter. It does NOT fix a systematic bias (see the "note"
fields already in calibration_data.json for poses where MediaPipe's z is
just wrong in a consistent direction, like a bent elbow near the face
reading as straight every time) - a tight std on a wrong number is still a
wrong number. That's what the std is for: if it's small, the pose was
stable and the mean is trustworthy; if it's large, don't just trust the
average blindly - re-check whether the pose itself is unstable to track.

Run (from anywhere):   python capture_point.py
Quit:                  press 'q' with the webcam window focused
"""

import os
import statistics
import sys
import time

import cv2
import mediapipe as mp

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import track  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

BURST_SIZE = 10  # confident samples collected per capture

# A burst's standard deviation above this (in raw degrees) gets flagged as
# unstable rather than silently averaged - same rough scale as the jitter
# already seen on face-adjacent poses (BOXING_HOOK_SETUP, ELBOW_BENT_REST).
HIGH_VARIANCE_THRESHOLD = 8.0

# Captures fire automatically every this many seconds instead of waiting for
# SPACE - frees both hands for actually holding the pose. The countdown
# resets the moment a capture STARTS, not when it finishes (a burst takes
# ~0.3s at 30fps, so the difference doesn't matter in practice). If the
# timer hits zero while no confident pose is visible (mid-transition,
# briefly occluded), it just keeps waiting for the next confident frame
# rather than skipping that cycle - so "every 10 seconds" is really "at
# least every 10 seconds, plus however long a good reading takes to show up".
AUTO_CAPTURE_INTERVAL_SEC = 10.0

# SPACE still works too, for forcing an early capture without waiting out
# the countdown - it's additive, not a replacement for the timer.

# ============================================================


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

    pan_filter = track.OneEuroFilter(track.PAN_MIN_CUTOFF, track.PAN_BETA, track.D_CUTOFF)
    tilt_filter = track.OneEuroFilter(track.TILT_MIN_CUTOFF, track.TILT_BETA, track.D_CUTOFF)
    yaw_filter = track.OneEuroFilter(track.YAW_MIN_CUTOFF, track.YAW_BETA, track.D_CUTOFF)
    elbow_filter = track.OneEuroFilter(track.ELBOW_MIN_CUTOFF, track.ELBOW_BETA, track.D_CUTOFF)

    elbow_l1 = None
    elbow_l2 = None
    last_smooth_yaw = 0.0
    last_smooth_elbow = 150.0
    tracked_side = track.SIDE_A
    start_time = time.monotonic()

    burst = []  # list of (pan, tilt, yaw, elbow) while a capture is active
    next_capture_time = time.monotonic() + AUTO_CAPTURE_INTERVAL_SEC
    last_countdown_print = 0.0

    print(f"No Arduino needed - camera/tracking only. Auto-captures {BURST_SIZE} "
          f"samples every {AUTO_CAPTURE_INTERVAL_SEC:.0f}s - hold each pose steady "
          f"as the countdown reaches 0. SPACE forces an early capture. 'q' to quit.")

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

        smooth_vec = None

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

            shoulder = lm[tracked_side["shoulder"]]
            elbow = lm[tracked_side["elbow"]]
            wrist = lm[tracked_side["wrist"]]
            hip = lm[tracked_side["hip"]]

            low_confidence = (
                shoulder.visibility < track.VISIBILITY_MIN
                or wrist.visibility < track.VISIBILITY_MIN
                or hip.visibility < track.VISIBILITY_MIN
            )
            elbow_visible = elbow.visibility >= track.VISIBILITY_MIN

            if not low_confidence:
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

                smooth_pan = pan_filter(frame_time, raw_pan)
                smooth_tilt = tilt_filter(frame_time, raw_tilt)
                smooth_yaw = yaw_filter(frame_time, raw_yaw) if raw_yaw is not None else last_smooth_yaw
                smooth_elbow = elbow_filter(frame_time, raw_elbow) if raw_elbow is not None else last_smooth_elbow
                last_smooth_yaw = smooth_yaw
                last_smooth_elbow = smooth_elbow

                smooth_vec = (smooth_pan, smooth_tilt, smooth_yaw, smooth_elbow)

                side_label = "A" if tracked_side is track.SIDE_A else "B"
                status = f"[{side_label}] raw=[{smooth_pan:6.1f},{smooth_tilt:6.1f},{smooth_yaw:6.1f},{smooth_elbow:6.1f}]"
                if burst:
                    status += f"  CAPTURING {len(burst)}/{BURST_SIZE}"
                else:
                    status += f"  next capture: {max(next_capture_time - frame_time, 0.0):4.1f}s"
                cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "Low confidence - hold still", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
        else:
            cv2.putText(frame, "No pose detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(frame, "auto-captures every 10s   SPACE = capture now   q = quit",
                    (10, frame.shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        cv2.imshow("Calibration Point Capture", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        due = frame_time >= next_capture_time
        manual = key == ord(" ")

        if (due or manual) and not burst and smooth_vec is not None:
            burst = [smooth_vec]  # start a fresh burst, counting this frame
            next_capture_time = frame_time + AUTO_CAPTURE_INTERVAL_SEC
            print(f"\nCapturing {BURST_SIZE} samples - hold the pose...")
        elif burst and smooth_vec is not None:
            burst.append(smooth_vec)
        elif not burst and frame_time - last_countdown_print >= 1.0:
            # Terminal countdown, throttled to once/sec so it doesn't flood
            # the console the way printing every frame would.
            remaining = max(next_capture_time - frame_time, 0.0)
            waiting_note = "" if smooth_vec is not None else "  (waiting for a confident pose)"
            print(f"\rNext capture in: {remaining:4.1f}s{waiting_note}   ", end="", flush=True)
            last_countdown_print = frame_time

        if len(burst) >= BURST_SIZE:
            report_burst(burst)
            burst = []

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()


def report_burst(burst):
    names = ("pan", "tilt", "yaw", "elbow")
    means = []
    print(f"=== {len(burst)} samples ===")
    for i, name in enumerate(names):
        values = [sample[i] for sample in burst]
        mean = statistics.mean(values)
        std = statistics.pstdev(values)
        means.append(round(mean))
        flag = "  <-- HIGH VARIANCE, pose may be unstable" if std > HIGH_VARIANCE_THRESHOLD else ""
        print(f"  {name:5s}: mean={mean:7.1f}  std={std:5.2f}{flag}")
    print(f'  "mediapipe": {means},')


if __name__ == "__main__":
    main()
