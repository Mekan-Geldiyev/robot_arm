"""
Boxing Robot Arm - Labeled Punch Dataset Collector
-------------------------------------------------------
Camera-only, no Arduino. The current punch_classifier.py thresholds were
guesses (adapted from EFIGHT's constants, never verified against real
throws) - this fixes that the same way capture_point.py fixed the servo
calibration data: throw a bunch of REAL, labeled punches and log the
actual numbers, instead of continuing to guess.

Every punch detected reuses punch_classifier.py's own PunchClassifier
(same speed/travel/cooldown gate, same feature extraction) - this tool
just tags each one with the TRUE label you're actually throwing and logs
it, plus what the current heuristic guessed, so you get a free "how often
is the current guess already right" check alongside the raw data.

Controls:
    h        - switch to HOOK labeling mode
    u        - switch to UPPERCUT labeling mode
    n        - switch to NONE mode (ordinary, non-punch movement - e.g. the
               HAND_DOWN-to-side-out transition that was triggering false
               "HOOK" calls) - anything the classifier fires on here is by
               definition a false positive, since you're not punching
    s        - save the dataset to CSV right now (also auto-saves on quit)
    q        - quit (auto-saves)

Throw ~40 of each (or however many) while in the matching mode, then hand
the resulting punch_dataset.csv back for analysis. For 'n' mode
specifically: do the actual movements that were triggering false
positives (arm transitions between rest poses, reaching, etc), not random
motion - the point is to capture what THOSE specific movements look like
in feature space.

Run:  python punch_dataset.py
"""

import csv
import os
import sys
import time

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import track  # noqa: E402
from punch_classifier import PunchClassifier  # noqa: E402

OUTPUT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "punch_dataset.csv")
FIELDNAMES = ["label", "predicted", "speed", "travel", "dx", "dy", "dtilt", "yaw"]


def save_csv(rows):
    if not rows:
        return
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def main():
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

    label = None
    rows = []
    counts = {"hook": 0, "uppercut": 0, "none": 0}
    flash_text = ""
    flash_until = 0.0

    print("Press 'h' HOOK, 'u' UPPERCUT, or 'n' NONE (ordinary movement) before doing anything. 's' saves, 'q' quits+saves.")

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
                if label is None:
                    print("No label selected - press 'h' or 'u' first. Discarded this one.")
                    flash_text, flash_color = "NO LABEL SET - press h or u", (0, 0, 255)
                else:
                    rows.append({
                        "label": label,
                        "predicted": result["type"],
                        "speed": result["speed"],
                        "travel": result["travel"],
                        "dx": result["dx"],
                        "dy": result["dy"],
                        "dtilt": result["dtilt"],
                        "yaw": result["yaw"] if result["yaw"] is not None else "",
                    })
                    counts[label] = counts.get(label, 0) + 1
                    match = result["type"] == label
                    if label == "none":
                        # There's no "none" classifier output - any result
                        # here is by definition a false positive, not a
                        # "wrong guess between punch types".
                        flash_text = f"NONE #{counts[label]} - FALSE POSITIVE: called {result['type'].upper()}"
                        flash_color = (0, 0, 255)
                    else:
                        flash_text = f"{label.upper()} #{counts[label]} recorded" + ("" if match else f" (guessed {result['type']})")
                        flash_color = (0, 200, 0) if match else (0, 140, 255)
                    print(
                        f"[{flash_text}] speed={result['speed']:.2f} travel={result['travel']:.2f} "
                        f"dx={result['dx']:+.2f} dy={result['dy']:+.2f} dtilt={result['dtilt']:+.1f} yaw={result['yaw']}"
                    )
                flash_until = frame_time + 1.2
        else:
            cv2.putText(frame, "No pose detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        mode_str = label.upper() if label else "NO LABEL - press h/u/n"
        cv2.putText(frame, f"MODE: {mode_str}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(frame, f"hooks: {counts['hook']}   uppercuts: {counts['uppercut']}   none: {counts['none']}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if frame_time < flash_until:
            cv2.putText(frame, flash_text, (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.8, flash_color, 2)
        cv2.putText(frame, "h=hook  u=uppercut  n=none  s=save  q=quit", (10, frame.shape[0] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        cv2.imshow("Punch Dataset Collector", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("h"):
            label = "hook"
            print("Mode: HOOK - throw real hooks")
        elif key == ord("u"):
            label = "uppercut"
            print("Mode: UPPERCUT - throw real uppercuts")
        elif key == ord("n"):
            label = "none"
            print("Mode: NONE - do the ordinary movements that were triggering false positives (not punches)")
        elif key == ord("s"):
            save_csv(rows)
            print(f"Saved {len(rows)} rows to {OUTPUT_CSV}")

    save_csv(rows)
    print(f"Final counts: {counts}. Saved {len(rows)} rows to {OUTPUT_CSV}")

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()


if __name__ == "__main__":
    main()
