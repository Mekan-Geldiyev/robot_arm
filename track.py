"""
Boxing Robot Arm - Real-Time Motion Mirroring
-----------------------------------------------
Tracks the user's LEFT arm with MediaPipe Pose and streams
"pan,tilt\n" servo angles to the Arduino over serial, which then
smoothly drives the shoulder servos to match.

Run:   python track.py
Quit:  press 'q' with the webcam window focused

Install dependencies:
    pip install opencv-python mediapipe pyserial

Note: mediapipe >=0.10.30 removed the old mp.solutions.pose API. This
script uses the newer Tasks API (mp.tasks.vision.PoseLandmarker), which
needs a small model file. It's downloaded automatically to
pose_landmarker_lite.task next to this script the first time you run it.
"""

import math
import os
import time
import urllib.request

import cv2
import mediapipe as mp
import serial

# ==================== TUNABLE SETTINGS ====================

# --- Serial ---
SERIAL_PORT = "COM3"       # <-- change this to match your Arduino's port
BAUD_RATE = 9600           # must match Serial.begin() in robot_arm.ino

# --- Camera ---
CAMERA_INDEX = 0

# --- Pose model (downloaded automatically on first run) ---
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pose_landmarker_lite.task")
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"

# --- Deadband: don't send a new value unless it changed by at least this much ---
DEADBAND_DEG = 2

# --- Smoothing: exponential moving average applied to the computed angles
# before they're mapped/sent. Raw single-frame pose angles are noisy (a
# hand or elbow can jump 10-50 degrees between two frames from normal
# pose-estimation jitter) - without this the arm chases every jitter
# instead of your actual motion, which is what "not smooth" looks like.
# Lower = smoother but more lag, higher = snappier but jitterier. 1.0 disables it.
SMOOTH_ALPHA = 0.25

# --- Don't flood the serial link faster than this, independent of camera FPS ---
SEND_INTERVAL_MS = 40   # ~25 updates/sec - plenty for smooth servo motion

# --- Elbow (channel 2) servo is not installed yet ---
SEND_ELBOW = False

# --- Calibration -----------------------------------------------------------
# Raw angles from MediaPipe rarely land cleanly on 0-180. Watch the printed
# "raw" values in the terminal while moving your arm through its full boxing
# range of motion (jab, hook, uppercut) and set MIN/MAX to match what you see.
# IMPORTANT: if a raw value ever falls outside [*_MIN, *_MAX], it gets
# clamped to a servo extreme (0 or 180) - if MIN/MAX don't bracket your
# actual range of motion, the servo will get stuck pinned at one end
# (this is what happened with the tilt defaults below before calibration).
#
#   *_MIN / *_MAX        -> raw MediaPipe angle range (degrees)
#   *_SERVO_MIN / _MAX    -> resulting servo angle range (0-180)
#   INVERT_*              -> flip direction if the arm moves the wrong way
#
# To calibrate: run track.py, watch the "raw:" values while you move
# through arm-down -> arm-out-horizontal -> arm-raised-overhead (for tilt)
# and jab -> hook (for pan), and set MIN/MAX to the extremes you actually see.
#
# Pan/tilt are now both measured shoulder->WRIST (not shoulder->elbow), so a
# bent-elbow raise still counts as the arm going up/around instead of only
# straight-arm motion. This shifts the raw numbers a bit from earlier
# calibration sessions - redo the raw-value readout below if angles feel off.

PAN_MIN, PAN_MAX = 133, 178           # raw shoulder horizontal angle range
PAN_SERVO_MIN, PAN_SERVO_MAX = 0, 180
INVERT_PAN = False
# Fitted from real Serial Monitor ground truth (see README "Calibration
# Reference Values"): raw=178 <-> servo=180 (hand at side), raw=133 <->
# servo=0 (hand out to the side). Narrower than it used to be because the
# pan/tilt formulas now measure shoulder->WRIST instead of shoulder->elbow.

# Tilt formula: more negative = arm hanging down, less negative/positive =
# arm raised. Fitted the same way: raw=-84 <-> servo=0 (hand down at side),
# raw=-16 <-> servo=90 (hand out to the side). TILT_MAX extrapolates a bit
# past that point to leave headroom for raising the arm further (uppercut/
# overhead) while staying under the mechanical stall limit found earlier -
# recheck via Serial Monitor (tilt=140/150/160/170, pan held still) if the
# arm jams before reaching TILT_SERVO_MAX.
TILT_MIN, TILT_MAX = -84, 30          # raw shoulder elevation angle range
TILT_SERVO_MIN, TILT_SERVO_MAX = 0, 150
INVERT_TILT = False

ELBOW_MIN, ELBOW_MAX = 30, 180        # raw elbow flex angle range (for later)
ELBOW_SERVO_MIN, ELBOW_SERVO_MAX = 0, 180
INVERT_ELBOW = False

# ============================================================

# MediaPipe Pose landmark indices for tracking the user's real LEFT arm + hip.
# Note: frame is horizontally flipped (mirror view) before pose detection,
# and MediaPipe's left/right labeling is appearance-based (not geometric),
# so it comes out swapped on a flipped feed - its "RIGHT_*" landmarks are
# what actually correspond to the user's true left arm here.
LEFT_SHOULDER = 12
LEFT_ELBOW = 14
LEFT_WRIST = 16
LEFT_HIP = 24


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def map_range(value, in_min, in_max, out_min, out_max, invert=False):
    """Linearly map value from [in_min, in_max] to [out_min, out_max]."""
    value = clamp(value, min(in_min, in_max), max(in_min, in_max))
    if invert:
        value = in_max - (value - in_min)
    span_in = in_max - in_min
    if span_in == 0:
        return out_min
    result = (value - in_min) / span_in * (out_max - out_min) + out_min
    return clamp(result, min(out_min, out_max), max(out_min, out_max))


def angle_3pt(a, b, c, keys=("x", "y")):
    """Angle at point b, between vectors b->a and b->c, using two axes."""
    k1, k2 = keys
    v1 = (getattr(a, k1) - getattr(b, k1), getattr(a, k2) - getattr(b, k2))
    v2 = (getattr(c, k1) - getattr(b, k1), getattr(c, k2) - getattr(b, k2))
    mag1 = math.hypot(*v1)
    mag2 = math.hypot(*v2)
    if mag1 * mag2 == 0:
        return 0.0
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    cos_angle = clamp(dot / (mag1 * mag2), -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


def elevation_angle(shoulder, wrist):
    """Angle of the whole arm (shoulder->wrist) above/below horizontal
    (image x-y plane). Uses the wrist rather than the elbow so a bent-elbow
    raise (forearm swings up while the upper arm barely moves - e.g. a
    salute-style motion) still registers as "arm going up", since it's the
    hand/fist position that boxing motion actually cares about."""
    dx = wrist.x - shoulder.x
    dy = shoulder.y - wrist.y  # image y grows downward, so flip it
    return math.degrees(math.atan2(dy, abs(dx) + 1e-6))


def ensure_model_downloaded():
    if os.path.exists(MODEL_PATH):
        return
    print(f"Downloading pose model to {MODEL_PATH} (one-time, ~5.5 MB)...")
    urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)
    print("Done.")


def main():
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, write_timeout=1)
        time.sleep(2)  # give the Arduino time to reset after the port opens
        print(f"Connected to Arduino on {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"WARNING: could not open {SERIAL_PORT} ({e})")
        print("Continuing without serial - tracking/display only.")

    ensure_model_downloaded()

    BaseOptions = mp.tasks.BaseOptions
    PoseLandmarker = mp.tasks.vision.PoseLandmarker
    PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
    VisionRunningMode = mp.tasks.vision.RunningMode
    drawing_utils = mp.tasks.vision.drawing_utils
    POSE_CONNECTIONS = mp.tasks.vision.PoseLandmarksConnections.POSE_LANDMARKS

    landmarker = PoseLandmarker.create_from_options(
        PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=VisionRunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
    )

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("ERROR: could not open webcam")
        return

    last_sent_pan = None
    last_sent_tilt = None
    last_send_time = 0.0
    smooth_pan = None
    smooth_tilt = None
    smooth_elbow = None
    start_time = time.monotonic()

    print("Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)  # mirror view, feels more natural to stand in front of
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((time.monotonic() - start_time) * 1000)
        results = landmarker.detect_for_video(mp_image, timestamp_ms)

        if results.pose_landmarks:
            lm = results.pose_landmarks[0]
            drawing_utils.draw_landmarks(frame, lm, POSE_CONNECTIONS)

            shoulder = lm[LEFT_SHOULDER]
            elbow = lm[LEFT_ELBOW]
            wrist = lm[LEFT_WRIST]
            hip = lm[LEFT_HIP]

            raw_pan = angle_3pt(hip, shoulder, wrist, keys=("x", "z"))
            raw_tilt = elevation_angle(shoulder, wrist)
            raw_elbow = angle_3pt(shoulder, elbow, wrist, keys=("x", "y"))

            # Low-pass filter the noisy per-frame angles before mapping/sending,
            # so the arm follows your overall motion instead of chasing jitter.
            if smooth_pan is None:
                smooth_pan, smooth_tilt, smooth_elbow = raw_pan, raw_tilt, raw_elbow
            else:
                smooth_pan += SMOOTH_ALPHA * (raw_pan - smooth_pan)
                smooth_tilt += SMOOTH_ALPHA * (raw_tilt - smooth_tilt)
                smooth_elbow += SMOOTH_ALPHA * (raw_elbow - smooth_elbow)

            pan = int(map_range(smooth_pan, PAN_MIN, PAN_MAX, PAN_SERVO_MIN, PAN_SERVO_MAX, INVERT_PAN))
            tilt = int(map_range(smooth_tilt, TILT_MIN, TILT_MAX, TILT_SERVO_MIN, TILT_SERVO_MAX, INVERT_TILT))
            elbow_angle = int(map_range(smooth_elbow, ELBOW_MIN, ELBOW_MAX, ELBOW_SERVO_MIN, ELBOW_SERVO_MAX, INVERT_ELBOW))

            print(
                f"raw: pan={raw_pan:6.1f} tilt={raw_tilt:6.1f} elbow={raw_elbow:6.1f}  ->  "
                f"servo: pan={pan:3d} tilt={tilt:3d} elbow={elbow_angle:3d}"
            )

            pan_changed = last_sent_pan is None or abs(pan - last_sent_pan) >= DEADBAND_DEG
            tilt_changed = last_sent_tilt is None or abs(tilt - last_sent_tilt) >= DEADBAND_DEG
            now = time.monotonic()
            time_ok = (now - last_send_time) * 1000 >= SEND_INTERVAL_MS

            if ser and time_ok and (pan_changed or tilt_changed):
                line = f"{pan},{tilt},{elbow_angle}\n" if SEND_ELBOW else f"{pan},{tilt}\n"
                try:
                    ser.write(line.encode())
                    last_sent_pan = pan
                    last_sent_tilt = tilt
                    last_send_time = now
                except serial.SerialException as e:
                    print(f"WARNING: serial write failed ({e}) - is the Arduino still connected?")

            cv2.putText(
                frame,
                f"pan:{pan} tilt:{tilt} elbow:{elbow_angle}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            cv2.putText(
                frame, "No pose detected", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2,
            )

        cv2.imshow("Boxing Robot Arm - Left Arm Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    landmarker.close()
    if ser:
        ser.close()


if __name__ == "__main__":
    main()
