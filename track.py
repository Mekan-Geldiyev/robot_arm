"""
Boxing Robot Arm - Real-Time Motion Mirroring
-----------------------------------------------
Tracks the user's LEFT arm with MediaPipe Pose and streams
"pan,tilt,yaw,elbow\n" servo angles to the Arduino over serial, which then
smoothly drives the shoulder/elbow servos to match.

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

# --- Crop out the frame-right region before pose detection so the user's
# real RIGHT arm (e.g. holding a phone up to film) is never actually visible
# to MediaPipe, not just filtered out after the fact by SIDE_A/SIDE_B
# picking - that per-frame candidate selection only picks which label to
# trust, it can't stop a fully-visible second arm from pulling the tracked
# arm's own landmark ESTIMATES off toward it (a known monocular pose-model
# failure mode: occluded/ambiguous joints get inferred using cues from the
# other visible limb). After the mirror flip below, the user's true left
# arm sits on the frame-left side and the real right arm on frame-right, so
# cropping to the leftmost fraction removes the right arm at the source.
# 1.0 = no crop (both arms visible, old behavior). Lower this if the right
# arm is still creeping into the cropped region; raise it if a left hook's
# full swing is getting clipped off the right edge of the video window.
CROP_RIGHT_FRAC = 0.65

# --- Pose model (downloaded automatically on first run) ---
# Using "full" instead of "lite": meaningfully better landmark accuracy/
# confidence (fewer low-confidence holds, less jitter on tricky poses like
# a raised arm), at some extra CPU cost per frame - still real-time on a
# normal PC. Delete the old pose_landmarker_lite.task file if you want to
# free up the disk space, it's no longer used.
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pose_landmarker_full.task")
MODEL_URL = "https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"

# --- Deadband: don't send a new value unless it changed by at least this much.
# Applied per-channel (pan/tilt/elbow each gated independently against their
# own last SENT value) - a channel that hasn't moved enough holds its last
# sent value even when another channel's motion triggers a send. Without
# per-channel gating, any send triggered by pan/tilt would carry along
# whatever the elbow's current (possibly noisy) value happened to be,
# vibrating the elbow servo even though nothing elbow-related crossed its
# own threshold. ---
DEADBAND_DEG = 2

# --- Smoothing: a flat exponential moving average can't satisfy both
# "don't vibrate when the hand is still" and "keep up with an actual fast
# punch" - those pull the same constant in opposite directions (smooth
# enough to kill stationary jitter is also too slow to follow a jab).
# The One Euro Filter (Casiez et al. 2012 - the standard technique for
# exactly this in motion capture/AR pose tracking) fixes that by
# estimating how fast the signal is currently moving and adapting its
# cutoff frequency per frame: heavy filtering near zero speed, which
# automatically loosens up as speed increases.
#
#   MIN_CUTOFF -> filtering strength when nearly still, in Hz. Lower = more
#                 stationary-jitter rejection (plays the role SMOOTH_ALPHA
#                 used to). Too low makes the very start of a punch feel
#                 delayed.
#   BETA       -> how fast filtering loosens up as speed increases. Higher =
#                 snappier during real motion (less lag on a fast punch),
#                 but too high lets more jitter leak through while moving.
#   D_CUTOFF   -> smooths the internal speed estimate itself; 1.0 is the
#                 standard default, rarely worth changing.
#
# Elbow gets its own (much larger) BETA because its calibration range is
# only ~10 raw degrees across the full 0-90 servo sweep (see ELBOW_MIN/MAX
# below, ~9x amplification) - the same real elbow motion produces a much
# smaller raw degrees/sec than pan/tilt see for equivalent motion, so it
# needs a proportionally bigger speed coefficient to unlock the same
# responsiveness during a punch.
PAN_MIN_CUTOFF, PAN_BETA = 0.4, 0.03
TILT_MIN_CUTOFF, TILT_BETA = 0.4, 0.03
YAW_MIN_CUTOFF, YAW_BETA = 0.4, 0.03
ELBOW_MIN_CUTOFF, ELBOW_BETA = 0.15, 0.10
# ELBOW_MIN_CUTOFF/BETA history: 0.2/0.15 (with the IK fix) -> 0.15/0.10 on
# 2026-08-05, user reported it still "tweaking"/switching fast - lowered
# both a notch for more baseline smoothing (MIN_CUTOFF) and less speed-
# triggered snapping (BETA). Some of that switching may actually have been
# the SIDE_A/SIDE_B arm-identity flicker below rather than the filter
# itself - re-evaluate once that's confirmed fixed before tuning further.
D_CUTOFF = 1.0

# --- Depth (z-axis) instrumentation - not wired to any servo yet, there's no
# depth-capable joint on the hardware. This just filters + prints/overlays a
# depth signal so we can see how usable MediaPipe's built-in landmark.z is
# before deciding how (or whether) to act on it - see README item 5. ---
DEPTH_MIN_CUTOFF, DEPTH_BETA = 0.4, 0.03

# --- Don't flood the serial link faster than this, independent of camera FPS ---
SEND_INTERVAL_MS = 40   # ~25 updates/sec - plenty for smooth servo motion

# --- Skip frames where MediaPipe isn't confident about a landmark's
# position (e.g. a hand raised near/above the top of the frame, or an arm
# briefly occluded) instead of computing angles from a bad guess. Without
# this, a single bad low-confidence estimate can flip the sign of the tilt
# calculation for a frame or two - which is what raising your arm straight
# overhead (wrist near/past the top frame edge) looks like. ---
VISIBILITY_MIN = 0.5

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
INVERT_PAN = True

# Hardware note (2026-08-12): pan is the base joint - the tilt/yaw/elbow
# assembly and forearm are all mounted on top of it, so on the current
# build sweeping pan doesn't just point the arm left/right, it rolls the
# entire downstream assembly with it, which is what flips the hand's
# orientation past a certain point. Locking pan (PAN_LOCKED=True) avoids
# the flip but loses all front-vs-side distinction, since pan is the only
# channel that encodes horizontal aim - re-opened per user request
# (distinction matters more right now than the flip). Real fix is
# mechanical (decouple pan's axis from roll on the next rebuild), not
# software - re-lock this if the flip becomes the bigger problem again.
PAN_LOCKED = False
PAN_LOCK_VALUE = 0

# Fitted from real Serial Monitor ground truth (see README "Calibration
# Reference Values"): with INVERT_PAN off, raw=178 <-> servo=180 (hand at
# side), raw=133 <-> servo=0 (hand out to the side). Narrower than it used
# to be because the pan/tilt formulas now measure shoulder->WRIST instead
# of shoulder->elbow.
# INVERT_PAN flipped on 2026-08-03: user reported the pan direction felt
# backwards, so this now flips to raw=178 <-> servo=0, raw=133 <-> servo=180.

# Tilt formula: elevation_angle(shoulder, wrist) is atan2 of vertical vs
# horizontal offset, so it's bounded by construction: -90 = arm hanging
# straight down, 0 = arm straight out to the side (horizontal), +90 = arm
# straight up overhead. Re-fit on 2026-08-03 from direct Serial Monitor
# ground truth (bypassing track.py, sending servo angles straight to the
# Arduino and matching the physical arm against the described real pose):
#   hand hanging down at side              -> servo tilt = 180
#   hand straight out to the side           -> servo tilt = 90
#   hand straight up overhead               -> servo tilt = 0
# Previously TILT_SERVO_MAX was capped at 90, which made 180 physically
# unreachable no matter what the tracked hand did - that's why the
# hand-at-side position "didn't work at all". Using the full theoretical
# -90..+90 raw range (rather than the old narrow empirically-fit -84..-16)
# also means less MediaPipe noise gets clamped into the servo extremes.
TILT_MIN, TILT_MAX = -90, 90          # raw shoulder elevation angle range
TILT_SERVO_MIN, TILT_SERVO_MAX = 0, 180
INVERT_TILT = True
# This is the servo's full mechanical range end-to-end (0 and 180 both
# now reachable) - watch closely the first time you test hand-down and
# hand-up-overhead in case the linkage binds/stalls at either extreme
# (the old code avoided the top half of the range over a stall concern,
# though that concern was based on the previous, wrong-direction mapping).

# Elbow raw angle is now computed via 2-link IK (law of cosines on the
# shoulder<->wrist distance, using fixed upper-arm/forearm lengths measured
# from the shoulder/elbow/wrist landmarks) instead of reading the elbow
# landmark's own angle directly every frame - see elbow_ik_angle() below and
# the 2026-08-05 note above VISIBILITY_MIN-adjacent code for why. This makes
# the raw signal a genuine anatomical flex angle (~180 deg = arm straight,
# smaller = more bent), a wide, natural range instead of the old 10-degree
# span that amplified noise ~9x.
#
# Placeholder range below is a reasonable boxing guess (guard ~90-100 deg
# bent, punch extension ~170-180 deg) - NOT yet hardware-verified. Watch the
# printed "raw: ... elbow=" values while moving through guard -> full
# extension and correct these two numbers before trusting the servo output.
ELBOW_MIN, ELBOW_MAX = 90, 178        # raw elbow flex angle range (needs re-verification)
ELBOW_SERVO_MIN, ELBOW_SERVO_MAX = 0, 90
INVERT_ELBOW = False
# Direction should carry over unchanged from the old calibration: increasing
# raw (more bent -> more straight) still maps to increasing servo (0 -> 90,
# where 90 = straight), same as before.

# When the arm is straight (jab), the elbow sits almost exactly on the
# shoulder->wrist line, so its offset FROM that line - what yaw_angle()
# measures - shrinks toward zero and its direction becomes dominated by
# landmark noise (a near-zero vector's angle is inherently unstable, not a
# calibration problem). That's why yaw looked like it "wasn't moving" when
# the arm was held straight out - there's no real hinge-plane signal there
# to move. Fade raw_yaw toward neutral as the elbow (from the IK above)
# straightens past this threshold, so a jab reads as intentionally centered
# instead of chasing noise; below it, the hook/guard bend is enough for the
# plane angle to mean something and the full signal is used.
YAW_ELBOW_FADE_START = 170  # raw elbow degrees - fade begins here (170 = nearly straight)
YAW_ELBOW_FADE_END = 150    # raw elbow degrees - full yaw signal at/below this bend

# Yaw (3rd shoulder DOF, added 2026-08-12): rotates the whole shoulder
# assembly around the shoulder->wrist axis, which is what actually swings
# the elbow's hinge plane out to the side for a hook - pan/tilt alone can
# already point the arm anywhere, but can't rotate that plane since it's
# fixed by how the elbow is physically glued on. See yaw_angle() below for
# how the raw signal is computed.
#
# Placeholder range below is a guess (small rotation for guard/jab, wider
# for a hook) - NOT yet hardware-verified. Watch the printed "raw: ...
# yaw=" values while moving through guard -> jab -> hook and correct these
# two numbers before trusting the servo output, same as ELBOW_MIN/MAX above.
# YAW_MIN widened -90 -> -120 on 2026-08-12: real terminal output regularly
# showed raw yaw down to -115, well past the old -90 floor - map_range()
# clamps anything past YAW_MIN to the same output, so the servo was pegged
# at one extreme (then the safety floor below) almost the entire time
# instead of tracking real motion. Still a guess on the exact edge, but
# matches the data actually observed instead of the original blind guess.
YAW_MIN, YAW_MAX = -120, 90           # raw hinge-plane rotation range (needs verification)
YAW_SERVO_MIN, YAW_SERVO_MAX = 0, 180
INVERT_YAW = False

# The yaw servo horn was hand-glued onto the outside of the shoulder
# bracket, so which physical servo angle actually makes the arm's real-world
# rotation "correct" (e.g. palm down, not upside-down, when the arm's out to
# the side) is a mounting fact, not something the raw_yaw formula can know -
# it assumes raw_yaw==0 (its neutral/guard reference) belongs at the servo
# sweep's plain midpoint (90), which only holds if the horn happened to get
# glued on dead-center. It didn't, based on the reported upside-down hand.
#
# To fix: hold the arm straight out to the side (elbow extended enough that
# YAW_ELBOW_FADE_START above forces raw_yaw toward 0 - this is the pose
# YAW_SERVO_OFFSET is measured against), then use the Arduino Serial
# Monitor to send yaw servo values directly until the hand's orientation
# actually looks right (palm down). Whatever value that is, minus 90 (the
# old assumed center), is YAW_SERVO_OFFSET below.
YAW_SERVO_OFFSET = 0  # degrees added to the mapped yaw output - PLACEHOLDER, set from the test above

# TEMPORARY hardware safety floor (2026-08-12): the yaw servo horn is
# hand-glued on with limited real range right now (yaw:0 was confirmed
# "hella bad" - binds/strains against the mount) - a proper reglue for
# full range of motion is planned but hasn't happened yet. Until then,
# never let the commanded yaw servo value go below this, no matter what
# raw_yaw/mapping/offset computed - 40 is a value the user confirmed looks
# and feels fine on the current glue job. Remove this floor (or set it back
# to 0) once the mount is rebuilt for full range.
YAW_SAFE_FLOOR = 40

# EMA smoothing factor for the L1/L2 arm-segment-length estimates used by
# the elbow IK below. Very slow on purpose (0.01 = ~99% history) - segment
# lengths are basically constant for a given person at a given distance
# from the camera, so heavy averaging lets a single noisy/occluded elbow
# landmark reading (e.g. elbow tucked in front of the torso during a hook)
# barely move the estimate, instead of directly corrupting the angle output
# the way reading the elbow's raw angle every frame used to.
ELBOW_LENGTH_ALPHA = 0.01

# ============================================================

# MediaPipe Pose landmark indices for tracking the user's real LEFT arm + hip.
# Note: frame is horizontally flipped (mirror view) before pose detection,
# and MediaPipe's left/right labeling is appearance-based (not geometric),
# so it comes out swapped on a flipped feed - its "RIGHT_*" landmarks are
# normally what correspond to the user's true left arm here.
#
# "Normally" because that label is a per-frame guess, not a fixed fact - on
# a confusing pose (arms crossing, one arm low-confidence, etc) MediaPipe
# can flip which side it calls which, and the tracked arm silently jumps to
# the user's other arm for a frame or several. Instead of trusting either
# label blindly, both candidate sets are computed each frame and picked by
# comparing SHOULDER x-position (see tracked_side selection in main() below)
# - shoulders essentially never swap left-right order during normal
# front-facing motion, unlike wrists, which are *supposed* to cross the
# body's midline during a hook/cross (an earlier version of this compared
# wrist positions instead and could get fooled by exactly that).
SIDE_A = {"shoulder": 12, "elbow": 14, "wrist": 16, "hip": 24}  # "RIGHT_*" - true left arm, normally
SIDE_B = {"shoulder": 11, "elbow": 13, "wrist": 15, "hip": 23}  # "LEFT_*" - true right arm, normally

# Require the other side's shoulder to be meaningfully further left/right
# (not just marginally) before switching tracked side - otherwise near-ties
# from ordinary jitter would flicker the tracked arm back and forth.
SIDE_SWITCH_MARGIN = 0.05  # normalized image-space units


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


def landmark_distance(a, b, keys=("x", "y")):
    return math.hypot(*(getattr(a, k) - getattr(b, k) for k in keys))


def landmark_vec3(a, b):
    return (b.x - a.x, b.y - a.y, b.z - a.z)


def v_sub(u, v):
    return (u[0] - v[0], u[1] - v[1], u[2] - v[2])


def v_scale(u, s):
    return (u[0] * s, u[1] * s, u[2] * s)


def v_dot(u, v):
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def v_cross(u, v):
    return (
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    )


def v_len(u):
    return math.sqrt(v_dot(u, u))


def yaw_angle(shoulder, elbow, wrist, hip):
    """Signed rotation (degrees) of the shoulder-elbow-wrist hinge plane
    around the shoulder->wrist axis, measured against shoulder->hip as the
    0-degree reference (elbow hanging toward the body, like a guard/jab).
    This is the "elbow hinge plane" angle - the pan/tilt gimbal can already
    point the arm anywhere in 3D, but only rotating around this axis swings
    the elbow's bend out to the side (a hook), since the elbow's hinge
    plane is otherwise fixed by how it's physically mounted."""
    d = landmark_vec3(shoulder, wrist)
    d_len = v_len(d)
    if d_len < 1e-6:
        return None
    d_hat = v_scale(d, 1.0 / d_len)

    e = landmark_vec3(shoulder, elbow)
    e_perp = v_sub(e, v_scale(d_hat, v_dot(e, d_hat)))
    r = landmark_vec3(shoulder, hip)
    r_perp = v_sub(r, v_scale(d_hat, v_dot(r, d_hat)))

    e_len = v_len(e_perp)
    r_len = v_len(r_perp)
    if e_len < 1e-6 or r_len < 1e-6:
        return None

    cos_angle = clamp(v_dot(e_perp, r_perp) / (e_len * r_len), -1.0, 1.0)
    angle = math.degrees(math.acos(cos_angle))

    # Sign distinguishes the elbow swinging out to one side of the
    # reference plane vs the other (e.g. a right hook vs a left hook).
    sign = 1.0 if v_dot(v_cross(r_perp, e_perp), d_hat) >= 0 else -1.0
    return sign * angle


def elbow_ik_angle(l1, l2, d):
    """Elbow flex angle (degrees) from the law of cosines, given the fixed
    upper-arm length l1 (shoulder->elbow), forearm length l2 (elbow->wrist),
    and the current shoulder->wrist distance d. 180 = arm fully straight
    (d == l1 + l2), smaller = more bent. Only shoulder and wrist positions
    are needed per-frame - the elbow landmark itself (frequently occluded
    or noisy, e.g. tucked in front of the torso during a hook) is only used
    to establish l1/l2 once, slowly, not for the live angle."""
    if l1 <= 0 or l2 <= 0:
        return None
    cos_angle = (l1 * l1 + l2 * l2 - d * d) / (2 * l1 * l2)
    cos_angle = clamp(cos_angle, -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


class OneEuroFilter:
    """Adaptive low-pass filter: strong smoothing when the input is nearly
    still, progressively less (lower lag) the faster it's moving. See
    Casiez, Roussel & Vogel, "1 Euro Filter: A Simple Speed-based Low-pass
    Filter for Noisy Input in Interactive Systems" (CHI 2012)."""

    def __init__(self, min_cutoff, beta, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        r = 2 * math.pi * cutoff * dt
        return r / (r + 1)

    def __call__(self, t, x):
        if self.x_prev is None:
            self.x_prev = x
            self.t_prev = t
            return x

        dt = t - self.t_prev
        if dt <= 0:
            return self.x_prev

        a_d = self._alpha(self.d_cutoff, dt)
        dx = (x - self.x_prev) / dt
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


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
    last_sent_yaw = None
    last_sent_elbow = None
    last_send_time = 0.0
    pan_filter = OneEuroFilter(PAN_MIN_CUTOFF, PAN_BETA, D_CUTOFF)
    tilt_filter = OneEuroFilter(TILT_MIN_CUTOFF, TILT_BETA, D_CUTOFF)
    yaw_filter = OneEuroFilter(YAW_MIN_CUTOFF, YAW_BETA, D_CUTOFF)
    elbow_filter = OneEuroFilter(ELBOW_MIN_CUTOFF, ELBOW_BETA, D_CUTOFF)
    depth_filter = OneEuroFilter(DEPTH_MIN_CUTOFF, DEPTH_BETA, D_CUTOFF)
    elbow_l1 = None  # slow EMA estimate of shoulder->elbow length (IK)
    elbow_l2 = None  # slow EMA estimate of elbow->wrist length (IK)
    tracked_side = SIDE_A  # which candidate landmark set we're currently following
    start_time = time.monotonic()

    print("Press 'q' to quit.")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame_time = time.monotonic()
        frame = cv2.flip(frame, 1)  # mirror view, feels more natural to stand in front of
        if CROP_RIGHT_FRAC < 1.0:
            frame = frame[:, : int(frame.shape[1] * CROP_RIGHT_FRAC)]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int((frame_time - start_time) * 1000)
        results = landmarker.detect_for_video(mp_image, timestamp_ms)

        if results.pose_landmarks:
            lm = results.pose_landmarks[0]
            drawing_utils.draw_landmarks(frame, lm, POSE_CONNECTIONS)

            # Pick the tracked side from SHOULDER x-position, not wrist - wrists
            # are supposed to cross the body's midline during a hook/cross, so
            # continuity-tracking the wrist could latch onto the wrong arm at
            # exactly the moment (a real punch) we most need it to be right.
            # Shoulders essentially never swap left-right order during normal
            # front-facing motion, so this is a plain per-frame geometric fact,
            # not a fragile guess: whichever shoulder sits further toward the
            # frame-left edge (smaller x) is the user's true left shoulder.
            shoulder_a = lm[SIDE_A["shoulder"]]
            shoulder_b = lm[SIDE_B["shoulder"]]
            if tracked_side is SIDE_A:
                if shoulder_b.x + SIDE_SWITCH_MARGIN < shoulder_a.x:
                    tracked_side = SIDE_B
            else:
                if shoulder_a.x + SIDE_SWITCH_MARGIN < shoulder_b.x:
                    tracked_side = SIDE_A

            shoulder = lm[tracked_side["shoulder"]]
            elbow = lm[tracked_side["elbow"]]
            wrist = lm[tracked_side["wrist"]]
            hip = lm[tracked_side["hip"]]

            # Elbow is deliberately NOT part of this gate (see below) - the whole
            # point of the IK approach is that pan/tilt/elbow-IK only need
            # shoulder/wrist/hip, so an occluded elbow (e.g. tucked in front of
            # the torso mid-hook) shouldn't freeze pan/tilt tracking too.
            low_confidence = (
                shoulder.visibility < VISIBILITY_MIN
                or wrist.visibility < VISIBILITY_MIN
                or hip.visibility < VISIBILITY_MIN
            )
            elbow_visible = elbow.visibility >= VISIBILITY_MIN

            if low_confidence:
                cv2.putText(
                    frame, "Low confidence - holding last position", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2,
                )
            else:
                raw_pan = angle_3pt(hip, shoulder, wrist, keys=("x", "z"))
                raw_tilt = elevation_angle(shoulder, wrist)

                # Elbow IK: shoulder->wrist distance is the fast per-frame
                # signal; l1/l2 (arm segment lengths) only update - slowly -
                # when the elbow landmark itself is confidently visible, so a
                # momentary bad elbow read can't corrupt this frame's angle.
                # Uses x/y/z (not just x/y) - an arm pointed mostly along the
                # camera's depth axis (e.g. hanging straight down close to
                # the body) foreshortens in the image plane even when fully
                # extended, which was reading as "bent" using x/y alone.
                if elbow_visible:
                    l1_frame = landmark_distance(shoulder, elbow, keys=("x", "y", "z"))
                    l2_frame = landmark_distance(elbow, wrist, keys=("x", "y", "z"))
                    if elbow_l1 is None:
                        elbow_l1, elbow_l2 = l1_frame, l2_frame  # bootstrap
                    else:
                        elbow_l1 += ELBOW_LENGTH_ALPHA * (l1_frame - elbow_l1)
                        elbow_l2 += ELBOW_LENGTH_ALPHA * (l2_frame - elbow_l2)

                wrist_shoulder_dist = landmark_distance(shoulder, wrist, keys=("x", "y", "z"))
                raw_elbow = (
                    elbow_ik_angle(elbow_l1, elbow_l2, wrist_shoulder_dist)
                    if elbow_l1 is not None else None
                )

                # Yaw needs the elbow's actual current position (unlike the
                # elbow IK above, there's no slow length estimate to fall
                # back on), so it's only computed on frames the elbow is
                # confidently visible - held at its last servo value otherwise.
                raw_yaw = yaw_angle(shoulder, elbow, wrist, hip) if elbow_visible else None
                if raw_yaw is not None and raw_elbow is not None:
                    fade = (YAW_ELBOW_FADE_START - raw_elbow) / (YAW_ELBOW_FADE_START - YAW_ELBOW_FADE_END)
                    raw_yaw *= clamp(fade, 0.0, 1.0)

                # Depth instrumentation only (see DEPTH_MIN_CUTOFF above) - not
                # sent anywhere yet, just observed. More negative z = farther
                # from the camera than the hip-relative origin MediaPipe uses;
                # a jab toward the camera should show this trending one way.
                raw_depth = wrist.z - shoulder.z
                smooth_depth = depth_filter(frame_time, raw_depth)

                # Low-pass filter the noisy per-frame angles before mapping/sending,
                # so the arm follows your overall motion instead of chasing jitter -
                # speed-adaptive (see OneEuroFilter) so it stays smooth when still
                # but doesn't lag behind an actual fast punch.
                smooth_pan = pan_filter(frame_time, raw_pan)
                smooth_tilt = tilt_filter(frame_time, raw_tilt)

                pan = PAN_LOCK_VALUE if PAN_LOCKED else int(map_range(smooth_pan, PAN_MIN, PAN_MAX, PAN_SERVO_MIN, PAN_SERVO_MAX, INVERT_PAN))
                tilt = int(map_range(smooth_tilt, TILT_MIN, TILT_MAX, TILT_SERVO_MIN, TILT_SERVO_MAX, INVERT_TILT))

                if raw_elbow is not None:
                    smooth_elbow = elbow_filter(frame_time, raw_elbow)
                    elbow_angle = int(map_range(smooth_elbow, ELBOW_MIN, ELBOW_MAX, ELBOW_SERVO_MIN, ELBOW_SERVO_MAX, INVERT_ELBOW))
                else:
                    # No elbow-length estimate yet (elbow hasn't been visible
                    # since startup) - hold the last known servo position
                    # instead of guessing.
                    elbow_angle = last_sent_elbow if last_sent_elbow is not None else (ELBOW_SERVO_MIN + ELBOW_SERVO_MAX) // 2

                if raw_yaw is not None:
                    smooth_yaw = yaw_filter(frame_time, raw_yaw)
                    mapped_yaw = map_range(smooth_yaw, YAW_MIN, YAW_MAX, YAW_SERVO_MIN, YAW_SERVO_MAX, INVERT_YAW)
                    yaw = int(clamp(mapped_yaw + YAW_SERVO_OFFSET, 0, 180))
                else:
                    # Elbow not visible this frame - hold the last known yaw
                    # servo position instead of guessing.
                    yaw = last_sent_yaw if last_sent_yaw is not None else (YAW_SERVO_MIN + YAW_SERVO_MAX) // 2

                yaw = max(yaw, YAW_SAFE_FLOOR)

                side_label = "A" if tracked_side is SIDE_A else "B"
                raw_elbow_str = f"{raw_elbow:6.1f}" if raw_elbow is not None else "  n/a "
                raw_yaw_str = f"{raw_yaw:6.1f}" if raw_yaw is not None else "  n/a "
                print(
                    f"[side {side_label}] raw: pan={raw_pan:6.1f} tilt={raw_tilt:6.1f} yaw={raw_yaw_str} elbow={raw_elbow_str} depth={smooth_depth:+.3f}  ->  "
                    f"servo: pan={pan:3d} tilt={tilt:3d} yaw={yaw:3d} elbow={elbow_angle:3d}"
                )

                # Each channel is gated against its OWN last sent value, independently.
                # A channel that hasn't moved past its deadband holds steady at its
                # last sent value even when a send is triggered by a different channel
                # - otherwise e.g. pan crossing its threshold would also flush out
                # whatever noisy elbow value happened to be sitting there, vibrating
                # the elbow servo for motion that was never actually elbow motion.
                pan_changed = last_sent_pan is None or abs(pan - last_sent_pan) >= DEADBAND_DEG
                tilt_changed = last_sent_tilt is None or abs(tilt - last_sent_tilt) >= DEADBAND_DEG
                yaw_changed = last_sent_yaw is None or abs(yaw - last_sent_yaw) >= DEADBAND_DEG
                elbow_changed = last_sent_elbow is None or abs(elbow_angle - last_sent_elbow) >= DEADBAND_DEG

                send_pan = pan if pan_changed else last_sent_pan
                send_tilt = tilt if tilt_changed else last_sent_tilt
                send_yaw = yaw if yaw_changed else last_sent_yaw
                send_elbow = elbow_angle if elbow_changed else last_sent_elbow

                time_ok = (frame_time - last_send_time) * 1000 >= SEND_INTERVAL_MS

                if ser and time_ok and (pan_changed or tilt_changed or yaw_changed or elbow_changed):
                    line = f"{send_pan},{send_tilt},{send_yaw},{send_elbow}\n"
                    try:
                        ser.write(line.encode())
                        last_sent_pan = send_pan
                        last_sent_tilt = send_tilt
                        last_sent_yaw = send_yaw
                        last_sent_elbow = send_elbow
                        last_send_time = frame_time
                    except serial.SerialException as e:
                        print(f"WARNING: serial write failed ({e}) - is the Arduino still connected?")

                cv2.putText(
                    frame,
                    f"[{side_label}] pan:{pan} tilt:{tilt} yaw:{yaw} elbow:{elbow_angle}  depth:{smooth_depth:+.2f}",
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
