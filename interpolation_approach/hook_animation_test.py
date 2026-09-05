"""
Boxing Robot Arm - Hook Animation Test
-----------------------------------------
Standalone test to lock down the timing/feel of the hook's yaw animation
BEFORE wiring in live hook detection. No camera, no MediaPipe - just sends
a scripted yaw sweep to the Arduino so the animation can be tuned in
isolation, the same way boxing_moves.py tests canned punches.

Why yaw only, holding pan/tilt/elbow fixed: the 4 HOOK_* calibration
points (HOOK_WINDUP -> HOOK_MID_1 -> HOOK_MID_2 -> HOOK_EXTENSION) all
share the same pan=0, tilt=90, elbow=0 - the entire hook, in servo terms,
is just a yaw sweep with the elbow held fully bent the whole way. That's
exactly the part live tracking couldn't hold steady (see the HOOK_MID
notes in calibration_data.json) - this locks it down as a fixed, verified
animation instead of live-tracking it.

SAFETY: STRIKE_END_YAW defaults to track.YAW_SAFE_FLOOR (40), NOT
HOOK_EXTENSION's raw calibration value of 0 - that 0 was captured before
YAW_SAFE_FLOOR existed as a guard against mechanically straining the
glued yaw horn. Only lower it if you've separately confirmed the current
mount handles yaw=0 safely.

Run (from anywhere):   python hook_animation_test.py
Trigger:                press ENTER to play the animation
Quit:                   type 'q' then ENTER
"""

import os
import sys
import time

import serial

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import track  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

# Fixed for the whole animation - matches every HOOK_* calibration point's
# pan/tilt/elbow (only yaw actually changes during a real hook, per the
# calibration data).
FIXED_PAN = 0
FIXED_TILT = 90
FIXED_ELBOW = 0

# STRIKE_START_YAW used to be a hardcoded animation start (145, guard-ish).
# Dropped 2026-08-26: detection only fires after you've already committed
# real travel into the swing (the speed+travel gate needs that), so your
# real yaw is usually already well past 145 by the time it triggers -
# forcing the animation to start there yanked the arm BACK to 145 first
# ("cocks back to load the hook") before ever striking. The strike now
# starts from wherever yaw actually is (passed in per-call as start_yaw),
# so there's no artificial windup - just the actual leftover distance to
# the target. STRIKE_START_YAW is kept only as the default for the
# standalone blocking test below, which has no live tracking to grab a
# real starting point from.
STRIKE_START_YAW = 145
STRIKE_END_YAW = 20               # deepened 30 -> 20 (2026-08-26) - NOT yet
                                  # confirmed safe. track.YAW_SAFE_FLOOR is
                                  # still 30, so the INTEGRATED pipeline will
                                  # keep clamping to 30 regardless of this
                                  # value until 20 is tested standalone here
                                  # (no floor in this script) and confirmed
                                  # not straining, same as the 40->30 move.
                                  # Don't lower YAW_SAFE_FLOOR to match until
                                  # that's actually been checked.

# Duration now scales with actual distance traveled instead of being fixed
# - since the start point varies (see above), a fixed duration would make
# a short leftover distance feel sluggish (slow relative motion) and a
# long one feel rushed. STRIKE_SPEED_DEG_PER_SEC keeps the FEEL consistent
# regardless of how far there is to go. MIN_STRIKE_DURATION_MS floors it
# so an already-very-close start doesn't produce a near-instant, glitchy
# snap.
# Raised 400 -> 450 (2026-08-26) alongside the FAST_STEP_SIZE bump in
# robot_arm.ino - keeps the software curve's requested rate roughly matched
# to the new hardware cap instead of asking for a rate the servo can't
# reach anyway.
STRIKE_SPEED_DEG_PER_SEC = 450
MIN_STRIKE_DURATION_MS = 80

HOLD_DURATION_MS = 120           # brief pause at extension, then hand back to live tracking
# No scripted return anymore (dropped same day, same reasoning as above) -
# live tracking has been watching your real arm this entire time, so once
# the hold ends we just resume it immediately instead of easing back to a
# fixed guard pose over several hundred ms. Whatever gap exists between
# the held extension and your real (already-retracting) arm position gets
# picked up as a bit of natural filter lag on the very next live frame,
# not a separate multi-hundred-ms animated phase.

SEND_INTERVAL_MS = 20            # how often to send an updated yaw during the animation

# ============================================================


def ease_in_cubic(t):
    """Slow start, ACCELERATING into the end - the strike's slowest moment
    used to be right at full extension (ease_out_cubic decelerates into
    the end), which read as weak/uncommitted right when it should feel
    like impact. This snaps hardest at the very end instead - windup, then
    the actual hit."""
    return t ** 3


def ease_in_out_cubic(t):
    """Smooth accelerate-then-decelerate - for the controlled return."""
    if t < 0.5:
        return 4 * t ** 3
    return 1 - ((-2 * t + 2) ** 3) / 2


def send(ser, pan, tilt, yaw, elbow):
    ser.write(f"{pan},{tilt},{yaw},{elbow}\n".encode())


def hook_animation_frame(elapsed_ms, start_yaw=None):
    """Pure/stateless: given elapsed ms since the animation started (and
    the ACTUAL yaw value at the moment it started), return
    (yaw, elbow, finished). Single source of truth for the animation curve
    - both the blocking standalone test below AND track_interpolation.py's
    per-frame integration call this.

    start_yaw: wherever yaw actually was when the hook was detected -
    defaults to STRIKE_START_YAW for the standalone test (no live tracking
    to grab a real value from there). The strike eases from THIS to
    STRIKE_END_YAW, not from a fixed guard pose - see the comment above
    STRIKE_START_YAW for why that matters.

    Written as a pure function of elapsed time (not a sleep-loop) so the
    live tracker can query "what should yaw/elbow be right now" once per
    camera frame without blocking - pan/tilt keep coming from live tracking
    the whole time the hook animation is playing."""
    if start_yaw is None:
        start_yaw = STRIKE_START_YAW

    distance = abs(STRIKE_END_YAW - start_yaw)
    strike_duration_ms = max(distance / STRIKE_SPEED_DEG_PER_SEC * 1000.0, MIN_STRIKE_DURATION_MS)

    if elapsed_ms < strike_duration_ms:
        t = elapsed_ms / strike_duration_ms
        yaw = start_yaw + (STRIKE_END_YAW - start_yaw) * ease_in_cubic(t)
        return yaw, FIXED_ELBOW, False

    elapsed_ms -= strike_duration_ms
    if elapsed_ms < HOLD_DURATION_MS:
        return STRIKE_END_YAW, FIXED_ELBOW, False

    return STRIKE_END_YAW, FIXED_ELBOW, True


def play_hook_animation(ser):
    """Blocking version for the standalone test only - loops calling
    hook_animation_frame() and sending until it reports finished. The
    live-tracking integration doesn't use this function directly (it can't
    block the camera loop); it calls hook_animation_frame() itself once per
    frame instead."""
    print(f"Strike -> hold: yaw {STRIKE_START_YAW} -> {STRIKE_END_YAW}, then done "
          f"(no scripted return - live tracking would take over here)")
    start_time = time.monotonic()
    while True:
        elapsed_ms = (time.monotonic() - start_time) * 1000.0
        yaw, elbow, finished = hook_animation_frame(elapsed_ms)
        send(ser, FIXED_PAN, FIXED_TILT, int(round(yaw)), elbow)
        if finished:
            break
        time.sleep(SEND_INTERVAL_MS / 1000.0)


def main():
    ser = serial.Serial(track.SERIAL_PORT, track.BAUD_RATE, timeout=1, write_timeout=1)
    time.sleep(2)  # give the Arduino time to reset after the port opens
    print(f"Connected to Arduino on {track.SERIAL_PORT}")

    send(ser, FIXED_PAN, FIXED_TILT, STRIKE_START_YAW, FIXED_ELBOW)
    time.sleep(1)  # settle at guard before the first strike

    print("Press ENTER to play the hook animation, 'q' + ENTER to quit.")
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "q":
            break
        play_hook_animation(ser)

    ser.close()


if __name__ == "__main__":
    main()
