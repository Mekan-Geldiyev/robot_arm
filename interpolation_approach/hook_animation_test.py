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

STRIKE_START_YAW = 145           # HOOK_WINDUP's yaw - guard/chambered
STRIKE_END_YAW = 30              # requested deeper than track.YAW_SAFE_FLOOR (40) -
                                  # that floor exists because yaw values below it
                                  # were previously reported to strain the glued
                                  # horn. Confirm the mount actually handles this
                                  # before testing, especially if the servo isn't
                                  # responding right now for an unrelated reason -
                                  # see the chat history before running this again.
STRIKE_DURATION_MS = 300         # snap out fast - real hooks are ~200-400ms
HOLD_DURATION_MS = 120           # brief pause at extension before returning
RETURN_DURATION_MS = 450         # ease back to guard - slower/more controlled than the strike

SEND_INTERVAL_MS = 20            # how often to send an updated yaw during the animation

# ============================================================


def ease_out_cubic(t):
    """Fast start, decelerating into the end - a natural "snap out" feel."""
    return 1 - (1 - t) ** 3


def ease_in_out_cubic(t):
    """Smooth accelerate-then-decelerate - for the controlled return."""
    if t < 0.5:
        return 4 * t ** 3
    return 1 - ((-2 * t + 2) ** 3) / 2


def send(ser, pan, tilt, yaw, elbow):
    ser.write(f"{pan},{tilt},{yaw},{elbow}\n".encode())


TOTAL_DURATION_MS = STRIKE_DURATION_MS + HOLD_DURATION_MS + RETURN_DURATION_MS


def hook_animation_frame(elapsed_ms):
    """Pure/stateless: given elapsed ms since the animation started, return
    (yaw, elbow, finished). This is the single source of truth for the
    animation curve - both the blocking standalone test below AND
    track_interpolation.py's per-frame integration call this, so tuning the
    constants above changes the feel in both places identically.

    Written as a pure function of elapsed time (not a sleep-loop) so the
    live tracker can query "what should yaw/elbow be right now" once per
    camera frame without blocking - pan/tilt keep coming from live tracking
    the whole time the hook animation is playing."""
    if elapsed_ms < STRIKE_DURATION_MS:
        t = elapsed_ms / STRIKE_DURATION_MS
        yaw = STRIKE_START_YAW + (STRIKE_END_YAW - STRIKE_START_YAW) * ease_out_cubic(t)
        return yaw, FIXED_ELBOW, False

    elapsed_ms -= STRIKE_DURATION_MS
    if elapsed_ms < HOLD_DURATION_MS:
        return STRIKE_END_YAW, FIXED_ELBOW, False

    elapsed_ms -= HOLD_DURATION_MS
    if elapsed_ms < RETURN_DURATION_MS:
        t = elapsed_ms / RETURN_DURATION_MS
        yaw = STRIKE_END_YAW + (STRIKE_START_YAW - STRIKE_END_YAW) * ease_in_out_cubic(t)
        return yaw, FIXED_ELBOW, False

    return STRIKE_START_YAW, FIXED_ELBOW, True


def play_hook_animation(ser):
    """Blocking version for the standalone test only - loops calling
    hook_animation_frame() and sending until it reports finished. The
    live-tracking integration doesn't use this function directly (it can't
    block the camera loop); it calls hook_animation_frame() itself once per
    frame instead."""
    print(f"Strike -> hold -> return: yaw {STRIKE_START_YAW} -> {STRIKE_END_YAW} -> "
          f"{STRIKE_START_YAW} over {TOTAL_DURATION_MS}ms total")
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
