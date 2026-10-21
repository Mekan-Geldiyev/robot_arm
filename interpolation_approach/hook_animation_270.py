"""
Boxing Robot Arm (270-degree build) - Hook Animation Test
----------------------------------------------------------
Standalone test to lock down the timing/feel of the hook's yaw animation
on the new hardware, BEFORE wiring it into anything live - same reasoning
and same proven curve as the original hook_animation_test.py (kept intact
in the 180-degree robot_arm repo), just re-parameterized for this build's
real centered values (see positions_270.py).

Why yaw only, holding pan/tilt/elbow fixed: same as before - a hook is,
in servo terms, a yaw sweep with pan/tilt/elbow held at the real "Hook"
reference position (positions_270.POSITIONS["HOOK"] = pan=169, tilt=120,
elbow=55). That's the fixed baseline for the whole animation.

SAFETY: STRIKE_END_YAW defaults to 30, NOT yaw's literal mechanical floor
(0). Yaw's neutral on this build is 70, with ~70 degrees of real room
toward the body (the crash-risk direction, see positions_270.py's header)
before anything would hit - 30 uses less than half of that budget,
leaving real margin. This is a FIRST GUESS to test and deepen from, the
same way the original hook's depth was only lowered (40->30->20) after
each value was separately confirmed safe standalone - do not assume 30 is
either too shallow or already at the limit without testing it for real.

Run (from anywhere):   python hook_animation_270.py
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
from positions_270 import POSITIONS  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

FIXED_PAN, FIXED_TILT, _HOOK_YAW, FIXED_ELBOW = POSITIONS["HOOK"]

# STRIKE_START_YAW used to be a hardcoded animation start on the old
# hardware. Kept here only as the standalone test's default (no live
# tracking to grab a real value from) - the real integration later passes
# wherever yaw actually is at the moment a hook is detected, same as
# before, so there's no artificial windup.
STRIKE_START_YAW = _HOOK_YAW    # 70 on this build - the real "Hook" reference yaw
STRIKE_END_YAW = 30             # see SAFETY above - untested first guess, confirm standalone

STRIKE_SPEED_DEG_PER_SEC = 450   # same feel as the original hook animation
MIN_STRIKE_DURATION_MS = 80
HOLD_DURATION_MS = 120          # brief pause at extension before handing back to live tracking
SEND_INTERVAL_MS = 20           # how often to send an updated yaw during the animation

# ============================================================


def ease_in_cubic(t):
    """Slow start, ACCELERATING into the end - same curve as the original
    hook animation, proven to feel like a real snap rather than a weak
    decelerating finish."""
    return t ** 3


def send(ser, pan, tilt, yaw, elbow):
    ser.write(f"{pan},{tilt},{yaw},{elbow}\n".encode())


def hook_animation_frame(elapsed_ms, start_yaw=None, target_yaw=None):
    """Pure/stateless: given elapsed ms since the animation started (and
    the ACTUAL yaw value at the moment it started), return
    (yaw, elbow, finished). Same signature/behavior as the original
    hook_animation_test.hook_animation_frame, including the target_yaw
    override for a future voice-cued integration - see that file's
    docstring for the full reasoning, unchanged here."""
    if start_yaw is None:
        start_yaw = STRIKE_START_YAW
    if target_yaw is None:
        target_yaw = STRIKE_END_YAW

    distance = abs(target_yaw - start_yaw)
    strike_duration_ms = max(distance / STRIKE_SPEED_DEG_PER_SEC * 1000.0, MIN_STRIKE_DURATION_MS)

    if elapsed_ms < strike_duration_ms:
        t = elapsed_ms / strike_duration_ms
        yaw = start_yaw + (target_yaw - start_yaw) * ease_in_cubic(t)
        return yaw, FIXED_ELBOW, False

    elapsed_ms -= strike_duration_ms
    if elapsed_ms < HOLD_DURATION_MS:
        return target_yaw, FIXED_ELBOW, False

    return target_yaw, FIXED_ELBOW, True


def play_hook_animation(ser):
    """Blocking version for the standalone test only - loops calling
    hook_animation_frame() and sending until it reports finished."""
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
    time.sleep(1)  # settle at the Hook reference position before the first strike

    print("Press ENTER to play the hook animation, 'q' + ENTER to quit.")
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "q":
            break
        play_hook_animation(ser)

    ser.close()


if __name__ == "__main__":
    main()
