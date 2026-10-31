"""
Boxing Robot Arm (270-degree build) - Punch Angle Test
----------------------------------------------------------
Generalizes hook_animation_270.py's single hook into the full 4-punch
family (Uppercut/High Overhand/Overhand/Hook) - the 270-degree equivalent
of the original punch_angle_test.py. Standalone, keyboard-testable, BEFORE
wiring into anything live.

Each punch's ready position (pan, tilt, yaw, elbow) is pulled directly
from positions_270.POSITIONS - real, hand-jogged values captured on this
build, not formula-derived. Unlike the original 180-degree family (which
fixed pan/yaw-start/elbow and only varied tilt), this build's 4 punches
have genuinely different pan/tilt/yaw/elbow combinations each - UPPERCUT
in particular has its own yaw (90) and elbow (30), not shared with the
other three (yaw=70, elbow=55).

Two phases per punch, same idea as the original:
  1. reposition: pan/tilt/elbow ease to the punch's own ready values, yaw
     eases back to that punch's own ready yaw - "get into position," not
     the strike itself.
  2. strike: once in position, yaw sweeps fast from the ready yaw down to
     STRIKE_END_YAW (same untested-but-reasoned value as
     hook_animation_270.py - 30; confirm that's actually safe for EACH of
     these 4 before assuming it, since Uppercut/High Overhand/Overhand
     start from a different ready yaw than Hook and swing a different
     distance) - pan/tilt/elbow stay fixed at the punch's ready values
     throughout.

Run (from anywhere):   python punch_angle_270.py
Throw a punch:          type 1/2/3/4 then Enter
Reset to ready:         type 'r' then Enter
Quit:                   type 'q' then Enter
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

# key -> (label, (pan, tilt, yaw, elbow)) - pulled directly from
# positions_270.POSITIONS, not re-derived, so this file can't drift out of
# sync with the single source of truth there.
PUNCHES = {
    "1": ("Uppercut", POSITIONS["UPPERCUT"]),
    "2": ("High Overhand", POSITIONS["HIGH_OVERHAND"]),
    "3": ("Overhand", POSITIONS["OVERHAND"]),
    "4": ("Hook", POSITIONS["HOOK"]),
}

READY_PAN, READY_TILT, READY_YAW, READY_ELBOW = POSITIONS["HOOK"]  # default/reset position

# Standalone-test-only: strikes start from here instead of each punch's own
# (narrow) ready yaw, to simulate what a real detection would actually see
# - in live use, the strike starts from wherever your real arm was when a
# fast hook-shaped motion got detected, which is naturally much wider than
# any fixed neutral pose (punch_animation_frame() already takes start_pos
# as a real argument for exactly this reason - this constant only affects
# the keyboard test's go_ready()/throw(), not that function). 180 matches
# REST_GUARD/HAND_OUT_SIDE's own yaw - a plausible real pre-punch position,
# not an arbitrary huge number - and gives a much bigger, more visible
# swing (150 degrees for Hook, vs. 40 before) since the narrow version was
# too subtle to see clearly.
STRIKE_START_YAW = 180

STRIKE_END_YAW = 30  # same as hook_animation_270.py - see that file's SAFETY note

# Pan swings DURING the strike too (synced with the yaw sweep, same
# ease_in_cubic curve, same duration) instead of sitting fixed the whole
# time - UPPERCUT ONLY. The other 3 punches don't want this (applying it
# as a blanket constant the first time visibly changed Hook, which was
# wrong) - every key other than "1" gets 0 here, meaning
# punch_animation_frame()'s pan output for them is just tgt_pan, identical
# to before this existed. -30 -> -40 -> -60 (more follow-through each
# round, on request) - UPPERCUT's base pan is 169 (positions_270.POSITIONS),
# so -60 lands at 109, still well inside the 0-SERVO_ANGLE_MAX clamp - confirm
# in THIS file's keyboard test before it goes anywhere near live again.
PAN_SWING_BY_KEY = {
    "1": -60,  # Uppercut only
}

STRIKE_SPEED_DEG_PER_SEC = 450 * 1.25  # 1.25x per request - bigger swing needs to still feel snappy, not slower
MIN_STRIKE_DURATION_MS = 80
HOLD_DURATION_MS = 120

# Per-punch override for HOLD_DURATION_MS - UPPERCUT ONLY, same pattern as
# PAN_SWING_BY_KEY. Longer hold = the strike lands and stays extended a
# beat longer before the animation ends, i.e. a follow-through instead of
# snapping back immediately. Every key other than "1" falls back to
# HOLD_DURATION_MS via .get(key, HOLD_DURATION_MS) - untested guess,
# confirm in THIS file's keyboard test before it goes anywhere near live
# again.
HOLD_DURATION_BY_KEY = {
    "1": 220,  # Uppercut only
}

SEND_INTERVAL_MS = 20
READY_SETTLE_SEC = 0.6  # keyboard test only - time given to reach ready before striking

# Gentler than the strike - repositioning is "get ready," not the hit.
# Used only by punch_animation_frame() below (the live per-frame path);
# the keyboard test's go_ready() just sends the target directly and
# sleeps, since it has no per-frame caller to ease for.
REPOSITION_SPEED_DEG_PER_SEC = 300
MIN_REPOSITION_DURATION_MS = 60

# ============================================================


def ease_in_cubic(t):
    """Slow start, ACCELERATING into the end - same curve proven on the
    original hook animation and hook_animation_270.py."""
    return t ** 3


def ease_in_out_cubic(t):
    """Smooth accelerate-then-decelerate - for the reposition phase only,
    which is "get ready," not the strike."""
    if t < 0.5:
        return 4 * t ** 3
    return 1 - ((-2 * t + 2) ** 3) / 2


def send(ser, pan, tilt, yaw, elbow):
    ser.write(f"{pan},{tilt},{yaw},{elbow}\n".encode())


def go_ready(ser, pan, tilt, yaw, elbow):
    """Blocking, keyboard-test only - send the target position directly
    and sleep. The live per-frame path uses punch_animation_frame()'s
    reposition phase instead, which eases rather than snaps."""
    send(ser, round(pan), round(tilt), round(yaw), round(elbow))
    time.sleep(READY_SETTLE_SEC)


def throw(ser, pan, tilt, elbow, start_yaw, pan_swing_deg=0.0, hold_duration_ms=HOLD_DURATION_MS):
    """Blocking, keyboard-test only - same proven curve as
    hook_animation_270.py's play_hook_animation, PLUS an optional pan
    swing (see PAN_SWING_BY_KEY - 0 for every punch except Uppercut) - this
    duplicates punch_animation_frame()'s strike-phase math rather than
    calling it, since that function also owns the reposition phase
    (already handled by go_ready() here) and takes a full target_pos tuple
    this blocking caller doesn't build. Keep this in sync with
    punch_animation_frame()'s strike phase by hand if that ever changes -
    this was exactly the kind of drift that caused the pan swing to go
    live but not show up here the first time."""
    distance = abs(STRIKE_END_YAW - start_yaw)
    strike_duration_ms = max(distance / STRIKE_SPEED_DEG_PER_SEC * 1000.0, MIN_STRIKE_DURATION_MS)
    start_time = time.monotonic()
    while True:
        elapsed_ms = (time.monotonic() - start_time) * 1000.0
        if elapsed_ms < strike_duration_ms:
            t = ease_in_cubic(elapsed_ms / strike_duration_ms)
            yaw = start_yaw + (STRIKE_END_YAW - start_yaw) * t
            swung_pan = pan + pan_swing_deg * t
            send(ser, round(swung_pan), round(tilt), round(yaw), round(elbow))
        elif elapsed_ms < strike_duration_ms + hold_duration_ms:
            send(ser, round(pan + pan_swing_deg), round(tilt), STRIKE_END_YAW, round(elbow))
        else:
            break
        time.sleep(SEND_INTERVAL_MS / 1000.0)


def punch_animation_frame(elapsed_ms, start_pos, target_pos, target_yaw=None, pan_swing_deg=0.0,
                           hold_duration_ms=HOLD_DURATION_MS):
    """Pure/stateless, for a FUTURE live per-frame integration (not yet
    wired in - see the project's workflow rule: changes to this animation
    get tested here, in this file's keyboard test, BEFORE ever touching
    track_interpolation_270.py again). start_pos/target_pos are each
    (pan, tilt, yaw, elbow) tuples. Two phases:
      1. reposition: pan/tilt/elbow ease from start_pos to target_pos's
         own ready values, yaw eases back to target_pos's own ready yaw.
      2. strike: once in position, yaw sweeps fast from that ready yaw to
         target_yaw (defaults to STRIKE_END_YAW) while tilt/elbow hold at
         target_pos's values; pan holds UNLESS pan_swing_deg is non-zero
         (see PAN_SWING_BY_KEY - 0 for every punch except Uppercut),  in
         which case it eases by that many degrees on the same curve/timing
         as the yaw strike. hold_duration_ms (see HOLD_DURATION_BY_KEY -
         same Uppercut-only pattern) controls how long the struck pose is
         held before the animation reports finished.
    Returns (pan, tilt, yaw, elbow, finished)."""
    start_pan, start_tilt, start_yaw, start_elbow = start_pos
    tgt_pan, tgt_tilt, tgt_yaw, tgt_elbow = target_pos
    if target_yaw is None:
        target_yaw = STRIKE_END_YAW

    reposition_distance = max(
        abs(tgt_pan - start_pan), abs(tgt_tilt - start_tilt),
        abs(tgt_yaw - start_yaw), abs(tgt_elbow - start_elbow),
    )
    reposition_duration_ms = max(
        reposition_distance / REPOSITION_SPEED_DEG_PER_SEC * 1000.0, MIN_REPOSITION_DURATION_MS
    )

    if elapsed_ms < reposition_duration_ms:
        eased = ease_in_out_cubic(elapsed_ms / reposition_duration_ms)
        pan = start_pan + (tgt_pan - start_pan) * eased
        tilt = start_tilt + (tgt_tilt - start_tilt) * eased
        yaw = start_yaw + (tgt_yaw - start_yaw) * eased
        elbow = start_elbow + (tgt_elbow - start_elbow) * eased
        return pan, tilt, yaw, elbow, False

    strike_elapsed_ms = elapsed_ms - reposition_duration_ms
    strike_distance = abs(target_yaw - tgt_yaw)
    strike_duration_ms = max(strike_distance / STRIKE_SPEED_DEG_PER_SEC * 1000.0, MIN_STRIKE_DURATION_MS)

    if strike_elapsed_ms < strike_duration_ms:
        strike_t = ease_in_cubic(strike_elapsed_ms / strike_duration_ms)
        yaw = tgt_yaw + (target_yaw - tgt_yaw) * strike_t
        pan = tgt_pan + pan_swing_deg * strike_t
        return pan, tgt_tilt, yaw, tgt_elbow, False

    struck_pan = tgt_pan + pan_swing_deg
    hold_elapsed_ms = strike_elapsed_ms - strike_duration_ms
    if hold_elapsed_ms < hold_duration_ms:
        return struck_pan, tgt_tilt, target_yaw, tgt_elbow, False

    return struck_pan, tgt_tilt, target_yaw, tgt_elbow, True


def main():
    ser = serial.Serial(track.SERIAL_PORT, track.BAUD_RATE, timeout=1, write_timeout=1)
    time.sleep(2)  # give the Arduino time to reset after the port opens
    print(f"Connected to Arduino on {track.SERIAL_PORT}")

    print("\nLegend:")
    for key, (label, (pan, tilt, yaw, elbow)) in PUNCHES.items():
        print(f"  {key} = {label:<14} (pan={pan} tilt={tilt} yaw={yaw}->{STRIKE_END_YAW} elbow={elbow})")
    print("  r = reset to ready   q = quit\n")

    go_ready(ser, READY_PAN, READY_TILT, READY_YAW, READY_ELBOW)

    while True:
        cmd = input("> ").strip().lower()
        if cmd == "q":
            break
        elif cmd == "r":
            go_ready(ser, READY_PAN, READY_TILT, READY_YAW, READY_ELBOW)
        elif cmd in PUNCHES:
            label, (pan, tilt, _ready_yaw, elbow) = PUNCHES[cmd]
            pan_swing_deg = PAN_SWING_BY_KEY.get(cmd, 0.0)
            hold_duration_ms = HOLD_DURATION_BY_KEY.get(cmd, HOLD_DURATION_MS)
            # Reposition to this punch's own pan/tilt/elbow, but hold yaw
            # at the wide STRIKE_START_YAW instead of the punch's own
            # narrow ready yaw - see that constant's comment for why.
            swing_note = f", pan swings {pan_swing_deg:+.0f}" if pan_swing_deg else ""
            print(f"Throwing {label} (pan={pan} tilt={tilt} yaw={STRIKE_START_YAW}->{STRIKE_END_YAW} elbow={elbow}{swing_note}, hold={hold_duration_ms}ms)...")
            go_ready(ser, pan, tilt, STRIKE_START_YAW, elbow)
            throw(ser, pan, tilt, elbow, STRIKE_START_YAW, pan_swing_deg=pan_swing_deg, hold_duration_ms=hold_duration_ms)
        else:
            print("Unknown command - use 1/2/3/4/r/q")

    ser.close()


if __name__ == "__main__":
    main()
