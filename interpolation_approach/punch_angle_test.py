"""
Boxing Robot Arm - Punch Angle Test
--------------------------------------
Keyboard-controlled exploration of a punch FAMILY built from two axes
instead of one: tilt sets which height/angle the strike comes from, yaw
provides the actual fast strike sweep - the same yaw-sweep mechanism
already proven in hook_animation_test.py, just replayed at different tilt
positions instead of a single fixed one (tilt=90 there).

Mechanical basis (from real hands-on testing): tilt=180 puts the arm in
the "folded over" position - see the README's tilt calibration notes,
tilt passes a mechanical top-dead-center around 90 and folds the other
way, bottoming out at 180 - which is what's actually in position for yaw
to swing into an uppercut shape. The motors can't do a true mirror-image
"inverse uppercut", hence the one-directional tilt range below rather than
a symmetric one. tilt=45 is the low end of what's usable here - not a
guess: calibration_data.json's HAND_LEFT_UP (tilt=40) and
HAND_UP_LEFT_DIAGONAL (tilt=50) already use nearby values safely, so 45
sits inside already-verified territory, not past it. tilt=180 is likewise
already a real calibration value (HAND_DOWN).

Tilt values are hand-tuned per punch from real testing, not an evenly-
spaced formula - went through two rounds of corrections already (first
pass had 3/4 swapped; second pass had "high overhand" at tilt=135, which
read as a body uppercut instead of a real overhand). Net rule that came
out of that testing: lower tilt (closer to 0) reads as more overhand,
tilt=90 reads as a normal hook, tilt=180 (arm folded fully over) is the
uppercut position:
    1 = Uppercut       (tilt=180, straight vertical rise + strike)
    2 = High Overhand  (tilt=40, most overhand)
    3 = Overhand       (tilt=45)
    4 = Hook           (tilt=90, most horizontal)
Rename/retune freely once you've seen how each actually looks - these
labels are a working guess, not a verified boxing taxonomy. Single source
of truth for these 4 is PUNCH_TILTS below - voice_punch.py imports it
directly so the keyboard legend and voice commands can't drift apart.

SAFETY: yaw's real strike target is clamped to track.YAW_SAFE_FLOOR (20),
NOT the requested 0. That floor exists because the hand-glued yaw horn
mechanically strains below it - confirmed standalone at tilt=90 for the
existing hook animation, but NOT separately confirmed at tilt=180/90/45/40
here. Every previous floor change in this project was only loosened after
that kind of standalone confirmation, never assumed (see CLAUDE.md) -
watch each of the 4 keys for straining before ever lowering this to
actually reach 0.

Run (from anywhere):   python punch_angle_test.py
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
from hook_animation_test import ease_in_cubic, ease_in_out_cubic  # noqa: E402 - reuse the proven curves

# ==================== TUNABLE SETTINGS ====================

FIXED_PAN = 0
FIXED_ELBOW = 0        # fully bent for every punch in this family - same
                        # "compact fist, less moment of inertia" reasoning
                        # already established for the hook animation

TILT_READY = 180        # default/reset ready position - also Uppercut's own tilt

YAW_START = 180         # cocked/ready position, the strike sweeps FROM here
YAW_END_REQUESTED = 0   # what was asked for
YAW_END = max(YAW_END_REQUESTED, track.YAW_SAFE_FLOOR)  # actually clamped target - see SAFETY above

STRIKE_SPEED_DEG_PER_SEC = 450   # same feel as the proven hook animation
MIN_STRIKE_DURATION_MS = 80
HOLD_DURATION_MS = 120
SEND_INTERVAL_MS = 20
READY_SETTLE_SEC = 0.6   # time given for tilt/yaw to reach the ready position before striking (keyboard test only)

# Gentler than the strike itself - repositioning tilt into place is "get
# ready," not the hit. Used only by punch_animation_frame() below (the
# live per-frame path); the keyboard test's go_ready() just sends the
# target directly and sleeps, since it has no per-frame caller to ease for.
REPOSITION_SPEED_DEG_PER_SEC = 300
MIN_REPOSITION_DURATION_MS = 60

# type name -> tilt. Single source of truth for which punches exist and
# what tilt each one uses - voice_punch.py imports this directly rather
# than keeping its own parallel list, so the keyboard legend, voice
# commands, and text-input mode can never drift out of sync with each
# other. Hand-tuned from real testing, not a formula: lower tilt (closer
# to 0) reads as more overhand, tilt=90 reads as a normal hook, tilt=180
# (arm folded fully over - see the mechanical note above) is the uppercut
# position. "high overhand" and "overhand" ended up close together (40 vs
# 45) because that's where the real motion actually changed character, not
# because they were spaced by formula.
PUNCH_TILTS = {
    "uppercut": TILT_READY,
    "high overhand": 40,
    "overhand": 45,
    "hook": 90,
}

# key -> (label, tilt) for the keyboard legend/dispatch only - derived from
# PUNCH_TILTS above so the two can't disagree with each other.
PUNCHES = {
    "1": ("Uppercut", PUNCH_TILTS["uppercut"]),
    "2": ("High Overhand", PUNCH_TILTS["high overhand"]),
    "3": ("Overhand", PUNCH_TILTS["overhand"]),
    "4": ("Hook", PUNCH_TILTS["hook"]),
}

# ============================================================


def punch_animation_frame(elapsed_ms, start_tilt, target_tilt, start_yaw, target_yaw=None):
    """Pure/stateless, for the LIVE per-frame integration (see
    track_interpolation.py) - the non-blocking equivalent of
    go_ready()+throw() below. Two phases:
      1. reposition: tilt eases to target_tilt while yaw eases back to
         YAW_START at the same time - this is "get into position for this
         punch," not the strike itself.
      2. strike: once in position, yaw sweeps fast from YAW_START to
         target_yaw (defaults to YAW_END) - same proven curve as
         hook_animation_test.hook_animation_frame - while tilt holds at
         target_tilt.
    Returns (tilt, yaw, elbow, finished)."""
    if target_yaw is None:
        target_yaw = YAW_END

    reposition_distance = max(abs(target_tilt - start_tilt), abs(YAW_START - start_yaw))
    reposition_duration_ms = max(
        reposition_distance / REPOSITION_SPEED_DEG_PER_SEC * 1000.0, MIN_REPOSITION_DURATION_MS
    )

    if elapsed_ms < reposition_duration_ms:
        eased = ease_in_out_cubic(elapsed_ms / reposition_duration_ms)
        tilt = start_tilt + (target_tilt - start_tilt) * eased
        yaw = start_yaw + (YAW_START - start_yaw) * eased
        return tilt, yaw, FIXED_ELBOW, False

    strike_elapsed_ms = elapsed_ms - reposition_duration_ms
    strike_distance = abs(target_yaw - YAW_START)
    strike_duration_ms = max(strike_distance / STRIKE_SPEED_DEG_PER_SEC * 1000.0, MIN_STRIKE_DURATION_MS)

    if strike_elapsed_ms < strike_duration_ms:
        yaw = YAW_START + (target_yaw - YAW_START) * ease_in_cubic(strike_elapsed_ms / strike_duration_ms)
        return target_tilt, yaw, FIXED_ELBOW, False

    hold_elapsed_ms = strike_elapsed_ms - strike_duration_ms
    if hold_elapsed_ms < HOLD_DURATION_MS:
        return target_tilt, target_yaw, FIXED_ELBOW, False

    return target_tilt, target_yaw, FIXED_ELBOW, True


def send(ser, pan, tilt, yaw, elbow):
    ser.write(f"{pan},{tilt},{yaw},{elbow}\n".encode())


def go_ready(ser, tilt):
    """Position tilt + yaw for the next strike (yaw back to YAW_START) -
    not itself a fast committed motion, just getting into position."""
    send(ser, FIXED_PAN, round(tilt), YAW_START, FIXED_ELBOW)
    time.sleep(READY_SETTLE_SEC)


def throw(ser, tilt):
    """Same proven curve as hook_animation_test.hook_animation_frame,
    applied to yaw only - tilt is already in position from go_ready()."""
    distance = abs(YAW_END - YAW_START)
    strike_duration_ms = max(distance / STRIKE_SPEED_DEG_PER_SEC * 1000.0, MIN_STRIKE_DURATION_MS)
    start_time = time.monotonic()
    while True:
        elapsed_ms = (time.monotonic() - start_time) * 1000.0
        if elapsed_ms < strike_duration_ms:
            t = elapsed_ms / strike_duration_ms
            yaw = YAW_START + (YAW_END - YAW_START) * ease_in_cubic(t)
            send(ser, FIXED_PAN, round(tilt), int(round(yaw)), FIXED_ELBOW)
        elif elapsed_ms < strike_duration_ms + HOLD_DURATION_MS:
            send(ser, FIXED_PAN, round(tilt), YAW_END, FIXED_ELBOW)
        else:
            break
        time.sleep(SEND_INTERVAL_MS / 1000.0)


def main():
    ser = serial.Serial(track.SERIAL_PORT, track.BAUD_RATE, timeout=1, write_timeout=1)
    time.sleep(2)  # give the Arduino time to reset after the port opens
    print(f"Connected to Arduino on {track.SERIAL_PORT}")

    if YAW_END != YAW_END_REQUESTED:
        print(f"NOTE: yaw target clamped to {YAW_END} (track.YAW_SAFE_FLOOR), "
              f"not the requested {YAW_END_REQUESTED} - see this file's SAFETY comment.")

    print("\nLegend:")
    for key, (label, tilt) in PUNCHES.items():
        print(f"  {key} = {label:<14} (tilt={tilt:.0f})")
    print("  r = reset to ready   q = quit\n")

    go_ready(ser, TILT_READY)

    while True:
        cmd = input("> ").strip().lower()
        if cmd == "q":
            break
        elif cmd == "r":
            go_ready(ser, TILT_READY)
        elif cmd in PUNCHES:
            label, tilt = PUNCHES[cmd]
            print(f"Throwing {label} (tilt={tilt:.0f}, yaw {YAW_START} -> {YAW_END})...")
            go_ready(ser, tilt)
            throw(ser, tilt)
        else:
            print("Unknown command - use 1/2/3/4/r/q")

    ser.close()


if __name__ == "__main__":
    main()
