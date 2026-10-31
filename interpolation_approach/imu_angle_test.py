"""
Boxing Robot Arm - Wrist IMU Angle Mapping, Live Dry-Run
------------------------------------------------------------
Standalone test for imu_angle_map.py - reads the real PUNCH,... lines
printed by imu_punch_classify_test.ino over serial and prints what
(pan, tilt, yaw, elbow) target that throw would map to, side by side with
the raw type/angle. Throw real hooks and uppercuts with this running and
watch whether the mapped target looks right (a wide, obvious hook should
land close to POSITIONS["HOOK"]; a light hook near the ANGLE_HOOK_MIN_DEG
boundary should land somewhere between HOOK and UPPERCUT, not snap all
the way to either).

DRY RUN ONLY - no serial write happens here, by design, not as a
TODO. The Arduino connected during this test is running
imu_punch_classify_test.ino, which only ever reads the IMU and prints -
it has no command parser listening for "pan,tilt,yaw,elbow" the way
robot_arm_270.ino does, so sending anything here would do nothing useful.
This is the same "test standalone and get it confirmed BEFORE wiring into
the live system" rule this project always uses for punch-animation
changes (see punch_angle_270.py, imu_punch_classify_test.ino itself) -
only once this mapping is confirmed does it make sense to fold IMU
reading into robot_arm_270.ino's own setup()/loop() alongside the
PCA9685 code, so the mapped target can actually be sent somewhere.

Run (from anywhere):   python imu_angle_test.py
Quit:                   Ctrl+C
"""

import os
import sys

import serial

_PARENT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

import track  # noqa: E402
import imu_angle_map  # noqa: E402

# ==================== TUNABLE SETTINGS ====================

# Straight "punch" (jab/cross) has no ground-truth anchor at all - see
# imu_angle_map.py's docstring for why. Printed as a clear note instead of
# silently skipped, so a real jab thrown during testing doesn't look like
# this script just missed it.
UNMAPPED_TYPES = {"punch"}

# ============================================================


def parse_punch_line(line):
    """Returns (punch_type, angle_deg) from a raw "PUNCH,<type>,<angleDeg>,..."
    serial line, or None if the line isn't a PUNCH line or fails to parse
    (a torn/partial line at the moment the port opens is normal, not an
    error worth stopping for)."""
    if not line.startswith("PUNCH,"):
        return None
    fields = line.split(",")
    if len(fields) < 3:
        return None
    try:
        return fields[1], float(fields[2])
    except ValueError:
        return None


def main():
    ser = serial.Serial(track.SERIAL_PORT, track.BAUD_RATE, timeout=1)
    print(f"Connected to Arduino on {track.SERIAL_PORT}")
    print("Dry run only - nothing is sent back to the Arduino. Throw real "
          "hooks/uppercuts and watch the mapped target below. Ctrl+C to quit.\n")

    try:
        while True:
            raw = ser.readline().decode(errors="replace").strip()
            if not raw:
                continue

            parsed = parse_punch_line(raw)
            if parsed is None:
                continue  # non-PUNCH line (calibration/debug output) - ignore

            punch_type, angle_deg = parsed
            if punch_type in UNMAPPED_TYPES:
                print(f"type={punch_type:<8} angle={angle_deg:6.1f}  -> no mapping (straight punch, no anchor exists)")
                continue

            pan, tilt, yaw, elbow = imu_angle_map.angle_to_target(angle_deg)
            print(f"type={punch_type:<8} angle={angle_deg:6.1f}  -> "
                  f"pan={pan:6.1f} tilt={tilt:6.1f} yaw={yaw:6.1f} elbow={elbow:6.1f}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
