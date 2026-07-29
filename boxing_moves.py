"""
Boxing Robot Arm - Canned Punch Demo
-------------------------------------
Standalone, camera-free demo: sends pre-programmed pan/tilt/elbow
sequences over serial to throw a jab, hook, or uppercut on command. No
MediaPipe/webcam involved - this is for showing the hardware off
reliably while gesture detection (matching a real jab/hook/uppercut from
the camera) is worked on separately later.

Run:   python boxing_moves.py
Then type a command and press Enter:
    j = jab       h = hook      u = uppercut
    r = rest/guard position
    q = quit

Install dependencies:
    pip install pyserial
"""

import time

import serial

# ==================== TUNABLE SETTINGS ====================

SERIAL_PORT = "COM3"     # <-- change this to match your Arduino's port
BAUD_RATE = 9600         # must match Serial.begin() in robot_arm.ino

# How long a full-range move takes to settle, based on robot_arm.ino's
# smoothing speed (STEP_SIZE degrees every STEP_DELAY_MS). If you change
# STEP_SIZE/STEP_DELAY_MS there, update this too so the waits below still
# give the arm enough time to actually arrive before the next move.
DEGREES_PER_SEC = (2 / 15) * 1000

# --- Named servo positions (pan, tilt, elbow) ---
# REST/JAB_OUT/HOOK_OUT use the confirmed-safe values from the README
# "Calibration Reference Values" table. UPPER_OUT is a first-pass style
# choice (no dedicated uppercut ground truth exists) - all of these are
# just a starting point, retune freely to change how a punch looks.
REST      = (180, 0, 90)   # hand hanging straight at your side
JAB_OUT   = (90, 0, 90)    # arm straight out to the front
HOOK_OUT  = (0, 90, 30)    # arm out to the side, elbow bent for a hook shape
UPPER_OUT = (90, 60, 20)   # partway up + tight elbow bend, snapping up

HOLD_SECONDS = 0.15        # how long to hold the extended position before retracting

# ============================================================


def wait_for(pos_from, pos_to):
    """Rough seconds for the Arduino's smoothing to reach pos_to, based on
    the largest single-servo travel distance."""
    travel = max(abs(a - b) for a, b in zip(pos_from, pos_to))
    return travel / DEGREES_PER_SEC + 0.05  # small buffer for serial/latency


def send(ser, pos):
    pan, tilt, elbow = pos
    ser.write(f"{pan},{tilt},{elbow}\n".encode())


def move_to(ser, current, target):
    send(ser, target)
    time.sleep(wait_for(current, target))
    return target


def throw(ser, current, extended):
    current = move_to(ser, current, extended)
    time.sleep(HOLD_SECONDS)
    current = move_to(ser, current, REST)
    return current


def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, write_timeout=1)
    time.sleep(2)  # give the Arduino time to reset after the port opens
    print(f"Connected to Arduino on {SERIAL_PORT}")

    current = move_to(ser, REST, REST)

    print("j = jab   h = hook   u = uppercut   r = rest   q = quit")
    while True:
        cmd = input("> ").strip().lower()
        if cmd == "j":
            current = throw(ser, current, JAB_OUT)
        elif cmd == "h":
            current = throw(ser, current, HOOK_OUT)
        elif cmd == "u":
            current = throw(ser, current, UPPER_OUT)
        elif cmd == "r":
            current = move_to(ser, current, REST)
        elif cmd == "q":
            break
        else:
            print("Unknown command - use j/h/u/r/q")

    ser.close()


if __name__ == "__main__":
    main()
