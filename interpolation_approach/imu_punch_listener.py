"""
Boxing Robot Arm - Wrist IMU Punch Listener
------------------------------------------------
Reads the "PUNCH,<type>,..." lines the wrist IMU classifier (merged into
robot_arm_270.ino - see CLAUDE.md's "Wrist IMU punch detector" section)
prints over serial, and exposes the most recent one as a pending punch
command. track_interpolation_270.py pops this EVERY frame (not just when
its own vision-based gate also fires) and, if present, lets it trigger
the punch animation on its own - that's the whole point of this chapter:
the wrist IMU reacts in the same ~20ms tick it senses the spike, while
the vision gate needs a real multi-frame window
(punch_classifier.HOOK_WINDOW_SEC=0.12s, plus camera/MediaPipe inference
time on top) before it can fire at all. The IMU is meant to beat it there.

Reads off the SAME serial connection track_interpolation_270.py already
has open for sending "pan,tilt,yaw,elbow" commands - this is no longer a
second Arduino or a second port, the IMU logic was merged into
robot_arm_270.ino itself. One background thread owns all reading from
that connection; the main loop keeps writing to it - the standard safe
pattern for a duplex serial link (one reader thread, one writer thread,
no lock needed since they only ever touch their own direction).

Only the CONFIRMED-GOOD binary classifier (hook/uppercut/punch) is used
here - the continuous signed-angle experiment was tried and shelved (see
CLAUDE.md), nothing from that line of work is wired in.
"""

import threading
import time

import serial

# Only "hook" maps to a different canned position (PUNCHES["4"]) - both
# "uppercut" and the low-rotation "punch" fallback map to PUNCHES["1"],
# same reasoning track_interpolation_270.py's own vision-based fallback
# already uses (no separate jab/cross animation exists yet).
TYPE_TO_KEY = {"hook": "4", "uppercut": "1", "punch": "1"}

# A pending punch expires if the main loop doesn't consume it within this
# long - mirrors voice_punch_270.PENDING_EXPIRY_SEC's reasoning, though in
# practice this should be consumed within one frame (a fraction of a
# second), since the main loop checks every frame, not just when its own
# gate fires. This is just a safety net against a genuinely stuck/stale
# read.
PENDING_EXPIRY_SEC = 2.0


class IMUPunchListener:
    """Background-thread reader for 'PUNCH,<type>,...' lines arriving on
    an ALREADY-OPEN serial connection. start()/stop() control the thread;
    pop_next_punch() is the only method the main loop needs."""

    def __init__(self, ser: serial.Serial):
        self._ser = ser
        self._lock = threading.Lock()
        self._pending = None  # {"type", "key", "armed_at"} or None
        self._thread = None
        self._running = False

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def pop_next_punch(self):
        """Returns {"type", "key"} and clears it, or None if nothing's
        pending or it's gone stale - consumed at most once, so the same
        real punch can't accidentally fire two animations."""
        with self._lock:
            pending = self._pending
            self._pending = None
        if pending is None:
            return None
        if time.monotonic() - pending["armed_at"] > PENDING_EXPIRY_SEC:
            return None
        return {"type": pending["type"], "key": pending["key"]}

    def _listen_loop(self):
        while self._running:
            try:
                raw = self._ser.readline()
            except serial.SerialException:
                break  # port closed/disconnected - stop quietly, main loop already handles that case
            if not raw:
                continue  # read timeout with no data - normal, just loop again
            line = raw.decode(errors="replace").strip()
            if not line.startswith("PUNCH,"):
                continue  # servo-command echoes ("name -> value"), calibration lines, etc. - not for us
            fields = line.split(",")
            if len(fields) < 2:
                continue
            punch_type = fields[1]
            key = TYPE_TO_KEY.get(punch_type)
            if key is None:
                continue  # unrecognized type string - ignore rather than guess
            with self._lock:
                self._pending = {"type": punch_type, "key": key, "armed_at": time.monotonic()}
            print(f"[imu] punch detected: {punch_type}")
