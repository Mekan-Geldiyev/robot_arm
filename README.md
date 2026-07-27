# Boxing Robot Arm — Real-Time Motion Mirroring

A webcam tracks your left arm with MediaPipe Pose; a Lego + MG996R servo
arm mirrors it in real time. Long-term goal: throw jabs, hooks, and
uppercuts by copying your punches.

## Hardware

- Elegoo Uno R3 (Arduino clone), USB to PC, port `COM3` (may vary)
- PCA9685 16-channel PWM servo driver
- 2x MG996R servos (pan + tilt), 3rd elbow servo coming later
- Breadboard as the Arduino <-> PCA9685 wiring hub
- 5V 4A power supply -> barrel jack -> screw terminal -> PCA9685 `V+`
- Shared ground: Arduino `GND` -> PCA9685 `GND` screw terminal
- Aluminum pan-tilt bracket, mounted **sideways** (rotated 90° from a
  normal camera-mount orientation) so the two servos give shoulder-like
  range for jab / hook / uppercut motion
- Lego upper arm hot-glued to the tilt servo horn, forearm + hand below it

### Wiring diagram

```
                 5V 4A PSU
                     |
              barrel jack -> screw terminal
                     |
        +-----------------------------+
        |      PCA9685 (V+ screw)     |
        |                              |
        |  V+   GND   SDA  SCL  VCC   |
        +---+-----+----+----+----+----+
            |     |    |    |    |
            |     |    |    |    |
            |     |    |    |    +---- 5V  --------+
            |     |    |    +--------- A5 (SCL) ----+
            |     |    +-------------- A4 (SDA) ----+   Elegoo Uno R3
            |     +------------------- GND ----------+   (breadboard hub)
            |                                        |
            +---- shared ground rail ----------------+

  PCA9685 channel 0 (PAN)  -> aluminum bracket PAN servo  (horizontal axis)
  PCA9685 channel 1 (TILT) -> aluminum bracket TILT servo (vertical axis)
  PCA9685 channel 2 (ELBOW)-> not installed yet, placeholder in code
```

- Channel 0 = **PAN** — horizontal rotation axis — jab (forward/back
  sweep) and hook (side sweep)
- Channel 1 = **TILT** — vertical rotation axis — uppercut (arm raise)
- Channel 2 = **ELBOW** — not installed; code has a placeholder ready

## Software

- `robot_arm.ino` — Arduino sketch. Reads `"pan,tilt\n"` (or
  `"pan,tilt,elbow\n"` later) over serial, smooths motion toward the
  target angle instead of snapping, and drives the PCA9685.
- `track.py` — Python webcam tracker. Uses OpenCV + MediaPipe Pose to
  read your left shoulder/elbow/wrist/hip, computes pan/tilt angles,
  and streams them to the Arduino over serial.

### MediaPipe landmarks used

`track.py` flips the webcam frame horizontally for a natural mirror view
before running pose detection. MediaPipe's left/right labeling is
appearance-based (learned from mostly non-flipped training images), so on
a flipped feed its labels come out swapped - its `RIGHT_*` indices are
what actually correspond to the user's true left arm here.

| Body part (your real left arm) | MediaPipe index |
|---------------------------------|------------------|
| Shoulder                        | 12 (`RIGHT_SHOULDER`) |
| Elbow                           | 14 (`RIGHT_ELBOW`)    |
| Wrist                           | 16 (`RIGHT_WRIST`)    |
| Hip                              | 24 (`RIGHT_HIP`)      |

- **Pan** = shoulder horizontal angle: hip -> shoulder -> elbow,
  projected onto the horizontal (x/z) plane
- **Tilt** = shoulder elevation angle: how far the upper arm is raised
  above/below horizontal
- **Elbow** (placeholder) = shoulder -> elbow -> wrist flex angle

## Install

```
pip install -r requirements.txt
```

Also install the **Adafruit PWM Servo Driver Library** in the Arduino
IDE (Library Manager -> search "Adafruit PWM Servo Driver").

> Note: recent `mediapipe` versions (0.10.30+) dropped the old
> `mp.solutions.pose` API in favor of the newer Tasks API
> (`mp.tasks.vision.PoseLandmarker`), which `track.py` uses. It needs a
> small model file (`pose_landmarker_lite.task`, ~5.5 MB), which
> `track.py` downloads automatically next to itself the first time you
> run it — no manual step needed, just make sure you have an internet
> connection on first launch.

## Running it

1. Plug in the Arduino, upload `robot_arm.ino` from the Arduino IDE.
2. Open `track.py` and check `SERIAL_PORT` at the top matches your
   Arduino's COM port (Device Manager on Windows, or the Arduino IDE's
   Port menu).
3. Run:
   ```
   python track.py
   ```
4. Stand in front of the webcam so your left shoulder, elbow, and hip
   are visible. The servo arm should mirror your arm movement.
5. Press `q` in the video window to quit.

## Tuning

All tunable values live at the **top of each file** so you don't need
to dig through the code:

**`track.py`**
- `SERIAL_PORT`, `BAUD_RATE` — serial connection
- `DEADBAND_DEG` — minimum angle change (degrees) before a new value is
  sent; reduces jitter
- `PAN_MIN/MAX`, `TILT_MIN/MAX`, `ELBOW_MIN/MAX` — raw MediaPipe angle
  ranges seen during a full range-of-motion test
- `PAN_SERVO_MIN/MAX`, `TILT_SERVO_MIN/MAX`, `ELBOW_SERVO_MIN/MAX` —
  resulting servo angle ranges (0-180)
- `INVERT_PAN`, `INVERT_TILT`, `INVERT_ELBOW` — flip direction if the
  arm moves the wrong way
- `SEND_ELBOW` — set `True` once the channel 2 servo is installed

**`robot_arm.ino`**
- `SERVOMIN` / `SERVOMAX` — PWM pulse range (currently 150 / 600)
- `PWM_FREQ` — PCA9685 frequency (60 Hz)
- `PAN_CHANNEL` / `TILT_CHANNEL` / `ELBOW_CHANNEL` — PCA9685 channel
  numbers
- `STEP_SIZE` / `STEP_DELAY_MS` — smoothing speed (bigger step or
  shorter delay = faster but less smooth motion)

### Calibration workflow

1. Run `track.py` and watch the `raw: pan=... tilt=... elbow=...`
   values printed to the terminal while moving your arm through a jab,
   hook, and uppercut.
2. Note the min/max raw values for each and set `PAN_MIN/MAX`,
   `TILT_MIN/MAX` (and `ELBOW_MIN/MAX` later) accordingly.
3. If the servo moves the opposite direction from your arm, flip the
   matching `INVERT_*` flag.

<a id="calibration-reference-values"></a>
### 📌 Calibration reference values (measured 2026-07-27)

Ground-truth pose -> servo command pairs, found by moving the physical arm
by hand into each pose and finding the `pan,tilt` command via Serial
Monitor that reproduces it. Used to fit `PAN_MIN/MAX` and `TILT_MIN/MAX` in
`track.py`. **If you recalibrate, redo this table** - the raw numbers only
mean what they mean for the current pan/tilt angle formulas (shoulder ->
wrist).

| Pose | Serial Monitor command (pan,tilt) | Live raw reading (pan,tilt) | Reliable? |
|---|---|---|---|
| Hand down at side (resting) | `180,0` | `178, -84` | Yes |
| Hand out to the side (horizontal) | `0,90` | `133, -16` | Yes |
| Hand out to the front (facing camera) | `90,0` | `160, -9` | **No** - see note below |

> **Why "hand out to the front" is unreliable:** that motion is mostly
> toward/away from the camera (depth/z-axis), which a single monocular
> webcam can't resolve nearly as well as side-to-side motion. Its raw
> numbers don't fit the same line as the other two poses and were
> deliberately *not* used to fit `PAN_MIN/MAX`/`TILT_MIN/MAX`. If this pose
> needs to work well, it likely needs its own handling (e.g. leaning more
> on `z` depth data) rather than just wider MIN/MAX ranges.

## Status / roadmap

- [x] Hardware wired and tested
- [x] Both servos confirmed moving via tester sketch
- [x] Pan-tilt bracket assembled (sideways orientation)
- [x] Lego arm partially built and attached
- [ ] Full motion tracking system (this repo)
- [ ] Elbow servo (channel 2) installed and enabled
- [ ] Second arm (right side)
- [ ] Full body tracking
