# DEMO VIDEO

LINK TO DEMO: [
https://www.dropbox.com/scl/fi/22eq9liee4u3cuzz4cdll/demovid2.mp4?rlkey=ak88vl7npl2zlx96b5431a55y&st=5my26a4f&dl=0
<img width="1336" height="868" alt="image" src="https://github.com/user-attachments/assets/35d313b4-ae53-4bc4-81ac-a3bf6648b8b2" />




# CONSTRAINTS
Everything is built out of lego, a 3d printer would be super helpful to be able to build custom parts, and also parts that are a little bit stronger with a heavier filament.




# Boxing Robot Arm — Real-Time Motion Mirroring

A webcam tracks your arm with MediaPipe Pose; a Lego + MG996R servo arm
mirrors it in real time and can throw jabs, hooks, uppercuts, and
overhands - either by watching you throw one, or by you naming which one's
coming next. Built as a demo for a YC application / robotics portfolio -
it needs to look controlled and intentional on camera, not janky, so
tracking-quality and reliability bugs are treated as first-order problems,
not polish.

**If you're a fresh AI/reviewer reading this to give feedback**: the
["Known limitations & open questions"](#known-limitations--open-questions)
section near the bottom is written specifically for you - it's an honest
list of what's unsolved, what was deliberately deprioritized, and where
the real remaining risk is, rather than just what's built.

## Hardware

- Elegoo Uno R3 (Arduino clone), USB to PC, port `COM3` (may vary)
- PCA9685 16-channel PWM servo driver
- 4x MG996R servos: pan + tilt (shoulder gimbal), yaw (3rd shoulder DOF,
  added 2026-08-12), elbow
- Breadboard as the Arduino <-> PCA9685 wiring hub
- 5V 4A power supply -> barrel jack -> screw terminal -> PCA9685 `V+`
- Shared ground: Arduino `GND` -> PCA9685 `GND` screw terminal
- Aluminum pan-tilt bracket, mounted **sideways** (rotated 90° from a
  normal camera-mount orientation) so the two servos give shoulder-like
  range for jab / hook / uppercut motion
- Yaw servo glued to the **outside** of the pan/tilt bracket's metal U
  (didn't fit inside) - rotates the whole shoulder assembly around the
  upper-arm axis, which is what actually lets the elbow's hinge plane
  swing out to the side for a hook
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

  PCA9685 channel 0 (PAN)   -> aluminum bracket PAN servo  (horizontal axis)
  PCA9685 channel 1 (TILT)  -> aluminum bracket TILT servo (vertical axis)
  PCA9685 channel 2 (YAW)   -> shoulder YAW servo (glued to outside of bracket)
  PCA9685 channel 3 (ELBOW) -> elbow servo
```

- Channel 0 = **PAN** — horizontal rotation axis — jab (forward/back
  sweep) and hook (side sweep)
- Channel 1 = **TILT** — vertical rotation axis — which "height/angle" a
  punch comes from; also the axis that reaches the uppercut position (see
  the punch family below)
- Channel 2 = **YAW** — rotates the shoulder around the upper-arm axis —
  provides the actual fast strike sweep for every punch in that family
- Channel 3 = **ELBOW** — elbow flex

## Software

Two parallel control approaches exist side by side, not one deprecated in
favor of the other - check which one you're actually running before
assuming which file's constants matter.

**`track.py`** - the original approach: per-channel formulas + hand-tuned
calibration constants (`PAN_MIN/MAX`, `TILT_MIN/MAX`, etc., each with an
`INVERT_*` flag). Elbow uses 2-link inverse kinematics (law of cosines)
from the shoulder<->wrist distance instead of reading the elbow landmark's
angle directly - the elbow landmark is frequently occluded/noisy (e.g.
tucked in front of the torso during a hook), so only shoulder+wrist are
needed per live frame; the elbow landmark is only used, heavily smoothed,
to calibrate the two arm-segment lengths.

**`interpolation_approach/`** - a newer, more capable alternative and
where all of the punch-throwing/voice work below actually lives:

- `track_interpolation.py` — the main entry point. Instead of per-channel
  formulas, it loads hand-verified `(raw MediaPipe reading, correct servo
  output)` pairs from `interpolation_implement/calibration_data.json` and
  maps live input to servo output via inverse-distance-weighted
  interpolation (the closer your current pose is to a calibrated one, the
  more its known-good servo values count) - the result can never
  extrapolate past an already-confirmed-safe value, unlike a formula fit.
  Purely additive on top of `track.py` (imports its geometry/filters
  rather than duplicating them); switching back to the older approach is
  just running `track.py` directly.
- `punch_classifier.py` + `hook_detector.py` — watch wrist speed/travel (a
  reliable "did a strike just happen" gate, adapted from a separate
  webcam fighting-game project called EFIGHT) plus elbow bend and
  yaw/direction to guess punch *type* (hook / uppercut / unclassified
  "punch") from vision alone. Real labeled data (`punch_dataset.py`)
  showed this has a genuine information limit, not just a tuning gap - see
  [Known limitations](#known-limitations--open-questions).
- `hook_animation_test.py` — a scripted, stateless per-frame curve
  (`hook_animation_frame`) that takes over yaw+elbow only (pan/tilt stay
  live) when the classifier auto-detects a hook, because live-tracking a
  fast hook was too noisy to look right (real hook elbow angles sit inside
  a fade band that amplifies sensor noise into big yaw swings - see
  `track.YAW_ELBOW_FADE_START/END`). The strike starts from wherever yaw
  actually was, not a fixed guard pose, and duration scales with actual
  distance so it never feels sluggish or rushed.
- `punch_angle_test.py` — generalizes the hook idea into a 4-punch
  **family** built from two axes: tilt sets which height/angle a punch
  comes from, yaw provides the fast strike sweep. Hand-tuned from real
  testing (not a formula) into `PUNCH_TILTS`, the single source of truth
  every other file imports from:

  | Punch | Tilt | Feel |
  |---|---|---|
  | Uppercut | 180 | arm folded fully over (mechanical top-dead-center past 90°, folds back down) |
  | High Overhand | 40 | most overhand |
  | Overhand | 45 | |
  | Hook | 90 | most horizontal, "normal hook" |

  Also runnable standalone as a keyboard test (no camera) - `python
  punch_angle_test.py`, keys `1`/`2`/`3`/`4` throw the 4 punches, `r`
  resets to ready, `q` quits.
- `voice_punch.py` — say (or type) a punch name before throwing it, and
  the system uses that instead of guessing type from vision. See
  ["Implemented: voice/text-cued punch selection"](#implemented-voicetext-cued-punch-selection)
  below for the full design and why.
- `capture_point.py` — the tool that built `calibration_data.json`: bursts
  10 samples per pose, flags high-variance (unstable) captures
  automatically, counts down out loud in the terminal so you know when
  it's about to grab a sample.
- `punch_dataset.py` — labeled dataset collector for validating/tuning
  `punch_classifier.py` against real thrown punches instead of guessed
  thresholds (`h`/`u`/`n` to label hook/uppercut/none, logs what the
  classifier guessed alongside the true label).
- `boxing_moves.py` — standalone, camera-free demo predating the punch
  family above: sends hardcoded jab/hook/uppercut waypoint sequences on
  keypress (`j`/`h`/`u`/`r`/`q`). Kept for reliably showing the arm off
  without any camera/detection in the loop.
- `circle_test/circle_test.ino` — standalone Arduino sketch, no serial/
  Python involved. Traces a continuous pan/tilt circle to sanity-check
  the hardware (PCA9685/servos/wiring/power) in isolation.
- `robot_arm.ino` — Arduino sketch shared by everything above. Reads
  `"pan,tilt,yaw,elbow\n"` over serial, smooths motion toward the target
  angle instead of snapping, and drives the PCA9685.

### MediaPipe landmarks used

Both `track.py` and `track_interpolation.py` flip the webcam frame
horizontally for a natural mirror view before running pose detection.
MediaPipe's left/right labeling is appearance-based (learned from mostly
non-flipped training images), so on a flipped feed its labels come out
swapped - its `RIGHT_*` indices are what actually correspond to the user's
true left arm here.

| Body part (your real left arm, normally) | MediaPipe index |
|---------------------------------|------------------|
| Shoulder                        | 12 (`RIGHT_SHOULDER`) |
| Elbow                           | 14 (`RIGHT_ELBOW`)    |
| Wrist                           | 16 (`RIGHT_WRIST`)    |
| Hip                              | 24 (`RIGHT_HIP`)      |

"Normally" because MediaPipe's left/right label is a per-frame guess, not a
fixed fact - on a confusing pose it can flip which side it calls which,
which looked like the robot arm suddenly mirroring the user's *right* arm
instead of their left. Neither script trusts index 12/14/16/24 blindly
every frame: each also computes the alternate candidate (11/13/15/23,
`LEFT_*`) and picks whichever side's **shoulder** sits further toward the
frame-left edge (`SIDE_A`/`SIDE_B` + `SIDE_SWITCH_MARGIN`). Shoulder
position, not wrist position, on purpose - wrists are supposed to cross the
body's midline during a hook/cross, so tracking by wrist proximity could
latch onto the wrong arm at exactly the moment (a real punch) it mattered
most; shoulders essentially never swap left-right order during normal
front-facing motion. Watch the `[side A/B]` tag printed to the terminal
(and overlaid on the video) - it should read `A` almost all the time; if
it's flickering to `B` during normal motion, `SIDE_SWITCH_MARGIN` may need
to be raised. `track.py` also crops the frame's right edge
(`CROP_RIGHT_FRAC`) before pose detection so the user's real *right* arm
(e.g. holding a phone up to film) is never visible to MediaPipe at all,
not just filtered out after the fact.

- **Pan** = shoulder horizontal angle: hip -> shoulder -> wrist,
  projected onto the horizontal (x/z) plane
- **Tilt** = shoulder elevation angle: how far the upper arm is raised
  above/below horizontal
- **Yaw** = rotation of the shoulder-elbow-wrist plane around the
  shoulder->wrist axis, measured against shoulder->hip as the 0-degree
  reference (elbow hanging toward the body, like a guard/jab). This is
  **not** palm/wrist roll - MediaPipe's body-only landmarks structurally
  cannot see forearm twist, on any monocular pose model. It's a genuinely
  different, weaker signal that approximates the same intent: how far the
  elbow's hinge plane has swung out to the side. Explicitly faded toward
  zero as the elbow straightens (`YAW_ELBOW_FADE_START/END`), since a
  straight arm carries almost no plane-rotation signal at all - that's an
  information floor, not a smoothing bug. Needs the elbow landmark visible
  every frame (no slow fallback like elbow IK below), so it holds its last
  servo value when the elbow is occluded.
- **Elbow** = 2-link IK (law of cosines) from the shoulder<->wrist distance
  and slowly-averaged upper-arm/forearm lengths, NOT a direct read of the
  elbow landmark's angle (see `track.py`'s section above for why).

## Install

```
pip install -r requirements.txt
```

Also install the **Adafruit PWM Servo Driver Library** in the Arduino
IDE (Library Manager -> search "Adafruit PWM Servo Driver").

> Note: recent `mediapipe` versions (0.10.30+) dropped the old
> `mp.solutions.pose` API in favor of the newer Tasks API
> (`mp.tasks.vision.PoseLandmarker`), which this project uses. It needs a
> small model file (`pose_landmarker_lite.task`, ~5.5 MB), downloaded
> automatically next to itself the first time you run it — no manual step
> needed, just make sure you have an internet connection on first launch.

> Note: `mediapipe`'s own package init imports `matplotlib` transitively
> (for a 3D-plot debug helper this project never calls), which Windows
> **Smart App Control** can block as an unrecognized DLL, crashing on
> `import mediapipe` before any of this project's own code even runs. Both
> `track.py` and `track_interpolation.py` work around this by stubbing
> `matplotlib`/`matplotlib.pyplot` in `sys.modules` before mediapipe ever
> gets a chance to import the real ones - see the comment at the top of
> either file if this ever needs revisiting (e.g. mediapipe starts using
> matplotlib for something this project actually needs).

> Note: voice/text-cued punch selection additionally needs `vosk` and
> `PyAudio` (in `requirements.txt` already). `voice_punch.py` reuses an
> already-downloaded ~1.8GB Vosk speech model from a separate local
> project (`BackTPal`) if present, to avoid a redundant download - falls
> back to downloading its own copy into `interpolation_approach/` if that
> path doesn't exist (e.g. on a different machine).

## Running it

**Formula-based approach:**
1. Plug in the Arduino, upload `robot_arm.ino` from the Arduino IDE.
2. Open `track.py` and check `SERIAL_PORT` at the top matches your
   Arduino's COM port (Device Manager on Windows, or the Arduino IDE's
   Port menu).
3. Run `python track.py`.
4. Stand in front of the webcam so your left shoulder, elbow, and hip
   are visible. The servo arm should mirror your arm movement.
5. Press `q` in the video window to quit.

**Interpolation approach (more capable - punch classification, hook
animation, voice/text-cued punches):**
```
python interpolation_approach/track_interpolation.py
```
Same `SERIAL_PORT` setup as above (shared from `track.py`). A `>` prompt
will also appear in the terminal for typed punch commands (see below) -
type a punch name and Enter any time, or throw a real hook and let the
classifier catch it automatically.

## Tuning

All tunable values live at the **top of each file** so you don't need
to dig through the code:

**`track.py`**
- `SERIAL_PORT`, `BAUD_RATE` — serial connection
- `DEADBAND_DEG` — minimum angle change (degrees) before a new value is
  sent; reduces jitter. Applied per-channel (pan/tilt/yaw/elbow each
  gated independently), so one channel moving doesn't drag the others along
- `PAN_MIN_CUTOFF/BETA`, `TILT_MIN_CUTOFF/BETA`, `YAW_MIN_CUTOFF/BETA`,
  `ELBOW_MIN_CUTOFF/BETA`, `D_CUTOFF` — One Euro Filter params applied to
  raw angles before mapping/sending. `MIN_CUTOFF` controls smoothing when
  nearly still (lower = smoother but slower to react), `BETA` controls how
  fast filtering loosens up as speed increases (higher = snappier during a
  real punch, but more jitter can leak through while moving)
- `PAN_MIN/MAX`, `TILT_MIN/MAX`, `YAW_MIN/MAX`, `ELBOW_MIN/MAX` — raw
  MediaPipe angle ranges seen during a full range-of-motion test
- `ELBOW_LENGTH_ALPHA` — how slowly the IK's arm-segment-length estimate
  updates from the elbow landmark (0.01 = very slow/stable on purpose)
- `PAN_SERVO_MIN/MAX`, `TILT_SERVO_MIN/MAX`, `YAW_SERVO_MIN/MAX`,
  `ELBOW_SERVO_MIN/MAX` — resulting servo angle ranges (0-180)
- `INVERT_PAN`, `INVERT_TILT`, `INVERT_YAW`, `INVERT_ELBOW` — flip
  direction if the arm moves the wrong way
- `YAW_SAFE_FLOOR` — the hand-glued yaw horn mechanically strains below
  this value; never lower it without confirming a rebuilt mount handles
  real range first
- `CROP_RIGHT_FRAC` — crops the frame's right edge before pose detection
  so the user's real right arm is never visible to MediaPipe at all

**`interpolation_approach/track_interpolation.py`**
- `NEIGHBOR_COUNT` — how many nearest calibration points to blend (IDW)
- `IDW_POWER` — inverse-distance weighting exponent; lower spreads weight
  more evenly across neighbors, damping jumps when the nearest-neighbor
  set changes
- `SNAP_DISTANCE` / `SNAP_MARGIN` — use a single calibration point
  directly (skip blending/smoothing) when close enough AND clearly closer
  than the 2nd-nearest point
- `OUTPUT_MIN_CUTOFF` / `OUTPUT_BETA` — second-stage OneEuroFilter on the
  *interpolated servo output*, separate from `track.py`'s raw-signal
  filters - smooths the discontinuity that happens when the nearest-
  neighbor set changes, which raw-signal smoothing can't reach

**`interpolation_approach/punch_classifier.py`**
- `ELBOW_BENT_THRESHOLD` — elbow must be bent below this to ever be called
  a hook/uppercut; found necessary from real negative-example data (see
  Known limitations)
- `DIRECTION_DOMINANCE_RATIO` — how much one axis of wrist motion must
  dominate the other before calling the direction confidently
  horizontal/vertical

**`interpolation_approach/punch_angle_test.py`**
- `PUNCH_TILTS` — the 4-punch family's tilt values, single source of truth
  (voice_punch.py imports this directly)
- `YAW_END_REQUESTED` / `YAW_END` — requested vs. actually-clamped strike
  depth (see `track.YAW_SAFE_FLOOR`)
- `STRIKE_SPEED_DEG_PER_SEC`, `REPOSITION_SPEED_DEG_PER_SEC` — feel of the
  strike vs. the "get into position" phase before it

**`interpolation_approach/voice_punch.py`**
- `INPUT_MODE` — `"voice"` (microphone) or `"text"` (type a line + Enter);
  text mode exists for testing without speaking out loud
- `PENDING_EXPIRY_SEC` — an armed command expires if never thrown within
  this long, so a forgotten command can't misfire on a later, unrelated
  motion

**`robot_arm.ino`**
- `SERVOMIN` / `SERVOMAX` — PWM pulse range (currently 150 / 600)
- `PWM_FREQ` — PCA9685 frequency (60 Hz)
- `PAN_CHANNEL` / `TILT_CHANNEL` / `YAW_CHANNEL` / `ELBOW_CHANNEL` —
  PCA9685 channel numbers
- `STEP_SIZE` / `FAST_STEP_SIZE` / `FAST_JUMP_THRESHOLD` / `STEP_DELAY_MS`
  — smoothing speed; small corrections use `STEP_SIZE`, jumps past
  `FAST_JUMP_THRESHOLD` use the faster `FAST_STEP_SIZE`

### Calibration workflow (`track.py`)

1. Run `track.py` and watch the `raw: pan=... tilt=... yaw=... elbow=...`
   values printed to the terminal while moving your arm through a jab,
   hook, and uppercut.
2. Note the min/max raw values for each and set `PAN_MIN/MAX`,
   `TILT_MIN/MAX`, `YAW_MIN/MAX`, and `ELBOW_MIN/MAX` accordingly.
3. If the servo moves the opposite direction from your arm, flip the
   matching `INVERT_*` flag.

### Calibration workflow (`interpolation_approach/`)

1. Hold a real pose in front of the camera, run `capture_point.py` -
   it bursts 10 samples, flags high variance, and counts down out loud
   before grabbing one.
2. Separately verify the correct servo output by jogging channels via
   Serial Monitor (`robot_arm.ino` supports `name:value`, e.g. `yaw:90`,
   leaving the other 3 channels untouched) until it visually matches the
   pose, then send bare `print` to dump the current servo tuple.
3. Add the `(raw, servo)` pair to `calibration_data.json`. Both halves
   must come from the same real moment - several entries got corrupted
   early on by pairing a raw reading with a servo value someone typed
   based on what a pose "should" look like rather than a verified live
   match.

**Known monocular limitation, not a bug:** raw elbow readings are
unreliable whenever the wrist ends up close to the shoulder/head in real
3D space - MediaPipe's z-depth is least reliable exactly when the true
depth difference is small. Several `calibration_data.json` entries have a
`note` field documenting this when it applies.

<a id="calibration-reference-values"></a>
### 📌 Calibration reference values (measured 2026-07-27, `track.py`)

Ground-truth pose -> servo command pairs, found by moving the physical arm
by hand into each pose and finding the `pan,tilt` command via Serial
Monitor that reproduces it. **If you recalibrate, redo this table** - the
raw numbers only mean what they mean for the current pan/tilt angle
formulas (shoulder -> wrist).

| Pose | Serial Monitor command (pan,tilt) | Live raw reading (pan,tilt) | Reliable? |
|---|---|---|---|
| Hand down at side (resting) | `180,0` | `178, -84` | Yes |
| Hand out to the side (horizontal) | `0,90` | `133, -16` | Yes |
| Hand out to the front (facing camera) | `90,0` | `160, -9` | **No** - see note below |
| Elbow bent to 90 degrees | elbow=`0` | elbow raw=`58` | Yes |
| Elbow fully straight (hanging at side) | elbow=`90` | elbow raw=`68` | Yes |

> Elbow servo range is deliberately capped at 90 (not 180): a real elbow
> doesn't hyperextend, so 90 is the natural full range end, not just a
> mechanical safety margin like tilt's cap.

> **Confirmed: tilt servo~90 is a real mechanical peak, not just a
> cautious cap.** With pan held at 0, sweeping tilt past 90 makes the
> physical arm reverse and swing back down, bottoming out fully around
> tilt=180 - the bracket's linkage passes over a mechanical top-dead-center
> around 90 and folds the other way. This turned out to be a *feature*
> once discovered, not just a limit - `punch_angle_test.py`'s Uppercut
> deliberately uses tilt=180, the folded-over position, since that's
> exactly what's in position for yaw to swing into an uppercut shape.

> **Why "hand out to the front" is unreliable:** that motion is mostly
> toward/away from the camera (depth/z-axis), which a single monocular
> webcam can't resolve nearly as well as side-to-side motion. Its raw
> numbers don't fit the same line as the other two poses and were
> deliberately *not* used to fit `PAN_MIN/MAX`/`TILT_MIN/MAX`.

## Implemented: voice/text-cued punch selection

The hardest unsolved problem in the automatic system isn't detecting
*that* a fast strike happened - the speed/travel gate in
`punch_classifier.py` already does that reliably. It's classifying *what
kind* of punch it was from vision alone: a wide, straight-armed hook and
an ordinary fast reach are genuinely ambiguous in the signals available to
a single webcam (see `ELBOW_BENT_THRESHOLD`'s notes in
`punch_classifier.py`) - real labeled data confirmed this isn't a tunable
gap, it's an information limit on what a monocular camera can distinguish.

The approach: stop trying to infer punch type from vision, and just ask
the user. Name the punch before throwing it - e.g. "hook", or "high
overhand" - which arms a `{type, tilt, yaw}` command (`voice_punch.py`).
The camera keeps mirroring live exactly as it does otherwise. When the
existing speed/travel gate detects a real strike, instead of guessing the
type from ambiguous geometry, it plays `punch_angle_test.py`'s scripted
animation for whichever punch was named - tilt repositions, then yaw
strikes, then holds, then hands back to live tracking. To a viewer it
should still look and feel like the arm is reading the punch live - it
just also happens to already know what's coming.

Two ways to name the punch, controlled by `voice_punch.INPUT_MODE`:
- **Voice** (`"voice"`) - fully offline speech recognition (Vosk), no
  network dependency or per-request latency during a live demo.
- **Text** (`"text"`, current default) - type the name + Enter in the
  terminal. Exists for testing without speaking out loud (e.g. late at
  night); functionally identical otherwise - both expose the exact same
  pending-command interface, so nothing downstream cares which is active.

Saying/typing just the name is enough - each punch already has its own
pre-tuned tilt+yaw target (`punch_angle_test.PUNCH_TILTS`). An optional
trailing number overrides just that throw's yaw depth (e.g. "hook forty
five"), without changing which tilt it uses.

This only affects punches you explicitly name. If nothing's pending when
the classifier detects a hook on its own, the original automatic path
still runs exactly as before (yaw/elbow scripted, tilt/pan stay live) -
the voice-cued path is additive, not a replacement for the tuned,
already-verified automatic hook detection.

<a id="known-limitations--open-questions"></a>
## Known limitations & open questions

Written for a reviewer (human or AI) who wasn't in the room for any of
this - what's genuinely unsolved, what was a deliberate tradeoff under
time pressure (a YC-adjacent application deadline), and where more
scrutiny would actually be useful, rather than just what's built.

- **Pan/roll mechanical coupling, unsolved.** Pan is the base joint with
  tilt/yaw/elbow all mounted rigidly on top of it; because the shoulder
  bracket is mounted sideways (a hack to get shoulder-like range from just
  2 servos), sweeping pan doesn't just point the arm left/right - it rolls
  the entire downstream assembly, flipping the hand's orientation past a
  certain angle. This is a hardware design issue, not a software bug - a
  real fix needs pan's axis to pass through the same point tilt/yaw do
  (ball-joint style), which is out of scope for the current build.
  `track.PAN_LOCKED`/`PAN_LOCK_VALUE` is the stopgap (locks pan to a safe
  value, at the cost of losing front-vs-side aim distinction).
- **Automatic punch-type classification has a real ceiling, not just an
  undertuned threshold.** A wide/looping hook thrown with a straighter arm
  is kinematically indistinguishable from an ordinary fast reach in the
  data this system can observe - confirmed via a real labeled dataset
  (`punch_dataset.py`), not assumed. The voice/text-cued path exists
  specifically because this ceiling is real; it's a deliberate pivot away
  from the problem, not a claim that the classifier is fully solved.
  Resolving this "for real" would need a different signal entirely (wrist
  acceleration profile, or a wearable IMU) - worth a second opinion on
  whether that's worth pursuing given the payoff already achieved by
  voice/text input.
- **Automatic detection only covers hook.** Uppercut/overhand/high-
  overhand currently require voice or text input - there's no automatic
  geometric detection wired up for them (deliberately not attempted yet;
  see the note in `punch_classifier.py`/`CLAUDE.md` about not repeating
  the same unvalidated-guess mistake already made and corrected once for
  hook). Whether that's worth building, versus leaning further into
  voice/text as the primary interface, is an open product question, not
  just a technical one.
- **`punch_angle_test.py`'s 4-punch family is still being hand-tuned.**
  The tilt values (Uppercut=180, High Overhand=40, Overhand=45, Hook=90)
  went through two rounds of correction already from real testing (labels
  were swapped once, one tilt value moved by 95°) - worth assuming these
  will keep moving as more punches get thrown, not treating them as
  settled.
- **No real profiled latency numbers.** The response-time breakdown that
  exists is a theoretical budget calculated from configured constants
  (detection window, strike speed, serial send interval), not measured
  wall-clock timing - and there's a real, currently-unexplained gap
  between that calculated total and what the system feels like live
  (almost certainly camera capture + MediaPipe inference time, never
  instrumented). Real profiling - stage-boundary timestamps, percentiles
  not just averages, and awareness that `ser.write()` returning isn't the
  same as the Arduino receiving it - would replace a guess with a number.
- **The two control approaches (`track.py` vs. `interpolation_approach/`)
  have never been benchmarked against each other for latency**, only
  compared subjectively for tracking quality/smoothness. Interpolation
  does more per-frame work (a distance calculation against every
  calibration point), which is probably negligible next to camera/
  inference time, but "probably" isn't "measured."
- **Yaw's mechanical range is more limited than the software wants it to
  be.** `track.YAW_SAFE_FLOOR` (20) exists because the hand-glued yaw horn
  mechanically strains below it - every punch's real "depth" is bounded by
  this, not by the animation code. The extended-range positional servo
  upgrade (see Status/roadmap) is what actually lifts this, not further
  software tuning.
- **Hardware upgrade (extended-range servos, 3D-printed parts, second arm)
  is planned but not started** - see Status/roadmap below. The whole
  Lego + hot-glue build is a real constraint on both range of motion and
  reliability, independent of anything in software.

## Status / roadmap

- [x] Hardware wired and tested
- [x] Both servos confirmed moving via tester sketch
- [x] Pan-tilt bracket assembled (sideways orientation)
- [x] Lego arm partially built and attached
- [x] Full motion tracking system (this repo)
- [x] Elbow servo (channel 3) installed and enabled
- [x] Yaw servo (channel 2, 3rd shoulder DOF) installed and enabled
- [x] Hook/uppercut/punch classification + scripted hook-strike animation
      (`interpolation_approach/`)
- [x] 4-punch family (Uppercut/High Overhand/Overhand/Hook) built from
      tilt+yaw combinations, keyboard-testable (`punch_angle_test.py`)
- [x] Voice/text-cued punch selection (`voice_punch.py`) - name a punch
      before throwing it instead of relying on automatic classification
- [ ] Extended-range servos to reach the full intended range (mechanical
      floor currently caps real strike depth) - look for wide-range
      **positional** servos (e.g. 270-300°, still holding an exact
      commanded angle), NOT continuous-rotation "360°" servos. A true
      continuous-rotation servo is a geared motor with speed/direction
      control and no angle feedback at all - it can't hold a position,
      which would break the whole angle-based control model this codebase
      is built on
- [ ] 3D-printed arm parts (stronger, custom-fit - the current Lego +
      hot-glue construction is the load-bearing constraint on both range
      of motion and reliability)
- [ ] Second arm (mirroring the user's other arm - the current build
      already covers left)
- [ ] Automatic geometric detection for uppercut/overhand (open question -
      see Known limitations; may not be worth building given voice/text
      already covers this)
- [ ] Real latency instrumentation (see Known limitations)
- [ ] Full body tracking
