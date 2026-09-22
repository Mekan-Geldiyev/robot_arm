# Boxing Robot Arm

Teleoperated robot arm: a webcam watches the user's real left arm via MediaPipe
Pose, and a servo arm mirrors it in real time. Built as a demo for a YC
application / robotics portfolio - it needs to look controlled on camera, not
janky, so tracking-quality bugs are treated as first-order problems.

## Hardware

Elegoo Uno R3 + PCA9685 (I2C PWM driver) + 4x MG996R servos, arm structure
built from Lego (fragile - has broken twice from wire snags/mishandling).

PCA9685 channels: `0=pan  1=tilt  2=yaw  3=elbow`. Yaw is a 3rd shoulder DOF
added after the original pan/tilt-only build, hand-glued to the outside of
the pan/tilt bracket's metal U.

**Critical mechanical quirk - pan is coupled to roll.** The pan/tilt bracket
is mounted sideways (rotated 90° from a normal camera-mount orientation) to
fake shoulder-like range from just 2 servos. Because of this, and because
pan is the base joint with tilt/yaw/elbow/forearm all mounted rigidly on top
of it, **sweeping pan doesn't just point the arm left/right - it rolls the
entire downstream assembly with it**, flipping the hand's orientation past a
certain angle. This is a hardware design issue, not a software bug - a true
fix needs pan's axis to pass through the same point tilt/yaw do (ball-joint
style), not a calibration tweak. `track.py`'s `PAN_LOCKED`/`PAN_LOCK_VALUE`
is a stopgap (locks pan to a safe value, at the cost of losing all
front-vs-side aim distinction, since pan is the only channel that encodes
that). Don't "fix" the flip by touching pan's calibration constants - it
won't work.

**Yaw is not palm/wrist roll.** MediaPipe's body-only landmarks can never see
forearm twist (no hand/finger tracking), regardless of camera count. `yaw`
is a proxy signal instead: how far the elbow swings out of the straight-
hanging plane (`yaw_angle()` in `track.py`) - real and useful for
distinguishing a hook from a jab, but a genuinely different physical
quantity from "which way is the palm facing." Don't expect it to solve palm-
orientation complaints; that's an unmeasurable input, not a tuning problem.

`YAW_SAFE_FLOOR` (track.py) exists because the hand-glued yaw horn
mechanically strains below a certain servo value - never remove this without
confirming the mount has been rebuilt with real range. `YAW_SERVO_OFFSET`
corrects for the horn's unknown physical mounting angle (measured via
Serial Monitor ground truth, not computed).

## Two parallel control approaches

- **`track.py`** - per-channel formula + hand-tuned calibration constants
  (`PAN_MIN/MAX`, `TILT_MIN/MAX`, `YAW_MIN/MAX`, `ELBOW_MIN/MAX`, each with
  an `INVERT_*` flag). Elbow uses 2-link IK (law of cosines) instead of
  reading the elbow landmark's angle directly - the elbow landmark is
  frequently occluded/noisy, so only shoulder+wrist are needed per frame.
- **`interpolation_approach/track_interpolation.py`** - alternative approach:
  loads hand-verified `(raw MediaPipe reading, correct servo output)` pairs
  from `interpolation_implement/calibration_data.json` and does inverse-
  distance-weighted nearest-neighbor blending instead of per-channel
  formulas. Purely additive - imports `track.py` for shared geometry/filters
  rather than duplicating it, so switching back is just running `track.py`
  directly. Has its own `SNAP_DISTANCE` (use the single nearest calibration
  point directly when close enough, instead of blending) and
  `LOW_CONFIDENCE_DISTANCE` (flags when even the nearest calibration point is
  far away - a signal for "this pose needs a new calibration entry," not a
  servo-output change).

Both are real, working approaches - not one deprecated in favor of the
other. Check which one is actively being tested before assuming which
file's constants matter.

## Calibration methodology

`robot_arm.ino` supports three serial input formats:
- `pan,tilt,yaw,elbow` - all 4 targets at once (what both Python scripts send)
- `name:value` (e.g. `yaw:90`) - jogs a single channel via Serial Monitor,
  leaving the other 3 exactly where they were
- `print` - dumps the current actual angle of all 4 channels, comma-separated

To calibrate a new pose: hold the real pose in front of the camera and read
the live `raw: pan=... tilt=... yaw=... elbow=...` terminal line (the
MediaPipe-derived reading), *separately* verify the correct servo output by
jogging channels with `name:value` until it visually matches, then `print`
to grab that servo tuple. Both halves must come from the same real moment -
several `calibration_data.json` entries got corrupted by pairing a raw
reading with a servo value someone typed based on what a pose "should" look
like rather than a verified live match.

**Known monocular limitation, not a bug:** raw elbow readings are
unreliable whenever the wrist ends up close to the shoulder/head in real 3D
space (a bent-elbow pose with the hand near the face, or an arm hanging
close to the torso) - MediaPipe's z-depth is least reliable exactly when the
true depth difference is small. Re-recording more carefully will not fix
this; it's a sensing limit. Several `calibration_data.json` entries have a
`note` field documenting exactly this when it applies - read those before
assuming a bad-looking calibration point is a capture mistake.

Some calibrated poses (e.g. a proper boxing guard) currently can't be
reached at all because pan/tilt hit a mechanical hard-stop before the true
position - a hardware range limitation, not something fixable by
recalibrating. The fix is wide-range **positional** servos (e.g. 270-300°,
still holding an exact commanded angle via a pot/magnetic encoder) - NOT
continuous-rotation "360°" servos, which are geared motors with
speed/direction control and no angle feedback at all. Every script in this
repo commands and holds absolute angles; a true continuous-rotation servo
can't do that and would break the entire control model.

## Punch classification + hook animation (`interpolation_approach/`)

Layered on top of `track_interpolation.py`'s live mirroring:
`hook_detector.py`/`punch_classifier.py` watch wrist speed + travel (a
reliable, low-ambiguity "did a strike just happen" gate, adapted from a
separate webcam fighting-game project called EFIGHT) plus elbow bend and
yaw/direction to guess punch *type* (hook/uppercut/punch). Real labeled
data (`punch_dataset.py`) showed type-classification from vision alone has
a genuine information limit, not just a tuning gap - a wide, straight-armed
hook is kinematically indistinguishable from an ordinary fast reach.

When a hook is detected, `hook_animation_test.py`'s scripted, stateless
per-frame curve (`hook_animation_frame(elapsed_ms, start_yaw)`) takes over
yaw/elbow only - pan/tilt keep tracking live - because live-tracking a fast
hook was too noisy (real hook elbow angles sit inside `YAW_ELBOW_FADE`'s
transition band, amplifying noise into big yaw swings). The animation
starts from whatever yaw the arm was actually at, not a hardcoded value.
`punch_classifier.last_punch_at` gets re-armed when the animation
*finishes*, not when the hook was *detected* - otherwise a real arm
retraction that outlasts the cooldown reads as a second hook (the
"double-dip" bug).

## Current direction (as of 2026-09-21)

YC application (and others) close in ~1 month - priority is a demo that's
reliable and impressive on camera, not architectural completeness.

**Voice-cued punch selection** (planned, see README): the type-
classification ambiguity above is being sidestepped rather than solved -
the user declares the punch type + angle out loud before throwing
(`next_punch` state), the existing speed/travel gate just confirms *that* a
strike happened, then a generalized version of the hook animation (any
type/angle, not just the one hardcoded hook depth) plays. Other input
methods were considered (a physical button, resistance bands on a pole the
user holds to signal angle) - voice was picked for now as fastest to
prototype, not because the others were rejected outright.

**Decided sequencing**: build the voice-cued software on the *current*
single arm first (needs no new hardware, days not weeks, fixes the
project's biggest reliability weakness) before starting the hardware
upgrade below. Explicitly avoid starting both in parallel - two half-
finished arms is worse than one that's solid.

**Hardware upgrade (next, after voice-cued software works)**: 3D-printed
arm parts (current Lego + hot-glue is the load-bearing constraint on both
range and reliability) and wide-range positional servos (see above) to
finally reach guard/uppercut angles. A second arm (mirroring the user's
other side) is the stretch goal after that - not a dependency for a
working demo.

## Running it

```
pip install -r requirements.txt
python track.py                                    # formula-based approach
python interpolation_approach/track_interpolation.py  # interpolation-based approach
```

Upload `robot_arm.ino` to the Arduino first. `SERIAL_PORT` in `track.py`
(shared by both scripts) needs to match the Arduino's actual COM port.
