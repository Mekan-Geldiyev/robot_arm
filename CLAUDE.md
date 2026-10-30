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

Not every "hard stop" turned out to be a pure limitation, though: tilt
passes a mechanical top-dead-center around 90° and folds back down,
bottoming out around tilt=180 - originally logged as just a boundary to
respect, it turned out to be exactly the folded-over position needed for
an uppercut (see `punch_angle_test.py` below). Worth re-examining other
"hard stops" the same way before assuming they're pure downsides.

## Punch classification + hook animation (`interpolation_approach/`)

Layered on top of `track_interpolation.py`'s live mirroring:
`hook_detector.py`/`punch_classifier.py` watch wrist speed + travel (a
reliable, low-ambiguity "did a strike just happen" gate, adapted from a
separate webcam fighting-game project called EFIGHT) plus elbow bend and
yaw/direction to guess punch *type* (hook/uppercut/punch). Real labeled
data (`punch_dataset.py`) showed type-classification from vision alone has
a genuine information limit, not just a tuning gap - a wide, straight-armed
hook is kinematically indistinguishable from an ordinary fast reach.

When a hook is auto-detected (no voice/text command pending - see below),
`hook_animation_test.py`'s scripted, stateless per-frame curve
(`hook_animation_frame(elapsed_ms, start_yaw)`) takes over yaw/elbow only -
pan/tilt keep tracking live - because live-tracking a fast hook was too
noisy (real hook elbow angles sit inside `YAW_ELBOW_FADE`'s transition
band, amplifying noise into big yaw swings). The animation starts from
whatever yaw the arm was actually at, not a hardcoded value.
`punch_classifier.last_punch_at` gets re-armed when the animation
*finishes*, not when the hook was *detected* - otherwise a real arm
retraction that outlasts the cooldown reads as a second hook (the
"double-dip" bug, confirmed fixed via live retest 2026-09-21).

## Punch family + voice/text selection (`interpolation_approach/`)

`punch_angle_test.py` generalizes the single hook animation above into a
4-punch family built from two axes: tilt sets which height/angle a punch
comes from, yaw provides the fast strike sweep (same proven curve as the
hook animation). `PUNCH_TILTS` is the single source of truth - hand-tuned
from real testing, not a formula, and went through two rounds of
correction already (labels swapped once, one tilt moved 95°):
`{"uppercut": 180, "high overhand": 40, "overhand": 45, "hook": 90}`.
Also runnable standalone as a keyboard test (keys 1-4).

`voice_punch.py` lets the user *name* the punch instead of relying on
`punch_classifier.py`'s geometric guess - sidesteps the classification
ambiguity above rather than solving it, since real labeled data confirmed
that ambiguity is a genuine information limit, not a tunable gap. Naming a
punch (voice via Vosk, or typed - `INPUT_MODE`, text is current default
for testing without speaking out loud) arms a `{type, tilt, yaw}` command,
consumed only at the exact moment `punch_classifier`'s speed/travel gate
fires (never every frame, and it expires after `PENDING_EXPIRY_SEC` if
never thrown) - then `track_interpolation.py` plays
`punch_angle_test.punch_animation_frame()` (reposition tilt, then strike)
instead of the classifier's own guess. This is a SEPARATE code path from
the automatic hook-only one above: naming "hook" via voice now drives tilt
to 90 explicitly (full family treatment), while automatic hook detection
with no command pending still leaves tilt live, exactly as before - don't
merge these two paths without re-verifying the automatic one's already-
tuned feel doesn't regress.

## Current direction (as of 2026-09-21)

YC application (and others) close in ~1 month - priority is a demo that's
reliable and impressive on camera, not architectural completeness.

**Software-first sequencing was the right call**: voice/text-cued
selection (above) shipped on the *current* single arm with zero new
hardware, directly fixing the project's biggest reliability weakness
(punch-type classification) before any hardware work started. Other input
methods were considered (a physical button, resistance bands on a pole the
user holds to signal angle) - voice/text was picked as fastest to
prototype, not because the others were rejected outright; worth
reconsidering if voice/text proves unreliable in the actual demo
environment (background noise, etc).

**Hardware upgrade (next)**: not started. 3D-printed arm parts (current
Lego + hot-glue is the load-bearing constraint on both range and
reliability) and wide-range positional servos (see above) to finally lift
`YAW_SAFE_FLOOR` and reach full strike depth on every punch in the family.
A second arm (mirroring the user's other side) is the stretch goal after
that - not a dependency for a working demo. See the README's "Known
limitations & open questions" section for the fuller, reviewer-facing
version of this list.

## The 270-degree rebuild (as of 2026-10-20)

Same repo, deliberately separate files, not a separate build of the
original ones - the 180-degree hardware above stays a clean, working
fallback in case the 270-degree parts/print don't come together in time.
Every 270-specific file has a `_270` suffix and imports/references only
other `_270` files, never touching `track.py`, `track_interpolation.py`,
`calibration_data.json`, or `robot_arm.ino`:
- `robot_arm_270/robot_arm_270.ino` - firmware, 0-270 range (not 180)
- `interpolation_approach/positions_270.py` - the 16-pose reference set
  and real confirmed channel centering for this build (pan=145, tilt=120,
  yaw=70, elbow=135 - not assumed, physically verified)
- `interpolation_approach/hook_animation_270.py` - standalone hook
  animation test, not yet wired into live detection
- `interpolation_approach/track_interpolation_270.py` - the live
  mirroring system for this hardware; loads
  `interpolation_implement/calibration_data_270.json` specifically (not
  `calibration_data.json`)

`calibration_data_270.json` was built by reusing the ORIGINAL 180-degree
build's raw MediaPipe readings (those are about the user's body, unchanged
by the servo swap) paired with freshly hand-jogged servo values - caught
and fixed one real bug this way: `HAND_OUT_CENTER` (reaching forward) and
the T-pose had been silently merged into one entry during the first
rebuild, which is exactly the kind of mistake this pairing approach is
prone to if a raw value gets reused for the wrong named pose - double
check the pose NAME still matches the raw reading's actual real-world
pose, not just that a raw value exists.

`PUNCH_DETECTION_ENABLED` (top of `track_interpolation_270.py`) is
currently `False` - pure live mirroring only, hook/voice-cued detection
fully disabled. Flip it back to `True` to re-enable; nothing else needs
touching. Also added a real hold-time requirement (`HOLD_TIME_SEC=2.0`)
on top of `SNAP_DISTANCE`/`SNAP_MARGIN` - the same point now has to stay
the confident nearest match continuously for 2 full seconds before
locking, not just for one frame, which was causing flicker on jittery
readings near the snap boundary even while holding still.

**Open issue, not yet root-caused**: moving quickly from `HAND_OUT_SIDE`
toward `HAND_DOWN` (a 77-degree tilt swing) shows a real lag - the arm
appears to pause for 2-3 seconds partway through, then catches up to
`HAND_DOWN` 5-6 seconds after the real motion already happened. Checked
the IDW blend path with a straight-line simulation between the two points
and it transitions smoothly (`HAND_OUT_SIDE` -> `HAND_LEFT_DOWN` ->
`HAND_DOWN`, no flat spot, never gets near `SNAP_DISTANCE`) - so probably
NOT the interpolation math itself, though real human motion isn't a
straight line in raw-value space so this isn't fully ruled out. Best
current guess: the raw-signal OneEuroFilter's lag on a large, fast tilt
swing specifically (77 degrees is a lot to track smoothly) - this would
produce exactly this symptom, the output slowly catching up to where the
arm already was well after the real motion stopped, which looks like a
freeze-then-jump from the outside even though nothing is literally frozen.
Next diagnostic step, not yet done: log raw vs. smoothed tilt and the
chosen nearest-neighbor name every frame during a real `HAND_OUT_SIDE` ->
`HAND_DOWN` motion, to see directly which stage is actually lagging
instead of guessing further.

## Wrist IMU punch detector (new chapter, started 2026-10-29)

A GY-521/MPU-6050 taped to the user's left wrist brace, read over I2C on
the SAME Uno as the PCA9685 (shares A4/SDA, A5/SCL - two different I2C
addresses, 0x68 vs 0x40, coexist fine on one bus). This is not a second
Arduino; the plan is to eventually fold its setup()/loop() into
`robot_arm_270.ino`'s own setup()/loop() once it's proven out standalone.
**Why this exists**: the camera/MediaPipe pipeline is too slow/laggy to
catch a fast punch in time - the IMU is meant to be a faster trigger, not
just a more decisive one (an open question, not yet confirmed either way
- current priority is getting type classification right first, see below).

Files, same "keep the simpler known-good file standing" pattern as
`hook_animation_test.py`/`punch_angle_test.py`:
- `imu_punch_test.ino` - first version, spike-only (magnitude above a
  threshold = "PUNCH"), no type classification. Has a known, not-fixed-
  there baseline bug: it subtracts the full calibration vector (gravity
  included) from every accel axis, then subtracts another 1g from the
  magnitude, double-cancelling gravity - resting baseline actually sits
  at ~-1g, not ~0. Didn't block spike detection (real punches are huge
  relative to either baseline) but don't trust "still reads 0" in this
  file literally.
- `imu_punch_classify_test.ino` - current/active file. Adds hook vs
  uppercut vs low-rotation "punch" (straight) classification plus a
  diagnostic swing angle, fixes the baseline bug above (uses the
  calibration vector's measured length, `restMag`, instead of a
  hardcoded 1.0g), and prints `PUNCH,<type>,<angle_deg>,<ms_to_recognize>,
  <verticality>,<peak_gyro_dps>` per punch - CSV, matching this repo's
  existing serial-protocol convention. Type vocabulary ("hook"/
  "uppercut"/"punch") deliberately matches `punch_classifier.py`'s own
  strings for an eventual merge.

**Classifier model**: calibration (holding the wrist still) captures the
raw accel vector as `refUp` - a unit vector meaning "which way is up, in
this sensor's own local axes, as mounted on this wrist, in whatever pose
it was calibrated in." A hook swings through a roughly horizontal arc, so
by the right-hand rule its angular velocity vector points roughly
*parallel* to `refUp`; an uppercut swings vertically, so its angular
velocity points roughly *perpendicular* to `refUp`. `VERTICALITY_THRESHOLD`
and `GYRO_MIN_FOR_ROTATION_DPS` are the tunable cutoffs - still being
tuned as of this writing, not yet confirmed against real throws.

**Calibration pose is now standardized: arm straight out in front
(center), held still through the ~4s calibration window right after
reset/upload.** This was explicitly chosen for repeatability, not boxing
realism - it's easy to return to the exact same arm position every time,
unlike a guard stance where elbow/wrist angle varies subtly run to run.
This matters a lot here: `refUp` is only as good as the pose held during
THAT calibration, and an inconsistent calibration pose was confirmed to
flip which punch type the classifier is biased toward between sessions
(one session biased everything toward "hook," the next toward "uppercut,"
with no code change in between) - calibrate the same way, every time, or
the threshold tuning below won't hold across sessions.

**Root cause found (2026-10-29, user's own diagnostic): forearm ROLL, not
swing plane, was driving the result.** Two rounds of gyro-verticality
(projecting gyro onto refUp, first at the peak-accel instant, then at the
true peak-gyro instant) both failed - every real throw, regardless of
intended type, landed at nearly the same extreme verticality. The user
then tested directly: same uppercut motion, palm up vs. palm down (pure
forearm rotation, not a different swing) - got "hook" one way,
"uppercut" the other. `refUp` was frozen at one-time startup calibration,
so it goes stale the instant the wrist's rest orientation differs from
power-on time - rotating the forearm about its own long axis spins the
sensor's local axes relative to gravity even though the real-world swing
arc is unchanged. This directly mirrors the already-documented camera-
side limitation above ("Yaw is not palm/wrist roll... MediaPipe can't see
forearm twist") - same anatomical DOF, opposite failure mode (the wrist
IMU is too sensitive to roll, where the camera is blind to it). Confirmed
via photo that the physical mounting itself (GY-521 rigidly taped to the
dorsal side of the wrist brace) is fine - purely an algorithm problem.

**Fix**: `refUp` is now continuously adaptive, not a one-shot startup
calibration - while idle (never during a capture window), it slowly
drifts toward the current accel reading each loop tick
(`REFUP_ADAPT_RATE`), so each punch is judged against whatever
orientation the wrist is actually resting in right before THAT throw.
Classification also switched from gyro-verticality to `angleDeg` (accel-
direction vs refUp) after the verticality approach failed twice: a hook's
net strike acceleration should point roughly across the body
(`ANGLE_HOOK_MIN_DEG`, currently 60), an uppercut's roughly up it. Not
yet retested - next step is repeating the user's own palm-up/palm-down
A/B test and checking the now-printed live `refUp` values actually
tracked the change.

**Two-phase plan**: (1) get hook/uppercut/punch type classification
matching real throws first (in progress); (2) separately, build a real
angle-to-servo-degrees mapping from hand-paired (IMU reading, verified
correct robot-arm position) throws, same ground-truth methodology as
`calibration_data_270.json` - the diagnostic angle currently printed is
a comparison feature only, not yet a servo mapping. Not wired to
`punch_classifier.py` or `track_interpolation_270.py` yet.

## Running it

```
pip install -r requirements.txt
python track.py                                        # formula-based, 180-degree
python interpolation_approach/track_interpolation.py      # interpolation-based, 180-degree
python interpolation_approach/track_interpolation_270.py  # interpolation-based, 270-degree build
```

Upload `robot_arm.ino` (180-degree) or `robot_arm_270/robot_arm_270.ino`
(270-degree) to the Arduino depending on which hardware is actually
plugged in - uploading the wrong one for the physically-connected servos
is a real, easy-to-make mistake, not just a config error. `SERIAL_PORT` in
`track.py` (shared by everything above) needs to match the Arduino's
actual COM port either way.
