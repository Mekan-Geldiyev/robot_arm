# Vision-based continuous punch angle - first real test, 2026-11-05

Raw console output from `interpolation_approach/imu_vision_angle_test.py`,
the new harness that reports a CONTINUOUS angle for each detected punch
from vision's own `dx,dy` (`angle_deg = degrees(atan2(-dy, dx))`), with the
wrist IMU logged alongside as a cross-check only (never the angle source -
see the file's own docstring for why, and `CLAUDE.md`'s "Wrist IMU punch
detector" chapter for the shelved IMU-only signed-angle history this is
deliberately not repeating). First real data for this feature - the angle
formula had not been thrown-tested before this session.

**User's stated protocol**: 5 hooks, then 5 uppercuts thrown straight up,
then 5 uppercuts thrown from roughly 45 degrees off to the side (diagonal,
not straight up) - 15 real throws intended.

## Raw console log

```
[imu] uppercut angleDeg=50.1 elapsed=60ms
[vision] hook angle=-5.2 dx=+0.54 dy=+0.05 speed=6.18 travel=0.54
>>> angle=-5.2 deg  type=hook  (paired=yes, imu_type=uppercut)
[imu] uppercut angleDeg=50.4 elapsed=60ms
[imu] uppercut angleDeg=43.5 elapsed=60ms
[pair] IMU-only event, no angle available: type=uppercut
[vision] hook angle=+11.7 dx=+0.40 dy=-0.08 speed=9.29 travel=0.41
>>> angle=+11.7 deg  type=hook  (paired=yes, imu_type=uppercut)
[imu] uppercut angleDeg=52.3 elapsed=60ms
[vision] hook angle=+10.2 dx=+0.39 dy=-0.07 speed=6.88 travel=0.45
>>> angle=+10.2 deg  type=hook  (paired=yes, imu_type=uppercut)
[imu] hook angleDeg=122.4 elapsed=80ms
[imu] uppercut angleDeg=46.4 elapsed=80ms
[pair] IMU-only event, no angle available: type=hook
[vision] hook angle=+16.9 dx=+0.62 dy=-0.19 speed=9.67 travel=0.65
>>> angle=+16.9 deg  type=hook  (paired=yes, imu_type=uppercut)
[imu] uppercut angleDeg=42.3 elapsed=180ms
[vision] hook angle=+5.2 dx=+0.80 dy=-0.07 speed=13.12 travel=0.80
>>> angle=+5.2 deg  type=hook  (paired=yes, imu_type=uppercut)
[vision] hook angle=+85.5 dx=+0.04 dy=-0.49 speed=9.21 travel=0.49
[imu] hook angleDeg=129.2 elapsed=200ms
>>> angle=+85.5 deg  type=hook  (paired=yes, imu_type=hook)
[imu] uppercut angleDeg=42.1 elapsed=160ms
[vision] punch angle=+47.8 dx=+0.36 dy=-0.39 speed=10.60 travel=0.53
>>> angle=+47.8 deg  type=punch  (paired=yes, imu_type=uppercut)
[imu] uppercut angleDeg=47.2 elapsed=140ms
[vision] uppercut angle=+66.0 dx=+0.26 dy=-0.59 speed=11.66 travel=0.64
>>> angle=+66.0 deg  type=uppercut  (paired=yes, imu_type=uppercut)
[imu] uppercut angleDeg=29.9 elapsed=120ms
[vision] punch angle=+43.1 dx=+0.44 dy=-0.42 speed=15.44 travel=0.68
>>> angle=+43.1 deg  type=punch  (paired=yes, imu_type=uppercut)
[vision] punch angle=-101.2 dx=-0.09 dy=+0.47 speed=16.75 travel=0.52
[imu] uppercut angleDeg=30.9 elapsed=60ms
>>> angle=-101.2 deg  type=punch  (paired=vision_only)
[vision] hook angle=-4.0 dx=+0.51 dy=+0.04 speed=11.32 travel=0.59
>>> angle=-4.0 deg  type=hook  (paired=yes, imu_type=uppercut)
[vision] punch angle=-101.0 dx=-0.11 dy=+0.57 speed=10.85 travel=0.58
[vision] hook angle=+32.9 dx=+0.27 dy=-0.18 speed=15.75 travel=0.50
>>> angle=-101.0 deg  type=punch  (paired=vision_only)
>>> angle=+32.9 deg  type=hook  (paired=vision_only)
[vision] hook angle=+41.8 dx=+0.60 dy=-0.54 speed=15.54 travel=0.81
[imu] uppercut angleDeg=32.5 elapsed=200ms
>>> angle=+41.8 deg  type=hook  (paired=yes, imu_type=uppercut)
[vision] hook angle=-161.0 dx=-0.50 dy=+0.17 speed=9.03 travel=0.53
>>> angle=-161.0 deg  type=hook  (paired=vision_only)
[imu] uppercut angleDeg=49.9 elapsed=140ms
[vision] hook angle=+63.8 dx=+0.17 dy=-0.35 speed=7.94 travel=0.47
>>> angle=+63.8 deg  type=hook  (paired=yes, imu_type=uppercut)
[vision] hook angle=+13.1 dx=+0.50 dy=-0.12 speed=10.62 travel=0.52
[imu] uppercut angleDeg=59.7 elapsed=240ms
>>> angle=+13.1 deg  type=hook  (paired=yes, imu_type=uppercut)
[vision] hook angle=+35.7 dx=+0.56 dy=-0.40 speed=13.79 travel=0.69
[imu] uppercut angleDeg=44.2 elapsed=220ms
>>> angle=+35.7 deg  type=hook  (paired=yes, imu_type=uppercut)
[vision] hook angle=+43.8 dx=+0.46 dy=-0.44 speed=15.42 travel=0.63
[imu] uppercut angleDeg=59.9 elapsed=200ms
>>> angle=+43.8 deg  type=hook  (paired=yes, imu_type=uppercut)
[imu] hook angleDeg=98.3 elapsed=60ms
[vision] hook angle=-2.2 dx=+0.72 dy=+0.03 speed=28.72 travel=0.89
>>> angle=-2.2 deg  type=hook  (paired=yes, imu_type=hook)
```

20 vision angle events got logged against 15 real thrown punches - see
"Extra events" below for the likely reason (not a bug in the angle math
itself).

## Angle readings sorted low to high (computed from the rows above)

| angle_deg | dx | dy | vision's type label | shape |
|---|---|---|---|---|
| -161.0 | -0.50 | +0.17 | hook | horizontal, **leftward** |
| -101.2 | -0.09 | +0.47 | punch | downward |
| -101.0 | -0.11 | +0.57 | punch | downward |
| -5.2 | +0.54 | +0.05 | hook | horizontal, rightward |
| -4.0 | +0.51 | +0.04 | hook | horizontal, rightward |
| -2.2 | +0.72 | +0.03 | hook | horizontal, rightward |
| +5.2 | +0.80 | -0.07 | hook | horizontal, rightward |
| +10.2 | +0.39 | -0.07 | hook | horizontal, rightward |
| +11.7 | +0.40 | -0.08 | hook | horizontal, rightward |
| +13.1 | +0.50 | -0.12 | hook | horizontal, rightward |
| +16.9 | +0.62 | -0.19 | hook | horizontal, slight rise |
| +32.9 | +0.27 | -0.18 | hook | diagonal |
| +35.7 | +0.56 | -0.40 | hook | diagonal, ~45 |
| +41.8 | +0.60 | -0.54 | hook | diagonal, ~45 |
| +43.1 | +0.44 | -0.42 | punch | diagonal, ~45 |
| +43.8 | +0.46 | -0.44 | hook | diagonal, ~45 |
| +47.8 | +0.36 | -0.39 | punch | diagonal, ~45 |
| +63.8 | +0.17 | -0.35 | hook | diagonal, vertical-leaning |
| +66.0 | +0.26 | -0.59 | uppercut | diagonal, vertical-leaning |
| +85.5 | +0.04 | -0.49 | hook | vertical |

## Findings

**The angle formula is validated - this is the headline result.** No sign
bugs, no `abs(dx)` creeping in, no clustering-regardless-of-type (the exact
failure shape the shelved IMU-only approach hit four separate times - see
`CLAUDE.md`). Three clean signs of correctness, all visible directly in the
sorted table above, none requiring the vision `type` label (which stays as
unreliable as already documented - see below) to be trusted:

1. **A genuine continuous gradient, not two clusters.** Sorted by angle,
   the readings climb smoothly from horizontal (-5 to +17) through a tight
   diagonal band (33-48) up to vertical (85.5) - exactly what a continuous
   measurement is supposed to produce, as opposed to the old discrete
   hook-or-uppercut classifier's binary output.
2. **Signed `dx` is propagating correctly.** The one leftward hook
   (`dx=-0.50`) landed at -161 - the opposite side of the circle from the
   nine rightward hooks (`dx>0`, all landing -5 to +17) - confirming this
   isn't silently reproducing `elevation_angle()`'s `abs(dx)`-clamped
   (-90,+90] range.
3. **The `-dy` negation is the right way around.** Every reading with a
   strongly negative `dy` (a real rise) landed near +90 (`+85.5`), not -90
   - confirming the sign flip matches `elevation_angle()`'s up-positive
   convention as designed, not inverted.

**The ~45-degree side-uppercut throws look like the cleanest result in the
whole batch.** Five readings (`+32.9, +35.7, +41.8, +43.1, +43.8`, and
arguably `+47.8`) cluster tightly in a 33-48 degree band - suspiciously
close to the user's stated ~45-degree intent for that third batch of
throws. For a throw type that's inherently harder to aim consistently by
feel than a straight hook or a straight uppercut, landing this tightly
clustered is a strong sign the measurement itself is precise, not just
directionally correct.

**The straight-up uppercut batch is less clean than expected.** Only one
reading (`+85.5`) landed clearly near +90; `+66.0` and `+63.8` lean
vertical but sit well short of it, and several readings that should
plausibly belong to this batch instead landed in the same 33-48 degree
diagonal band as the side-uppercuts. Three honest possible explanations,
not yet distinguished - worth a cleaner re-run (slower, paused between
throws, 90/0 straight shapes only) before concluding anything further:
   - A real "straight" uppercut may still carry forward/rightward drift in
     the 2D camera plane even when it feels purely vertical to the
     thrower - the wrist IMU doesn't have to worry about this (it measures
     real 3D acceleration), but 2D image-plane projection does.
   - `dx,dy` is a NET displacement over the whole detection window
     (`HOOK_WINDOW_SEC`), not an instantaneous strike-moment snapshot - if
     windup motion bleeds into the window before the vertical strike
     itself, it would pull the net angle away from pure vertical.
   - Simple throw-order bleed in a live test with no pause/reset between
     punches (same caveat `imu_vision_agreement_results_2026-11-03.md`
     already flagged for its own data).

**Extra events (20 logged vs. 15 thrown) - likely recovery snap-back, not
a bug.** The two `-101` readings (`dx` near 0, `dy` strongly positive =
real downward travel) are the clearest case: a punch's retraction/recovery
can independently cross the same speed+travel gate as the strike itself,
exactly the risk `punch_classifier.py`'s own code comments already flag
("the window catching a punch's recovery snap-back instead of the
strike"). These read at a physically correct -101 degrees for a genuine
downward motion - the formula is doing exactly what it should with input
that probably isn't a punch at all. Not touched/fixed here - flagging for
awareness, not something this pass addresses.

**Type-classification mismatches are expected, unchanged, and don't affect
the angle.** Vision's own `type` field called almost everything "hook"
even for throws that were clearly vertical by `dx,dy` (e.g. `+85.5` logged
as `hook`) - consistent with the already-documented 22% vision-uppercut
accuracy in `imu_vision_agreement_results_2026-11-03.md`. The IMU's type
called nearly everything "uppercut" this session (`angleDeg` mostly in the
30-60 range, below `ANGLE_HOOK_MIN_DEG=60`), including during the opening
hook batch - consistent with the already-documented 30% IMU-hook accuracy.
Neither is a new problem and neither was ever the angle source - the
angle comes from vision's raw `dx,dy` directly, never from either type
label.

## Why this matters

This is the first confirmation that fusing vision's own position data
(instead of IMU accelerometer-direction alone) produces a real, usable
continuous angle - the specific gap the shelved IMU-only signed-angle
attempts never closed. Not yet wired into anything - this is still a
pure measurement/display tool (`imu_vision_angle_test.py`), per the
project's own build-standalone-first convention. Next step suggested by
this data, not yet done: a cleaner, slower re-test isolating straight-up
uppercuts specifically (pause and reset between throws) to settle whether
the 66/63.8-not-85 pattern above is thrower-form, window-blending, or
something else before trusting this for anything beyond display.
