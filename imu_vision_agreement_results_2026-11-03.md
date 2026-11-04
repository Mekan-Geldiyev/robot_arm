# IMU vs vision punch-type agreement test - 2026-11-03

Raw console output from `interpolation_approach/imu_vision_agreement_test.py`,
the harness that pairs the wrist IMU's confirmed-good binary classifier
(`robot_arm_270.ino`, `angleDeg` vs adaptive `refUp`, UNCHANGED) against the
vision classifier (`punch_classifier.PunchClassifier`, UNCHANGED) for the
SAME real throws, labeled by keyboard (`h`/`u`, sticky mode) as ground
truth. First real paired-agreement data collected for this project - until
now, IMU and vision type-classification accuracy had only ever been
measured separately, never against each other on the same throws.

4 hooks thrown, then 4 uppercuts thrown (per the user's own count) - 12 rows
total got logged because one hook throw's vision fired without a matching
IMU read, and one uppercut throw's IMU fired without a matching vision
read (both logged as one-sided misses, `agree=n/a`, not disagreements).

## Raw console log

```
=== Label mode: HOOK ===

[imu] hook angle=78.5
[vision] hook speed=12.12 travel=0.45
>>> [hook] imu=hook vision=hook agree=yes  |  running acc: imu=100%(1) vision=100%(1) combined=100%(1)
[vision] hook speed=17.15 travel=0.84
[imu] uppercut angle=55.1
>>> [hook] imu=uppercut vision=hook agree=no  |  running acc: imu=50%(2) vision=100%(2) combined=50%(2)
[vision] hook speed=8.05 travel=0.49
[imu] hook angle=120.6
>>> [hook] imu=hook vision=hook agree=yes  |  running acc: imu=67%(3) vision=100%(3) combined=67%(3)
[vision] hook speed=6.90 travel=0.47
[imu] hook angle=123.6
>>> [hook] imu=hook vision=hook agree=yes  |  running acc: imu=75%(4) vision=100%(4) combined=75%(4)
[vision] hook speed=9.24 travel=1.13
>>> [hook] imu=- vision=hook agree=n/a  |  running acc: imu=75%(4) vision=100%(5) combined=60%(5)

=== Label mode: UPPERCUT ===

[imu] hook angle=62.6
[vision] hook speed=9.91 travel=0.53
>>> [uppercut] imu=hook vision=hook agree=yes  |  running acc: imu=60%(5) vision=83%(6) combined=50%(6)
[vision] hook speed=8.42 travel=0.58
[imu] uppercut angle=29.7
>>> [uppercut] imu=uppercut vision=hook agree=no  |  running acc: imu=67%(6) vision=71%(7) combined=43%(7)
[vision] punch speed=11.62 travel=0.49
[imu] uppercut angle=44.2
[vision] hook speed=19.12 travel=0.68
>>> [uppercut] imu=- vision=punch agree=n/a  |  running acc: imu=67%(6) vision=62%(8) combined=38%(8)
>>> [uppercut] imu=uppercut vision=hook agree=no  |  running acc: imu=71%(7) vision=56%(9) combined=33%(9)
[imu] uppercut angle=40.2
[vision] hook speed=23.64 travel=0.71
>>> [uppercut] imu=uppercut vision=hook agree=no  |  running acc: imu=75%(8) vision=50%(10) combined=30%(10)
[imu] uppercut angle=8.7
[vision] punch speed=10.71 travel=0.63
>>> [uppercut] imu=uppercut vision=punch agree=no  |  running acc: imu=78%(9) vision=45%(11) combined=27%(11)
[imu] uppercut angle=36.8
[vision] uppercut speed=6.44 travel=0.43
>>> [uppercut] imu=uppercut vision=uppercut agree=yes  |  running acc: imu=80%(10) vision=50%(12) combined=33%(12)

==================== FINAL SUMMARY ====================
IMU alone:      80% (8/10)
Vision alone:   50% (6/12)
Combined (must agree): 33% (4/12)
```

## Per-type breakdown (computed from the rows above, not printed by the script)

**Hook throws (5):** IMU correct 3/4 fired (1 miss) - the one IMU miss on a
hook called it `uppercut` at `angle=55.1`, just under `ANGLE_HOOK_MIN_DEG=60`.
Vision correct 5/5 fired (0 misses) - vision was flawless on hooks this run.

**Uppercut throws (7):** IMU correct 5/6 fired (1 miss) - the one IMU miss
on an uppercut called it `hook` at `angle=62.6`, just OVER
`ANGLE_HOOK_MIN_DEG=60` - a near-boundary case, not a wild miss. Vision
correct only **1/7** - it called 5 of the 7 real uppercuts `hook` and 2 of
them `punch`, never settling on `uppercut` until the last, most clearly
upward throw.

**The "combined, must-agree" rule scored WORSE (33%) than IMU alone (80%).**
Not a bug - mechanically correct given how bad vision is specifically on
uppercuts here: every time IMU correctly called `uppercut` but vision
wrongly called `hook`/`punch`, the agree-only rule threw away IMU's correct
answer. A rule that lets the weaker signal veto the stronger one on exactly
the case where they're most likely to disagree is worse than just trusting
the stronger signal - useful to know BEFORE building anything smarter.

## User's own observation (2026-11-03), verbatim intent preserved

Vision "recorded hook perfectly" but needs real hip/foot drive behind an
uppercut to call it `uppercut` at all - a "light" uppercut (no foot
movement, hand just raised, palm facing self) gets misread as `hook` by
vision almost every time. The wrist IMU did NOT have this problem - it
correctly read the same light, no-hip uppercuts as `uppercut`. Some rows
were flagged ambiguous by the user as "just moving around" rather than a
clean thrown punch - not separated out in the log above since the script
has no way to mark that after the fact; worth a cleaner re-run with a short
pause and reset between throws if this run's exact numbers ever need
re-verifying.

## Why this matters

This is a NEW, more specific vision-side limitation than the one already
documented in `CLAUDE.md` (the "wide straight-armed hook is kinematically
indistinguishable from an ordinary fast reach" finding from
`punch_dataset.py`). That one was about hook false-positives on non-punches.
This one is about uppercut false-negatives on real, intentional uppercuts
specifically when they're thrown compact/light rather than with a big body
commitment - `punch_classifier._classify()`'s direction heuristic
(`rising > horizontal * DIRECTION_DOMINANCE_RATIO`) apparently doesn't get
satisfied by a light uppercut's actual 2D wrist displacement in camera
space, even though the real-world motion is clearly vertical-dominant to a
person watching (and to the wrist IMU, which measures real 3D acceleration
direction, not 2D image displacement).

**Not yet acted on** - this was a first, fairly small sample (12 throws).
See `CLAUDE.md`'s "Wrist IMU punch detector" chapter for the standing
rule this reinforces: trust the IMU's binary call over vision's on
uppercuts specifically, pending a larger confirming batch.
