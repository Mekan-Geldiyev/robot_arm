"""
Boxing Robot Arm (270-degree build) - Named Reference Positions
---------------------------------------------------------------
Servo (pan, tilt, yaw, elbow) tuples only - captured by hand-jogging the
new DS3218/Miuzei 270-degree servos into each pose via Serial Monitor on
robot_arm_270.ino, then reading it back with `print`. These are servo
targets, NOT paired with a raw MediaPipe reading the way
interpolation_implement/calibration_data.json's entries are - that's a
separate, still-needed step (hold each pose in front of the camera and
run capture_point.py) before the live camera-mirroring interpolation
system can work on this hardware. Until then, this file is useful on its
own for anything that just needs to reach a named servo position directly
- the hook animation (hook_animation_270.py) and a canned-position demo.

Channel neutral/direction reference (centered 2026-10-20, confirmed via
real testing on this specific build - see CLAUDE.md):
    pan=145   (down=clockwise/hand up, up=counterclockwise/hand down)
    tilt=120  (down=arm rises/faces sky, up=arm lowers/faces ground)
    yaw=70    (down=rotates inward/toward body, up=rotates outward)
    elbow=135 (down=clockwise/moves right, up=counterclockwise/moves left)

Yaw's real safe room around its 70 neutral: ~70 degrees toward the body
(down to 0) before crashing, ~200 degrees away from it (up to 270) - the
inward/crash-risk direction is LOWER yaw values, not higher.
"""

POSITIONS = {
    "REST_GUARD":            (145, 220, 180, 135),  # hand hanging straight down at the side
    "HAND_OUT_SIDE":          (145, 120, 180, 135),  # arm straight out to the side, elbow straight
    "ELBOW_BENT_90":          (145, 220, 180, 45),   # upper arm at rest, forearm folded up toward shoulder
    "ELBOW_STRAIGHT":         (145, 220, 180, 135),  # same as REST_GUARD - elbow's fully-extended reading
    "HAND_MID_CENTER":        (145, 120, 30, 135),   # mid-height, pulled inward from straight-out
    "RAISED_DIAGONAL_UP":     (110, 120, 70, 135),   # arm raised diagonally, halfway between side and overhead
    "LOWERED_DIAGONAL_DOWN":  (200, 120, 70, 135),   # arm angled down and outward, dropped guard
    "STEEP_DIAGONAL_UP":      (90, 120, 70, 135),    # arm raised steeply, close to straight overhead
    "HOOK_WINDUP":            (145, 135, 200, 15),   # fist near ear/cheek, elbow bent tight - winding up a hook
    "FORWARD_LOW":            (169, 120, 70, 135),   # reaching mostly forward, angled slightly down
    "HAND_EXTENDED_SIDE":     (145, 120, 180, 135),  # same as HAND_OUT_SIDE on this build
    "UPPERCUT":               (169, 220, 90, 30),    # arm folded fully over, set to rise and strike upward
    "HIGH_OVERHAND":          (169, 65, 70, 55),     # steepest overhand angle
    "OVERHAND":               (169, 90, 70, 55),     # less steep than HIGH_OVERHAND
    "HOOK":                   (169, 120, 70, 55),    # roughly horizontal, elbow bent, set to sweep across
}
