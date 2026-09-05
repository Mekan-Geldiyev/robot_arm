/*
  Boxing Robot Arm - Motion Mirror
  Elegoo Uno R3 + PCA9685 + 4x MG996R (Pan/Tilt/Yaw shoulder + Elbow)

  Receives one line of ASCII over serial:
      "pan,tilt,yaw,elbow\n"

  Each value is an angle in degrees (0-180). The Arduino smoothly
  steps the real servo position toward the target instead of
  snapping to it, so motion looks natural instead of jerky.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdlib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ==================== TUNABLE SETTINGS ====================

#define SERVOMIN      150   // pulse length count for 0 degrees
#define SERVOMAX      600   // pulse length count for 180 degrees
#define PWM_FREQ      60    // Hz, standard for analog servos

#define PAN_CHANNEL   0     // horizontal sweep - jab (forward/back) + hook (side)
#define TILT_CHANNEL  1     // vertical raise - uppercut
#define YAW_CHANNEL   2     // shoulder rotation - swings the elbow's hinge plane for hooks
#define ELBOW_CHANNEL 3     // elbow flex

#define BAUD_RATE     115200  // raised from 9600 (2026-08-25) - must match BAUD_RATE in track.py

#define STEP_SIZE     2     // degrees per smoothing step for ORDINARY moves (small target changes)
#define STEP_DELAY_MS 20    // ms between smoothing steps
// Adaptive step size (2026-08-25): STEP_SIZE alone is a single fixed speed
// for everything, tuned gentle to protect the glued joints from ordinary
// jitter - but that same gentleness is what makes a fast punch take ~600ms
// to physically arrive (see the "Interpolation Math" writeup, section 4).
// Rather than raising STEP_SIZE globally (which reintroduces jitter/shock
// on ordinary small corrections), stepToward() below picks a bigger step
// ONLY when the target is currently far from where the arm actually is -
// self-gating, since ordinary jitter never gets this large regardless of
// how STEP_SIZE itself is tuned.
#define FAST_STEP_SIZE      9   // degrees per step once FAST_JUMP_THRESHOLD is exceeded
#define FAST_JUMP_THRESHOLD 15  // degrees remaining - at/above this, use FAST_STEP_SIZE instead
// FAST_STEP_SIZE=9 / STEP_DELAY_MS=20 -> 450 deg/sec during a real fast
// move, vs. 100 deg/sec (STEP_SIZE=2) the rest of the time. Raised 7->9
// (2026-08-26) specifically because the hook animation's software curve
// (STRIKE_SPEED_DEG_PER_SEC in hook_animation_test.py) was already asking
// for 400+ deg/sec while this cap held it to 350 - the servo was
// physically trailing behind the already-eased software curve, adding lag
// on top of lag. NOTE: this is a SHARED, global cap (any big/fast jump on
// any channel uses it, not just hook animations) - there's no per-source
// speed override yet, so this also speeds up any other large live-tracking
// jump the same way. 450 deg/sec is a real, meaningful jump in torque/
// shock on the joints - if breakage resumes, lower this back down (try
// 7-8) before touching STEP_SIZE, which governs the safe default the arm
// spends most of its time at.
// FAST_JUMP_THRESHOLD=15 is comfortably above the ~2 degree deadband
// track.py/track_interpolation.py already gate sends behind, so
// ordinary held-still noise can't accidentally trigger fast mode.
// STEP_SIZE history: 2 (original) -> 5 on 2026-08-03 (too aggressive -
// felt jerky/overshooting) -> 3 same day. 3 deg / 15ms = ~200 deg/sec max
// slew, still notably faster than the original 133 deg/sec without going
// all the way to 5's harsh landing. If punches still feel sluggish, try 4
// before going back up to 5; if 3 still feels aggressive, drop to 2.
// STEP_DELAY_MS bumped 15 -> 17 on 2026-08-12 (yaw axis added) - a small,
// deliberate slowdown (~200 -> ~176 deg/sec max slew) per user request to
// take the edge off, without undoing the 2026-08-03 speed-up above.
// Bumped again 17 -> 18 same day after a re-glue, explicitly requested as
// the smallest perceptible nudge (~176 -> ~167 deg/sec, ~5% slower) to
// take a little more shock off the joints without meaningfully hurting
// responsiveness - go back to STEP_SIZE (bigger effect per step) if this
// is still too small to matter.
// Dropped STEP_SIZE 3->2 and bumped STEP_DELAY_MS 18->20 same day again
// after "insanely finnicky" feedback testing the interpolation approach -
// this is a bigger cut than the earlier "littlest bit" nudges (~167 ->
// ~100 deg/sec max slew, ~40% slower) since the complaint was stronger.
// This affects BOTH track.py and track_interpolation.py equally, since
// it's the same firmware regardless of which Python script is sending
// commands - the real jitter fix (interpolation output flickering between
// nearest-neighbor sets) was addressed in track_interpolation.py itself,
// this just slows the physical response on top of that.

#define ANGLE_MIN     0
#define ANGLE_MAX     180

// =============================================================

int currentPan   = 0;
int currentTilt  = 90;
int currentYaw   = 45;
int currentElbow = 90;
int targetPan    = 0;
int targetTilt   = 90;
int targetYaw    = 45;
int targetElbow  = 90;

unsigned long lastStepTime = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(BAUD_RATE);

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  // Move to a safe, centered starting position.
  moveServo(PAN_CHANNEL, currentPan);
  moveServo(TILT_CHANNEL, currentTilt);
  moveServo(YAW_CHANNEL, currentYaw);
  moveServo(ELBOW_CHANNEL, currentElbow);

  // Boot confirmation: 3 blinks means setup() finished (pwm.begin() didn't
  // hang on I2C). Uses the LED, not Serial, so it's safe at any send rate.
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }
}

void loop() {
  readSerial();

  if (millis() - lastStepTime >= STEP_DELAY_MS) {
    lastStepTime = millis();
    stepToward(currentPan, targetPan, PAN_CHANNEL);
    stepToward(currentTilt, targetTilt, TILT_CHANNEL);
    stepToward(currentYaw, targetYaw, YAW_CHANNEL);
    stepToward(currentElbow, targetElbow, ELBOW_CHANNEL);
  }
}

// Reads whatever bytes are currently waiting, one loop() pass at a time,
// and never blocks. A blocking read (e.g. Serial.readStringUntil) would
// stall the smoothing loop below and cause the arm to freeze/stutter
// while data backs up - this drains only what's already in the buffer.
//
// Uses a fixed-size char buffer instead of the Arduino String class on
// purpose: String grows/shrinks the heap on every character, and on an
// Uno's 2KB of RAM that fragments the heap after running for a while,
// silently hanging serial parsing (works at first, then just stops).
#define SERIAL_BUF_SIZE 32
char serialBuf[SERIAL_BUF_SIZE];
uint8_t serialBufLen = 0;

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuf[serialBufLen] = '\0';
      parseLine(serialBuf);
      serialBufLen = 0;
    } else if (c != '\r') {
      if (serialBufLen < SERIAL_BUF_SIZE - 1) {
        serialBuf[serialBufLen++] = c;
      } else {
        serialBufLen = 0; // line too long/garbled - drop it and resync
      }
    }
  }
}

// Parses "pan,tilt,yaw,elbow" (all 4 targets at once, what track.py sends),
// a single named jog command "name:value" e.g. "yaw:90" (moves just that
// one channel), or the bare word "print" (dumps the current actual angle of
// all 4 channels, comma-separated - after jogging channels one at a time to
// hand-build a pose, this is how you read back the full "pan,tilt,yaw,elbow"
// combination to record as calibration ground truth).
void parseLine(char *line) {
  if (strcmp(line, "print") == 0) {
    Serial.print(currentPan);
    Serial.print(',');
    Serial.print(currentTilt);
    Serial.print(',');
    Serial.print(currentYaw);
    Serial.print(',');
    Serial.println(currentElbow);
    return;
  }

  char *colon = strchr(line, ':');
  if (colon != NULL) {
    *colon = '\0';
    char *name = line;
    int value = constrain(atoi(colon + 1), ANGLE_MIN, ANGLE_MAX);

    if (strcmp(name, "pan") == 0) {
      targetPan = value;
    } else if (strcmp(name, "tilt") == 0) {
      targetTilt = value;
    } else if (strcmp(name, "yaw") == 0) {
      targetYaw = value;
    } else if (strcmp(name, "elbow") == 0) {
      targetElbow = value;
    } else {
      Serial.print("unrecognized channel: ");
      Serial.println(name);
      return;
    }

    // Echo back what was actually parsed - jog commands used to be silent,
    // which made it impossible to tell "command worked, servo is moving"
    // apart from "nothing arrived / arrived garbled" (e.g. a baud rate
    // mismatch between this sketch and whatever's sending). Cheap at
    // human-typing speed; the frequent auto-sent "pan,tilt,yaw,elbow" path
    // below deliberately does NOT get this same treatment, since printing
    // on every ~25/sec tracking update would flood the monitor.
    Serial.print(name);
    Serial.print(" -> ");
    Serial.println(value);

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    return;
  }

  char *panStr = strtok(line, ",");
  char *tiltStr = strtok(NULL, ",");
  char *yawStr = strtok(NULL, ",");
  char *elbowStr = strtok(NULL, ",");
  if (panStr == NULL || tiltStr == NULL || yawStr == NULL || elbowStr == NULL) {
    return; // malformed line, ignore
  }

  targetPan   = constrain(atoi(panStr), ANGLE_MIN, ANGLE_MAX);
  targetTilt  = constrain(atoi(tiltStr), ANGLE_MIN, ANGLE_MAX);
  targetYaw   = constrain(atoi(yawStr), ANGLE_MIN, ANGLE_MAX);
  targetElbow = constrain(atoi(elbowStr), ANGLE_MIN, ANGLE_MAX);

  // Toggle the LED every time a valid command is parsed - safe at any
  // send rate (unlike Serial.println, which caused the earlier hang).
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

// Moves 'current' one STEP_SIZE closer to 'target' and writes the result
// to the given PCA9685 channel.
void stepToward(int &current, int target, uint8_t channel) {
  if (current == target) return;

  int remaining = abs(target - current);
  int stepSize = (remaining >= FAST_JUMP_THRESHOLD) ? FAST_STEP_SIZE : STEP_SIZE;

  if (remaining <= stepSize) {
    current = target;
  } else if (target > current) {
    current += stepSize;
  } else {
    current -= stepSize;
  }

  moveServo(channel, current);
}

void moveServo(uint8_t channel, int angleDeg) {
  angleDeg = constrain(angleDeg, ANGLE_MIN, ANGLE_MAX);
  int pulse = map(angleDeg, ANGLE_MIN, ANGLE_MAX, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}
