/*
  Boxing Robot Arm - Motion Mirror (270-DEGREE SERVO VARIANT)
  Elegoo Uno R3 + PCA9685 + 4x DS3218/Miuzei 270-degree servos
  (Pan/Tilt/Yaw shoulder + Elbow)

  Separate sketch from robot_arm.ino on purpose - the original MG996R
  (180-degree) servos are being kept as a fallback in case the 270-degree
  swap and/or the still-printing upper arm part aren't ready in time, so
  the proven 180-degree firmware stays completely untouched. This is the
  only file that changed; everything else (robot_arm.ino, the Python
  scripts) is unaware this sketch exists until it's actually chosen.

  Receives one line of ASCII over serial:
      "pan,tilt,yaw,elbow\n"

  Each value is an angle in degrees (0-270 - see ANGLE_MAX below). The
  Arduino smoothly steps the real servo position toward the target instead
  of snapping to it, so motion looks natural instead of jerky.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdlib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ==================== TUNABLE SETTINGS ====================

#define SERVOMIN      150   // pulse length count for 0 degrees
#define SERVOMAX      600   // pulse length count for ANGLE_MAX degrees (270 - see below)
// SERVOMIN/SERVOMAX themselves are UNCHANGED from the old 180-degree
// MG996R setup (~610-2441us pulse range at PWM_FREQ=60Hz) - most 270-
// degree servos (including the new DS3218/Miuzei ones) are designed to
// use that same standard pulse range and just pack more degrees into it,
// so this is a reasonable starting assumption, but it hasn't been
// physically verified yet. Command 0, 135, and 270 on each channel once
// mounted and confirm the servo actually reaches its true mechanical
// extremes without straining/binding - if it stops short or overshoots,
// these two values (not ANGLE_MAX) are what need adjusting.
#define PWM_FREQ      60    // Hz, standard for analog servos

#define PAN_CHANNEL   0     // horizontal sweep - jab (forward/back) + hook (side)
#define TILT_CHANNEL  1     // vertical raise - uppercut
#define YAW_CHANNEL   2     // shoulder rotation - swings the elbow's hinge plane for hooks
#define ELBOW_CHANNEL 3     // elbow flex

#define BAUD_RATE     115200  // must match BAUD_RATE in track.py - unchanged from robot_arm.ino

#define STEP_SIZE     2     // degrees per smoothing step for ORDINARY moves (small target changes)
#define STEP_DELAY_MS 20    // ms between smoothing steps
#define FAST_STEP_SIZE      9   // degrees per step once FAST_JUMP_THRESHOLD is exceeded
#define FAST_JUMP_THRESHOLD 15  // degrees remaining - at/above this, use FAST_STEP_SIZE instead
// STEP_SIZE/FAST_STEP_SIZE/STEP_DELAY_MS/FAST_JUMP_THRESHOLD copied as-is
// from robot_arm.ino's tuned values (see that file's full tuning history
// if needed) - these are angular speed/deadband concepts, independent of
// the servo's total degree range, so there's no reason to assume they need
// to change just because ANGLE_MAX did. Re-tune here separately if the new
// servos feel different (different torque/speed characteristics) - don't
// assume robot_arm.ino's history still applies exactly.

#define ANGLE_MIN     0
#define ANGLE_MAX     270   // the whole reason this is a separate sketch from
                            // robot_arm.ino (180) - see file header. Real
                            // mechanical range caveats this project has
                            // already hit on the OLD hardware (YAW_SAFE_FLOOR
                            // in track.py, the pan/roll coupling, tilt's old
                            // fold-over point) all need re-verifying on the
                            // new servos/mount, not assumed to still apply
                            // at the same angle values.

// =============================================================

// Startup/reset default: dead center of the 0-270 range (135) on every
// channel - gives the most equal mechanical travel in both directions once
// horns are attached at this position, and there's no old-hardware-tuned
// per-channel default that means anything on the new servos/mount yet.
int currentPan   = 135;
int currentTilt  = 135;
int currentYaw   = 135;
int currentElbow = 135;
int targetPan    = 135;
int targetTilt   = 135;
int targetYaw    = 135;
int targetElbow  = 135;

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
