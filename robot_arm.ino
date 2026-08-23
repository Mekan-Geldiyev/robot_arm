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

#define BAUD_RATE     9600

#define STEP_SIZE     3     // degrees per smoothing step (bigger = faster, less smooth)
#define STEP_DELAY_MS 18    // ms between smoothing steps
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

// Parses either "pan,tilt,yaw,elbow" (all 4 targets at once, what track.py
// sends) or a single named command "name:value" e.g. "yaw:90" (jogs just
// that one channel, leaving the other 3 targets exactly where they were) -
// the named form is for hand-calibrating one servo at a time from the
// Serial Monitor without having to know/resend the other three every time.
void parseLine(char *line) {
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
      return; // unrecognized channel name, ignore
    }

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

  if (abs(target - current) <= STEP_SIZE) {
    current = target;
  } else if (target > current) {
    current += STEP_SIZE;
  } else {
    current -= STEP_SIZE;
  }

  moveServo(channel, current);
}

void moveServo(uint8_t channel, int angleDeg) {
  angleDeg = constrain(angleDeg, ANGLE_MIN, ANGLE_MAX);
  int pulse = map(angleDeg, ANGLE_MIN, ANGLE_MAX, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}
