/*
  Boxing Robot Arm - Motion Mirror
  Elegoo Uno R3 + PCA9685 + 2x MG996R (Pan/Tilt shoulder)

  Receives one line of ASCII over serial:
      "pan,tilt\n"            (elbow servo not attached)
      "pan,tilt,elbow\n"      (elbow servo attached)

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
#define ELBOW_CHANNEL 2     // NOT INSTALLED YET - placeholder for later

#define BAUD_RATE     9600

#define STEP_SIZE     3     // degrees per smoothing step (bigger = faster, less smooth)
#define STEP_DELAY_MS 15    // ms between smoothing steps
// STEP_SIZE history: 2 (original) -> 5 on 2026-08-03 (too aggressive -
// felt jerky/overshooting) -> 3 same day. 3 deg / 15ms = ~200 deg/sec max
// slew, still notably faster than the original 133 deg/sec without going
// all the way to 5's harsh landing. If punches still feel sluggish, try 4
// before going back up to 5; if 3 still feels aggressive, drop to 2.

#define ANGLE_MIN     0
#define ANGLE_MAX     180

// =============================================================

int currentPan   = 90;
int currentTilt  = 90;
int currentElbow = 90;
int targetPan    = 90;
int targetTilt   = 90;
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

// Parses "pan,tilt" or "pan,tilt,elbow" and updates target angles.
void parseLine(char *line) {
  char *panStr = strtok(line, ",");
  char *tiltStr = strtok(NULL, ",");
  char *elbowStr = strtok(NULL, ",");
  if (panStr == NULL || tiltStr == NULL) return; // malformed line, ignore

  targetPan  = constrain(atoi(panStr), ANGLE_MIN, ANGLE_MAX);
  targetTilt = constrain(atoi(tiltStr), ANGLE_MIN, ANGLE_MAX);

  if (elbowStr != NULL) {
    targetElbow = constrain(atoi(elbowStr), ANGLE_MIN, ANGLE_MAX);
    // Channel 2 servo not installed yet - value stored but not sent.
  }

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
