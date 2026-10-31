/*
  Boxing Robot Arm - Motion Mirror (270-DEGREE SERVO VARIANT)
  Elegoo Uno R3 + PCA9685 + 4x DS3218/Miuzei 270-degree servos
  (Pan/Tilt/Yaw shoulder + Elbow) + GY-521/MPU-6050 wrist IMU punch detector

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

  ==================== WRIST IMU PUNCH DETECTOR (2026-10-31) ====================
  Folded in from imu_punch_classify_test.ino (that file is left standing,
  unchanged, as the confirmed-good standalone reference - see CLAUDE.md's
  "Wrist IMU punch detector" section for the full history of how this
  classifier logic was found/tuned/confirmed on real throws). Same chip
  (MPU-6050 at I2C address 0x68), same bus as the PCA9685 (0x40) - no
  address conflict, confirmed low-risk to combine since both halves
  already used the identical non-blocking timestamp-gate loop() idiom
  before merging.

  This is ONLY the confirmed-good binary classifier (hook / uppercut /
  low-rotation "punch"). The separate continuous-signed-angle experiment
  (imu_punch_signed_angle_test.ino) was tried and SHELVED after extensive
  real-data testing - see CLAUDE.md - and is deliberately NOT part of this
  merge.

  Whenever a real punch is detected, this prints the same
  "PUNCH,<type>,..." line the standalone test did, over the SAME serial
  connection the host computer already uses to send "pan,tilt,yaw,elbow"
  commands - the Python side reads for these lines on a background thread
  while still writing servo commands on the main thread (see
  interpolation_approach/imu_punch_listener.py).

  One real behavior change from the old servo-only sketch: setup() now
  blocks for about 4 seconds (CALIBRATION_SAMPLES) right after boot while
  the IMU calibrates - the wrist needs to be held still during that
  window (see calibrate() below). The arm centers itself and the LED
  blinks its 3-blink boot confirmation BEFORE this calibration starts, so
  "arm reaches center, then pauses a few seconds" is expected, not a bug.
  ================================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <string.h>
#include <stdlib.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ==================== TUNABLE SETTINGS - SERVOS ====================

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

// ==================== TUNABLE SETTINGS - WRIST IMU ====================
// Unchanged from imu_punch_classify_test.ino - same confirmed-good values,
// not re-tuned for this merge. See that file for the full tuning history.

#define PUNCH_THRESHOLD_G   1.5   // magnitude-above-gravity spike that starts
                                   // a capture window
#define PUNCH_COOLDOWN_MS   300   // minimum gap between two separate punch reports
#define CAPTURE_MAX_MS      400   // hard cap on a capture window's length
#define CAPTURE_MIN_MS      60    // floor so a 1-2 sample noise blip can't finalize early
#define ANGLE_HOOK_MIN_DEG  60.0  // angle-vs-refUp (degrees) above which a punch is "hook"
#define GYRO_MIN_FOR_ROTATION_DPS 60.0  // below this peak gyro, call it "punch" (straight)
#define REFUP_ADAPT_RATE    0.03  // EMA rate refUp drifts toward current accel while idle

#define IMU_LOOP_INTERVAL_MS 20   // ~50 Hz IMU sample rate - separate timer from the
                                   // servo step gate below so neither blocks the other
#define CALIBRATION_SAMPLES  200  // samples averaged at boot while the wrist is held still

#define MPU_ADDR        0x68  // GY-521 I2C address with AD0 unconnected/low
#define REG_PWR_MGMT_1  0x6B
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define ACCEL_CONFIG_VALUE 0x08  // AFS_SEL=1 -> +/-4g
#define ACCEL_SENSITIVITY  8192.0
#define GYRO_CONFIG_VALUE  0x08  // FS_SEL=1 -> +/-500 deg/s
#define GYRO_SENSITIVITY   65.5

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

// ---- Wrist IMU state (unchanged from imu_punch_classify_test.ino) ----
float refUpX = 0.0, refUpY = 0.0, refUpZ = 1.0;
float restMag = 1.0;
float gxOffset = 0.0, gyOffset = 0.0, gzOffset = 0.0;

unsigned long lastImuLoopTime = 0;
unsigned long lastPunchTime = 0;

enum ImuState { IMU_IDLE, IMU_CAPTURING };
ImuState imuState = IMU_IDLE;
unsigned long captureStartTime = 0;
float peakSignal = 0.0;
float peakAx = 0, peakAy = 0, peakAz = 0;
float peakGx = 0, peakGy = 0, peakGz = 0;
float peakGyroMag = 0.0;
float swingGx = 0, swingGy = 0, swingGz = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(BAUD_RATE);
  Wire.begin();  // shared bus - PCA9685 (0x40) and MPU-6050 (0x68) both need this once

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  // Move to a safe, centered starting position.
  moveServo(PAN_CHANNEL, currentPan);
  moveServo(TILT_CHANNEL, currentTilt);
  moveServo(YAW_CHANNEL, currentYaw);
  moveServo(ELBOW_CHANNEL, currentElbow);

  // Boot confirmation: 3 blinks means the servo half of setup() finished
  // (pwm.begin() didn't hang on I2C). Uses the LED, not Serial, so it's
  // safe at any send rate.
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);
  }

  // Wrist IMU wake/config, then a ~4s BLOCKING calibration (wrist must be
  // held still) - happens AFTER the arm is already centered and the boot
  // blinks are done, so "arm reaches center, then pauses a few seconds"
  // is expected behavior, not a hang.
  writeRegister(REG_PWR_MGMT_1, 0x00);
  writeRegister(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE);
  writeRegister(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE);
  calibrateImu();
}

void loop() {
  readSerial();  // always, every pass - never gated, see that function's own comment

  if (millis() - lastStepTime >= STEP_DELAY_MS) {
    lastStepTime = millis();
    stepToward(currentPan, targetPan, PAN_CHANNEL);
    stepToward(currentTilt, targetTilt, TILT_CHANNEL);
    stepToward(currentYaw, targetYaw, YAW_CHANNEL);
    stepToward(currentElbow, targetElbow, ELBOW_CHANNEL);
  }

  // IMU sampling - its own independent ~50Hz gate, falls through without
  // ever blocking the serial-read or servo-step work above (same
  // non-blocking timestamp-gate idiom both halves already used before
  // this merge).
  unsigned long now = millis();
  if (now - lastImuLoopTime >= IMU_LOOP_INTERVAL_MS) {
    lastImuLoopTime = now;
    imuLoopTick(now);
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

// ==================== WRIST IMU - unchanged logic from imu_punch_classify_test.ino ====================
// See that file's header for the full reasoning (refUp adaptivity, why
// angleDeg instead of gyro-verticality, etc.) - not re-explained here to
// keep this production file from being dominated by history that already
// lives elsewhere. Function/variable names prefixed/suffixed with "Imu"
// where they'd otherwise collide with the servo code's own naming.

void imuLoopTick(unsigned long now) {
  float ax, ay, az, gx, gy, gz;
  readAccelGyro(ax, ay, az, gx, gy, gz);
  gx -= gxOffset;
  gy -= gyOffset;
  gz -= gzOffset;

  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  float signal = magnitude - restMag;
  float gMagSample = sqrt(gx * gx + gy * gy + gz * gz);

  if (imuState == IMU_IDLE) {
    if (signal < PUNCH_THRESHOLD_G && magnitude > 0.1) {
      float invMag = 1.0 / magnitude;
      refUpX += REFUP_ADAPT_RATE * (ax * invMag - refUpX);
      refUpY += REFUP_ADAPT_RATE * (ay * invMag - refUpY);
      refUpZ += REFUP_ADAPT_RATE * (az * invMag - refUpZ);
      float refMag = sqrt(refUpX * refUpX + refUpY * refUpY + refUpZ * refUpZ);
      if (refMag > 0.01) {
        refUpX /= refMag; refUpY /= refMag; refUpZ /= refMag;
      }
    }

    if (signal > PUNCH_THRESHOLD_G && (now - lastPunchTime >= PUNCH_COOLDOWN_MS)) {
      imuState = IMU_CAPTURING;
      captureStartTime = now;
      peakSignal = signal;
      peakAx = ax; peakAy = ay; peakAz = az;
      peakGx = gx; peakGy = gy; peakGz = gz;
      peakGyroMag = gMagSample;
      swingGx = gx; swingGy = gy; swingGz = gz;
    }
    return;
  }

  // imuState == IMU_CAPTURING
  if (signal > peakSignal) {
    peakSignal = signal;
    peakAx = ax; peakAy = ay; peakAz = az;
    peakGx = gx; peakGy = gy; peakGz = gz;
  }
  if (gMagSample > peakGyroMag) {
    peakGyroMag = gMagSample;
    swingGx = gx; swingGy = gy; swingGz = gz;
  }

  unsigned long elapsed = now - captureStartTime;
  bool decayed = signal < PUNCH_THRESHOLD_G && elapsed >= CAPTURE_MIN_MS;
  bool timedOut = elapsed >= CAPTURE_MAX_MS;
  if (decayed || timedOut) {
    classifyAndPrintPunch(elapsed);
    lastPunchTime = now;
    imuState = IMU_IDLE;
  }
}

// Runs once per punch, on the single peak sample captured above.
void classifyAndPrintPunch(unsigned long elapsedMs) {
  float pax = peakAx - refUpX * restMag;
  float pay = peakAy - refUpY * restMag;
  float paz = peakAz - refUpZ * restMag;
  float pMag = sqrt(pax * pax + pay * pay + paz * paz);

  float angleDeg = 0.0;
  if (pMag > 0.05) {
    float cosAngle = (pax * refUpX + pay * refUpY + paz * refUpZ) / pMag;
    cosAngle = constrain(cosAngle, -1.0, 1.0);
    angleDeg = acos(cosAngle) * RAD_TO_DEG;
  }

  float impactGMag = sqrt(peakGx * peakGx + peakGy * peakGy + peakGz * peakGz);
  float vertImpact = 0.0;
  if (impactGMag > 1.0) {
    vertImpact = (peakGx * refUpX + peakGy * refUpY + peakGz * refUpZ) / impactGMag;
  }

  float vertSwing = 0.0;
  if (peakGyroMag > 1.0) {
    vertSwing = (swingGx * refUpX + swingGy * refUpY + swingGz * refUpZ) / peakGyroMag;
  }

  const char *type;
  if (peakGyroMag < GYRO_MIN_FOR_ROTATION_DPS) {
    type = "punch";  // straight-ish - not enough rotation to call hook/uppercut
  } else if (angleDeg > ANGLE_HOOK_MIN_DEG) {
    type = "hook";
  } else {
    type = "uppercut";
  }

  // Same CSV shape imu_punch_classify_test.ino printed, unchanged - the
  // Python-side listener parses this exact format.
  Serial.print("PUNCH,");
  Serial.print(type);
  Serial.print(',');
  Serial.print(angleDeg, 1);
  Serial.print(',');
  Serial.print(elapsedMs);
  Serial.print(',');
  Serial.print(vertSwing, 2);
  Serial.print(',');
  Serial.print(vertImpact, 2);
  Serial.print(',');
  Serial.print(peakGyroMag, 1);
  Serial.print(',');
  Serial.print(refUpX, 2);
  Serial.print(',');
  Serial.print(refUpY, 2);
  Serial.print(',');
  Serial.println(refUpZ, 2);
}

// Averages CALIBRATION_SAMPLES accel+gyro readings while the wrist is
// held still. Blocking is fine - runs once in setup(), not in loop().
void calibrateImu() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    readAccelGyro(ax, ay, az, gx, gy, gz);
    sumAx += ax; sumAy += ay; sumAz += az;
    sumGx += gx; sumGy += gy; sumGz += gz;
    delay(IMU_LOOP_INTERVAL_MS);
  }
  float avgAx = sumAx / CALIBRATION_SAMPLES;
  float avgAy = sumAy / CALIBRATION_SAMPLES;
  float avgAz = sumAz / CALIBRATION_SAMPLES;
  restMag = sqrt(avgAx * avgAx + avgAy * avgAy + avgAz * avgAz);
  if (restMag < 0.01) restMag = 1.0;  // guard divide-by-zero on a bad read
  refUpX = avgAx / restMag;
  refUpY = avgAy / restMag;
  refUpZ = avgAz / restMag;

  gxOffset = sumGx / CALIBRATION_SAMPLES;
  gyOffset = sumGy / CALIBRATION_SAMPLES;
  gzOffset = sumGz / CALIBRATION_SAMPLES;

  // restMag should be close to 1.000 (real gravity is 1g) - if it reads
  // noticeably lower, the wrist was moving during calibration. See
  // imu_punch_classify_test.ino's calibrate() comment for the full
  // explanation if this ever needs debugging again.
  Serial.println("imu calibrated");
  Serial.print("restMag=");
  Serial.print(restMag, 3);
  Serial.print(" refUp=(");
  Serial.print(refUpX, 3);
  Serial.print(',');
  Serial.print(refUpY, 3);
  Serial.print(',');
  Serial.print(refUpZ, 3);
  Serial.println(')');
}

// Single 14-byte burst read starting at ACCEL_XOUT_H: accel X/Y/Z (6
// bytes), temperature (2 bytes, skipped - not used here), gyro X/Y/Z (6
// bytes). One I2C transaction instead of two separate ones.
void readAccelGyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // TEMP_OUT_H/L, unused
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = rawAx / ACCEL_SENSITIVITY;
  ay = rawAy / ACCEL_SENSITIVITY;
  az = rawAz / ACCEL_SENSITIVITY;
  gx = rawGx / GYRO_SENSITIVITY;
  gy = rawGy / GYRO_SENSITIVITY;
  gz = rawGz / GYRO_SENSITIVITY;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}
