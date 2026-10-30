/*
  Wrist IMU Punch Classifier - standalone test (GY-521 / MPU-6050)

  Builds on imu_punch_test.ino's working spike detector (same wake/config
  registers, same non-blocking ~50Hz loop, same Wire-only no-external-
  library approach) and adds punch TYPE classification (hook vs uppercut
  vs a low-rotation "punch" fallback) plus a numeric swing angle, so the
  eventual goal - replicating the real swing on the robot arm - has
  something to select WHICH canned animation (punch_angle_270.PUNCHES) to
  play, not just "a punch happened."

  Kept as its own file rather than editing imu_punch_test.ino in place -
  same reason this project keeps hook_animation_test.py standing alongside
  punch_angle_test.py instead of overwriting it: the simpler working file
  stays as a known-good fallback while this one is actively tuned. Also
  fixes a real bug found while extending it (see the magnitude comment in
  loop() below) - imu_punch_test.ino's resting baseline actually sits at
  about -1g, not 0, because it subtracts the full calibration vector
  (gravity included) from every axis AND THEN subtracts another 1g from
  the magnitude. Punch detection still worked there because real punches
  are large spikes regardless of where the baseline sits, but this file
  does it correctly since the classifier math below needs a real
  direction reference, not just "something that spikes."

  Standalone - no Adafruit_PWMServoDriver, no servo code touched. Shares
  the Uno's A4(SDA)/A5(SCL) bus with the PCA9685 (0x40) the same way
  imu_punch_test.ino does; this is still meant to fold into
  robot_arm_270.ino's setup()/loop() later, not run on a second Arduino.

  ==================== HOW THE CLASSIFIER WORKS ====================
  Calibration (holding the wrist still, ideally in guard/ready position)
  seeds refUp, a unit vector that means "which way is up, in THIS
  sensor's own local axes, as mounted on THIS wrist right now." That's
  the trick that makes the rest of this work without knowing or caring
  how the GY-521 chip happens to be glued to the wrist brace (confirmed
  fine from a photo of the actual mount, 2026-10-29 - rigidly taped to
  the brace, not a wiring/mounting problem).

  refUp is NOT frozen at that one startup moment, though - see
  REFUP_ADAPT_RATE above. A direct test (2026-10-29) threw the same
  uppercut motion with palm up vs. palm down - a pure forearm roll, not a
  different swing - and got hook one way, uppercut the other. Rotating
  the forearm about its own long axis spins the sensor's local axes
  relative to gravity even though the real swing arc didn't change, so a
  one-shot calibration goes stale the moment the wrist's rest/guard
  orientation differs from whatever it was at power-on. refUp now drifts
  slowly toward the current accel reading whenever idle (never during a
  capture window, so a punch's own motion can't drag it), so each throw
  gets judged against a fresh reference instead of a stale one. Printed
  per-punch (the last 3 CSV fields) specifically so a palm-up-vs-down
  comparison can show directly whether it actually followed this time.

  TWO ROUNDS of a gyro-rotation-axis model (project a gyro reading onto
  refUp - big projection -> hook, small -> uppercut, on the reasoning
  that a hook's angular velocity should point roughly parallel to refUp
  and an uppercut's roughly perpendicular) were tried and BOTH failed on
  real throws (2026-10-29): first projecting the gyro sample from the
  same instant as peak accel ("vertImpact" below), then projecting
  whichever sample had the single highest gyro magnitude anywhere in the
  window ("vertSwing", tracked independently of accel timing). Both put
  nearly every real throw - hooks AND uppercuts alike - at the same
  extreme value (~-0.9 to -1.0), AND vertSwing kept landing almost
  identical to vertImpact anyway, meaning peak rotation and peak
  acceleration were co-occurring for real throws, not separated in time
  the way the "wrist snap happens later" theory assumed. Both fields are
  still computed and printed (comparison only) in case more data changes
  this conclusion, but classification no longer uses either.

  Classification now runs on angleDeg instead - the angle between the
  punch's own (gravity-subtracted) acceleration vector and refUp. NOT a
  world-frame compass angle (no magnetometer/fusion here): 0 degrees
  means the punch's extra acceleration pointed the same way gravity does
  at rest (physically: roughly UP the body - what an uppercut should do),
  90 degrees means perpendicular to that (roughly ACROSS the body - what
  a hook should do). The one data point available for a real (if
  truncated - see CAPTURE_MIN_MS) uppercut read 44.4 degrees against a
  75-108 degree hook cluster, which is WHY this switched from gyro to
  accel-direction - but still only one data point, not yet confirmed
  with a clean, non-truncated uppercut. See ANGLE_HOOK_MIN_DEG below.

  A real straight punch (jab/cross) has very little rotation either way
  (the forearm just extends), so a small total gyro reading regardless of
  direction gets classified "punch" (straight) rather than being forced
  into hook or uppercut - deliberately mirroring the vision-based
  classifier's own "hook" / "uppercut" / unclassified "punch" vocabulary
  (see punch_classifier.py) so the two signals could eventually agree or
  disagree on the same event, not speak different languages.

  ====================================================================

  Throw real hooks and uppercuts with this running and watch the "PUNCH,"
  lines - that's the ground-truth data needed to tell whether
  ANGLE_HOOK_MIN_DEG/GYRO_MIN_FOR_ROTATION_DPS are in the right
  ballpark, the same "watch it, then tune the constant" process this repo
  already uses everywhere else (see STRIKE_END_YAW, YAW_SAFE_FLOOR_270,
  SNAP_DISTANCE, etc. in the Python side for the established pattern).
*/

#include <Wire.h>

// ==================== TUNABLE SETTINGS ====================

#define PUNCH_THRESHOLD_G   1.5   // magnitude-above-gravity spike that starts
                                   // a capture window - same starting guess
                                   // as imu_punch_test.ino
#define PUNCH_COOLDOWN_MS   300   // minimum gap between two separate punch
                                   // reports

#define CAPTURE_MAX_MS 400  // hard cap on how long a capture window can run
                             // before being force-finalized, in case the
                             // signal never drops back under threshold
                             // (e.g. the arm stays tensed) - without this
                             // the classifier would just hang mid-punch
                             // forever and never print a result

// Floor on how long a capture window must run before the "signal decayed
// below threshold" condition is allowed to finalize it. Added after a
// real test produced a ms=20 "uppercut" that was actually just a 1-2
// sample noise blip crossing PUNCH_THRESHOLD_G and immediately dropping
// back out - far too little data for peak-tracking to find a real peak,
// so it produced a near-zero, meaningless verticality/angle reading
// instead of a genuine classification. Real hooks/uppercuts in that same
// test ran 60-320ms, so 60 is a conservative floor, not a tight fit -
// tighten once more real duration data comes in.
#define CAPTURE_MIN_MS 60

// Angle-vs-refUp (degrees, from the printed "angle" field) ABOVE which a
// punch counts as "hook" rather than "uppercut" - physically, a hook's
// net strike acceleration should point roughly ACROSS the body
// (perpendicular to refUp, i.e. close to 90 degrees), while an uppercut's
// should point roughly UP the body (aligned with refUp, i.e. close to 0).
// Switched from gyro-verticality to this after two different ways of
// reading the gyro (impact-synchronized, then swing-peak-decoupled) both
// came back nearly identical for every real throw regardless of intended
// type - the one data point available for a (mostly-)real uppercut read
// 44.4 degrees versus every hook landing 75-108, so 60 is a first guess
// splitting those two groups, NOT yet confirmed on a clean, non-truncated
// uppercut (CAPTURE_MIN_MS above should fix that next time).
#define ANGLE_HOOK_MIN_DEG 60.0

// Total peak gyro magnitude (deg/s) below which a punch is called
// "punch" (straight/low-rotation) instead of being forced into hook or
// uppercut - a real jab/cross should land well below whatever a real
// hook/uppercut's peak gyro reads. UNTESTED GUESS, same tuning process as
// VERTICALITY_THRESHOLD above.
#define GYRO_MIN_FOR_ROTATION_DPS 60.0

// How fast refUp (the "which way is up" reference, see file header) drifts
// toward the CURRENT accel reading while idle, each loop tick (0..1, EMA
// weight on the new sample). Added after a direct test (2026-10-29):
// throwing the same uppercut motion with palm up vs. palm down - a pure
// forearm roll, not a different swing - flipped the classification
// between hook and uppercut. A one-shot startup calibration can't handle
// that, since any guard/rest orientation change between throws (which
// naturally happens, e.g. the user was deliberately varying palm
// orientation) leaves refUp pointing at a stale, no-longer-true "up."
// Low on purpose: fast enough to resync within a second or two of resting
// between punches, slow enough that it won't meaningfully move DURING a
// single fast punch (it also only updates in the IDLE branch below, never
// while CAPTURING, so a punch's own motion can't drag it off target).
#define REFUP_ADAPT_RATE 0.03

#define LOOP_INTERVAL_MS        20   // ~50 Hz sample rate
#define DEBUG_PRINT_INTERVAL_MS 100  // how often the idle "ax,ay,az,mag"
                                     // debug line prints
#define CALIBRATION_SAMPLES     200  // samples averaged at startup while
                                     // the wrist is held still

#define BAUD_RATE 115200  // matches every other sketch/script in this repo

// ============================================================

#define MPU_ADDR        0x68  // GY-521 I2C address with AD0 unconnected/low
#define REG_PWR_MGMT_1  0x6B  // power management - bit 6 (SLEEP) starts set
#define REG_GYRO_CONFIG  0x1B  // bits 4:3 (FS_SEL) pick the gyro's +/-deg/s range
#define REG_ACCEL_CONFIG 0x1C  // bits 4:3 (AFS_SEL) pick the accel's +/-g range
#define REG_ACCEL_XOUT_H 0x3B  // first of 14 contiguous bytes read each
                                // sample: accel X/Y/Z (6 bytes), temp
                                // (2 bytes, unused here), gyro X/Y/Z
                                // (6 bytes) - one burst read instead of
                                // two separate transactions

// AFS_SEL=1 -> +/-4g, 8192 LSB/g (datasheet: +/-2g=16384, +/-4g=8192,
// +/-8g=4096, +/-16g=2048 LSB/g). Must match ACCEL_SENSITIVITY below.
#define ACCEL_CONFIG_VALUE 0x08
#define ACCEL_SENSITIVITY  8192.0

// FS_SEL=1 -> +/-500 deg/s, 65.5 LSB/(deg/s) (datasheet: +/-250=131,
// +/-500=65.5, +/-1000=32.8, +/-2000=16.4 LSB/(deg/s)).
#define GYRO_CONFIG_VALUE  0x08
#define GYRO_SENSITIVITY   65.5

// refUp: unit vector pointing "up" in the sensor's own local axes, as
// measured at calibration time (see calibrate() below) - the reference
// everything in classify() is measured against. restMag is that same
// calibration vector's raw length (should be close to 1.0g; using the
// MEASURED value here instead of hardcoding 1.0 absorbs this specific
// sensor's own small scale error for free).
float refUpX = 0.0, refUpY = 0.0, refUpZ = 1.0;
float restMag = 1.0;

// Gyro bias (deg/s) measured at the same calibration moment - unlike
// accel, gyro has no "gravity" component to worry about, a still wrist
// should read genuinely ~0 on all 3 axes, so straight subtraction is
// correct here (no refUp-style trick needed).
float gxOffset = 0.0, gyOffset = 0.0, gzOffset = 0.0;

unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastPunchTime = 0;

// Capture-window state machine: IDLE watches for the next spike; once one
// crosses PUNCH_THRESHOLD_G, CAPTURING tracks the peak sample (the
// coherent accel+gyro vector pair at whichever instant had the single
// highest magnitude) until the spike decays or CAPTURE_MAX_MS elapses,
// then classify() runs once on that peak and prints the result.
enum State { IDLE, CAPTURING };
State state = IDLE;
unsigned long captureStartTime = 0;
float peakSignal = 0.0;
float peakAx = 0, peakAy = 0, peakAz = 0;   // raw accel (g) at the peak-ACCEL sample
float peakGx = 0, peakGy = 0, peakGz = 0;   // bias-corrected gyro (deg/s) at that SAME peak-accel sample

// Tracked SEPARATELY from the accel peak above - the sample with the
// highest total GYRO magnitude anywhere in the capture window, which is
// often NOT the same instant as peak accel. Peak accel lines up with
// impact/full-extension, which is also when a real punch's wrist
// pronates/"snaps over" for power - a fast roll around the forearm's own
// long axis that happens at the end of basically every punch type, not
// just hooks. If that roll was swamping the shoulder-swing-plane signal
// (the thing that's actually supposed to differ between hook and
// uppercut) by riding along on the same sample as peak accel, tracking
// the true gyro peak independently should catch the mid-swing rotation
// instead, before the end-of-punch snap dominates.
float peakGyroMag = 0.0;
float swingGx = 0, swingGy = 0, swingGz = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();

  writeRegister(REG_PWR_MGMT_1, 0x00);                  // wake the chip (clear SLEEP)
  writeRegister(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE);  // +/-4g
  writeRegister(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE);    // +/-500 deg/s

  calibrate();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_INTERVAL_MS) return;  // non-blocking ~50Hz gate
  lastLoopTime = now;

  float ax, ay, az, gx, gy, gz;
  readAccelGyro(ax, ay, az, gx, gy, gz);
  gx -= gxOffset;
  gy -= gyOffset;
  gz -= gzOffset;

  // Raw vector magnitude minus restMag (the MEASURED "1g", not a
  // hardcoded one) - gravity's magnitude is always ~1g regardless of
  // which way the wrist is oriented, so this reads ~0 at rest no matter
  // the orientation and spikes positive under real extra acceleration.
  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  float signal = magnitude - restMag;

  if (now - lastPrintTime >= DEBUG_PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print(ax, 3);
    Serial.print(',');
    Serial.print(ay, 3);
    Serial.print(',');
    Serial.print(az, 3);
    Serial.print(',');
    Serial.println(signal, 3);
  }

  float gMagSample = sqrt(gx * gx + gy * gy + gz * gz);

  if (state == IDLE) {
    // Keep refUp tracking whatever orientation the wrist is CURRENTLY
    // resting in - see REFUP_ADAPT_RATE above. Skipped on a sample that's
    // already spiking (about to become a punch) so the start of real
    // motion never nudges the reference itself.
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
      state = CAPTURING;
      captureStartTime = now;
      peakSignal = signal;
      peakAx = ax; peakAy = ay; peakAz = az;
      peakGx = gx; peakGy = gy; peakGz = gz;
      peakGyroMag = gMagSample;
      swingGx = gx; swingGy = gy; swingGz = gz;
    }
    return;
  }

  // state == CAPTURING
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
    classifyAndPrint(elapsed);
    lastPunchTime = now;
    state = IDLE;
  }
}

// Runs once per punch, on the single peak sample captured above. See the
// file header's "HOW THE CLASSIFIER WORKS" section for the physical
// reasoning behind verticality/angle.
void classifyAndPrint(unsigned long elapsedMs) {
  // Gravity-subtracted punch acceleration vector - approximate, since
  // refUp was measured at calibration (pre-punch) orientation and the
  // wrist has likely rotated some by the time of peak, but close enough
  // for a first-pass direction feature.
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

  // "Impact" verticality - gyro at the SAME instant as peak accel (full
  // extension/impact, where the end-of-punch wrist snap lives). Kept and
  // printed for comparison only - no longer what classification runs on.
  float impactGMag = sqrt(peakGx * peakGx + peakGy * peakGy + peakGz * peakGz);
  float vertImpact = 0.0;
  if (impactGMag > 1.0) {
    vertImpact = (peakGx * refUpX + peakGy * refUpY + peakGz * refUpZ) / impactGMag;
  }

  // "Swing" verticality - gyro at whichever instant had the highest
  // rotation rate anywhere in the capture window, independent of when
  // accel peaked. This is the one classification actually uses now - see
  // the comment on peakGyroMag/swingG* above for why.
  float vertSwing = 0.0;
  if (peakGyroMag > 1.0) {
    vertSwing = (swingGx * refUpX + swingGy * refUpY + swingGz * refUpZ) / peakGyroMag;
  }

  // Classification now runs on angleDeg (accel-direction vs refUp), not
  // the gyro verticality fields - see ANGLE_HOOK_MIN_DEG above for why.
  // vertSwing/vertImpact are still computed and printed below purely so
  // they can keep being compared against angle-based results.
  const char *type;
  if (peakGyroMag < GYRO_MIN_FOR_ROTATION_DPS) {
    type = "punch";  // straight-ish - not enough rotation to call hook/uppercut
  } else if (angleDeg > ANGLE_HOOK_MIN_DEG) {
    type = "hook";
  } else {
    type = "uppercut";
  }

  // CSV, matching this repo's existing serial-protocol convention
  // (robot_arm_270.ino's "pan,tilt,yaw,elbow", etc.): type, angle (deg,
  // drives classification now), ms-to-recognize, vertSwing, vertImpact
  // (both comparison-only now), peak gyro magnitude (deg/s - the
  // classification straight-punch gate uses this, printed here as
  // gyroMag), then the LIVE refUp this specific punch was actually judged
  // against (see REFUP_ADAPT_RATE) - printed per-event, not just at
  // calibration, so a palm-up-vs-palm-down comparison can show directly
  // whether the reference actually followed the orientation change.
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
// held still. refUp/restMag capture "which way is up, right now, in this
// sensor's own axes" (see file header) - gxOffset/gyOffset/gzOffset
// capture pure gyro bias (a still wrist has genuinely ~0 angular
// velocity, unlike accel which always reads gravity). Blocking is fine -
// runs once in setup(), not in the non-blocking loop() above.
void calibrate() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    readAccelGyro(ax, ay, az, gx, gy, gz);
    sumAx += ax; sumAy += ay; sumAz += az;
    sumGx += gx; sumGy += gy; sumGz += gz;
    delay(LOOP_INTERVAL_MS);
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

  Serial.println("calibrated");

  // Diagnostic only - lets you SEE calibration quality instead of
  // inferring it from the debug line's math. restMag should be close to
  // 1.000 (real gravity is 1g) - if it reads noticeably lower (e.g.
  // 0.85), the wrist was moving during the 200-sample/~4s window, which
  // averages a vector that's pointing a slightly different direction
  // each sample into a SHORTER net vector (classic vector-cancellation:
  // averaging unit vectors that point different ways never adds back up
  // to length 1). A bad restMag means refUp is pointing in a skewed
  // direction too, not just scaled wrong - that biases every single
  // verticality calculation afterward, which can easily look like
  // "everything classifies as hook" even when real punch types differ.
  // If this prints below ~0.95, redo calibration holding the wrist
  // completely still (rest it on something solid) until "calibrated"
  // actually appears.
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
// bytes). One I2C transaction instead of two separate ones for accel vs
// gyro, which matters a little more here than in imu_punch_test.ino since
// this bus is shared with the PCA9685.
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
