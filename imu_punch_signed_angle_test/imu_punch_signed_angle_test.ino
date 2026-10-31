/*
  Wrist IMU Punch Classifier - SIGNED ANGLE variant (GY-521 / MPU-6050)

  Full copy of imu_punch_classify_test.ino (left untouched as the confirmed-
  good fallback - that file's hook/uppercut/punch type classification and
  unsigned angleDeg are both confirmed working on real throws, see
  imu_results_2026-10-29.md) PLUS a new signed, continuous angle - see
  "WHAT'S NEW IN THIS FILE" below. Same standalone constraints as that file:
  no Adafruit_PWMServoDriver, no servo code touched, shares the Uno's
  A4(SDA)/A5(SCL) bus with the PCA9685 (0x40) without conflict.

  ==================== WHAT'S NEW IN THIS FILE ====================
  The old angleDeg (still computed below, unchanged) is UNSIGNED (0-180,
  acos against refUp alone) - it can only say "how far this punch's
  acceleration was from straight up," which is enough to split hook vs
  uppercut with a threshold but can't express a real spectrum (uppercut vs
  hook vs overhand vs some 45-degree diagonal) as one continuous signed
  number the way the user actually wants it (their own anchors: uppercut
  ~= -90 degrees, hook ~= 0 degrees, diagonals in between/beyond).

  Getting a SIGNED angle needs a second reference axis, refForward,
  orthogonal to refUp - without it there's no way to tell "which side" of
  refUp a punch's direction falls on, only "how far." refForward can't be
  measured directly: a static accelerometer reading only ever observes
  ONE real-world direction (gravity = refUp) no matter how carefully or
  consistently the calibration pose is held - this is a hard physical
  limit of a 3-axis accelerometer with no magnetometer, not a tuning
  problem. So refForward is instead SEEDED from one of the chip's own raw
  axes at calibration time, then Gram-Schmidt-orthogonalized against refUp
  (subtract its refUp component, renormalize) to make it horizontal
  relative to "up" - see "SECOND UPDATE" below for why this is now done
  for all 3 raw axes in parallel rather than one guessed axis. This is NOT a true
  geographic "forward" - it's a fixed, ARBITRARY, but CONSISTENT
  horizontal reference, consistent only because the chip is rigidly
  mounted and the calibration pose is already standardized ("arm straight
  out in front, center, held still ~4s" - see CLAUDE.md). refUp itself
  works the same way - it never needed true north either, just a
  consistent meaning for "current up" in local axes.

  WHICH raw axis (X/Y/Z) actually lands horizontal on THIS mounting has
  never been measured. First attempt picked X and guessed wrong - see
  "SECOND UPDATE" below for why this is now resolved by testing all 3
  candidates AT ONCE instead of guessing one, re-flashing, failing,
  guessing another.

  UPDATE (2026-10-30, same day): refForward STARTED as one-shot
  (computed once in calibrate(), never touched in loop()), same "ship
  simplest version first" reasoning refUp itself followed before going
  adaptive. First real hardware test (after an unrelated wiring fault was
  fixed) showed why that wasn't enough on its own: refUp drifted
  substantially over one test session (its own adaptation working as
  designed), while the frozen refForward stayed put, so the two drifted
  out of orthogonality (~36 degrees off by the end of that session) -
  enough to erase all hook/uppercut separation in signedAngleDeg. Fixed
  by making refForward ALSO adaptive - re-derived from the same fixed
  seed axis against the CURRENT refUp every idle tick, right after refUp's
  own EMA update. This keeps the two axes orthogonal as refUp drifts; it
  does NOT track genuine twist about the up axis the way a gyro-
  integration approach would (a real, more complex fallback if this isn't
  enough) - simpler fix tried first since it directly addressed the error
  actually observed.

  SECOND UPDATE (2026-10-30, same day): the adaptive fix above was real
  and necessary but NOT sufficient - a second real test (5 hook/4
  uppercut/5 diagonal, written-down order) still showed no separation,
  actually WORSE/tighter clustering than before (nearly everything -74 to
  -125 degrees regardless of intended type). Root cause: seeding
  refForward from local X alone means forwardComponent only captures
  however much of a punch's horizontal acceleration happens to align with
  THAT one direction - if the real hook-swing plane is rotated away from
  local X on this specific mounting (very plausible, never measured),
  forwardComponent reads small/noisy for every punch regardless of type,
  and atan2 of a near-zero denominator is maximally sensitive to that
  noise - exactly the unstable, non-separating result observed.

  Fix: stopped guessing a single seed axis. refForwardX/Y/Z are now
  length-3 arrays - updateAllRefForwards() computes all three candidates
  (seeded from local X, Y, and Z respectively) every time, both at
  calibration and every idle tick. classifyAndPrint() computes a signed
  angle for EACH of the three and prints all three
  (signedAngleX,signedAngleY,signedAngleZ) so one real test session shows
  directly which seed axis actually separates real throws, instead of
  three separate re-flash-and-rethrow cycles to try each one blind.

  Given refUp and a candidate refForward, one candidate's signed angle:
    upComponent = dot(punchAccel, refUp)            // same for all 3
    forwardComponent = dot(punchAccel, refForward)   // differs per candidate
    signedAngleDeg = atan2(-upComponent, forwardComponent) * RAD_TO_DEG
  Hand-verified: a pure "up the body" punch (upComponent>0, forward~=0)
  gives atan2(negative, ~0) = -90 degrees (uppercut). A pure "across the
  body" punch (up~=0, forwardComponent>0) gives atan2(~0, positive) = 0
  degrees (hook). A 45-degree diagonal between them gives atan2(-x,x) =
  -45 degrees - but only for whichever candidate axis actually captures
  the real hook-swing plane; the other two candidates will look noisy/
  non-separating for the same real throws, which is exactly the signal
  that tells us which one to keep.

  CSV changed again - signedAngleDeg/refForwardX/Y/Z (single-candidate)
  replaced by three parallel fields, one per seed-axis candidate. Old
  field positions before that point (type, angleDeg, ms, vertSwing,
  vertImpact, peakGyroMag, refUpX/Y/Z) are unchanged:
    PUNCH,<type>,<angleDeg>,<ms>,<vertSwing>,<vertImpact>,<peakGyroMag>,
    <refUpX>,<refUpY>,<refUpZ>,<signedAngleX>,<signedAngleY>,<signedAngleZ>
  ====================================================================

  Everything below this point (calibration pose, capture-window state
  machine, old angleDeg/type classification, abandoned gyro-verticality
  fields) is UNCHANGED from imu_punch_classify_test.ino - see that file's
  own header for the full history of why each of those exists/failed/was
  fixed. Only look here for what's new above.
*/

#include <Wire.h>

// ==================== TUNABLE SETTINGS ====================

#define PUNCH_THRESHOLD_G   1.5   // magnitude-above-gravity spike that OPENS a
                                   // capture window (and, on the way back down,
                                   // the level it must decay below to CLOSE one -
                                   // see "decayed" below). Back to the original
                                   // 1.5 (2026-10-30) - briefly raised to 3.5 to
                                   // reject false triggers from fast non-punch
                                   // arm movement, but that broke something else:
                                   // with the onset bar that high, a real punch's
                                   // signal only stayed above it for a few ms,
                                   // collapsing every capture window down to the
                                   // CAPTURE_MIN_MS floor - not enough samples
                                   // left for signedAngleAvgX's window-averaging
                                   // to do anything (confirmed on real data: every
                                   // single capture read ms=60 exactly, and even
                                   // the OLD, previously-reliable angleDeg started
                                   // misreading hooks as uppercut). Rejecting false
                                   // triggers is now PUNCH_ACCEPT_G's job instead
                                   // (checked once, after the window closes, not
                                   // during it) - see that constant.
#define PUNCH_ACCEPT_G      3.5   // peakSignal (the actual max reached during a
                                   // capture window) must clear THIS before the
                                   // result gets classified/printed at all - a
                                   // capture that opened (crossed PUNCH_THRESHOLD_G)
                                   // but never got this strong is silently
                                   // discarded, not reported as a PUNCH. Same real
                                   // measured gap as before (hard hooks 4.31-5.39g,
                                   // deliberate non-punch fast movement 2.25-2.81g,
                                   // nothing between) - just applied as a POST-hoc
                                   // accept/reject gate instead of the window's own
                                   // start/end threshold, so the window itself can
                                   // stay long enough to average over.
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
#define DEBUG_PRINT_ENABLED     0    // idle "ax,ay,az,signal" debug line -
                                     // OFF by default (cluttered Serial
                                     // Monitor during real punch-angle
                                     // testing per user request). Flip to 1
                                     // to bring it back if diagnosing a dead
                                     // sensor again (see the wiring-fault
                                     // memory entry this debug line helped
                                     // catch).
#define DEBUG_PRINT_INTERVAL_MS 100  // how often the idle "ax,ay,az,mag"
                                     // debug line prints, when enabled above
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

// NEW (this file only) - THREE candidate second-reference axes, each
// orthogonal to refUp, needed to turn the single-axis unsigned angleDeg
// into a full signed angle (see file header). Index 0/1/2 = seeded from
// local X/Y/Z respectively, each Gram-Schmidt-orthogonalized against
// refUp - NOT a true geographic "forward," just a fixed-per-axis,
// arbitrary-but-consistent horizontal reference. All 3 are kept (rather
// than picking just one) because which raw axis actually captures this
// mounting's real hook-swing plane is unmeasured - see "SECOND UPDATE" in
// the file header for why one axis alone (tried first) wasn't enough.
// Re-derived every idle tick (updateAllRefForwards(), called alongside
// refUp's own adaptation) so all 3 track refUp's drift instead of
// freezing at calibration.
float refForwardX[3] = {1.0, 0.0, 0.0};
float refForwardY[3] = {0.0, 1.0, 0.0};
float refForwardZ[3] = {0.0, 0.0, 1.0};

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

// NEW (2026-10-30, real-data-driven) - raw accel summed over the WHOLE
// capture window (every sample, not just the peak), so classifyAndPrint()
// can compute a window-AVERAGED signed angle as an alternative to the
// single-peak-instant one. Real ordered-throw data showed the peak-instant
// signedAngleX only reads correctly for an unusually "super straight"
// hook - anything less exaggerated (including every intended 45-degree
// diagonal throw) collapsed toward the uppercut range. Same underlying
// pattern as the two already-abandoned gyro approaches (vertImpact/
// vertSwing above): a single sampled instant keeps getting dominated by
// the terminal wrist-snap at impact rather than reflecting the actual
// swing-plane geometry. Averaging over the whole window is the same
// "average several samples instead of trusting one" reasoning calibrate()
// already uses, tried here as the next hypothesis before concluding this
// sensor modality has a hard ceiling on resolving more than 2 categories.
float sumCapAx = 0, sumCapAy = 0, sumCapAz = 0;
int capSampleCount = 0;

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

  if (DEBUG_PRINT_ENABLED && now - lastPrintTime >= DEBUG_PRINT_INTERVAL_MS) {
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

      // NEW - keep all 3 candidate refForward axes in lockstep with refUp's
      // own drift, same reason refUp itself needs to adapt (a frozen
      // reference drifts out of orthogonality with a moving one - confirmed
      // as a real problem on real data, 2026-10-30). Mags not needed here
      // (only useful as a one-time calibration diagnostic, see calibrate()).
      updateAllRefForwards(NULL);
    }

    if (signal > PUNCH_THRESHOLD_G && (now - lastPunchTime >= PUNCH_COOLDOWN_MS)) {
      state = CAPTURING;
      captureStartTime = now;
      peakSignal = signal;
      peakAx = ax; peakAy = ay; peakAz = az;
      peakGx = gx; peakGy = gy; peakGz = gz;
      peakGyroMag = gMagSample;
      swingGx = gx; swingGy = gy; swingGz = gz;
      sumCapAx = ax; sumCapAy = ay; sumCapAz = az;  // first sample of the window average
      capSampleCount = 1;
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
  // Accumulate EVERY sample in the window (not just peak updates) for the
  // window-averaged signed angle - see sumCapAx/capSampleCount's comment.
  sumCapAx += ax; sumCapAy += ay; sumCapAz += az;
  capSampleCount++;

  unsigned long elapsed = now - captureStartTime;
  bool decayed = signal < PUNCH_THRESHOLD_G && elapsed >= CAPTURE_MIN_MS;
  bool timedOut = elapsed >= CAPTURE_MAX_MS;
  if (decayed || timedOut) {
    // Accept/reject gate - see PUNCH_ACCEPT_G's comment. A capture that
    // opened (crossed the low PUNCH_THRESHOLD_G) but never reached real-
    // punch strength is silently dropped, not printed, and does NOT reset
    // lastPunchTime/the cooldown - a weak non-punch blip shouldn't be able
    // to block a real punch that follows shortly after.
    if (peakSignal >= PUNCH_ACCEPT_G) {
      classifyAndPrint(elapsed);
      lastPunchTime = now;
    }
    state = IDLE;
  }
}

// Runs once per punch, on the single peak sample captured above. See the
// file header's "HOW THE CLASSIFIER WORKS" / "WHAT'S NEW IN THIS FILE"
// sections for the physical reasoning behind each field.
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

  // NEW (this file) - signed angle, computed once per CANDIDATE refForward
  // (index 0/1/2 = seeded from local X/Y/Z - see file header's "SECOND
  // UPDATE") so a single real test shows which candidate axis actually
  // separates hook/uppercut/diagonal, instead of guessing one and
  // re-testing blind. upComponent doesn't depend on which candidate (only
  // refUp is involved); forwardComponent does.
  float upComponent = pax * refUpX + pay * refUpY + paz * refUpZ;
  float signedAngleDeg[3];
  for (int axis = 0; axis < 3; axis++) {
    float forwardComponent = pax * refForwardX[axis] + pay * refForwardY[axis] + paz * refForwardZ[axis];
    signedAngleDeg[axis] = 0.0;
    if (pMag > 0.05) {
      signedAngleDeg[axis] = atan2(-upComponent, forwardComponent) * RAD_TO_DEG;
    }
  }

  // NEW - window-averaged alternative to the peak-instant signedAngleDeg[0]
  // (X candidate, already confirmed the best of the 3 - see "SECOND
  // UPDATE" above) - see sumCapAx's comment for why. Mirrors calibrate()'s
  // own averaging exactly: sum raw accel over the window, average, THEN
  // subtract gravity - not the other way around.
  float avgCapAx = (capSampleCount > 0) ? sumCapAx / capSampleCount : peakAx;
  float avgCapAy = (capSampleCount > 0) ? sumCapAy / capSampleCount : peakAy;
  float avgCapAz = (capSampleCount > 0) ? sumCapAz / capSampleCount : peakAz;
  float avgPax = avgCapAx - refUpX * restMag;
  float avgPay = avgCapAy - refUpY * restMag;
  float avgPaz = avgCapAz - refUpZ * restMag;
  float avgPMag = sqrt(avgPax * avgPax + avgPay * avgPay + avgPaz * avgPaz);
  float signedAngleAvgX = 0.0;
  if (avgPMag > 0.05) {
    float avgUpComponent = avgPax * refUpX + avgPay * refUpY + avgPaz * refUpZ;
    float avgForwardComponent = avgPax * refForwardX[0] + avgPay * refForwardY[0] + avgPaz * refForwardZ[0];
    signedAngleAvgX = atan2(-avgUpComponent, avgForwardComponent) * RAD_TO_DEG;
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

  // Classification still runs on the old unsigned angleDeg (accel-
  // direction vs refUp) - unchanged, confirmed-good logic, not replaced
  // by signedAngleDeg above. vertSwing/vertImpact are still computed and
  // printed below purely so they can keep being compared against
  // angle-based results.
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
  // drives classification), ms-to-recognize, vertSwing, vertImpact (both
  // comparison-only), peak gyro magnitude, the LIVE refUp this specific
  // punch was judged against, the three candidate PEAK-instant signed
  // angles (seeded from local X/Y/Z - X already confirmed best of the 3),
  // THEN (new) the WINDOW-AVERAGED X candidate - compare the last field
  // against the first of the three signedAngleDeg fields to see whether
  // averaging over the whole capture window (vs. just the peak sample)
  // recovers real separation for throws that aren't an exaggerated,
  // "super straight" hook.
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
  Serial.print(refUpZ, 2);
  Serial.print(',');
  Serial.print(signedAngleDeg[0], 1);
  Serial.print(',');
  Serial.print(signedAngleDeg[1], 1);
  Serial.print(',');
  Serial.print(signedAngleDeg[2], 1);
  Serial.print(',');
  Serial.print(signedAngleAvgX, 1);
  Serial.print(',');
  Serial.println(peakSignal, 2);  // NEW - how far above gravity this capture's peak spike actually
                                    // reached (same units as PUNCH_THRESHOLD_G=1.5). Added because the
                                    // user confirmed (2026-10-30) fast non-punch arm movement was
                                    // false-triggering captures, contaminating every batch so far with
                                    // non-punch data - this exposes the gap (or lack of one) between
                                    // real punches and incidental fast movement so PUNCH_THRESHOLD_G can
                                    // be raised to a REAL measured value instead of another guess.
}

// Recomputes ALL THREE candidate refForward axes (index 0/1/2 = seeded from
// local X/Y/Z) via Gram-Schmidt against the CURRENT refUp - called both
// once from calibrate() and every idle loop tick alongside refUp's own
// adaptation, so all 3 stay orthogonal to refUp even as refUp drifts,
// instead of freezing at whatever they were at calibration. outMag, if
// non-NULL, receives each candidate's PRE-normalization magnitude - small
// means that candidate's seed axis is nearly parallel to refUp right now, a
// degenerate reference for that one specific candidate (the other two are
// unaffected). Pass NULL from loop() where this diagnostic isn't needed
// every tick.
void updateAllRefForwards(float outMag[3]) {
  for (int axis = 0; axis < 3; axis++) {
    float seedX = (axis == 0) ? 1.0 : 0.0;
    float seedY = (axis == 1) ? 1.0 : 0.0;
    float seedZ = (axis == 2) ? 1.0 : 0.0;
    float seedDotUp = seedX * refUpX + seedY * refUpY + seedZ * refUpZ;
    float fx = seedX - seedDotUp * refUpX;
    float fy = seedY - seedDotUp * refUpY;
    float fz = seedZ - seedDotUp * refUpZ;
    float mag = sqrt(fx * fx + fy * fy + fz * fz);
    if (mag > 0.01) {
      fx /= mag;
      fy /= mag;
      fz /= mag;
    }
    refForwardX[axis] = fx;
    refForwardY[axis] = fy;
    refForwardZ[axis] = fz;
    if (outMag != NULL) outMag[axis] = mag;
  }
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

  // NEW (this file) - compute all 3 candidate refForward axes (local
  // X/Y/Z, Gram-Schmidt'd against refUp) - see updateAllRefForwards()'s
  // own comment and the file header's "SECOND UPDATE" for why there are 3
  // candidates instead of one guessed axis.
  float refForwardMag[3];
  updateAllRefForwards(refForwardMag);

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

  // NEW (this file) - diagnostic for all 3 candidate reference axes. If
  // ANY of these three magnitudes prints small (say below ~0.3), that ONE
  // candidate's seed axis is nearly parallel to refUp in THIS calibration
  // pose and will be a noisy/degenerate reference for that candidate only
  // - the other two candidates are unaffected and still worth checking
  // after a real throw batch.
  Serial.print("refForwardMag=(X:");
  Serial.print(refForwardMag[0], 3);
  Serial.print(",Y:");
  Serial.print(refForwardMag[1], 3);
  Serial.print(",Z:");
  Serial.print(refForwardMag[2], 3);
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
