/*
  USB/hardware isolation test - continuous "hand circle" animation.

  This sketch does NOT use Serial at all - no Serial.begin(), no reading
  commands, nothing. Once uploaded, the pan/tilt servos trace a smooth
  circle forever using only values baked into this code.

  Why this test matters: if the arm traces a clean, uninterrupted circle
  for several minutes straight with zero glitches, that proves the
  PCA9685 + servos + wiring + power are completely solid. Any remaining
  flakiness (uploads needing a USB replug, "test"/"0,90" not registering
  in Serial Monitor) is then isolated specifically to the serial DATA
  link (cable/port/driver) - not the robot hardware itself.

  Upload this once, then just watch it run. You can also try swapping
  USB cables while it's running - if a different cable makes the NEXT
  upload/Serial Monitor session behave better, that confirms the cable.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150   // pulse length count for 0 degrees
#define SERVOMAX 600   // pulse length count for 180 degrees
#define PWM_FREQ 60

#define PAN_CHANNEL  0
#define TILT_CHANNEL 1

// ==================== CIRCLE SHAPE - TUNE THESE ====================
// Pan/tilt will swing CENTER +/- RADIUS. Widen these once you've watched
// the first run and confirmed the bracket has room to spare.
#define PAN_CENTER    90
#define PAN_RADIUS    30   // -> pan sweeps 50 to 130

#define TILT_CENTER   90
#define TILT_RADIUS   40   // -> tilt sweeps 60 to 120

#define STEP_DELAY_MS   20    // ms between steps - smaller = faster circle
#define ANGLE_STEP_DEG  2.0   // degrees advanced around the circle per step
// =====================================================================

float angleDeg = 0;

void setup() {
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQ);

  // Ease to the circle's starting point before the loop takes over.
  moveServo(PAN_CHANNEL, PAN_CENTER + PAN_RADIUS);
  moveServo(TILT_CHANNEL, TILT_CENTER);
  delay(500);
}

void loop() {
  float rad = angleDeg * PI / 180.0;
  int pan  = PAN_CENTER  + (int)(PAN_RADIUS  * cos(rad));
  int tilt = TILT_CENTER + (int)(TILT_RADIUS * sin(rad));

  moveServo(PAN_CHANNEL, pan);
  moveServo(TILT_CHANNEL, tilt);

  angleDeg += ANGLE_STEP_DEG;
  if (angleDeg >= 360.0) angleDeg -= 360.0;

  delay(STEP_DELAY_MS);
}

void moveServo(uint8_t channel, int angleDegVal) {
  angleDegVal = constrain(angleDegVal, 0, 180);
  int pulse = map(angleDegVal, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}
