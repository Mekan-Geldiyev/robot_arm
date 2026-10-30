/*
  Wrist IMU Punch Detector - standalone test (GY-521 / MPU-6050)

  Goal: tape a GY-521 to the left wrist brace and detect a punch (a short
  acceleration spike) from raw accelerometer data alone - no external MPU
  library, just Wire.h register reads, so this has the same dependency
  footprint as robot_arm_270.ino's own I2C use.

  Standalone on purpose, same workflow this project already uses for new
  motion code (see punch_angle_270.py / CLAUDE.md: prove it alone first,
  THEN wire it into the live system) - this sketch does NOT touch
  robot_arm_270.ino or the PCA9685/servo code at all. It doesn't even
  #include Adafruit_PWMServoDriver. Once punch detection feels right here,
  the plan is to fold setup()/loop() below into robot_arm_270.ino's own
  setup()/loop() so the Uno drives servos AND reads the wrist IMU at the
  same time - NOT to run this as a second, separate Arduino. That's why
  the MPU6050 (0x68) and the PCA9685 (0x40) are being kept on the same
  bus/pins (A4=SDA, A5=SCL) from the start: two different I2C addresses
  can share one bus with no conflict, so the real robot_arm_270.ino will
  be able to call pwm.begin() and mpuInit() in the same setup() later.

  Wiring: GY-521 VCC->5V, GND->GND, SDA->A4, SCL->A5, AD0 unconnected
  (leaves the I2C address at 0x68 - if AD0 is tied high instead it becomes
  0x69 and MPU_ADDR below would need to change).
*/

#include <Wire.h>

// ==================== TUNABLE SETTINGS ====================

#define PUNCH_THRESHOLD_G   1.5   // magnitude spike (in g, gravity already
                                   // removed) that counts as a punch - raise
                                   // this if light hand motion false-triggers,
                                   // lower it if real punches aren't firing
#define PUNCH_COOLDOWN_MS   300   // minimum time between two separate punch
                                   // reports, so one spike can't be counted
                                   // twice while the arm is still decelerating

#define LOOP_INTERVAL_MS        20   // ~50 Hz sample rate
#define DEBUG_PRINT_INTERVAL_MS 100  // how often the "ax,ay,az,mag" debug
                                     // line gets printed (every loop would
                                     // flood Serial Monitor at 50 Hz)
#define CALIBRATION_SAMPLES     200  // samples averaged at startup while
                                     // the wrist is held still

#define BAUD_RATE 115200  // matches every other sketch/script in this repo

// ============================================================

#define MPU_ADDR        0x68  // GY-521 I2C address with AD0 unconnected/low
#define REG_PWR_MGMT_1  0x6B  // power management - bit 6 (SLEEP) starts set,
                               // the chip is asleep until this is cleared
#define REG_GYRO_CONFIG  0x1B  // bits 4:3 (FS_SEL) pick the gyro's +/-deg/s range
#define REG_ACCEL_CONFIG 0x1C  // bits 4:3 (AFS_SEL) pick the accel's +/-g range
#define REG_ACCEL_XOUT_H 0x3B  // first of 6 contiguous bytes: XH,XL,YH,YL,ZH,ZL

// AFS_SEL=1 -> +/-4g range, which uses 8192 LSB per g (datasheet table:
// +/-2g=16384, +/-4g=8192, +/-8g=4096, +/-16g=2048 LSB/g). Must match
// ACCEL_SENSITIVITY below - they're two halves of the same setting.
#define ACCEL_CONFIG_VALUE 0x08
#define ACCEL_SENSITIVITY  8192.0

// FS_SEL=1 -> +/-500 deg/s range. Gyro data isn't read/used yet (punch
// detection here is accel-only), but the range is set now anyway so the
// register is in a known state before any future code reads GYRO_XOUT etc.
#define GYRO_CONFIG_VALUE 0x08

// Offset removed from every raw accel reading, measured once at startup
// (see calibrateAccel() below) - corrects for this specific sensor's own
// zero-g bias on each axis, NOT for gravity itself (gravity is handled
// separately, in the magnitude step below, since it can't be cancelled
// per-axis without assuming a fixed wrist orientation).
float axOffset = 0.0;
float ayOffset = 0.0;
float azOffset = 0.0;

unsigned long lastLoopTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastPunchTime = 0;  // millis() of the last reported punch,
                                   // 0 = none yet this session
bool punchActive = false;         // true while the current spike is still
                                   // above threshold - keeps one spike from
                                   // being reported more than once as it
                                   // rides above PUNCH_THRESHOLD_G for a
                                   // few consecutive loop() passes

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();

  // REG_PWR_MGMT_1 = 0x00 clears SLEEP (bit 6) and leaves CLKSEL at 0
  // (internal 8MHz oscillator) - this is the standard "wake the chip up"
  // write every MPU6050 init does first, before touching any other register.
  writeRegister(REG_PWR_MGMT_1, 0x00);

  writeRegister(REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE);  // +/-4g
  writeRegister(REG_GYRO_CONFIG, GYRO_CONFIG_VALUE);    // +/-500 deg/s

  calibrateAccel();
}

void loop() {
  unsigned long now = millis();
  if (now - lastLoopTime < LOOP_INTERVAL_MS) return;  // non-blocking ~50Hz gate
  lastLoopTime = now;

  float ax, ay, az;
  readAccelG(ax, ay, az);
  ax -= axOffset;
  ay -= ayOffset;
  az -= azOffset;

  // Vector magnitude of the (bias-corrected) accel reading, then subtract
  // 1g for gravity - a still wrist sits at roughly (0,0,1)g no matter which
  // way it's rotated (gravity is always pulling on *some* combination of
  // the 3 axes with total magnitude 1g), so magnitude-1g reads ~0 at rest
  // and spikes positive under any sudden extra acceleration, regardless of
  // wrist orientation at the moment of the punch.
  float magnitude = sqrt(ax * ax + ay * ay + az * az) - 1.0;

  if (now - lastPrintTime >= DEBUG_PRINT_INTERVAL_MS) {
    lastPrintTime = now;
    Serial.print(ax, 3);
    Serial.print(',');
    Serial.print(ay, 3);
    Serial.print(',');
    Serial.print(az, 3);
    Serial.print(',');
    Serial.println(magnitude, 3);
  }

  // Latch so a single spike (which stays above threshold for several
  // consecutive loop() passes while it rises/falls) reports exactly once,
  // and PUNCH_COOLDOWN_MS blocks a second, separate spike from counting if
  // it arrives unrealistically soon after the last one.
  if (magnitude > PUNCH_THRESHOLD_G) {
    if (!punchActive && (now - lastPunchTime >= PUNCH_COOLDOWN_MS)) {
      Serial.println("PUNCH");
      lastPunchTime = now;
    }
    punchActive = true;
  } else {
    punchActive = false;
  }
}

// Averages CALIBRATION_SAMPLES readings while the wrist is held still and
// stores the result as axOffset/ayOffset/azOffset. Blocking is fine here -
// this only runs once in setup(), not in the non-blocking loop() above.
void calibrateAccel() {
  float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float ax, ay, az;
    readAccelG(ax, ay, az);
    sumX += ax;
    sumY += ay;
    sumZ += az;
    delay(LOOP_INTERVAL_MS);
  }
  axOffset = sumX / CALIBRATION_SAMPLES;
  ayOffset = sumY / CALIBRATION_SAMPLES;
  azOffset = sumZ / CALIBRATION_SAMPLES;
  Serial.println("calibrated");
}

// Reads the 6 raw accel bytes starting at REG_ACCEL_XOUT_H (X high, X low,
// Y high, Y low, Z high, Z low) and converts each axis to g by combining
// the high/low bytes into a signed 16-bit value, then dividing by
// ACCEL_SENSITIVITY (8192 LSB/g at the +/-4g range set in setup()).
void readAccelG(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);  // false = keep the bus held for the read that follows
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  ax = rawX / ACCEL_SENSITIVITY;
  ay = rawY / ACCEL_SENSITIVITY;
  az = rawZ / ACCEL_SENSITIVITY;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}
