/* =============================================================================
   SELF-BALANCING ROBOT - MPU6050 + L298N + PID + Bluetooth (ESP32)
   =============================================================================
   Balances a two-wheeled robot with a PID controller fed by an MPU6050 IMU and
   lets you drive it from a phone over classic Bluetooth Serial (for example the
   "Arduino Bluetooth Controller" app in pad mode).

   Robot-specific settings (pins, motor directions, IMU orientation) live in
   config.h. Tunable values are changed over USB Serial (115200 baud): type
   "help" for the command list and "save" to keep them after a reboot.

   Bluetooth phone app (pair with "DengeRobot"):
     W = forward   S = backward   A = left   D = right   L = stop
   ========================================================================= */

#include <Wire.h>

#include "BluetoothSerial.h"
#include "battery.h"
#include "command.h"
#include "config.h"
#include "control.h"
#include "imu.h"
#include "motors.h"
#include "settings.h"
#include "settings_store.h"
#include "tuning.h"

BluetoothSerial SerialBT;
Mpu6050 imu;

// Live-tunable values (gains, base angle, drive limits). See settings.h for
// defaults and ranges; change them over Serial, keep them with "save".
//   Kp: bigger = reacts harder to the current tilt error
//   Ki: bigger = removes a steady lean faster, too high = slow oscillation
//   Kd: bigger = damps fast tilt changes, too high = twitchy/noisy
// Tune in this order: raise Kp until it balances but wobbles, add Kd to
// damp the wobble, then add a small amount of Ki to kill any steady lean.
Settings settings;
LineReader serialLine;

// --- Angle estimation -------------------------------------------------------
enum class ImuRead { Ok, Skip, Fault };
ComplementaryFilter tiltFilter;
GyroCalibrator gyroCalibrator;
bool calibratingGyro = true;  // the first second after boot: keep the robot still
float gyroBias = 0.0f;        // deg/s read by the gyro at rest
int imuReadErrors = 0;
bool imuFault = false;
uint32_t lastImuRetryMs = 0;

float currentAngle = 0.0f;  // filtered tilt angle, deg
float pitchRate = 0.0f;     // bias-corrected tilt rate, deg/s
float targetAngle = 0.0f;   // angle the PID is chasing right now

// --- Balance PID ------------------------------------------------------------
float error = 0.0f, prevError = 0.0f;
float integral = 0.0f;
float pidOutput = 0.0f;

// --- Auto-trim (drift correction) -------------------------------------------
// If the robot consistently drifts one way while "balanced", baseAngle is not
// quite the real balance point. Auto-trim watches the long-term average PID
// output and, if enabled, slowly nudges baseAngle to make it zero.
const float trimRate = 0.0005f;  // per control step. Keep VERY small.
float outputAvg = 0.0f;          // slow rolling average of pidOutput

// --- Bluetooth drive control ------------------------------------------------
// Driving forward/backward gently LEANS the balance target angle; the balance
// PID then rolls the wheels to chase that lean, which gives smooth
// acceleration. Turning gives the two wheels small opposite offsets around the
// shared PID output, which changes yaw only, so it never fights the balance.
float moveTilt = 0.0f;  // current forward/backward lean added to baseAngle
float turnBias = 0.0f;  // current left/right wheel speed difference

char btCommand = 'L';
uint32_t lastBtCommandMs = 0;

// --- Fixed-rate control loop ------------------------------------------------
uint32_t lastControlUs = 0;
uint32_t nextControlUs = 0;

void setup() {
  // A bigger TX buffer lets status prints return at once instead of waiting
  // for the UART, which used to stretch some control loop iterations.
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(500);

  if (loadSettings(settings)) {
    Serial.println("Saved settings loaded.");
  } else {
    Serial.println("No saved settings, using defaults.");
  }

  motorsBegin();
  batteryBegin();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);  // a broken wire must not stall the loop
  if (imu.begin(Wire, IMU_I2C_ADDRESS, IMU_LOW_PASS_CONFIG)) {
    Serial.printf("IMU found (WHO_AM_I = 0x%02X).\n", imu.whoAmI());
  } else {
    imuFault = true;  // keep retrying in the loop; motors stay off meanwhile
    Serial.println("! IMU (MPU6050) not found. Check SDA/SCL/VCC/GND; retrying...");
  }
  startGyroCalibration();

  SerialBT.begin("DengeRobot");  // Bluetooth device name - pair your phone to this
  Serial.println("Bluetooth is active. Pair your phone with 'DengeRobot'.");

  Serial.println("\n=== Self-Balancing Robot Started ===");
  printHelp(Serial);
  Serial.println("Bluetooth commands: W forward, S backward, A left, D right, L stop");

  lastControlUs = micros();
  nextControlUs = lastControlUs + CONTROL_PERIOD_US;
}

void loop() {
  // Everything here returns immediately; only the control step is timed.
  readSerialCommands();
  readBluetoothCommands();
  batteryPoll();

  const uint32_t now = micros();
  if (static_cast<int32_t>(now - nextControlUs) >= 0) {
    // Real elapsed time, limited so one slow iteration cannot kick the
    // integrators (the old loop could see dt = 1 s after a blocking read).
    const float dt = clampf((now - lastControlUs) * 1e-6f, 0.001f, 0.02f);
    lastControlUs = now;
    nextControlUs += CONTROL_PERIOD_US;
    if (static_cast<int32_t>(now - nextControlUs) >= 0) nextControlUs = now + CONTROL_PERIOD_US;
    controlStep(dt);
  }

  logStatus();
}

// ---------------------------------------------------------------------------
// One 200 Hz control step: read the IMU, estimate the angle, run the PID.
// ---------------------------------------------------------------------------
void controlStep(float dt) {
  ImuSample sample;
  switch (readImu(sample)) {
    case ImuRead::Ok: break;
    case ImuRead::Skip: return;  // one glitch: keep the last motor command
    case ImuRead::Fault:
      motorsStop();
      resetBalance();
      return;
  }

  const float rawRate = pitchRateFromGyro(sample.gx, sample.gy, sample.gz);
  if (calibratingGyro) {
    motorsStop();
    runGyroCalibration(rawRate);
    return;
  }

  pitchRate = rawRate - gyroBias;
  currentAngle = tiltFilter.update(pitchFromAccel(sample.ax, sample.ay, sample.az), pitchRate,
                                   dt, IMU_FILTER_TIME_CONSTANT_S);
  updateDriveRamps(dt);
  targetAngle = settings.baseAngle + moveTilt;

  // Safety cutoff: tipped over too far to recover (picked up or already
  // fallen) or the battery is empty: stop instead of fighting the ground.
  if (fabsf(currentAngle - targetAngle) > FALL_ANGLE_DEG || batteryLow()) {
    motorsStop();
    resetBalance();
    return;
  }

  // Positive error = leaning forward past the target, and a positive output
  // drives forward: the wheels roll under the falling body.
  error = currentAngle - targetAngle;
  integral = clampf(integral + error * dt, -60.0f, 60.0f);  // anti-windup clamp
  const float derivative = (error - prevError) / dt;
  prevError = error;
  pidOutput = settings.kp * error + settings.ki * integral + settings.kd * derivative;
  motorsDrive(pidOutput - turnBias, pidOutput + turnBias, batteryCompensation());

  outputAvg = outputAvg * 0.999f + pidOutput * 0.001f;
  if (settings.autoTrim) {
    settings.baseAngle += outputAvg * trimRate;
  }
}

void resetBalance() {
  integral = 0.0f;
  prevError = 0.0f;
  pidOutput = 0.0f;
}

// ---------------------------------------------------------------------------
// IMU reading with error handling: a single failed read keeps the previous
// motor command; several in a row stop the motors until the sensor answers.
// ---------------------------------------------------------------------------
ImuRead readImu(ImuSample& sample) {
  if (imuFault) {
    if (millis() - lastImuRetryMs < IMU_RETRY_MS) return ImuRead::Fault;
    lastImuRetryMs = millis();
    if (!imu.begin(Wire, IMU_I2C_ADDRESS, IMU_LOW_PASS_CONFIG)) return ImuRead::Fault;
    Serial.println("-> IMU answers again.");
    imuFault = false;
    imuReadErrors = 0;
    tiltFilter.reset();
  }
  if (imu.read(sample)) {
    imuReadErrors = 0;
    return ImuRead::Ok;
  }
  if (++imuReadErrors < IMU_MAX_READ_ERRORS) return ImuRead::Skip;
  imuFault = true;
  lastImuRetryMs = millis();
  Serial.println("! IMU stopped answering: motors off. Check the SDA/SCL wires.");
  return ImuRead::Fault;
}

void startGyroCalibration() {
  gyroCalibrator.reset();
  calibratingGyro = true;
  Serial.println("Calibrating gyro: keep the robot still for a second...");
}

void runGyroCalibration(float rawRate) {
  gyroCalibrator.add(rawRate);
  if (gyroCalibrator.samples < GYRO_CALIBRATION_SAMPLES) return;
  if (!gyroCalibrator.stillEnough(GYRO_STILL_SPREAD_DPS)) {
    Serial.println("! The robot moved during gyro calibration. Keep it still; retrying...");
    gyroCalibrator.reset();
    return;
  }
  gyroBias = gyroCalibrator.bias();
  calibratingGyro = false;
  tiltFilter.reset();
  Serial.printf("-> Gyro calibrated: offset %.2f deg/s.\n", gyroBias);
}

void logStatus() {
  static uint32_t lastLogMs = 0;
  if (millis() - lastLogMs < 100) return;
  lastLogMs = millis();
  Serial.printf("Angle: %.2f | Target: %.2f | OutAvg: %.2f | Kp: %.2f | Ki: %.2f | Kd: %.2f | PID: %.2f\n",
                currentAngle, targetAngle, outputAvg, settings.kp, settings.ki, settings.kd, pidOutput);
}

// ---------------------------------------------------------------------------
// USB Serial commands. Characters are collected into a line without waiting,
// so typing a command never pauses the balance loop (Serial.parseFloat()
// used to block for up to 1 s and the robot fell over).
// ---------------------------------------------------------------------------
void handleCommandLine(const char* line) {
  Command cmd;
  switch (parseCommand(line, cmd)) {
    case ParseStatus::Empty: return;
    case ParseStatus::BadName:
    case ParseStatus::BadValue:
      Serial.printf("! Cannot understand '%s'. Type 'help'.\n", line);
      return;
    case ParseStatus::Ok: break;
  }

  if (strcmp(cmd.name, "calib") == 0) {
    startGyroCalibration();
    return;
  }

  const float oldKi = settings.ki;
  const bool oldAutoTrim = settings.autoTrim;
  if (!handleSettingsCommand(cmd, settings, Serial)) {
    Serial.printf("! Unknown command '%s'. Type 'help'.\n", cmd.name);
    return;
  }
  if (settings.ki != oldKi) integral = 0.0f;  // a big Ki change must not cause a kick
  if (settings.autoTrim != oldAutoTrim) outputAvg = 0.0f;
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    switch (serialLine.push(static_cast<char>(Serial.read()))) {
      case LineStatus::Ready: handleCommandLine(serialLine.line()); break;
      case LineStatus::Overflow: Serial.println("! Line too long, ignored."); break;
      case LineStatus::Pending: break;
    }
  }
}

// ---------------------------------------------------------------------------
// Bluetooth drive commands. A pad-style phone app re-sends the held key while
// it is pressed and sends 'L' on release. The last recognised command is kept;
// if nothing valid arrives for remoteTimeoutMs, it counts as released.
// ---------------------------------------------------------------------------
void readBluetoothCommands() {
  while (SerialBT.available() > 0) {
    const char c = static_cast<char>(toupper(SerialBT.read()));
    if (c == 'W' || c == 'S' || c == 'A' || c == 'D' || c == 'L') {
      btCommand = c;
      lastBtCommandMs = millis();
    }
  }
  if (millis() - lastBtCommandMs > settings.remoteTimeoutMs) {
    btCommand = 'L';  // connection dropped or button released: stop driving
  }
}

// Ramps the drive lean and the turn offset toward the current command at a
// constant rate per second, so the balance loop is never jolted.
void updateDriveRamps(float dt) {
  float tiltTarget = 0.0f;
  if (btCommand == 'W') tiltTarget = settings.moveTiltMax;
  else if (btCommand == 'S') tiltTarget = -settings.moveTiltMax;
  moveTilt = rampToward(moveTilt, tiltTarget, settings.moveTiltMax / settings.tiltRampTime * dt);

  float turnTarget = 0.0f;
  if (btCommand == 'D') turnTarget = settings.turnMax;
  else if (btCommand == 'A') turnTarget = -settings.turnMax;
  turnBias = rampToward(turnBias, turnTarget, settings.turnMax / settings.turnRampTime * dt);
}
