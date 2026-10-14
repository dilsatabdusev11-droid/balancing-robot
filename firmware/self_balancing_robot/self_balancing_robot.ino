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
     W = forward   S = backward   A = left   D = right   L = stop driving
     X = emergency stop (motors off)   G = go (allow balancing again)

   The motors start only after the robot is held upright for half a second.
   ========================================================================= */

#include <Wire.h>

#include "BluetoothSerial.h"
#include "balance.h"
#include "battery.h"
#include "command.h"
#include "config.h"
#include "control.h"
#include "drive_math.h"
#include "encoders.h"
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

// --- Balance ----------------------------------------------------------------
BalancePid pid;
AutoTrim autoTrim;  // learns a small correction of baseAngle (see balance.h)
Arming arming;      // decides when the motors may run
MotorTest motorTest;
bool stopRequested = false;  // emergency stop from Serial ("stop") or the phone (X)
float pidOutput = 0.0f;

constexpr ArmingConfig kArmingConfig = {ARM_ANGLE_DEG, ARM_HOLD_S, FALL_ANGLE_DEG};

// --- Speed loop (only with wheel encoders) ------------------------------------
constexpr float kMetersPerCount = 3.14159265f * WHEEL_DIAMETER_M / ENCODER_COUNTS_PER_WHEEL_REV;
WheelSpeed wheelSpeed;
SpeedLoop speedLoop;
float wantedSpeed = 0.0f;  // m/s, ramped toward the phone command
float speedLean = 0.0f;    // lean the speed loop asks for, deg

// --- Bluetooth drive control ------------------------------------------------
// Driving forward/backward gently LEANS the balance target angle; the balance
// PID then rolls the wheels to chase that lean, which gives smooth
// acceleration. With encoders the phone sets a speed instead and the speed
// loop picks the lean. Turning gives the two wheels small opposite offsets around the
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
  encodersBegin();

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
  Serial.println("Bluetooth commands: W forward, S backward, A left, D right, L stop driving,");
  Serial.println("                    X emergency stop, G go");

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
// One 200 Hz control step: read the IMU, estimate the angle, decide whether
// the motors may run, run the PID.
// ---------------------------------------------------------------------------
void controlStep(float dt) {
  ImuSample sample;
  const ImuRead imuStatus = readImu(sample);
  if (imuStatus == ImuRead::Skip) return;  // one glitch: keep the last motor command

  if (imuStatus == ImuRead::Ok) {
    const float rawRate = pitchRateFromGyro(sample.gx, sample.gy, sample.gz);
    if (calibratingGyro) {
      runGyroCalibration(rawRate);
    } else {
      pitchRate = rawRate - gyroBias;
      currentAngle = tiltFilter.update(pitchFromAccel(sample.ax, sample.ay, sample.az), pitchRate,
                                       dt, IMU_FILTER_TIME_CONSTANT_S);
    }
  }

  updateDriveRamps(dt);
  const float uprightAngle = settings.baseAngle + autoTrim.offset;
  if (ENCODERS_ENABLED) {
    int32_t leftCounts = 0, rightCounts = 0;
    encodersTake(leftCounts, rightCounts);
    wheelSpeed.update(leftCounts, rightCounts, dt, kMetersPerCount, SPEED_FILTER_S);
    speedLean = speedLoop.update(wantedSpeed, wheelSpeed.speed, dt, settings.speedKp,
                                 settings.speedKi, SPEED_TILT_LIMIT_DEG);
    targetAngle = uprightAngle + speedLean;
  } else {
    targetAngle = uprightAngle + moveTilt;
  }

  const RobotState previous = arming.state;
  const ArmingInputs inputs = {imuStatus == ImuRead::Ok, calibratingGyro,
                               stopRequested || motorTest.running(), batteryLow(),
                               currentAngle - uprightAngle};
  const RobotState state = arming.update(inputs, dt, kArmingConfig);
  if (state != previous) onStateChanged(state);

  if (motorTest.running()) {
    float left = 0.0f, right = 0.0f;
    if (motorTest.step(dt, MOTOR_TEST_COMMAND, left, right)) {
      motorsDrive(left, right, 1.0f);
    } else {
      motorsStop();
      Serial.println("-> Motor test done. Each wheel should have rolled the robot FORWARD.");
    }
    return;
  }

  if (state != RobotState::Balancing) {
    motorsStop();
    return;
  }

  // Positive error = leaning forward past the target, and a positive output
  // drives forward: the wheels roll under the falling body.
  const PidGains gains = {settings.kp, settings.ki, settings.kd};
  pidOutput = pid.update(currentAngle - targetAngle, pitchRate, dt, gains, PID_INTEGRAL_LIMIT,
                         kMaxCommand);
  motorsDrive(pidOutput - turnBias, pidOutput + turnBias, batteryCompensation());
  // With encoders the speed loop's integral already finds the balance point.
  autoTrim.update(pidOutput, dt, settings.autoTrim && !ENCODERS_ENABLED && btCommand == 'L',
                  AUTOTRIM_RATE, AUTOTRIM_AVERAGE_S, AUTOTRIM_LIMIT_DEG);
}

void onStateChanged(RobotState state) {
  pid.reset();  // every (re)start of balancing begins from a clean controller
  speedLoop.reset();
  pidOutput = 0.0f;
  speedLean = 0.0f;
  Serial.printf("-> State: %s\n", stateName(state));
  if (state == RobotState::Waiting) {
    Serial.println("   Hold the robot upright: the motors start after half a second.");
  }
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
  Serial.printf("%s | Angle: %.2f | Target: %.2f | Trim: %.2f | PID: %.2f | Speed: %.2f\n",
                stateName(arming.state), currentAngle, targetAngle, autoTrim.offset, pidOutput,
                wheelSpeed.speed);
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

  if (handleActionCommand(cmd.name)) return;

  // "save" keeps the learned auto-trim correction as part of baseAngle.
  if (strcmp(cmd.name, "save") == 0) {
    settings.baseAngle = clampf(settings.baseAngle + autoTrim.offset, -45.0f, 45.0f);
    autoTrim.reset();
  }
  const float oldKi = settings.ki;
  const float oldBase = settings.baseAngle;
  const bool oldAutoTrim = settings.autoTrim;
  if (!handleSettingsCommand(cmd, settings, Serial)) {
    Serial.printf("! Unknown command '%s'. Type 'help'.\n", cmd.name);
    return;
  }
  if (settings.ki != oldKi) pid.reset();  // a big Ki change must not cause a kick
  if (settings.baseAngle != oldBase || settings.autoTrim != oldAutoTrim) autoTrim.reset();
}

// Commands that act on the robot rather than change a setting.
bool handleActionCommand(const char* name) {
  const bool balancing = arming.state == RobotState::Balancing;
  if (strcmp(name, "stop") == 0) {
    requestStop(true);
  } else if (strcmp(name, "go") == 0) {
    requestStop(false);
  } else if (strcmp(name, "calib") == 0) {
    if (balancing) {
      Serial.println("! Stop the robot first ('stop'), then 'calib'.");
    } else {
      startGyroCalibration();
    }
  } else if (strcmp(name, "zero") == 0) {
    if (balancing) {
      Serial.println("! Stop the robot first ('stop'), hold it at its balance point, then 'zero'.");
    } else {
      settings.baseAngle = clampf(currentAngle, -45.0f, 45.0f);
      autoTrim.reset();
      Serial.printf("-> base = %.2f (current angle). Type 'save' to keep it.\n", settings.baseAngle);
    }
  } else if (strcmp(name, "encoders") == 0) {
    if (!ENCODERS_ENABLED) {
      Serial.println("! Encoders are off: set ENCODERS_ENABLED in config.h.");
    } else {
      int32_t left = 0, right = 0;
      encodersReadTotals(left, right);
      Serial.printf("-> Counts since last check: left %ld, right %ld (one wheel turn = %.0f).\n",
                    static_cast<long>(left), static_cast<long>(right), ENCODER_COUNTS_PER_WHEEL_REV);
      Serial.println("   Roll the robot FORWARD and check again: both must be positive.");
    }
  } else if (strcmp(name, "motortest") == 0) {
    if (balancing) {
      Serial.println("! Stop the robot first ('stop') and hold it with the wheels in the air.");
    } else {
      Serial.println("-> Motor test: left wheel, pause, right wheel.");
      motorTest.start();
    }
  } else {
    return false;
  }
  return true;
}

void requestStop(bool stop) {
  if (stop == stopRequested) return;
  stopRequested = stop;
  Serial.println(stop ? "-> Emergency stop: motors off. 'go' (or G on the phone) to continue."
                      : "-> Go: hold the robot upright to start balancing.");
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
    } else if (c == 'X') {
      requestStop(true);
    } else if (c == 'G') {
      requestStop(false);
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

  float speedTarget = 0.0f;
  if (btCommand == 'W') speedTarget = settings.maxSpeed;
  else if (btCommand == 'S') speedTarget = -settings.maxSpeed;
  wantedSpeed = rampToward(wantedSpeed, speedTarget, settings.maxSpeed / settings.tiltRampTime * dt);

  float turnTarget = 0.0f;
  if (btCommand == 'D') turnTarget = settings.turnMax;
  else if (btCommand == 'A') turnTarget = -settings.turnMax;
  turnBias = rampToward(turnBias, turnTarget, settings.turnMax / settings.turnRampTime * dt);
}
