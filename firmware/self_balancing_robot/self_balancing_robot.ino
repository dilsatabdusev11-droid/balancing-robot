/* =============================================================================
   SELF-BALANCING ROBOT - MPU6050 + L298N/TB6612 + PID + phone remote (ESP32)
   =============================================================================
   Balances a two-wheeled robot with a PID controller fed by an MPU6050 IMU and
   lets you drive it from a phone: classic Bluetooth with a pad app, or a Wi-Fi
   web page (REMOTE_MODE in config.h).

   Robot-specific settings (pins, motor directions, IMU orientation, remote)
   live in config.h. Tunable values are changed over USB Serial (115200 baud):
   type "help" for the command list and "save" to keep them after a reboot.

   Phone commands (the robot's name is BalanceBot-XXXX, printed at boot):
     W = forward   S = backward   A = left   D = right   L = stop driving
     X = emergency stop (motors off)   G = go (allow balancing again)

   The motors start only after the robot is held upright for half a second.
   ========================================================================= */

#include <Wire.h>

#include "balance.h"
#include "battery.h"
#include "command.h"
#include "config.h"
#include "control.h"
#include "drive_math.h"
#include "encoders.h"
#include "imu.h"
#include "motors.h"
#include "remote.h"
#include "settings.h"
#include "settings_store.h"
#include "tuning.h"

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

// --- Phone drive control ----------------------------------------------------
// Driving forward/backward gently LEANS the balance target angle; the balance
// PID then rolls the wheels to chase that lean, which gives smooth
// acceleration. With encoders the phone sets a speed instead and the speed
// loop picks the lean. Turning gives the two wheels small opposite offsets around the
// shared PID output, which changes yaw only, so it never fights the balance.
float moveTilt = 0.0f;  // current forward/backward lean added to baseAngle
float turnBias = 0.0f;  // current left/right wheel speed difference

char driveCommand = 'L';  // last W/S/A/D/L from the phone
uint32_t lastDriveCommandMs = 0;

// --- Serial status output ("log 0/1/2") ------------------------------------------
enum class LogMode { Off, Text, Plotter };
LogMode logMode = LogMode::Text;

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

  remoteBegin(robotName(), handleRemoteCommand);

  Serial.println("\n=== Self-Balancing Robot Started ===");
  printHelp(Serial);
  Serial.println("Phone commands: W forward, S backward, A left, D right, L stop driving,");
  Serial.println("                X emergency stop, G go");

  lastControlUs = micros();
  nextControlUs = lastControlUs + CONTROL_PERIOD_US;
}

void loop() {
  // Everything here returns immediately; only the control step is timed.
  readSerialCommands();
  remotePoll({stateName(arming.state), currentAngle, wheelSpeed.speed, batteryVolts()});
  checkRemoteTimeout();
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
  // Encoder counts are taken every step, even when the IMU read is skipped,
  // so the speed is always counts over the same dt.
  if (ENCODERS_ENABLED) {
    int32_t leftCounts = 0, rightCounts = 0;
    encodersTake(leftCounts, rightCounts);
    wheelSpeed.update(leftCounts, rightCounts, dt, kMetersPerCount, SPEED_FILTER_S);
  }

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
  autoTrim.update(pidOutput, dt, settings.autoTrim && !ENCODERS_ENABLED && driveCommand == 'L',
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
  const uint32_t period = logMode == LogMode::Plotter ? 50 : 100;
  if (logMode == LogMode::Off || millis() - lastLogMs < period) return;
  lastLogMs = millis();
  if (logMode == LogMode::Plotter) {
    // Arduino IDE Serial Plotter format: name:value pairs, one line per sample.
    Serial.printf("angle:%.2f,target:%.2f,output:%.1f,rate:%.1f,speed:%.2f,battery:%.2f\n",
                  currentAngle, targetAngle, pidOutput / 10.0f, pitchRate / 10.0f,
                  wheelSpeed.speed, batteryVolts());
    return;
  }
  Serial.printf("%s | Angle: %.2f | Target: %.2f | Trim: %.2f | PID: %.2f | Speed: %.2f\n",
                stateName(arming.state), currentAngle, targetAngle, autoTrim.offset, pidOutput,
                wheelSpeed.speed);
}

// ---------------------------------------------------------------------------
// Phone commands. A pad app (or the web page) repeats the held key while it is
// pressed and sends 'L' on release. The last drive command is kept; if nothing
// arrives for remoteTimeoutMs it counts as released.
// ---------------------------------------------------------------------------
void handleRemoteCommand(char command) {
  switch (command) {
    case 'W':
    case 'S':
    case 'A':
    case 'D':
    case 'L':
      driveCommand = command;
      lastDriveCommandMs = millis();
      break;
    case 'X': requestStop(true); break;
    case 'G': requestStop(false); break;
    default: break;  // apps also send other characters: ignore them
  }
}

void checkRemoteTimeout() {
  if (millis() - lastDriveCommandMs > settings.remoteTimeoutMs) {
    driveCommand = 'L';  // connection dropped or button released: stop driving
  }
}

// BalanceBot-XXXX, XXXX = last two bytes of this chip's MAC address.
const char* robotName() {
  static char name[32];
  const uint64_t mac = ESP.getEfuseMac();
  snprintf(name, sizeof(name), "%s-%02X%02X", ROBOT_NAME_PREFIX,
           static_cast<unsigned>((mac >> 32) & 0xFF), static_cast<unsigned>((mac >> 40) & 0xFF));
  return name;
}

// Ramps the drive lean and the turn offset toward the current command at a
// constant rate per second, so the balance loop is never jolted.
void updateDriveRamps(float dt) {
  float tiltTarget = 0.0f;
  if (driveCommand == 'W') tiltTarget = settings.moveTiltMax;
  else if (driveCommand == 'S') tiltTarget = -settings.moveTiltMax;
  moveTilt = rampToward(moveTilt, tiltTarget, settings.moveTiltMax / settings.tiltRampTime * dt);

  float speedTarget = 0.0f;
  if (driveCommand == 'W') speedTarget = settings.maxSpeed;
  else if (driveCommand == 'S') speedTarget = -settings.maxSpeed;
  wantedSpeed = rampToward(wantedSpeed, speedTarget, settings.maxSpeed / settings.tiltRampTime * dt);

  float turnTarget = 0.0f;
  if (driveCommand == 'D') turnTarget = settings.turnMax;
  else if (driveCommand == 'A') turnTarget = -settings.turnMax;
  turnBias = rampToward(turnBias, turnTarget, settings.turnMax / settings.turnRampTime * dt);
}
