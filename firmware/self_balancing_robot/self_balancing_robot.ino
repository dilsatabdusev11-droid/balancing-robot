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

#include "config.h"
#include "command.h"
#include "settings.h"
#include "settings_store.h"
#include "tuning.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;

// Live-tunable values (gains, base angle, drive limits). See settings.h for
// defaults and ranges; change them over Serial, keep them with "save".
//   Kp: bigger = reacts harder to the current tilt error
//   Ki: bigger = removes a steady lean faster, too high = slow oscillation
//   Kd: bigger = damps fast tilt changes, too high = twitchy/noisy
// Tune in this order: raise Kp until it balances but wobbles, add Kd to
// damp the wobble, then add a small amount of Ki to kill any steady lean.
Settings settings;
LineReader serialLine;

double targetAngle = 0.0;            // actual angle the PID is chasing right now
double currentAngle = 0.0;            // filtered current tilt angle (complementary filter)
double error = 0.0, prevError = 0.0;
double integral = 0.0, derivative = 0.0;
double pidOutput = 0.0;

// ---------------------------------------------------------------------------
// AUTO-TRIM (drift correction)
// ---------------------------------------------------------------------------
// If the robot consistently drifts/leans one way even while "balanced",
// it usually means settings.baseAngle isn't perfectly matched to the real
// mechanical balance point. Auto-trim watches the long-term average PID
// output and, if enabled, very slowly nudges settings.baseAngle toward the
// value that would make that average zero.
double trimRate = 0.0005;  // how fast auto-trim corrects. Keep VERY small.
                            // If it makes drift WORSE, flip the sign in the
                            // "if (settings.autoTrim)" block below.
double outputAvg = 0.0;    // slow (long-term) rolling average of pidOutput

unsigned long lastTime = 0;
double dt = 0.01; // loop time delta in seconds, recalculated every loop

// =============================================================================
// BLUETOOTH DRIVE CONTROL
// =============================================================================
// Design idea: instead of feeding raw speed to the motors (which would
// fight the balance PID and make the robot fall), driving forward/backward
// is done by very gently LEANING the balance target angle in that
// direction. The balance PID then naturally drives the wheels to "chase"
// that lean, producing smooth acceleration. Left/right turning is done by
// giving the two wheels a small opposite speed offset (turnBias) around
// their shared PID output -- this only affects yaw (spin), not pitch
// (front/back balance), so it doesn't fight the balance loop at all.

double moveTilt = 0.0;   // current forward/backward lean added to settings.baseAngle
double turnBias = 0.0;   // current left/right wheel speed difference

char btCommand = 'L';
unsigned long lastBtCommandTime = 0;
// Failsafe: if no valid BT command arrives for settings.remoteTimeoutMs,
// assume the connection dropped or the button was released, and stop.
// ===========================================================================

void driveMotors(double output);
void stopMotors();
void readSerialCommands();
void readBluetoothCommands();

void setup() {
  Serial.begin(115200);
  delay(500);

  if (loadSettings(settings)) {
    Serial.println("Saved settings loaded.");
  } else {
    Serial.println("No saved settings, using defaults.");
  }

  pinMode(PIN_LEFT_IN1, OUTPUT); pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT); pinMode(PIN_RIGHT_IN2, OUTPUT);

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_LEFT_PWM, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcAttach(PIN_RIGHT_PWM, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  #else
    ledcSetup(0, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS); ledcAttachPin(PIN_LEFT_PWM, 0);
    ledcSetup(1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS); ledcAttachPin(PIN_RIGHT_PWM, 1);
  #endif

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!mpu.begin(IMU_I2C_ADDRESS, &Wire)) {
    Serial.println("ERROR: MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  SerialBT.begin("DengeRobot"); // Bluetooth device name — pair your phone to this
  Serial.println("Bluetooth is active. Pair your phone with 'DengeRobot'.");

  lastTime = micros();
  Serial.println("\n=== Self-Balancing Robot Started ===");
  printHelp(Serial);
  Serial.println("Bluetooth commands: W forward, S backward, A left, D right, L stop");
}

void loop() {
  // --- Compute the loop time delta FIRST, so both the Bluetooth ramp logic
  //     and the PID controller use a fresh, consistent dt this iteration. ---
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  if (dt <= 0.0) dt = 0.005;
  lastTime = now;

  readSerialCommands();
  readBluetoothCommands(); // uses dt above to ramp moveTilt/turnBias smoothly

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Complementary filter: mostly trust the gyro (smooth, drifts slowly),
  // and slowly correct with the accelerometer (noisy, but doesn't drift).
  // 0.96/0.04 is a common starting split; raise the gyro weight (e.g. 0.98)
  // for a smoother but slower-to-correct angle estimate.
  double accAngle = pitchFromAccel(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  double gyroRate = pitchRateFromGyro(g.gyro.x * RAD_TO_DEG, g.gyro.y * RAD_TO_DEG, g.gyro.z * RAD_TO_DEG);
  currentAngle = 0.96 * (currentAngle + gyroRate * dt) + 0.04 * accAngle;

  // Safety cutoff: if the robot has tipped over too far to recover
  // (e.g. picked up, or already fallen), stop the motors instead of
  // fighting the ground, and reset the integral/derivative state so it
  // doesn't lurch when picked back up.
  if (abs(currentAngle - targetAngle) > 30.0) {
    stopMotors();
    integral = 0;
    prevError = 0;
  } else {
    error = targetAngle - currentAngle;

    integral += error * dt;
    integral = constrain(integral, -60.0, 60.0); // anti-windup clamp

    derivative = (error - prevError) / dt;
    prevError = error;

    pidOutput = (settings.kp * error) + (settings.ki * integral) + (settings.kd * derivative);
    driveMotors(pidOutput);

    // --- AUTO-TRIM ---
    // Track a very slow rolling average of pidOutput. If the robot is
    // balanced but the average output is consistently non-zero, that
    // means settings.baseAngle is slightly off from the true mechanical
    // balance point. When enabled, slowly nudge settings.baseAngle to
    // cancel that average out.
    outputAvg = outputAvg * 0.999 + pidOutput * 0.001;

    if (settings.autoTrim) {
      settings.baseAngle -= outputAvg * trimRate;
      // NOTE: if this makes drift WORSE instead of better, your motor/
      // mounting orientation needs the opposite sign — change the line
      // above to "+=".
    }
  }

  static unsigned long lastLog = 0;
  if (millis() - lastLog > 100) {
    lastLog = millis();
    Serial.print("Angle: ");   Serial.print(currentAngle);
    Serial.print(" | Target: "); Serial.print(targetAngle);
    Serial.print(" | OutAvg: "); Serial.print(outputAvg);
    Serial.print(" | Kp: ");   Serial.print(settings.kp);
    Serial.print(" | Ki: ");   Serial.print(settings.ki);
    Serial.print(" | Kd: ");   Serial.print(settings.kd);
    Serial.print(" | PID: "); Serial.println(pidOutput);
  }

  delay(4);
}

// ---------------------------------------------------------------------------
// Drives each wheel independently. Pitch (front/back balance) depends on
// the AVERAGE of the two wheels' commanded speed, while turning depends
// on the DIFFERENCE between them — so the two behaviors are fully
// independent and turning never fights the balance PID.
// ---------------------------------------------------------------------------
// Positive direction drives in1 HIGH / in2 LOW; see the *_MOTOR_REVERSED flags.
void setDirection(int in1, int in2, bool positive) {
  digitalWrite(in1, positive ? HIGH : LOW);
  digitalWrite(in2, positive ? LOW : HIGH);
}

void driveMotors(double output) {
  double leftCmd  = output + turnBias;
  double rightCmd = output - turnBias;

  int speedL = 0, speedR = 0;

  // Motors need a minimum PWM value to actually overcome friction and
  // start turning ("deadzone"). 70 is the minimum "starts moving" duty
  // cycle and 230 is the practical top speed (out of 255) — adjust these
  // two numbers to match your specific motors if they stall or are too
  // fast/slow near the limits.
  if (abs(leftCmd) >= 0.05) {
    speedL = map((int)abs(leftCmd), 0, 100, 70, 230);
    speedL = constrain(speedL, 70, 230);
  }
  if (abs(rightCmd) >= 0.05) {
    speedR = map((int)abs(rightCmd), 0, 100, 70, 230);
    speedR = constrain(speedR, 70, 230);
  }

  setDirection(PIN_LEFT_IN1, PIN_LEFT_IN2, (leftCmd >= 0) != LEFT_MOTOR_REVERSED);
  setDirection(PIN_RIGHT_IN1, PIN_RIGHT_IN2, (rightCmd >= 0) != RIGHT_MOTOR_REVERSED);

  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PIN_LEFT_PWM, speedL);
    ledcWrite(PIN_RIGHT_PWM, speedR);
  #else
    ledcWrite(0, speedL);
    ledcWrite(1, speedR);
  #endif
}

void stopMotors() {
  digitalWrite(PIN_LEFT_IN1, LOW); digitalWrite(PIN_LEFT_IN2, LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW); digitalWrite(PIN_RIGHT_IN2, LOW);
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PIN_LEFT_PWM, 0); ledcWrite(PIN_RIGHT_PWM, 0);
  #else
    ledcWrite(0, 0); ledcWrite(1, 0);
  #endif
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

  const float oldKi = settings.ki;
  const bool oldAutoTrim = settings.autoTrim;
  if (!handleSettingsCommand(cmd, settings, Serial)) {
    Serial.printf("! Unknown command '%s'. Type 'help'.\n", cmd.name);
    return;
  }
  if (settings.ki != oldKi) integral = 0;  // a big Ki change must not cause a kick
  if (settings.autoTrim != oldAutoTrim) outputAvg = 0;
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
// Reads the latest Bluetooth drive command and smoothly ramps the
// forward/backward lean (moveTilt) and turn offset (turnBias) toward it
// over time, instead of jumping instantly — this is what keeps driving
// from fighting/destabilizing the balance PID.
//
// A phone Bluetooth-control app (e.g. "Arduino Bluetooth Controller" in
// pad mode) typically re-sends the held key's character repeatedly while
// pressed, and sends a different character (here treated as anything
// other than W/S/A/D, commonly mapped to 'L') when released. This
// function keeps the last recognized command and, if nothing valid is
// heard for `settings.remoteTimeoutMs` ms, treats that as "released" and stops as a
// safety failsafe.
// ---------------------------------------------------------------------------
void readBluetoothCommands() {
  if (SerialBT.available()) {
    char c = toupper(SerialBT.read());
    if (c == 'W' || c == 'S' || c == 'A' || c == 'D' || c == 'L') {
      btCommand = c;
      lastBtCommandTime = millis();
    }
  }

  // Safety failsafe: if the connection drops or the command stream stops
  // arriving, automatically stop driving.
  if (millis() - lastBtCommandTime > settings.remoteTimeoutMs) {
    btCommand = 'L';
  }

  // --- FORWARD / BACKWARD: ramp moveTilt toward its target over time ---
  double tiltTarget = 0.0;
  if (btCommand == 'W') tiltTarget = settings.moveTiltMax;
  else if (btCommand == 'S') tiltTarget = -settings.moveTiltMax;

  // Time-based ramp: moves at a constant deg/second rate regardless of
  // how fast/slow the loop happens to be running this cycle.
  double tiltStep = (settings.moveTiltMax / settings.tiltRampTime) * dt;
  if (moveTilt < tiltTarget)      moveTilt = min(moveTilt + tiltStep, tiltTarget);
  else if (moveTilt > tiltTarget) moveTilt = max(moveTilt - tiltStep, tiltTarget);

  targetAngle = settings.baseAngle + moveTilt;

  // --- LEFT / RIGHT: ramp turnBias toward its target over time ---
  double turnTarget = 0.0;
  if (btCommand == 'D') turnTarget = settings.turnMax;
  else if (btCommand == 'A') turnTarget = -settings.turnMax;

  double turnStep = (settings.turnMax / settings.turnRampTime) * dt;
  if (turnBias < turnTarget)      turnBias = min(turnBias + turnStep, turnTarget);
  else if (turnBias > turnTarget) turnBias = max(turnBias - turnStep, turnTarget);
}
