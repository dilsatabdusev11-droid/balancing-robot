/* =============================================================================
   SELF-BALANCING ROBOT - MPU6050 + L298N + PID + Bluetooth (ESP32)
   =============================================================================
   Balances a two-wheeled robot with a PID controller fed by an MPU6050 IMU and
   lets you drive it from a phone over classic Bluetooth Serial (for example the
   "Arduino Bluetooth Controller" app in pad mode).

   Robot-specific settings (pins, motor directions, IMU orientation) live in
   config.h. Serial tuning commands (115200 baud):
     p<value> Kp   i<value> Ki   d<value> Kd   t<value> base target angle
     a1 / a0  auto-trim on/off   m<value> max lean (deg)   n<value> max turn
     r<value> drive ramp (s)     u<value> turn ramp (s)    x<value> BT timeout (ms)

   Bluetooth phone app (pair with "DengeRobot"):
     W = forward   S = backward   A = left   D = right   L = stop
   ========================================================================= */

#include <Wire.h>

#include "config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "BluetoothSerial.h"

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT;

// ---------------------------------------------------------------------------
// LIVE-TUNABLE PID VALUES (balance controller)
// ---------------------------------------------------------------------------
// These are the core "how hard does the robot correct itself" gains.
//   Kp -> proportional: bigger = reacts harder to the current tilt error
//   Ki -> integral: bigger = corrects small steady-state lean faster,
//         but too high causes slow oscillation / overshoot
//   Kd -> derivative: bigger = resists fast tilt changes (damping),
//         too high makes the robot twitchy/noisy
// Tune in this order: raise Kp until it balances but wobbles, add Kd to
// damp the wobble, then add a small amount of Ki to kill any steady lean.
double Kp = 15.0;
double Ki = 0.71;
double Kd = 0.44;

// The angle (in degrees, as measured by the IMU) at which the robot is
// considered "upright". This is NOT necessarily 0 -- it depends on where
// the IMU is physically mounted, so you'll need to find this value by
// trial and error (use the 't' serial command).
double baseTargetAngle = 3.0;

double targetAngle = baseTargetAngle; // actual angle the PID is chasing right now
double currentAngle = 0.0;            // filtered current tilt angle (complementary filter)
double error = 0.0, prevError = 0.0;
double integral = 0.0, derivative = 0.0;
double pidOutput = 0.0;

// ---------------------------------------------------------------------------
// AUTO-TRIM (drift correction)
// ---------------------------------------------------------------------------
// If the robot consistently drifts/leans one way even while "balanced",
// it usually means baseTargetAngle isn't perfectly matched to the real
// mechanical balance point. Auto-trim watches the long-term average PID
// output and, if enabled, very slowly nudges baseTargetAngle toward the
// value that would make that average zero.
bool autoTrim = false;     // enable with 'a1', disable with 'a0'
double trimRate = 0.0005;  // how fast auto-trim corrects. Keep VERY small.
                            // If it makes drift WORSE, flip the sign in the
                            // "if (autoTrim)" block below.
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

double moveTilt = 0.0;   // current forward/backward lean added to baseTargetAngle
double turnBias = 0.0;   // current left/right wheel speed difference

// --- Tunable via serial commands (m / n / r / u / x) ---
double moveTiltMax   = 1.0;   // 'm' — maximum forward/back lean angle (deg).
                               //       Bigger = more aggressive top speed,
                               //       but also easier to destabilize the
                               //       balance loop. Start small (0.3-0.6).
double turnMax        = 12.0; // 'n' — maximum turn strength.
                               //       Bigger = faster spin on A/D.
double moveTiltRampTime = 0.6; // 'r' — seconds to go from 0 to moveTiltMax.
                                //       Bigger = gentler/slower acceleration.
double turnRampTime     = 1.0; // 'u' — seconds to go from 0 to turnMax.
                                //       Bigger = gentler turn start.

char btCommand = 'L';
unsigned long lastBtCommandTime = 0;
// Failsafe: if no valid BT command is received for this long, assume the
// connection dropped or the button was released, and stop. Raised from the
// original 300 ms because classic Bluetooth Serial can have latency spikes
// that were causing false "release" events mid-drive. Tune with 'x'.
unsigned long btTimeout = 600;
// ===========================================================================

void driveMotors(double output);
void stopMotors();
void readSerialCommands();
void readBluetoothCommands();

void setup() {
  Serial.begin(115200);
  delay(500);

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
  Serial.println("Serial tuning commands: p / i / d / t / a / m / n / r / u / x  (see header comment)");
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

    pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
    driveMotors(pidOutput);

    // --- AUTO-TRIM ---
    // Track a very slow rolling average of pidOutput. If the robot is
    // balanced but the average output is consistently non-zero, that
    // means baseTargetAngle is slightly off from the true mechanical
    // balance point. When enabled, slowly nudge baseTargetAngle to
    // cancel that average out.
    outputAvg = outputAvg * 0.999 + pidOutput * 0.001;

    if (autoTrim) {
      baseTargetAngle -= outputAvg * trimRate;
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
    Serial.print(" | Kp: ");   Serial.print(Kp);
    Serial.print(" | Ki: ");   Serial.print(Ki);
    Serial.print(" | Kd: ");   Serial.print(Kd);
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
// USB Serial tuning commands. Type a letter immediately followed by a
// number and press Enter, e.g.  p15.5
// ---------------------------------------------------------------------------
void readSerialCommands() {
  if (Serial.available()) {
    char type = Serial.read();
    double val = Serial.parseFloat();

    if (type == 'p') {
      Kp = val;
      Serial.print("-> Kp updated: "); Serial.println(Kp);
    } else if (type == 'i') {
      Ki = val;
      integral = 0; // reset so a big Ki change doesn't cause a sudden kick
      Serial.print("-> Ki updated: "); Serial.println(Ki);
    } else if (type == 'd') {
      Kd = val;
      Serial.print("-> Kd updated: "); Serial.println(Kd);
    } else if (type == 't') {
      baseTargetAngle = val;
      Serial.print("-> Base target angle updated: "); Serial.println(baseTargetAngle);
    } else if (type == 'a') {
      autoTrim = (val != 0);
      outputAvg = 0;
      Serial.print("-> Auto-trim: "); Serial.println(autoTrim ? "ON" : "OFF");
    } else if (type == 'm') {
      moveTiltMax = val;
      Serial.print("-> Max forward/back lean updated: "); Serial.println(moveTiltMax);
    } else if (type == 'n') {
      turnMax = val;
      Serial.print("-> Max turn strength updated: "); Serial.println(turnMax);
    } else if (type == 'r') {
      moveTiltRampTime = max(val, 0.05); // avoid 0/negative -> divide-by-zero
      Serial.print("-> Forward/back ramp time (s) updated: "); Serial.println(moveTiltRampTime);
    } else if (type == 'u') {
      turnRampTime = max(val, 0.05);
      Serial.print("-> Turn ramp time (s) updated: "); Serial.println(turnRampTime);
    } else if (type == 'x') {
      btTimeout = (unsigned long) max(val, 50.0); // don't allow an unsafe 0 ms timeout
      Serial.print("-> Bluetooth failsafe timeout (ms) updated: "); Serial.println(btTimeout);
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
// heard for `btTimeout` ms, treats that as "released" and stops as a
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
  if (millis() - lastBtCommandTime > btTimeout) {
    btCommand = 'L';
  }

  // --- FORWARD / BACKWARD: ramp moveTilt toward its target over time ---
  double tiltTarget = 0.0;
  if (btCommand == 'W') tiltTarget = moveTiltMax;
  else if (btCommand == 'S') tiltTarget = -moveTiltMax;

  // Time-based ramp: moves at a constant deg/second rate regardless of
  // how fast/slow the loop happens to be running this cycle.
  double tiltStep = (moveTiltMax / moveTiltRampTime) * dt;
  if (moveTilt < tiltTarget)      moveTilt = min(moveTilt + tiltStep, tiltTarget);
  else if (moveTilt > tiltTarget) moveTilt = max(moveTilt - tiltStep, tiltTarget);

  targetAngle = baseTargetAngle + moveTilt;

  // --- LEFT / RIGHT: ramp turnBias toward its target over time ---
  double turnTarget = 0.0;
  if (btCommand == 'D') turnTarget = turnMax;
  else if (btCommand == 'A') turnTarget = -turnMax;

  double turnStep = (turnMax / turnRampTime) * dt;
  if (turnBias < turnTarget)      turnBias = min(turnBias + turnStep, turnTarget);
  else if (turnBias > turnTarget) turnBias = max(turnBias - turnStep, turnTarget);
}
