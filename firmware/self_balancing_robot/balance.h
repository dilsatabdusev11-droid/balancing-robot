// =============================================================================
// Balance logic: PID, auto-trim, when the motors may run, motor test.
// Pure C++ (no Arduino dependencies) so it is unit-tested on the host.
// =============================================================================
#pragma once

#include <math.h>

#include "control.h"

struct PidGains {
  float kp;
  float ki;
  float kd;
};

// Balance PID. error = angle - target: positive when leaning forward past the
// target, and a positive output drives forward so the wheels roll under the
// body. The D term uses the gyro's tilt rate instead of differentiating the
// error: same meaning, but no noise amplification and no kick when the target
// jumps or the controller restarts.
struct BalancePid {
  float integral = 0.0f;

  float update(float error, float tiltRate, float dt, const PidGains& gains, float integralLimit,
               float outputLimit) {
    const float candidate = clampf(integral + error * dt, -integralLimit, integralLimit);
    const float output = gains.kp * error + gains.ki * candidate + gains.kd * tiltRate;
    // Anti-windup: while the output is already saturated in the direction the
    // error pushes, do not keep growing the integral.
    const bool saturatedFurther = fabsf(output) > outputLimit && (output > 0.0f) == (error > 0.0f);
    if (!saturatedFurther) integral = candidate;
    return clampf(gains.kp * error + gains.ki * integral + gains.kd * tiltRate, -outputLimit,
                  outputLimit);
  }

  void reset() { integral = 0.0f; }
};

// Slowly corrects the upright angle while the robot stands. If the target
// leans forward of the true balance point, the robot has to keep driving
// forward to stay up, so the average output is positive: move the target
// back. The correction is bounded, and it only learns while standing still
// (while driving the output is non-zero on purpose).
struct AutoTrim {
  float average = 0.0f;  // slow average of the balance output
  float offset = 0.0f;   // added to the base angle, deg

  void update(float output, float dt, bool learning, float degPerSecondPerUnit,
              float averageSeconds, float limitDeg) {
    if (!learning) return;
    average += (output - average) * (dt / (averageSeconds + dt));
    offset = clampf(offset - average * degPerSecondPerUnit * dt, -limitDeg, limitDeg);
  }

  void reset() { *this = AutoTrim(); }
};

enum class RobotState { Calibrating, Waiting, Balancing, Fallen, Stopped, LowBattery, ImuFault };

inline const char* stateName(RobotState state) {
  switch (state) {
    case RobotState::Calibrating: return "CALIBRATING";
    case RobotState::Waiting: return "WAITING";
    case RobotState::Balancing: return "BALANCING";
    case RobotState::Fallen: return "FALLEN";
    case RobotState::Stopped: return "STOPPED";
    case RobotState::LowBattery: return "LOW_BATTERY";
    case RobotState::ImuFault: return "IMU_FAULT";
  }
  return "?";
}

struct ArmingInputs {
  bool imuOk;
  bool calibrating;
  bool stopRequested;  // emergency stop or motor test
  bool batteryLow;
  float tiltError;     // angle minus the upright angle, deg
};

struct ArmingConfig {
  float armAngleDeg;     // must be within this of upright ...
  float armHoldSeconds;  // ... for this long before the motors start
  float fallAngleDeg;    // beyond this the robot has fallen
};

// Decides when the motors may run. The original sketch restarted them as soon
// as the robot was within 30 deg of upright, so a robot lying at 25 deg (or
// being picked up) lurched at full power. Now it must be held near upright
// for a moment first.
struct Arming {
  RobotState state = RobotState::Calibrating;
  float uprightFor = 0.0f;

  RobotState update(const ArmingInputs& in, float dt, const ArmingConfig& config) {
    if (!in.imuOk) return enter(RobotState::ImuFault);
    if (in.calibrating) return enter(RobotState::Calibrating);
    if (in.stopRequested) return enter(RobotState::Stopped);
    if (in.batteryLow) return enter(RobotState::LowBattery);

    const float tilt = fabsf(in.tiltError);
    if (state == RobotState::Balancing) {
      return tilt > config.fallAngleDeg ? enter(RobotState::Fallen) : state;
    }
    if (state != RobotState::Fallen) enter(RobotState::Waiting);
    uprightFor = tilt < config.armAngleDeg ? uprightFor + dt : 0.0f;
    return uprightFor >= config.armHoldSeconds ? enter(RobotState::Balancing) : state;
  }

  RobotState enter(RobotState next) {
    if (next != state) {
      state = next;
      uprightFor = 0.0f;
    }
    return state;
  }
};

// Spins the left wheel, pauses, then spins the right wheel, both "forward".
// Used to check motor directions with the robot held in the air.
struct MotorTest {
  float elapsed = -1.0f;  // < 0: not running

  void start() { elapsed = 0.0f; }
  bool running() const { return elapsed >= 0.0f; }

  // Returns false once the test has finished.
  bool step(float dt, float command, float& left, float& right) {
    if (!running()) return false;
    elapsed += dt;
    left = right = 0.0f;
    if (elapsed < 0.8f) {
      left = command;
    } else if (elapsed >= 1.2f && elapsed < 2.0f) {
      right = command;
    } else if (elapsed >= 2.0f) {
      elapsed = -1.0f;
      return false;
    }
    return true;
  }
};
