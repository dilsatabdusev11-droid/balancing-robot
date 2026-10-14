// =============================================================================
// Motor command shaping and battery logic.
// Pure C++ (no Arduino dependencies) so it is unit-tested on the host.
// =============================================================================
#pragma once

#include <math.h>

// Motor commands run from -100 (full backward) to +100 (full forward).
constexpr float kMaxCommand = 100.0f;

struct DutyShape {
  float startCommand;  // command at which the motor just starts to turn
  int minDuty;         // duty that overcomes static friction
  int maxDuty;         // duty at full command
};

// Converts |command| to a PWM duty. Above startCommand the duty jumps past the
// motor's static friction (minDuty) and grows linearly up to maxDuty. Below it
// the duty grows from 0 to minDuty, so there is no step at zero: a step there
// makes the motors chatter around the balance point.
inline int commandToDuty(float command, const DutyShape& shape) {
  float magnitude = fabsf(command);
  if (!(magnitude > 0.0f)) return 0;  // also catches NaN
  if (magnitude > kMaxCommand) magnitude = kMaxCommand;
  float duty;
  if (magnitude < shape.startCommand) {
    duty = magnitude / shape.startCommand * shape.minDuty;
  } else {
    duty = shape.minDuty + (magnitude - shape.startCommand) * (shape.maxDuty - shape.minDuty) /
                               (kMaxCommand - shape.startCommand);
  }
  return static_cast<int>(lroundf(duty));
}

// Multiplier that keeps motor torque the same while the battery drains:
// a half-empty pack needs a bit more duty for the same push.
inline float voltageCompensation(float batteryVolts, float nominalVolts) {
  if (!(batteryVolts > 1.0f)) return 1.0f;  // no reading: do not guess
  float scale = nominalVolts / batteryVolts;
  if (scale < 0.8f) scale = 0.8f;
  if (scale > 1.3f) scale = 1.3f;
  return scale;
}

struct BatteryLimits {
  float cutoffVolts;  // below this (for holdSeconds) the robot stops
  float resumeVolts;  // ... and runs again only above this (hysteresis)
  float holdSeconds;  // ignore short dips while the motors pull hard
  float filterSeconds;
};

// Smooths the battery voltage and decides when it is too low to run.
struct BatteryGuard {
  float volts = 0.0f;
  bool low = false;
  float belowCutoffFor = 0.0f;

  void update(float measuredVolts, float dt, const BatteryLimits& limits) {
    if (volts <= 0.0f) {
      volts = measuredVolts;
    } else {
      volts += (measuredVolts - volts) * (dt / (limits.filterSeconds + dt));
    }
    if (low) {
      if (volts > limits.resumeVolts) low = false;
      belowCutoffFor = 0.0f;
      return;
    }
    belowCutoffFor = volts < limits.cutoffVolts ? belowCutoffFor + dt : 0.0f;
    if (belowCutoffFor >= limits.holdSeconds) low = true;
  }
};
