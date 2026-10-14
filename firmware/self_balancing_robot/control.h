// =============================================================================
// Control building blocks: angle filter, gyro calibration, ramps.
// Pure C++ (no Arduino dependencies) so it is unit-tested on the host.
// =============================================================================
#pragma once

#include <math.h>

// Fuses the accelerometer angle (noisy, but no drift) with the integrated gyro
// rate (smooth, but drifts). timeConstant says how long, in seconds, the gyro
// alone is trusted: the result no longer depends on how fast the loop runs.
struct ComplementaryFilter {
  float angle = 0.0f;
  bool started = false;

  float update(float accelAngle, float gyroRate, float dt, float timeConstant) {
    if (!started) {  // start from the accelerometer instead of from 0
      angle = accelAngle;
      started = true;
      return angle;
    }
    const float alpha = timeConstant / (timeConstant + dt);
    angle = alpha * (angle + gyroRate * dt) + (1.0f - alpha) * accelAngle;
    return angle;
  }

  void reset() { started = false; }
};

// Finds the gyro's zero offset by averaging it while the robot is still. A
// typical MPU-6050 reads 1-3 deg/s at rest; uncorrected, the filter turns that
// into a constant angle error and the robot slowly rolls away.
struct GyroCalibrator {
  int samples = 0;
  float sum = 0.0f;
  float minRate = 0.0f;
  float maxRate = 0.0f;

  void reset() { *this = GyroCalibrator(); }

  void add(float rate) {
    if (samples == 0) {
      minRate = maxRate = rate;
    } else {
      minRate = fminf(minRate, rate);
      maxRate = fmaxf(maxRate, rate);
    }
    sum += rate;
    ++samples;
  }

  bool stillEnough(float maxSpread) const { return maxRate - minRate <= maxSpread; }
  float bias() const { return samples > 0 ? sum / samples : 0.0f; }
};

// Moves `current` toward `target` by at most `maxStep`.
inline float rampToward(float current, float target, float maxStep) {
  if (current < target) return fminf(current + maxStep, target);
  if (current > target) return fmaxf(current - maxStep, target);
  return current;
}

inline float clampf(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}
