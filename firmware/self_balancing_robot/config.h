// =============================================================================
// Build configuration: everything that depends on YOUR robot lives here.
// Change pins, motor directions and IMU orientation in this file only.
// =============================================================================
#pragma once

#include <math.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Motor driver pins (L298N)
// ---------------------------------------------------------------------------
constexpr int PIN_LEFT_PWM = 14;   // ENA
constexpr int PIN_LEFT_IN1 = 27;   // IN1
constexpr int PIN_LEFT_IN2 = 26;   // IN2
constexpr int PIN_RIGHT_PWM = 12;  // ENB
constexpr int PIN_RIGHT_IN1 = 25;  // IN3
constexpr int PIN_RIGHT_IN2 = 33;  // IN4

constexpr uint32_t MOTOR_PWM_FREQ_HZ = 1000;
constexpr uint8_t MOTOR_PWM_BITS = 8;  // 8 bits -> duty 0..255

// Motor direction. A positive command drives IN1 (IN3) HIGH and IN2 (IN4) LOW;
// a reversed motor gets the opposite. Flip a flag instead of re-soldering.
constexpr bool LEFT_MOTOR_REVERSED = false;
constexpr bool RIGHT_MOTOR_REVERSED = true;

// ---------------------------------------------------------------------------
// IMU (MPU-6050) wiring and orientation
// ---------------------------------------------------------------------------
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr uint8_t IMU_I2C_ADDRESS = 0x68;  // 0x69 if AD0 is tied to 3.3 V

// Pitch angle (degrees) from the accelerometer and pitch rate (deg/s) from the
// gyro. These match an MPU-6050 mounted upside down on the lid. If your board
// sits differently, pick the axes so that BOTH values grow when the robot tilts
// the same way (see README, "First start").
inline float pitchFromAccel(float ax, float ay, float az) {
  (void)ay;
  return atan2f(ax, -az) * 57.2957795f;
}
inline float pitchRateFromGyro(float gx, float gy, float gz) {
  (void)gx;
  (void)gz;
  return gy;
}
