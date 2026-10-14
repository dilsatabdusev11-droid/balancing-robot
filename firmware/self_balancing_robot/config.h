// =============================================================================
// Build configuration: everything that depends on YOUR robot lives here.
// Change pins, motor directions and IMU orientation in this file only.
// =============================================================================
#pragma once

#include <math.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Phone remote
// ---------------------------------------------------------------------------
// REMOTE_BLUETOOTH: classic Bluetooth, Android pad apps. Only the original
//   ESP32 has it (not S2/S3/C3/C6), and ESP32 core 4.x drops it by default.
// REMOTE_WIFI: the robot creates its own Wi-Fi network with a control page
//   for any phone browser, iPhone included. Works on every ESP32.
#define REMOTE_BLUETOOTH 1
#define REMOTE_WIFI 2
#ifndef REMOTE_MODE
#define REMOTE_MODE REMOTE_BLUETOOTH
#endif
// Each robot is named <prefix>-XXXX from its own chip ID, so a class full of
// robots does not show ten identical names.
constexpr const char* ROBOT_NAME_PREFIX = "BalanceBot";
constexpr const char* WIFI_PASSWORD = "robot1234";  // 8+ characters (REMOTE_WIFI)

// ---------------------------------------------------------------------------
// Motor driver pins (L298N or TB6612FNG)
// ---------------------------------------------------------------------------
constexpr int PIN_LEFT_PWM = 14;   // L298N ENA / TB6612 PWMA
constexpr int PIN_LEFT_IN1 = 27;   // L298N IN1 / TB6612 AIN1
constexpr int PIN_LEFT_IN2 = 26;   // L298N IN2 / TB6612 AIN2
// Was GPIO12 in the original build. GPIO12 is a boot strapping pin: if the
// driver pulls it HIGH while the ESP32 resets, the board does not start.
constexpr int PIN_RIGHT_PWM = 13;  // L298N ENB / TB6612 PWMB
constexpr int PIN_RIGHT_IN1 = 25;  // L298N IN3 / TB6612 BIN1
constexpr int PIN_RIGHT_IN2 = 33;  // L298N IN4 / TB6612 BIN2
// TB6612FNG only: its STBY pin. -1 when STBY is wired straight to 3.3 V or
// when using an L298N.
constexpr int PIN_MOTOR_STBY = -1;

constexpr uint32_t MOTOR_PWM_FREQ_HZ = 1000;
constexpr uint8_t MOTOR_PWM_BITS = 8;  // 8 bits -> duty 0..255
// Duty that just gets the motors turning (static friction), duty at full
// command, and the command (0..100) at which MOTOR_PWM_MIN is reached.
constexpr int MOTOR_PWM_MIN = 70;
constexpr int MOTOR_PWM_MAX = 230;
constexpr float MOTOR_START_COMMAND = 2.0f;

// Motor direction. A positive (forward) command drives IN1 HIGH and IN2 LOW;
// a reversed motor gets the opposite. "Forward" is the side the robot leans
// to when the measured angle grows. Check with the "motortest" command and
// flip a flag instead of re-soldering.
constexpr bool LEFT_MOTOR_REVERSED = true;
constexpr bool RIGHT_MOTOR_REVERSED = false;

// ---------------------------------------------------------------------------
// Battery monitor (optional): pack+ -> R1 -> GPIO34 -> R2 -> GND
// ---------------------------------------------------------------------------
// Enable only after wiring the divider: an unconnected pin reads garbage.
constexpr bool BATTERY_MONITOR_ENABLED = false;
constexpr int PIN_BATTERY_SENSE = 34;                              // input-only ADC pin
constexpr float BATTERY_DIVIDER_RATIO = (100.0f + 47.0f) / 47.0f;  // R1 = 100k, R2 = 47k
constexpr float BATTERY_NOMINAL_VOLTS = 7.4f;  // 2S Li-ion
constexpr float BATTERY_CUTOFF_VOLTS = 6.4f;   // 3.2 V per cell: stop the motors
constexpr float BATTERY_RESUME_VOLTS = 6.8f;   // run again after charging

// ---------------------------------------------------------------------------
// IMU (MPU-6050) wiring and orientation
// ---------------------------------------------------------------------------
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr uint8_t IMU_I2C_ADDRESS = 0x68;  // 0x69 if AD0 is tied to 3.3 V
constexpr uint32_t I2C_CLOCK_HZ = 400000;   // fast mode: one IMU read takes ~0.5 ms
constexpr uint16_t I2C_TIMEOUT_MS = 10;

// MPU-6050 built-in low-pass filter (DLPF_CFG): 4 = ~21 Hz like the original
// build; 3 = ~43 Hz reacts faster but is noisier.
constexpr uint8_t IMU_LOW_PASS_CONFIG = 4;
// How long (s) the gyro is trusted before the accelerometer pulls the angle
// back. Longer = less disturbed by the robot's own acceleration.
constexpr float IMU_FILTER_TIME_CONSTANT_S = 0.5f;
constexpr int IMU_MAX_READ_ERRORS = 3;  // consecutive failed reads before motors stop
constexpr uint32_t IMU_RETRY_MS = 500;

// Gyro offset calibration at boot and with the "calib" command.
constexpr int GYRO_CALIBRATION_SAMPLES = 200;  // 1 s at 200 Hz
constexpr float GYRO_STILL_SPREAD_DPS = 2.0f;  // more spread = robot moved, retry

// Pitch angle (degrees) from the accelerometer and pitch rate (deg/s) from the
// gyro. These match an MPU-6050 mounted upside down on the lid. If your board
// sits differently, pick the axes so that BOTH values grow when the robot tilts
// the same way (README, first start, step 2).
inline float pitchFromAccel(float ax, float ay, float az) {
  (void)ay;
  return atan2f(ax, -az) * 57.2957795f;
}
inline float pitchRateFromGyro(float gx, float gy, float gz) {
  (void)gx;
  (void)gz;
  return gy;
}

// ---------------------------------------------------------------------------
// Wheel encoders (optional): motors with Hall encoders on channels A and B
// ---------------------------------------------------------------------------
// Without encoders the robot only holds its angle: it slowly rolls away and
// cannot stand on one spot. With them a speed loop picks the lean angle.
constexpr bool ENCODERS_ENABLED = false;
constexpr int PIN_ENC_LEFT_A = 18;
constexpr int PIN_ENC_LEFT_B = 19;
constexpr int PIN_ENC_RIGHT_A = 23;
constexpr int PIN_ENC_RIGHT_B = 32;
// Flip so that both counts grow when the robot rolls forward ("encoders").
constexpr bool LEFT_ENCODER_REVERSED = false;
constexpr bool RIGHT_ENCODER_REVERSED = false;
// Rising edges on channel A per wheel turn: motor pulses x gear ratio.
constexpr float ENCODER_COUNTS_PER_WHEEL_REV = 11.0f * 48.0f;
constexpr float WHEEL_DIAMETER_M = 0.065f;
constexpr float SPEED_FILTER_S = 0.05f;       // smoothing of the measured speed
constexpr float SPEED_TILT_LIMIT_DEG = 6.0f;  // max lean the speed loop may ask for

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
constexpr uint32_t CONTROL_PERIOD_US = 5000;  // 200 Hz
constexpr float FALL_ANGLE_DEG = 30.0f;       // beyond this the robot has fallen
// The motors start only after the robot is held within ARM_ANGLE_DEG of
// upright for ARM_HOLD_S seconds (after boot, a fall or "stop"/"go").
constexpr float ARM_ANGLE_DEG = 5.0f;
constexpr float ARM_HOLD_S = 0.5f;
constexpr float PID_INTEGRAL_LIMIT = 60.0f;   // anti-windup clamp (deg * s)

// Auto-trim ("autotrim 1"): how fast it moves the upright angle (deg/s per
// unit of average output), how long it averages, and how far it may go.
constexpr float AUTOTRIM_RATE = 0.01f;
constexpr float AUTOTRIM_AVERAGE_S = 5.0f;
constexpr float AUTOTRIM_LIMIT_DEG = 2.0f;

constexpr float MOTOR_TEST_COMMAND = 35.0f;   // "motortest" strength, 0..100
