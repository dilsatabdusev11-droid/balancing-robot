#include "motors.h"

#include <Arduino.h>

#include "config.h"
#include "drive_math.h"

namespace {

constexpr int kDutyLimit = (1 << MOTOR_PWM_BITS) - 1;
constexpr DutyShape kShape = {MOTOR_START_COMMAND, MOTOR_PWM_MIN, MOTOR_PWM_MAX};

struct Motor {
  int pwmPin;
  int in1;
  int in2;
  int channel;  // only used by the ESP32 Arduino core 2.x LEDC API
  bool reversed;
};

constexpr Motor kLeft = {PIN_LEFT_PWM, PIN_LEFT_IN1, PIN_LEFT_IN2, 0, LEFT_MOTOR_REVERSED};
constexpr Motor kRight = {PIN_RIGHT_PWM, PIN_RIGHT_IN1, PIN_RIGHT_IN2, 1, RIGHT_MOTOR_REVERSED};

void writeDuty(const Motor& motor, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(motor.pwmPin, duty);
#else
  ledcWrite(motor.channel, duty);
#endif
}

void setupMotor(const Motor& motor) {
  pinMode(motor.in1, OUTPUT);
  pinMode(motor.in2, OUTPUT);
  digitalWrite(motor.in1, LOW);
  digitalWrite(motor.in2, LOW);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(motor.pwmPin, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
#else
  ledcSetup(motor.channel, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(motor.pwmPin, motor.channel);
#endif
  writeDuty(motor, 0);
}

void driveMotor(const Motor& motor, float command, float voltageScale) {
  const bool forward = (command >= 0.0f) != motor.reversed;
  digitalWrite(motor.in1, forward ? HIGH : LOW);
  digitalWrite(motor.in2, forward ? LOW : HIGH);
  int duty = static_cast<int>(lroundf(commandToDuty(command, kShape) * voltageScale));
  if (duty > kDutyLimit) duty = kDutyLimit;
  writeDuty(motor, duty);
}

void stopMotor(const Motor& motor) {
  digitalWrite(motor.in1, LOW);
  digitalWrite(motor.in2, LOW);
  writeDuty(motor, 0);
}

}  // namespace

void motorsBegin() {
  setupMotor(kLeft);
  setupMotor(kRight);
  if (PIN_MOTOR_STBY >= 0) {  // TB6612FNG: the driver sleeps while STBY is LOW
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    digitalWrite(PIN_MOTOR_STBY, HIGH);
  }
}

void motorsDrive(float leftCommand, float rightCommand, float voltageScale) {
  driveMotor(kLeft, leftCommand, voltageScale);
  driveMotor(kRight, rightCommand, voltageScale);
}

void motorsStop() {
  stopMotor(kLeft);
  stopMotor(kRight);
}
