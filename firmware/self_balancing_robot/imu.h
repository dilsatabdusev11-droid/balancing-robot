// MPU-6050 driven through its registers, so every I2C error is visible
// (the Adafruit driver ignored them) and no extra libraries are needed.
#pragma once

#include <Wire.h>
#include <stdint.h>

struct ImuSample {
  float ax, ay, az;  // acceleration, g
  float gx, gy, gz;  // rotation rate, deg/s
};

class Mpu6050 {
 public:
  // Wakes the sensor and sets ranges (+-4 g, +-500 deg/s) and its low-pass
  // filter. Returns false when the chip does not answer.
  bool begin(TwoWire& wire, uint8_t address, uint8_t lowPassConfig);
  // Reads one sample. Returns false on any bus error (loose wire, noise).
  bool read(ImuSample& out);
  uint8_t whoAmI() const { return whoAmI_; }

 private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t* data, uint8_t count);

  TwoWire* wire_ = nullptr;
  uint8_t address_ = 0x68;
  uint8_t whoAmI_ = 0;
};
