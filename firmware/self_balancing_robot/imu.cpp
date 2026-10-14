#include "imu.h"

namespace {

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr uint8_t GYRO_500_DPS = 0x08;
constexpr uint8_t ACCEL_4_G = 0x08;
constexpr uint8_t CLOCK_PLL_GYRO_X = 0x01;  // also clears the sleep bit

constexpr float GYRO_LSB_PER_DPS = 65.5f;   // at +-500 deg/s
constexpr float ACCEL_LSB_PER_G = 8192.0f;  // at +-4 g

int16_t word(const uint8_t* bytes) {
  return static_cast<int16_t>((bytes[0] << 8) | bytes[1]);
}

}  // namespace

bool Mpu6050::begin(TwoWire& wire, uint8_t address, uint8_t lowPassConfig) {
  wire_ = &wire;
  address_ = address;
  // Many cheap boards carry an MPU-6500 or a clone that reports a different
  // WHO_AM_I value but works the same, so any answer is accepted here.
  return readRegisters(REG_WHO_AM_I, &whoAmI_, 1) &&
         writeRegister(REG_PWR_MGMT_1, CLOCK_PLL_GYRO_X) &&
         writeRegister(REG_SMPLRT_DIV, 0) &&
         writeRegister(REG_CONFIG, lowPassConfig & 0x07) &&
         writeRegister(REG_GYRO_CONFIG, GYRO_500_DPS) &&
         writeRegister(REG_ACCEL_CONFIG, ACCEL_4_G);
}

bool Mpu6050::read(ImuSample& out) {
  uint8_t raw[14];
  if (!readRegisters(REG_ACCEL_XOUT_H, raw, sizeof(raw))) return false;
  out.ax = word(raw + 0) / ACCEL_LSB_PER_G;
  out.ay = word(raw + 2) / ACCEL_LSB_PER_G;
  out.az = word(raw + 4) / ACCEL_LSB_PER_G;
  // raw + 6 is the temperature, not needed.
  out.gx = word(raw + 8) / GYRO_LSB_PER_DPS;
  out.gy = word(raw + 10) / GYRO_LSB_PER_DPS;
  out.gz = word(raw + 12) / GYRO_LSB_PER_DPS;
  return true;
}

bool Mpu6050::writeRegister(uint8_t reg, uint8_t value) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool Mpu6050::readRegisters(uint8_t reg, uint8_t* data, uint8_t count) {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(address_, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) {
    const int value = wire_->read();
    if (value < 0) return false;
    data[i] = static_cast<uint8_t>(value);
  }
  return true;
}
