#include "battery.h"

#include <Arduino.h>

#include "config.h"
#include "drive_math.h"

namespace {

constexpr uint32_t kSamplePeriodMs = 20;
constexpr BatteryLimits kLimits = {BATTERY_CUTOFF_VOLTS, BATTERY_RESUME_VOLTS, 2.0f, 1.0f};

BatteryGuard guard;
uint32_t lastSampleMs = 0;

}  // namespace

void batteryBegin() {
  if (!BATTERY_MONITOR_ENABLED) return;
  analogSetPinAttenuation(PIN_BATTERY_SENSE, ADC_11db);
  lastSampleMs = millis();
}

void batteryPoll() {
  if (!BATTERY_MONITOR_ENABLED) return;
  const uint32_t now = millis();
  if (now - lastSampleMs < kSamplePeriodMs) return;
  const float dt = (now - lastSampleMs) / 1000.0f;
  lastSampleMs = now;
  const float volts = analogReadMilliVolts(PIN_BATTERY_SENSE) / 1000.0f * BATTERY_DIVIDER_RATIO;
  const bool wasLow = guard.low;
  guard.update(volts, dt, kLimits);
  if (guard.low != wasLow) {
    Serial.printf(guard.low ? "! Battery low (%.2f V): motors stopped. Charge the battery.\n"
                            : "-> Battery OK again (%.2f V).\n",
                  guard.volts);
  }
}

float batteryVolts() { return BATTERY_MONITOR_ENABLED ? guard.volts : 0.0f; }

bool batteryLow() { return BATTERY_MONITOR_ENABLED && guard.low; }

float batteryCompensation() {
  return BATTERY_MONITOR_ENABLED ? voltageCompensation(guard.volts, BATTERY_NOMINAL_VOLTS) : 1.0f;
}
