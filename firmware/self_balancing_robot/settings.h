// =============================================================================
// Tunable settings: defaults, allowed ranges and the names used in commands.
// Pure C++ (no Arduino dependencies) so it is unit-tested on the host.
// =============================================================================
#pragma once

#include <stddef.h>
#include <string.h>

// Bump when the Settings layout changes: stored settings with another version
// are ignored and the defaults are used instead.
constexpr unsigned kSettingsVersion = 2;

struct Settings {
  float kp = 15.0f;             // balance PID gains
  float ki = 0.71f;
  float kd = 0.44f;
  float baseAngle = 3.0f;       // IMU angle (deg) at which the robot is upright
  float moveTiltMax = 1.0f;     // max forward/back lean while driving (deg)
  float turnMax = 12.0f;        // max turn strength (motor command units)
  float tiltRampTime = 0.6f;    // seconds from 0 to full drive
  float turnRampTime = 1.0f;    // seconds from 0 to full turn
  float remoteTimeoutMs = 600;  // stop driving if the phone goes quiet this long
  bool autoTrim = false;        // slowly correct baseAngle while standing
  // Speed loop, used only with wheel encoders (ENCODERS_ENABLED in config.h):
  // lean (deg) per m/s of speed error, its integral gain, and the top speed.
  float speedKp = 3.0f;
  float speedKi = 1.0f;
  float maxSpeed = 0.3f;  // m/s
};

struct ParamInfo {
  const char* name;
  const char* alias;  // one-letter command kept from the original sketch, or ""
  float Settings::*field;
  float minValue;
  float maxValue;
  const char* description;
};

inline const ParamInfo* paramTable(size_t& count) {
  static const ParamInfo kParams[] = {
      {"kp", "p", &Settings::kp, 0.0f, 100.0f, "balance P gain"},
      {"ki", "i", &Settings::ki, 0.0f, 20.0f, "balance I gain"},
      {"kd", "d", &Settings::kd, 0.0f, 10.0f, "balance D gain"},
      {"base", "t", &Settings::baseAngle, -45.0f, 45.0f, "upright angle, deg"},
      {"lean", "m", &Settings::moveTiltMax, 0.0f, 10.0f, "max drive lean, deg"},
      {"turn", "n", &Settings::turnMax, 0.0f, 100.0f, "max turn strength"},
      {"ramp", "r", &Settings::tiltRampTime, 0.05f, 10.0f, "drive ramp time, s"},
      {"turnramp", "u", &Settings::turnRampTime, 0.05f, 10.0f, "turn ramp time, s"},
      {"timeout", "x", &Settings::remoteTimeoutMs, 50.0f, 5000.0f, "remote timeout, ms"},
      {"vkp", "", &Settings::speedKp, 0.0f, 50.0f, "speed loop P, deg per m/s"},
      {"vki", "", &Settings::speedKi, 0.0f, 50.0f, "speed loop I, deg per m"},
      {"vmax", "", &Settings::maxSpeed, 0.05f, 1.5f, "top speed with encoders, m/s"},
  };
  count = sizeof(kParams) / sizeof(kParams[0]);
  return kParams;
}

inline const ParamInfo* findParam(const char* name) {
  size_t count = 0;
  const ParamInfo* params = paramTable(count);
  for (size_t i = 0; i < count; ++i) {
    if (strcmp(name, params[i].name) == 0 || strcmp(name, params[i].alias) == 0) {
      return &params[i];
    }
  }
  return nullptr;
}

enum class SetResult { Ok, Clamped };

// Writes value into the parameter, clamped to its allowed range.
inline SetResult setParam(Settings& settings, const ParamInfo& param, float value) {
  float applied = value;
  if (applied < param.minValue) applied = param.minValue;
  if (applied > param.maxValue) applied = param.maxValue;
  settings.*(param.field) = applied;
  return applied == value ? SetResult::Ok : SetResult::Clamped;
}

// Pulls every parameter back into its range (protects against corrupt storage).
inline void sanitize(Settings& settings) {
  size_t count = 0;
  const ParamInfo* params = paramTable(count);
  const Settings defaults;
  for (size_t i = 0; i < count; ++i) {
    const float value = settings.*(params[i].field);
    if (value != value) {  // NaN: clamping cannot fix it, use the default
      settings.*(params[i].field) = defaults.*(params[i].field);
    } else {
      setParam(settings, params[i], value);
    }
  }
}
