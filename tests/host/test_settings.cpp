// Tunable settings: lookup by name or alias, range limits, corrupt storage.
#include "check.h"
#include "settings.h"

TEST(finds_parameters_by_name_and_original_alias) {
  CHECK(findParam("kp") == findParam("p"));
  CHECK(findParam("timeout") == findParam("x"));
  CHECK(findParam("kp") != nullptr);
  CHECK(findParam("nope") == nullptr);
}

TEST(set_param_writes_value_inside_range) {
  Settings s;
  CHECK(setParam(s, *findParam("kd"), 0.9f) == SetResult::Ok);
  CHECK_NEAR(s.kd, 0.9, 1e-6);
}

TEST(set_param_clamps_out_of_range_values) {
  Settings s;
  CHECK(setParam(s, *findParam("ramp"), 0.0f) == SetResult::Clamped);
  CHECK_NEAR(s.tiltRampTime, 0.05, 1e-6);  // 0 would divide by zero in the ramp
  CHECK(setParam(s, *findParam("timeout"), 10.0f) == SetResult::Clamped);
  CHECK_NEAR(s.remoteTimeoutMs, 50.0, 1e-6);  // an unsafe 0 ms failsafe is impossible
}

TEST(sanitize_repairs_corrupt_values) {
  Settings s;
  s.kp = 1e9f;
  s.ki = 0.0f / 0.0f;  // NaN
  sanitize(s);
  CHECK_NEAR(s.kp, 100.0, 1e-6);
  CHECK_NEAR(s.ki, Settings().ki, 1e-6);
}

TEST(defaults_are_inside_their_ranges) {
  Settings s;
  const Settings before = s;
  sanitize(s);
  size_t count = 0;
  const ParamInfo* params = paramTable(count);
  for (size_t i = 0; i < count; ++i) {
    CHECK(s.*(params[i].field) == before.*(params[i].field));
  }
}
