// Motor duty shaping and the battery guard.
#include "check.h"
#include "drive_math.h"

namespace {
constexpr DutyShape kShape = {2.0f, 70, 230};
constexpr BatteryLimits kLimits = {6.4f, 6.8f, 2.0f, 0.0f};  // no filtering: easier to reason about
}  // namespace

TEST(zero_command_gives_zero_duty) {
  CHECK(commandToDuty(0.0f, kShape) == 0);
  CHECK(commandToDuty(0.0f / 0.0f, kShape) == 0);  // NaN must never spin a motor
}

TEST(duty_has_no_jump_near_zero) {
  // The original sketch jumped from 0 to 70 at |command| = 0.05, which made the
  // motors chatter around the balance point. Now tiny commands give tiny duty.
  CHECK(commandToDuty(0.05f, kShape) <= 2);
  CHECK(commandToDuty(1.0f, kShape) == 35);
}

TEST(duty_is_continuous_at_start_command) {
  const int below = commandToDuty(1.999f, kShape);
  const int at = commandToDuty(2.0f, kShape);
  CHECK(at == 70);
  CHECK(at - below <= 1);
}

TEST(duty_reaches_max_and_saturates) {
  CHECK(commandToDuty(100.0f, kShape) == 230);
  CHECK(commandToDuty(500.0f, kShape) == 230);
}

TEST(duty_ignores_direction) {
  CHECK(commandToDuty(-37.0f, kShape) == commandToDuty(37.0f, kShape));
}

TEST(voltage_compensation_boosts_a_drained_pack_within_limits) {
  CHECK_NEAR(voltageCompensation(7.4f, 7.4f), 1.0, 1e-6);
  CHECK(voltageCompensation(6.8f, 7.4f) > 1.0f);
  CHECK_NEAR(voltageCompensation(3.0f, 7.4f), 1.3, 1e-6);
  CHECK_NEAR(voltageCompensation(0.0f, 7.4f), 1.0, 1e-6);  // no reading: no change
}

TEST(battery_guard_ignores_short_dips) {
  BatteryGuard guard;
  guard.update(6.0f, 1.0f, kLimits);
  guard.update(7.2f, 0.5f, kLimits);
  guard.update(6.0f, 1.5f, kLimits);
  CHECK(!guard.low);
}

TEST(battery_guard_trips_after_hold_time_and_needs_recovery) {
  BatteryGuard guard;
  guard.update(6.2f, 1.0f, kLimits);
  CHECK(!guard.low);
  guard.update(6.2f, 1.0f, kLimits);
  CHECK(guard.low);
  guard.update(6.6f, 1.0f, kLimits);  // above cutoff but below resume: stay stopped
  CHECK(guard.low);
  guard.update(7.0f, 1.0f, kLimits);
  CHECK(!guard.low);
}
