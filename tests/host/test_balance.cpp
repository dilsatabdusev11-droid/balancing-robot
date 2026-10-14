// Balance PID, auto-trim, arming and the motor test.
#include "balance.h"
#include "check.h"

namespace {

constexpr PidGains kGains = {15.0f, 0.7f, 0.44f};
constexpr ArmingConfig kArm = {5.0f, 0.5f, 30.0f};
constexpr float kDt = 0.005f;

ArmingInputs upright(float tilt) { return {true, false, false, false, tilt}; }

RobotState hold(Arming& arming, const ArmingInputs& in, float seconds) {
  for (float t = 0.0f; t < seconds; t += kDt) arming.update(in, kDt, kArm);
  return arming.state;
}

}  // namespace

TEST(pid_drives_forward_when_leaning_forward) {
  BalancePid pid;
  CHECK(pid.update(2.0f, 0.0f, kDt, kGains, 60.0f, 100.0f) > 0.0f);
  pid.reset();
  CHECK(pid.update(-2.0f, 0.0f, kDt, kGains, 60.0f, 100.0f) < 0.0f);
}

TEST(pid_has_no_derivative_kick_on_target_jump) {
  // Differentiating the error turned a 5 deg jump into ~440 units of output
  // for one step. With the gyro rate as D input only P and I respond.
  BalancePid pid;
  const float output = pid.update(5.0f, 0.0f, kDt, kGains, 60.0f, 100.0f);
  CHECK_NEAR(output, 15.0f * 5.0f + 0.7f * 5.0f * kDt, 1e-3);
}

TEST(pid_damps_with_tilt_rate) {
  BalancePid pid;
  // Falling forward fast: the D term adds forward drive.
  CHECK_NEAR(pid.update(0.0f, 10.0f, kDt, kGains, 60.0f, 100.0f), 4.4, 1e-4);
}

TEST(pid_output_is_limited) {
  BalancePid pid;
  CHECK_NEAR(pid.update(50.0f, 0.0f, kDt, kGains, 60.0f, 100.0f), 100.0, 1e-6);
}

TEST(pid_integral_does_not_wind_up_while_saturated) {
  BalancePid pid;
  for (int i = 0; i < 1000; ++i) pid.update(20.0f, 0.0f, kDt, kGains, 60.0f, 100.0f);
  CHECK_NEAR(pid.integral, 0.0, 1e-6);  // output was saturated the whole time
}

TEST(autotrim_moves_target_back_when_robot_keeps_driving_forward) {
  AutoTrim trim;
  trim.update(10.0f, 1.0f, true, 0.01f, 0.0f, 2.0f);
  CHECK(trim.offset < 0.0f);
}

TEST(autotrim_converges_to_true_balance_point) {
  // Simple model: with an angle-only controller the robot has to keep driving
  // forward when the target leans forward of the balance point, so the
  // average output grows with (target - balancePoint).
  const float balancePoint = 1.0f;
  AutoTrim trim;
  for (float t = 0.0f; t < 120.0f; t += kDt) {
    const float output = 20.0f * (0.0f + trim.offset - balancePoint);
    trim.update(output, kDt, true, 0.01f, 5.0f, 2.0f);
  }
  CHECK_NEAR(trim.offset, balancePoint, 0.05);
}

TEST(autotrim_is_bounded_and_ignores_driving) {
  AutoTrim trim;
  for (int i = 0; i < 10000; ++i) trim.update(-100.0f, 0.1f, true, 0.01f, 0.0f, 2.0f);
  CHECK_NEAR(trim.offset, 2.0, 1e-6);
  AutoTrim idle;
  idle.update(50.0f, 1.0f, false, 0.01f, 0.0f, 2.0f);
  CHECK_NEAR(idle.offset, 0.0, 1e-6);
  CHECK_NEAR(idle.average, 0.0, 1e-6);
}

TEST(arming_waits_for_calibration_then_hold_time) {
  Arming arming;
  CHECK(arming.update({true, true, false, false, 0.0f}, kDt, kArm) == RobotState::Calibrating);
  CHECK(arming.update(upright(1.0f), kDt, kArm) == RobotState::Waiting);
  CHECK(hold(arming, upright(1.0f), 0.3f) == RobotState::Waiting);
  CHECK(hold(arming, upright(1.0f), 0.3f) == RobotState::Balancing);
}

TEST(robot_lying_at_25_degrees_never_starts) {
  // The original sketch drove at full power below 30 deg.
  Arming arming;
  arming.update(upright(25.0f), kDt, kArm);
  CHECK(hold(arming, upright(25.0f), 5.0f) == RobotState::Waiting);
}

TEST(fall_stops_and_needs_rearming) {
  Arming arming;
  hold(arming, upright(0.0f), 1.0f);
  CHECK(arming.update(upright(35.0f), kDt, kArm) == RobotState::Fallen);
  CHECK(arming.update(upright(1.0f), kDt, kArm) == RobotState::Fallen);  // not at once
  CHECK(hold(arming, upright(1.0f), 0.6f) == RobotState::Balancing);
}

TEST(emergency_stop_holds_until_released) {
  Arming arming;
  hold(arming, upright(0.0f), 1.0f);
  CHECK(arming.update({true, false, true, false, 0.0f}, kDt, kArm) == RobotState::Stopped);
  CHECK(hold(arming, {true, false, true, false, 0.0f}, 2.0f) == RobotState::Stopped);
  CHECK(arming.update(upright(0.0f), kDt, kArm) == RobotState::Waiting);
}

TEST(imu_fault_and_low_battery_block_balancing) {
  Arming arming;
  hold(arming, upright(0.0f), 1.0f);
  CHECK(arming.update({false, false, false, false, 0.0f}, kDt, kArm) == RobotState::ImuFault);
  CHECK(arming.update({true, false, false, true, 0.0f}, kDt, kArm) == RobotState::LowBattery);
  CHECK(hold(arming, {true, false, false, true, 0.0f}, 2.0f) == RobotState::LowBattery);
}

TEST(motor_test_runs_left_then_right_then_stops) {
  MotorTest test;
  float left = 0.0f, right = 0.0f;
  CHECK(!test.step(kDt, 35.0f, left, right));  // not started
  test.start();
  CHECK(test.step(0.1f, 35.0f, left, right) && left == 35.0f && right == 0.0f);
  CHECK(test.step(0.9f, 35.0f, left, right) && left == 0.0f && right == 0.0f);   // t = 1.0
  CHECK(test.step(0.5f, 35.0f, left, right) && left == 0.0f && right == 35.0f);  // t = 1.5
  CHECK(!test.step(1.0f, 35.0f, left, right));
  CHECK(!test.running());
}
