// Angle filter, gyro calibration and ramps.
#include "check.h"
#include "control.h"

namespace {

// Runs the filter for `seconds` with constant inputs and returns the angle.
float runFilter(ComplementaryFilter& filter, float accelAngle, float gyroRate, float dt,
                float seconds) {
  float angle = filter.angle;
  for (float t = 0.0f; t < seconds; t += dt) angle = filter.update(accelAngle, gyroRate, dt, 0.5f);
  return angle;
}

}  // namespace

TEST(filter_starts_from_accelerometer_angle) {
  ComplementaryFilter filter;
  CHECK_NEAR(filter.update(4.0f, 0.0f, 0.005f, 0.5f), 4.0, 1e-6);
}

TEST(filter_follows_gyro_in_the_short_term) {
  ComplementaryFilter filter;
  filter.update(0.0f, 0.0f, 0.005f, 0.5f);
  // 0.1 s at 20 deg/s while the accelerometer still says 0: mostly the gyro.
  const float angle = runFilter(filter, 0.0f, 20.0f, 0.005f, 0.1f);
  CHECK(angle > 1.5f && angle < 2.05f);
}

TEST(filter_settles_on_accelerometer_in_the_long_term) {
  ComplementaryFilter filter;
  filter.update(0.0f, 0.0f, 0.005f, 0.5f);
  CHECK_NEAR(runFilter(filter, 3.0f, 0.0f, 0.005f, 5.0f), 3.0, 0.01);
}

TEST(filter_response_does_not_depend_on_loop_rate) {
  // The original fixed 0.96/0.04 split behaved differently when the loop
  // slowed down; a time constant keeps the response the same.
  ComplementaryFilter fast, slow;
  fast.update(0.0f, 0.0f, 0.005f, 0.5f);
  slow.update(0.0f, 0.0f, 0.010f, 0.5f);
  const float a = runFilter(fast, 2.0f, 0.0f, 0.005f, 0.5f);
  const float b = runFilter(slow, 2.0f, 0.0f, 0.010f, 0.5f);
  CHECK_NEAR(a, b, 0.05);
}

TEST(uncorrected_gyro_bias_shifts_the_angle) {
  // Why calibration matters: 2 deg/s of bias leaves a steady 1 deg error.
  ComplementaryFilter filter;
  filter.update(0.0f, 0.0f, 0.005f, 0.5f);
  CHECK_NEAR(runFilter(filter, 0.0f, 2.0f, 0.005f, 10.0f), 1.0, 0.05);
}

TEST(calibrator_averages_the_resting_rate) {
  GyroCalibrator calibrator;
  for (int i = 0; i < 100; ++i) calibrator.add(i % 2 == 0 ? 1.9f : 2.1f);
  CHECK_NEAR(calibrator.bias(), 2.0, 1e-4);
  CHECK(calibrator.stillEnough(2.0f));
}

TEST(calibrator_notices_movement) {
  GyroCalibrator calibrator;
  calibrator.add(1.0f);
  calibrator.add(15.0f);  // someone bumped the robot
  CHECK(!calibrator.stillEnough(2.0f));
  calibrator.reset();
  CHECK(calibrator.samples == 0);
}

TEST(ramp_moves_by_at_most_one_step_and_stops_at_target) {
  CHECK_NEAR(rampToward(0.0f, 1.0f, 0.3f), 0.3, 1e-6);
  CHECK_NEAR(rampToward(0.9f, 1.0f, 0.3f), 1.0, 1e-6);
  CHECK_NEAR(rampToward(0.0f, -1.0f, 0.3f), -0.3, 1e-6);
  CHECK_NEAR(rampToward(0.5f, 0.5f, 0.3f), 0.5, 1e-6);
}
