// Two DC motors through an L298N or TB6612FNG driver.
#pragma once

void motorsBegin();
// Commands run from -100 to +100; positive drives the robot forward.
// voltageScale > 1 adds duty to make up for a drained battery.
void motorsDrive(float leftCommand, float rightCommand, float voltageScale);
void motorsStop();
