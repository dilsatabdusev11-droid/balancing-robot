// Battery voltage through a resistor divider (optional, see config.h).
#pragma once

void batteryBegin();
// Call often; samples the voltage every few milliseconds.
void batteryPoll();
// Filtered pack voltage, or 0 when the monitor is disabled.
float batteryVolts();
// True while the pack is below the cutoff (the robot must not run).
bool batteryLow();
// Motor duty multiplier for the current voltage (1.0 when disabled).
float batteryCompensation();
