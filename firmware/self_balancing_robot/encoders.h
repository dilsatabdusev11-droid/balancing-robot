// Wheel encoder counting (optional, see ENCODERS_ENABLED in config.h).
#pragma once

#include <stdint.h>

void encodersBegin();
// Counts since the previous call; positive = the wheel rolled forward.
void encodersTake(int32_t& left, int32_t& right);
// Counts since the previous encodersReadTotals() call, for the "encoders"
// direction check. Updated by encodersTake().
void encodersReadTotals(int32_t& left, int32_t& right);
