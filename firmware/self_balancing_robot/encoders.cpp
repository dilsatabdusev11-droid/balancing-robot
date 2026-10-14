#include "encoders.h"

#include <Arduino.h>

#include "config.h"

namespace {

portMUX_TYPE countLock = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
int32_t leftTotal = 0;
int32_t rightTotal = 0;

// On each rising edge of channel A, channel B tells the direction.
void IRAM_ATTR onLeftEdge() {
  const bool b = digitalRead(PIN_ENC_LEFT_B) == HIGH;
  portENTER_CRITICAL_ISR(&countLock);
  leftCount += (b != LEFT_ENCODER_REVERSED) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&countLock);
}

void IRAM_ATTR onRightEdge() {
  const bool b = digitalRead(PIN_ENC_RIGHT_B) == HIGH;
  portENTER_CRITICAL_ISR(&countLock);
  rightCount += (b != RIGHT_ENCODER_REVERSED) ? 1 : -1;
  portEXIT_CRITICAL_ISR(&countLock);
}

}  // namespace

void encodersBegin() {
  if (!ENCODERS_ENABLED) return;
  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), onLeftEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), onRightEdge, RISING);
}

void encodersTake(int32_t& left, int32_t& right) {
  portENTER_CRITICAL(&countLock);
  left = leftCount;
  right = rightCount;
  leftCount = 0;
  rightCount = 0;
  portEXIT_CRITICAL(&countLock);
  leftTotal += left;
  rightTotal += right;
}

void encodersReadTotals(int32_t& left, int32_t& right) {
  left = leftTotal;
  right = rightTotal;
  leftTotal = 0;
  rightTotal = 0;
}
