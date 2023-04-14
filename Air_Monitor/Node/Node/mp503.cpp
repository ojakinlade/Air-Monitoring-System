#include <Arduino.h>
#include "mp503.h"

MP503::MP503(uint8_t A_out, uint8_t B_out)
{
  this->A_out = A_out;
  this->B_out = B_out;

  pinMode(this->A_out, INPUT);
  pinMode(this->B_out, INPUT);
}

bool MP503::GetState(uint8_t pin)
{
  return digitalRead(pin);
}
