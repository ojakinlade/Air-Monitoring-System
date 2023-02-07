#include <Arduino.h>
#include "O3_Sensor.h"

O3::O3(uint8_t pin)
{
  MQ131.begin(2,pin,LOW_CONCENTRATION,1000000);
  //MQ131.calibrate();
}

float O3::GetValue(void)
{
  MQ131.sample();
  return MQ131.getO3(PPB);
}
