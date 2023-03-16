#include <Arduino.h>
#include "MQ7.h"

MQ7::MQ7(uint8_t analogPin)
{
 this->analogPin = analogPin;
 this->R0 = 55000.0;
}

float MQ7::GetPPM(void)
{
  uint16_t sensorRawValue = analogRead(this->analogPin);
  float sensorVolt = sensorRawValue/1024.0*5;
  float RS_gas = (5.0-sensorVolt)/sensorVolt;
  float ratio = RS_gas/this->R0;
  float x = 1538.46*ratio;
  float ppm = pow(x,-1.709);
  return ppm;
}
