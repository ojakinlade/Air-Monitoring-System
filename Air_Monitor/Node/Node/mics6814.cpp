#include <Arduino.h>
#include "mics6814.h"

/**
 * @brief Constructor.
 * @param NO2Pin: Analog pin number for reading NO2 data.
 * @param NH3Pin: Analog pin number for reading NH3 data.
 * @param COPin: Analog pin number for reading CO data.
 */
MICS6814::MICS6814(uint8_t NO2Pin,
                   uint8_t NH3Pin, 
                   uint8_t COPin)
{
  this->NO2Pin = NO2Pin;
  this->NH3Pin = NH3Pin;
  this->COPin = COPin;

  pinMode(this->NO2Pin, INPUT_PULLUP);
  pinMode(this->NH3Pin, INPUT_PULLUP);
  pinMode(this->COPin, INPUT_PULLUP);
}

/**
 * @brief Read gas data from the MICS6814 sensor.
 * @param value: The type of gas data to read (NO2, NH3, or CO).
 * @return The gas data, as a 16-bit unsigned integer in parts per million (ppm).
 */
uint16_t MICS6814::GetValue(uint8_t value)
{
  uint16_t gasData;
  switch(value)
  {
    case GAS::NO2:
      gasData = analogRead(this->NO2Pin);
      return map(gasData,0,1023,0.05,10);
    case GAS::NH3:
      gasData = analogRead(this->NH3Pin);
      return map(gasData,0,1023,1,500);
    case GAS::CO:
      gasData = analogRead(this->COPin);
      return map(gasData,0,1023,1,1000); 
    default:
      return 0; 
  }
}
