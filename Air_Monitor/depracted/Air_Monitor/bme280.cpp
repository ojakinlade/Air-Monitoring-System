#include <Arduino.h>
#include "bme280.h"

/**
 * @brief Constructor.
 * Initializes the BME280 sensor using the provided I2C address.
 * @param addr: I2C address of the BME280 sensor.
 */
BME::BME(Adafruit_BME280& bmePtr,uint8_t addr)
{
  Serial.begin(9600);
  Serial.print('1');
  //this->bmePtr = bmePtr;
  Serial.print('2');
  bmePtr.begin(addr);
  Serial.print('3');
}

/**
 * @brief Read temperature or humidity data from the BME280 sensor.
 * @param value: The type of data to read (temperature or humidity).
 * @return The temperature or humidity data, as a floating point value.
 */
float BME::GetValue(Adafruit_BME280& bmePtr,uint8_t value)
{
  float bmeData;
  switch(value)
  {
    case BME::TEMP:
      bmeData = bmePtr.readTemperature();
      break;

    case BME::HUM:
      bmeData = bmePtr.readHumidity();
      break;
  }
  return bmeData;
}
