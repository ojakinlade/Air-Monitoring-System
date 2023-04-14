#include <Arduino.h>
#include "bme280.h"

/**
 * @brief Constructor.
 * Initializes the BME280 sensor using the provided I2C address.
 * @param addr: I2C address of the BME280 sensor.
 */
BME::BME(uint8_t addr)
{
  this->deviceAddr = addr;
  bme.begin(this->deviceAddr);
}

/**
 * @brief Read temperature or humidity data from the BME280 sensor.
 * @param value: The type of data to read (temperature or humidity).
 * @return The temperature or humidity data, as a floating point value.
 */
float BME::GetValue(uint8_t value)
{
  switch(value)
  {
    case BME::TEMP:
      return bme.readTemperature();

    case BME::HUM:
      return bme.readHumidity();
  }
}
