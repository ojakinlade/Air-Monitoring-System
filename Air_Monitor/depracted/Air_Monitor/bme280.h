#pragma once

#include <Adafruit_BME280.h>

/**
 * @brief Class for reading data from a BME280 sensor.
*/
class BME
{
  private:
    /** I2C address of the BME280 sensor */
    
  public:
    /* Enumeration for the types of data that can be read from the sensor */
    enum{TEMP = 0, HUM};

    /**
     * @brief Constructor
     * @param value: addr I2C address of the BME280 sensor
    */
    BME(Adafruit_BME280& bmePtr,uint8_t addr);

    /**
     * @brief Read temperature or humidity data from the BME280 sensor.
     * @param value: The type of data to read (temperature or humidity).
     * @return The temperature or humidity data, as a floating point value.
     */
    float GetValue(Adafruit_BME280& bmePtr,uint8_t value); 
};
