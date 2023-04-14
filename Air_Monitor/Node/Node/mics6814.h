#pragma once

/**
 * @brief Class for reading data from a MICS 6814 sensor.
 */
class MICS6814
{
  private:
     /** Analog pin number for reading NO2 data */
    uint8_t NO2Pin;
    /** Analog pin number for reading NH3 data */
    uint8_t NH3Pin;
    /** Analog pin number for reading CO data */
    uint8_t COPin; 
  
  public: 
    /** Enumeration for the types of gas that can be measured */
    enum GAS{NO2 = 0, NH3, CO};

    MICS6814(uint8_t NO2Pin,
             uint8_t NH3Pin, 
             uint8_t COPin);

    uint16_t GetValue(uint8_t value);
};
