#pragma once

class MQ7
{
  private:
    uint8_t analogPin;
    float R0;
  public:
    MQ7(uint8_t analogPin);
    float GetPPM(void);
};
