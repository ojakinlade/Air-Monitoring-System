#pragma once

class MP503
{
  private:
    uint8_t A_out;
    uint8_t B_out;
    
  public:
    MP503(uint8_t A_out, uint8_t B_out);
    bool GetState(uint8_t pin);
};
