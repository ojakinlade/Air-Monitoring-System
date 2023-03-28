#include <Arduino.h>
#include "AQI_Calc.h"

/**
 * @brief Constructor for the AQI_Calc class.
*/
AQI_Calc::AQI_Calc(void)
  //Assign the range and breakpoint for each pollutant
  : PM10 {{0,54,0,50},{55,154,51,100},
          {155,254,101,150},{255,354,151,200},
          {355,424,201,300},{425,504,301,400},
          {505,604,401,500}},
   PM2_5 {{0.0,15.4,0,50},{15.5,40.4,51,100},
          {40.5,65.4,101,150},{65.5,150.4,151,200},
          {150.5,250.4,201,300},{250.5,350.4,301,400},
          {350.5,500.4,401,500}},
   CO    {{0.0,4.4,0,50},{4.5,9.4,51,100},
          {9.5,12.4,101,150},{12.5,15.4,151,200},
          {15.5,30.4,201,300},{30.5,40.4,301,400},
          {40.5,50.4,401,500}},
   NO2   {{0.0,0.0,0,50},{0.0,0.0,51,100},
          {0.0,0.0,101,150},{0.0,0.0,151,200},
          {0.65,1.24,201,300},{1.25,1.64,301,400},
          {1.65,2.04,401,500}}
{
  //Initialize private variables
  Pollutant[P_PM10] = PM10;
  Pollutant[P_PM2_5] = PM2_5;
  Pollutant[P_CO] = CO;
  Pollutant[P_NO2] = NO2;
}

/**
 * @brief Method to get the AQI level of a pollutant
 * @param pollutant Pollutant for which the AQI level is to be calculated
 * @param pollutantConc Concentration of the pollutant
 * @return AQI level of the pollutant
*/
AQI_LEVEL AQI_Calc::GetAQILevel(POLLUTANTS pollutant,float pollutantConc)
{
  AQI_LEVEL level = INVALID;
  switch(pollutant)
  {
    case P_PM10:
      for(uint8_t i = 0; i < NO_OF_AQI_LEVELS; i++)
      {
        if(pollutantConc >= PM10[i].BP_Low && pollutantConc <= PM10[i].BP_High)
        {
          level = (AQI_LEVEL)i;
        }
      }
    break;   
    case P_PM2_5:
      for(uint8_t i = 0; i < NO_OF_AQI_LEVELS; i++)
      {
        if(pollutantConc >= PM2_5[i].BP_Low && pollutantConc <= PM2_5[i].BP_High)
        {
          level = (AQI_LEVEL)i;
        }
      }
    break;
    case P_CO:
      for(uint8_t i = 0; i < NO_OF_AQI_LEVELS; i++)
      {
        if(pollutantConc >= CO[i].BP_Low && pollutantConc <= CO[i].BP_High)
        {
          level = (AQI_LEVEL)i;
        }
      }
    break;
    case P_NO2:
      for(uint8_t i = 0; i < NO_OF_AQI_LEVELS; i++)
      {
        if(pollutantConc >= NO2[i].BP_Low && pollutantConc <= NO2[i].BP_High)
        {
          level = (AQI_LEVEL)i;
        }
      }
    break;
  }
  return level;
}

/**
 * @brief Method to get the AQI Index of a pollutant
 * @param pollutant Pollutant for which the AQI level is to be calculated
 * @param pollutantConc Concentration of the pollutant
 * @return AQI Index of the pollutant
*/
float AQI_Calc::ComputeIndex(POLLUTANTS pollutant,float pollutantConc)
{
  AQI_LEVEL level;
  level = AQI_Calc::GetAQILevel(pollutant,pollutantConc);
  switch(pollutant)
  {
    case P_PM10:
      PM10[level].Index = (pollutantConc - PM10[level].BP_Low)*
                          ((PM10[level].I_High - PM10[level].I_Low)/
                          (PM10[level].BP_High - PM10[level].BP_Low)) +
                           PM10[level].I_Low;
    break;
    case P_PM2_5:
      PM2_5[level].Index = (pollutantConc - PM2_5[level].BP_Low)*
                           ((PM2_5[level].I_High - PM2_5[level].I_Low)/
                           (PM2_5[level].BP_High - PM2_5[level].BP_Low)) +
                            PM2_5[level].I_Low;
    break;
    case P_CO:
      CO[level].Index = (pollutantConc - CO[level].BP_Low)*
                        ((CO[level].I_High - CO[level].I_Low)/
                        (CO[level].BP_High - CO[level].BP_Low)) +
                         CO[level].I_Low;
    break;
    case P_NO2:
      NO2[level].Index = (pollutantConc - NO2[level].BP_Low)*
                         ((NO2[level].I_High - NO2[level].I_Low)/
                         (NO2[level].BP_High - NO2[level].BP_Low)) +
                          NO2[level].I_Low;
    break;
  }
  return Pollutant[pollutant][level].Index;
}
