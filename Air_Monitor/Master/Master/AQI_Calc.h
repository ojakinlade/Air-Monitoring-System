#pragma once

#define NO_OF_AQI_LEVELS  7
#define NO_OF_POLLUTANTS  4

/**
 * @brief Struct to hold range and breakpoint values
 * of a pollutant
*/
typedef struct
{
  float BP_Low;
  float BP_High;
  float I_Low;
  float I_High;
  float Index;
}AQI_Param_t;

enum AQI_LEVEL{INVALID = -1,GOOD = 0,MODERATE,UNHEALTY_SG,
                UNHEALTHY,VERY_UNHEALTY,HAZARDOUS,
                VERY_HAZARDOUS};
enum POLLUTANTS{P_PM10 = 0,P_PM2_5,P_CO,P_NO2};

class AQI_Calc
{
  private:
    AQI_Param_t PM10[NO_OF_AQI_LEVELS];
    AQI_Param_t PM2_5[NO_OF_AQI_LEVELS];
    AQI_Param_t CO[NO_OF_AQI_LEVELS];
    AQI_Param_t NO2[NO_OF_AQI_LEVELS];
    AQI_LEVEL GetAQILevel(POLLUTANTS pollutant,float pollutantConc);
    AQI_Param_t* Pollutant[NO_OF_POLLUTANTS];

  public:
    AQI_Calc(void);
    float ComputeIndex(POLLUTANTS pollutant,float pollutantConc);
};
