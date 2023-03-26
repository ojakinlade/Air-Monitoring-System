#include <iostream>
#include "AQI_Calc.h"

using namespace std;

AQI_Calc aqiCalc;
float aqi;

int main()
{
    aqi = aqiCalc.ComputeIndex(P_PM2_5,70);
    cout << aqi << endl;
    return 0;
}
