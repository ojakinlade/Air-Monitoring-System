/*******************************************************************************
 * Calibrate the MQ131 and start sampling every 60 seconds
 * 
 * Example code base on low concentration sensor (black bakelite)
 * and load resistance of 1MOhms
 * 
 * 
  */
#include <MQ131.h>

void setup() {
  Serial.begin(9600);

  // Init the sensor
  // - Heater control on pin 2
  // - Sensor analog read on pin A0
  // - Model LOW_CONCENTRATION
  // - Load resistance RL of 1MOhms (1000000 Ohms)
  MQ131.begin(2,A3, LOW_CONCENTRATION, 1000000);  

  Serial.println("Calibration in progress...");
  Serial.print("Initial Time: ");
  Serial.println(millis());
  MQ131.calibrate();
  Serial.print("Finish Time: ");
  Serial.println(millis());
  
  Serial.println("Calibration done!");
//  Serial.print("R0 = ");
//  Serial.print(MQ131.getR0());
//  Serial.println(" Ohms");
//  Serial.print("Time to heat = ");
//  Serial.print(MQ131.getTimeToRead());
//  Serial.println(" s");
}

void loop() {
  Serial.println("Sampling...");
  Serial.print("Data init Time: ");
  Serial.println(millis());
  MQ131.sample();
  Serial.print("Data Finish Time: ");
  Serial.println(millis());
  Serial.print("Concentration O3 : ");
//  Serial.print(MQ131.getO3(PPM));
//  Serial.println(" ppm");
  Serial.print("Concentration O3 : ");
  Serial.print(MQ131.getO3(PPB));
  Serial.println(" ppb");
//  Serial.print("Concentration O3 : ");
//  Serial.print(MQ131.getO3(MG_M3));
//  Serial.println(" mg/m3");
//  Serial.print("Concentration O3 : ");
//  Serial.print(MQ131.getO3(UG_M3));
//  Serial.println(" ug/m3");

  delay(2000);
}
