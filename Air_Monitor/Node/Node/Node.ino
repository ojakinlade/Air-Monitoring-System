#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <MQ131.h>
#include "MNI.h"
#include "mics6814.h"
#include "mp503.h"

#define BME280_ADDR 0x76

typedef struct
{
  uint16_t temp;
  uint16_t hum;
  uint16_t NO2;
  uint16_t NH3;
  uint16_t CO;
  uint16_t ozone;
}SensorData_t;

enum MICS_Data
{
  NO2 = 0,
  NH3,
  CO
};

enum MP503_PINS
{
  A = 0,
  B
};

namespace Pin
{
  const uint8_t NO2Pin = A0;
  const uint8_t NH3Pin = A1;
  const uint8_t COPin = A2;
  const uint8_t O3Sensor = A3;
  const uint8_t MP503_A = 4;
  const uint8_t MP503_B = 5;
};

//Object Instances of Sensors
Adafruit_BME280 bmeSensor;
MICS6814 micsSensor(Pin::NO2Pin,Pin::NH3Pin,Pin::COPin);
MP503 mp503(Pin::MP503_A,Pin::MP503_B);

SensorData_t dataToSend = {0};
static void Get_SensorData(SensorData_t& data);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Air Monitoring System...");
  bmeSensor.begin(BME280_ADDR);
  MQ131.begin(2,Pin::O3Sensor,LOW_CONCENTRATION,1000000);
  Serial.println("Calibrating.......");
  MQ131.calibrate();
  Serial.println("Done Calibrating.");
}

void loop() {
  // put your main code here, to run repeatedly:
  Get_SensorData(dataToSend);
  Serial.print("Temperature: ");
  Serial.println(dataToSend.temp);
  Serial.print("Humidity: ");
  Serial.println(dataToSend.hum);
  Serial.print("NO2 conc: ");
  Serial.println(dataToSend.NO2);
  Serial.print("NH3 conc: ");
  Serial.println(dataToSend.NH3);
  Serial.print("CO conc: ");
  Serial.println(dataToSend.CO);
  Serial.print("A: ");
  Serial.println(mp503.GetState(A));
  Serial.print("B: ");
  Serial.println(mp503.GetState(B));
  Serial.print("O3 conc: ");
  Serial.println(dataToSend.ozone);
  delay(2000);
}

void Get_SensorData(SensorData_t& data)
{
  data.temp = bmeSensor.readTemperature() * 100;
  data.hum = bmeSensor.readHumidity() * 100;
  data.NO2 = micsSensor.GetValue(NO2);
  data.NH3 = micsSensor.GetValue(NH3);
  data.CO = micsSensor.GetValue(CO);
  MQ131.sample();
  data.ozone = MQ131.getO3(PPB) * 100;
}
