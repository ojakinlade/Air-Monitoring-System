#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <MQ131.h>
#include "MNI.h"
#include "mics6814.h"
#include "mp503.h"

#define BME280_ADDR 0x76

typedef struct
{
  float temp;
  float hum;
  uint16_t NO2;
  uint16_t NH3;
  uint16_t CO;
  uint16_t pinAState;
  uint16_t pinBState;
  float O3;
}SensorData_t;

namespace Pin
{
  const uint8_t NO2Pin = A0;
  const uint8_t NH3Pin = A1;
  const uint8_t COPin = A2;
  const uint8_t O3Sensor = A3;
  const uint8_t nodeRx = 6;
  const uint8_t nodeTx = 7;
  const uint8_t MP503_A = 4;
  const uint8_t MP503_B = 5;
};

//Object Instances of Sensors
Adafruit_BME280 bmeSensor;
MICS6814 micsSensor(Pin::NO2Pin,Pin::NH3Pin,Pin::COPin);
MP503 mp503(Pin::MP503_A,Pin::MP503_B);
SoftwareSerial nodeSerial(Pin::nodeRx,Pin::nodeTx);
MNI mni(&nodeSerial);

SensorData_t dataToSend = {0};
static void Get_SensorData(SensorData_t& data);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Air Monitoring System...");
  bmeSensor.begin(BME280_ADDR);
  //MQ131.begin(2,Pin::O3Sensor,LOW_CONCENTRATION,1000000);
//  Serial.println("Calibrating.......");
//  MQ131.calibrate();
//  Serial.println("Done Calibrating.");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(mni.ReceivedData())
  {
    if(mni.DecodeData(MNI::RxDataId::DATA_QUERY) == MNI::QUERY)
    {
      Serial.println("Query Received");
      Get_SensorData(dataToSend);
      
      //Debug
//      Serial.print("Temperature: ");
//      Serial.println(dataToSend.temp,2);
//      Serial.print("Humidity: ");
//      Serial.println(dataToSend.hum,2);
//      Serial.print("NO2 conc: ");
//      Serial.println(dataToSend.NO2);
//      Serial.print("NH3 conc: ");
//      Serial.println(dataToSend.NH3);
//      Serial.print("CO conc: ");
//      Serial.println(dataToSend.CO);
//      Serial.print("A: ");
//      Serial.println(dataToSend.pinAState);
//      Serial.print("B: ");
//      Serial.println(dataToSend.pinBState);
//      Serial.print("O3 conc: ");
//      Serial.println(dataToSend.O3,2);

      mni.EncodeData(MNI::ACK,MNI::TxDataId::DATA_ACK);
      mni.EncodeData((dataToSend.temp * 100),MNI::TxDataId::TEMP);
      mni.EncodeData((dataToSend.hum * 100),MNI::TxDataId::HUM);
      mni.EncodeData(dataToSend.NO2,MNI::TxDataId::NO2);
      mni.EncodeData(dataToSend.NH3,MNI::TxDataId::NH3);
      mni.EncodeData(dataToSend.CO,MNI::TxDataId::CO);
      mni.EncodeData(dataToSend.pinAState,MNI::TxDataId::PIN_A_STATE);
      mni.EncodeData(dataToSend.pinBState,MNI::TxDataId::PIN_B_STATE);
      mni.EncodeData((dataToSend.O3 * 100),MNI::TxDataId::O3);
      mni.TransmitData();
    }
  }
}

void Get_SensorData(SensorData_t& data)
{
  data.temp = bmeSensor.readTemperature();
  data.hum = bmeSensor.readHumidity();
  data.NO2 = micsSensor.GetValue(MICS6814::GAS::NO2);
  data.NH3 = micsSensor.GetValue(MICS6814::GAS::NH3);
  data.CO = micsSensor.GetValue(MICS6814::GAS::CO);
  data.pinAState = (uint16_t) mp503.GetState(Pin::MP503_A);
  data.pinBState = (uint16_t)mp503.GetState(Pin::MP503_B);
//  MQ131.sample();
//  data.O3 = MQ131.getO3(PPB);
}
