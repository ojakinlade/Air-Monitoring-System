#include <SoftwareSerial.h>
#include <Adafruit_BME280.h>
#include <MQ131.h>
#include <PMS.h>
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
  uint16_t pms2_5;
  uint16_t pms10_0;
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
  const uint8_t pmsTx = 9;
  const uint8_t pmsRx = 10;
};

//Object Instances of Sensors
Adafruit_BME280 bmeSensor;
MICS6814 micsSensor(Pin::NO2Pin,Pin::NH3Pin,Pin::COPin);
MP503 mp503(Pin::MP503_A,Pin::MP503_B);
SoftwareSerial nodeSerial(Pin::nodeRx,Pin::nodeTx);
SoftwareSerial pmsSerial(Pin::pmsTx,Pin::pmsRx);
PMS pms(pmsSerial);
PMS::DATA pmsData;
MNI mni(&nodeSerial);

SensorData_t dataToSend = {0};
static void Get_SensorData(SensorData_t& data);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Air Monitoring System...");
  pmsSerial.begin(9600);
  bmeSensor.begin(BME280_ADDR);
  //MQ131.begin(2,Pin::O3Sensor,LOW_CONCENTRATION,1000000);
//  Serial.println("Calibrating.......");
//  MQ131.calibrate();
//  Serial.println("Done Calibrating.");
}

void loop() {
  // put your main code here, to run repeatedly:
  nodeSerial.listen();
  if(mni.ReceivedData())
  {
    if(mni.DecodeData(MNI::RxDataId::DATA_QUERY) == MNI::QUERY)
    {
      Serial.println("Query Received");
      Get_SensorData(dataToSend); 
      //Debug
      Serial.print("PM 2.5 (ug/m3): ");
      Serial.println(dataToSend.pms2_5);
      Serial.print("PM 10.0 (ug/m3): ");
      Serial.println(dataToSend.pms10_0);
      
      mni.EncodeData(MNI::ACK,MNI::TxDataId::DATA_ACK);
      mni.EncodeData((dataToSend.temp * 100),MNI::TxDataId::TEMP);
      mni.EncodeData((dataToSend.hum * 100),MNI::TxDataId::HUM);
      mni.EncodeData(dataToSend.NO2,MNI::TxDataId::NO2);
      mni.EncodeData(dataToSend.NH3,MNI::TxDataId::NH3);
      mni.EncodeData(dataToSend.CO,MNI::TxDataId::CO);
      mni.EncodeData(dataToSend.pinAState,MNI::TxDataId::PIN_A_STATE);
      mni.EncodeData(dataToSend.pinBState,MNI::TxDataId::PIN_B_STATE);
      mni.EncodeData((dataToSend.O3 * 100),MNI::TxDataId::O3);
      mni.EncodeData(dataToSend.pms2_5,MNI::TxDataId::PMS2_5);
      mni.EncodeData(dataToSend.pms10_0,MNI::TxDataId::PMS10_0);
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
  pmsSerial.listen();
  pms.requestRead();
  if(pms.readUntil(pmsData))
  {
    data.pms2_5 = pmsData.PM_AE_UG_2_5;
    data.pms10_0 = pmsData.PM_AE_UG_10_0;
  }
  
//  MQ131.sample();
//  data.O3 = MQ131.getO3(PPB);
}
