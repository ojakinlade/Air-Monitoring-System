#include <SoftwareSerial.h>
#include <PMS.h>

#define pmsTx 2
#define pmsRx 3

SoftwareSerial pmsSerial(pmsTx,pmsRx);
PMS pms(pmsSerial);
PMS::DATA data;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pmsSerial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  pms.requestRead();
  while(!pms.readUntil(data))
  {}
  
  Serial.print("PM 1.0 (ug/m3): ");
  Serial.println(data.PM_AE_UG_1_0);
  
  Serial.print("PM 2.5 (ug/m3): ");
  Serial.println(data.PM_AE_UG_2_5);

  Serial.print("PM 10.0 (ug/m3): ");
  Serial.println(data.PM_AE_UG_10_0);

  Serial.println();
  delay(1000);
}
