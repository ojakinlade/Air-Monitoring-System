#include "MQ7.h"

MQ7 mq7(A3);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(mq7.GetPPM());
  delay(1000);
}
