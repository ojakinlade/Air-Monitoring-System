float sensor_volt;
float RS_gas;
float R0;
int R2 = 2000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorRawValue = analogRead(A0);
  sensor_volt = (float)(sensorRawValue/1024.0)*5.0;
  RS_gas = ((5.0*R2)/sensor_volt) - R2;
  R0 = RS_gas/1;
  Serial.print("R0: ");
  Serial.println(R0);
  delay(1000);
}
