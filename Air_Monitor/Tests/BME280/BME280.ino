#include <Wire.h>
#include <Adafruit_BME280.h>

#define BME280_ADDR   0x76

Adafruit_BME280 bme;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bme.begin(BME280_ADDR);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Temperature: ");
  Serial.println(bme.readTemperature());
  Serial.print("Humidity: ");
  Serial.println(bme.readHumidity());
  delay(1000);
}
