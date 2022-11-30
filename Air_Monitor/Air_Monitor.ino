#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME280_ADDR   0x76

typedef struct
{
  uint32_t temp;
  uint32_t hum;
}airParam_t;

/* The values stored in members of airParam_t struct 
 * must be conveterted back to their double form before being 
 * used.
 */
airParam_t airParam;
Adafruit_BME280 bme;

void Get_SensorsData(void)
{
  airParam.temp = bme.readTemperature() * 100;
  airParam.hum = bme.readHumidity() * 100;
  
  Serial.print("Temperature = ");
  //Serial.print(bme.readTemperature());
  Serial.print(airParam.temp);
  Serial.println(" °C");
  
  Serial.print("Humidity = ");
  //Serial.print(bme.readHumidity());
  Serial.print(airParam.hum);
  Serial.println(" %");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  bme.begin(BME280_ADDR);
}

void loop() {
  // put your main code here, to run repeatedly:
  Get_SensorsData();

  delay(1000);
}
