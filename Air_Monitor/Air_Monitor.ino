#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME280_ADDR   0x76
#define SIZE_TOPIC    30

//RTOS
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE  0
#else
#define ARDUINO_RUNNING_CORE  1
#endif

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
  Serial.print(airParam.temp);
  Serial.println(" °C");
  
  Serial.print("Humidity = ");
  Serial.print(airParam.hum);
  Serial.println(" %");
}

void setup() {
  // put your setup code here, to run once:
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  bme.begin(BME280_ADDR);
  //Create Tasks
  xTaskCreatePinnedToCore(WiFiManagementTask,"",7000,NULL,1,NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(SensorsTask,"",7000,NULL,1,NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(MqttTask,"",7000,NULL,1,NULL,ARDUINO_RUNNING_CORE);
}

void loop() {
  // put your main code here, to run repeatedly:
}

/*
 * @brief Manages WiFi configurations (STA and AP modes). Connects
 * to an existing/saved network if available, otherwise it acts as
 * an AP in order to receive new network credentials.
*/
void WiFiManagementTask(void* pvParameters)
{
  const uint16_t accessPointTimeout = 50000; //millisecs
  static WiFiManager wm;
  WiFi.mode(WIFI_STA);  
  wm.setConfigPortalBlocking(false);  
  //Auto-connect to previous network if available.
  //If connection fails, ESP32 goes from being a station to being an access point.
  Serial.print(wm.autoConnect("AirMonitor")); 
  Serial.println("-->WiFi status");   
  bool accessPointMode = false;
  uint32_t startTime = 0;    
  
  while(1)
  {
    wm.process();
    if(WiFi.status() != WL_CONNECTED)
    {
      if(!accessPointMode)
      {
        if(!wm.getConfigPortalActive())
        {
          wm.autoConnect("AirMonitor"); 
        }
        accessPointMode = true; 
        startTime = millis(); 
      }
      else
      {
        //reset after a timeframe (device shouldn't spend too long as an access point)
        if((millis() - startTime) >= accessPointTimeout)
        {
          Serial.println("\nAP timeout reached, system will restart for better connection");
          vTaskDelay(pdMS_TO_TICKS(1000));
          esp_restart();
        }
      }
    }
    else
    {
      if(accessPointMode)
      {   
        accessPointMode = false;
        Serial.println("Successfully connected, system will restart now");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
      }
    }    
  }
}

void SensorsTask(void* pvParameters)
{
  while(1)
  {
   //Get_SensorsData(); 
  }
}

void MqttTask(void* pvParameters)
{
  static WiFiClient wifiClient;
  static PubSubClient mqttClient(wifiClient);

  char humTopic[SIZE_TOPIC] = "AirMonitor/hum";
  char tempTopic[SIZE_TOPIC] = "AirMonitor/temp";

  const char *mqttBroker = "broker.hivemq.com";
  const uint16_t mqttPort = 1883;  
  uint32_t prevTime = millis();
  while(1)
  {
    if(WiFi.status() == WL_CONNECTED)
    {       
      if(!mqttClient.connected())
      {
        mqttClient.setServer(mqttBroker,mqttPort);
        //mqttClient.setCallback(MqttCallback);
        while(!mqttClient.connected())
        {
          String clientID = String(WiFi.macAddress());
          if(mqttClient.connect(clientID.c_str()))
          {
            Serial.println("Connected to HiveMQ broker");
          }
        } 
      }
      else
      {
        if((millis() - prevTime) >= 5000)
        {
          mqttClient.publish(humTopic,"1");
          mqttClient.publish(tempTopic,"2");
          prevTime = millis();
        }
      }
    }
  }
}
