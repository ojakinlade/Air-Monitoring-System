#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MNI.h"

//Maximum number of characters for HiveMQ topic(s)
#define SIZE_TOPIC    30
//Define textbox for MQTT publish topic
WiFiManagerParameter subTopic("0","HiveMQ Subscription topic","",SIZE_TOPIC);
Preferences preferences; //for accessing ESP32 flash memory

//Type(s)
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

//RTOS Handle(s)
TaskHandle_t wifiTaskHandle;
TaskHandle_t nodeTaskHandle;
QueueHandle_t nodeToAppQueue;
QueueHandle_t nodeToMqttQueue;

/**
 * @brief Store new data to specifiedlocation in ESP32's flash memory 
 * if the new data is different  from the old data.
*/
static void StoreNewFlashData(const char* flashLoc,const char* newData,
                              const char* oldData,uint8_t dataSize)
{
  if(strcmp(newData,"") && strcmp(newData,oldData))
  {
    preferences.putBytes(flashLoc,newData,dataSize);                              
  }
}

void setup() {
  // put your setup code here, to run once:
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  preferences.begin("Air-Monitor", false);
  nodeToAppQueue = xQueueCreate(1,sizeof(SensorData_t));
  nodeToMqttQueue = xQueueCreate(1,sizeof(SensorData_t));
  if(nodeToAppQueue != NULL)
  {
    Serial.println("Node-Application Queue successfully creeated");
  }
  else
  {
    Serial.println("Node-Application Queue creation failed");
  }
  if(nodeToMqttQueue != NULL)
  {
    Serial.println("Node-MQTT Queue successfully creeated");
  }
  else
  {
    Serial.println("Node-MQTT Queue creation failed");
  }  
  xTaskCreatePinnedToCore(WiFiManagementTask,"",7000,NULL,1,&wifiTaskHandle,1);
  xTaskCreatePinnedToCore(ApplicationTask,"",30000,NULL,1,NULL,1);
  xTaskCreatePinnedToCore(NodeTask,"",25000,NULL,1,&nodeTaskHandle,1);
  xTaskCreatePinnedToCore(MqttTask,"",7000,NULL,1,NULL,1);
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
  wm.addParameter(&subTopic);
  wm.setConfigPortalBlocking(false);
  wm.setSaveParamsCallback(WiFiManagerCallback); 
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

/**
 * @brief Handles main application logic.
*/
void ApplicationTask(void* pvParameters)
{
  LiquidCrystal_I2C lcd(0x27,20,4);
  static SensorData_t sensorData;
  bool isWiFiTaskSuspended = false;

  //Startup message
  lcd.init();
  lcd.backlight();
  lcd.print(" AIR MONITOR");
  vTaskDelay(pdMS_TO_TICKS(1500));
  lcd.clear();
  lcd.print("STATUS: ");
  lcd.setCursor(0,1);
  lcd.print("LOADING...");
  vTaskDelay(pdMS_TO_TICKS(1500));
  lcd.clear();
  vTaskResume(nodeTaskHandle);

  //Simple FSM to periodically change parameters being displayed
  const uint8_t displayState1 = 0;
  const uint8_t displayState2 = 1;
  uint8_t displayState = displayState1;
  uint32_t prevTime = millis();
  while(1)
  {
    
  }
}

/**
 * @brief Handles communication between the master and the node via a 
 * serial interface (i.e. MNI).
 * MNI is an acronym for 'Master-Node-Interface'.
*/
void NodeTask(void* pvParameters)
{
  vTaskSuspend(NULL);
  static MNI mni(&Serial2);
  static SensorData_t sensorData;
  //Initial request for sensorData from the node
  mni.EncodeData(MNI::QUERY,MNI::TxDataId::DATA_QUERY);
  mni.TransmitData();
  uint32_t prevTime = millis();

  while(1)
  {
    //Request for sensor data from node periodically
    if(millis() - prevTime >= 5000)
    {
      mni.EncodeData(MNI::QUERY,MNI::TxDataId::DATA_QUERY);
      mni.TransmitData();
      prevTime = millis();
    }
    //Decode data received from node
    if(mni.ReceivedData())
    {
      if(mni.DecodeData(MNI::RxDataId::DATA_ACK) == MNI::ACK)
      {
        Serial.println("--Received serial data from node\n");
        sensorData.temp = mni.DecodeData(MNI::RxDataId::TEMP) / 100;
        sensorData.hum = mni.DecodeData(MNI::RxDataId::HUM) / 100;
        sensorData.NO2 = mni.DecodeData(MNI::RxDataId::NO2);
        sensorData.NH3 = mni.DecodeData(MNI::RxDataId::NH3);
        sensorData.CO = mni.DecodeData(MNI::RxDataId::CO);
        sensorData.pinAState = mni.DecodeData(MNI::RxDataId::PIN_A_STATE);
        sensorData.pinBState = mni.DecodeData(MNI::RxDataId::PIN_B_STATE);
        sensorData.O3 = mni.DecodeData(MNI::RxDataId::O3) / 100;
        //Debug
        Serial.print("Temperature: ");
        Serial.println(sensorData.temp);
        Serial.print("Humidity: ");
        Serial.println(sensorData.hum);
        Serial.print("NO2 conc: ");
        Serial.println(sensorData.NO2);
        Serial.print("NH3 conc: ");
        Serial.println(sensorData.NH3);
        Serial.print("CO conc: ");
        Serial.println(sensorData.CO);
        Serial.print("A: ");
        Serial.println(sensorData.pinAState);
        Serial.print("B: ");
        Serial.println(sensorData.pinBState);
        Serial.print("O3 conc: ");
        Serial.println(sensorData.O3);
        //Place sensor data in the Node-Application Queue
        if(xQueueSend(nodeToAppQueue,&sensorData,0) == pdPASS)
        {
          Serial.println("--Data successfully sent to Application task\n");
        }
        else
        {
          Serial.println("--Failed to send data to Application task\n");
        }
        //Place sensor data in the Node-MQTT Queue
        if(xQueueSend(nodeToMqttQueue,&sensorData,0) == pdPASS)
        {
          Serial.println("--Data successfully sent to MQTT task\n");
        }
        else
        {
          Serial.println("--Failed to send data to MQTT task\n");
        }
      }
    }
  }
}


void MqttTask(void* pvParameters)
{
  static SensorData_t sensorData;
  static WiFiClient wifiClient;
  static PubSubClient mqttClient(wifiClient);
  char prevSubTopic[SIZE_TOPIC] = {0};
  const char *mqttBroker = "broker.hivemq.com";
  const uint16_t mqttPort = 1883;

  while(1)
  {
    if(WiFi.status() == WL_CONNECTED)
    {       
      if(!mqttClient.connected())
      {
        preferences.getBytes("0",prevSubTopic,SIZE_TOPIC);
        mqttClient.setServer(mqttBroker,mqttPort);
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
        //Receive sensor data from the node-MQTT Queue.
        if(xQueueReceive(nodeToMqttQueue,&sensorData,0) == pdPASS)
        {
          Serial.println("--MQTT Task received data from node task\n");
          String dataToPublish = "TEMP: " + String(sensorData.temp) + " C\n" +
                                 "HUM: " + String(sensorData.hum) + " %\n" +
                                 "NO2 conc: " + String(sensorData.NO2) + " PPM\n" +
                                 "NH3 conc: " + String(sensorData.NH3) + " PPM\n" +
                                 "CO conc: " + String(sensorData.CO) + " PPM\n" +
                                 "O3 conc: " + String(sensorData.O3) + " PPM\n";
          mqttClient.publish(prevSubTopic,dataToPublish.c_str());
        }
      }
    }
  }
}

void WiFiManagerCallback(void)
{
  char prevSubTopic[SIZE_TOPIC] = {0};
  preferences.getBytes("0",prevSubTopic,SIZE_TOPIC);
  StoreNewFlashData("0",subTopic.getValue(),prevSubTopic,SIZE_TOPIC);
}
