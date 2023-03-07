#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ThingSpeak.h>
#include "MNI.h"

//Maximum number of characters for HiveMQ topic(s)
#define SIZE_TOPIC      30
//Maximum number of characters for Thingspeak credentials
#define SIZE_CHANNEL_ID 30
#define SIZE_API_KEY    50
//Define textbox for MQTT publish topic
WiFiManagerParameter pubTopic("0","HiveMQ Publish topic","",SIZE_TOPIC);
//Define textboxes for Thingspeak credentials
WiFiManagerParameter channelId("1","Thingspeak Channel ID","",SIZE_CHANNEL_ID);
WiFiManagerParameter apiKey("2","Thingspeak API key","",SIZE_API_KEY);
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
  uint16_t pm2_5;
  uint16_t pm10_0;
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
  xTaskCreatePinnedToCore(NodeTask,"",30000,NULL,1,&nodeTaskHandle,1);
  xTaskCreatePinnedToCore(DataToCloudTask,"",7000,NULL,1,NULL,1);
  //vTaskSuspend(&wifiTaskHandle);
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
  wm.addParameter(&pubTopic);
  wm.addParameter(&channelId);
  wm.addParameter(&apiKey);
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
  bool isWifiTaskSuspended = false;

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
  const uint8_t displayState3 = 2;
  uint8_t displayState = displayState1;
  uint32_t prevTime = millis();
  while(1)
  {
    /**
     * Suspend Wifi management task if the system is already
     * connected to a WiFi network
     */
    if(WiFi.status() == WL_CONNECTED && !isWifiTaskSuspended)
    {
      vTaskSuspend(wifiTaskHandle);
      Serial.println("WIFI TASK: SUSPENDED");
      isWifiTaskSuspended = true;
    }
    else if(WiFi.status() != WL_CONNECTED && isWifiTaskSuspended)
    {
      vTaskResume(wifiTaskHandle);
      Serial.println("WIFI TASK: RESUMED");
      isWifiTaskSuspended = false;
    }
    //Receive data from Node Task
    if(xQueueReceive(nodeToAppQueue,&sensorData,0) == pdPASS)
    {
      Serial.println("--Application task received data from Node task\n");
    }
    //FSM[Displays the received sensor data received on the LCD]
    switch(displayState)
    {
      case displayState1:
        lcd.setCursor(0,0);
        lcd.print("Temp: ");
        lcd.print(sensorData.temp);
        lcd.print(" C");
        lcd.setCursor(0,1);
        lcd.print("Hum: ");
        lcd.print(sensorData.hum);
        lcd.print(" %");
        lcd.setCursor(0,2);
        lcd.print("NO2 conc: ");
        lcd.print(sensorData.NO2);
        lcd.print(" ppm");
        lcd.setCursor(0,3);
        lcd.print("NH3 conc: ");
        lcd.print(sensorData.NH3);
        lcd.print(" ppm");
        if(millis() - prevTime >= 4000)
        {
          displayState = displayState2;
          prevTime = millis();
          lcd.clear();
        }
        break;
        
      case displayState2:
        lcd.setCursor(0,0);
        lcd.print("CO conc: ");
        lcd.print(sensorData.CO);
        lcd.print(" ppm");
        lcd.setCursor(0,1);
        lcd.print("Pin A: ");
        lcd.print(sensorData.pinAState);
        lcd.setCursor(0,2);
        lcd.print("Pin B: ");
        lcd.print(sensorData.pinBState);
        lcd.setCursor(0,3);
        lcd.print("O3 conc: ");
        lcd.print(sensorData.O3);
        lcd.print(" ppm");
        if(millis() - prevTime >= 4000)
        {
          displayState = displayState3;
          prevTime = millis();
          lcd.clear();
        }
        break;

      case displayState3:
        lcd.setCursor(0,0);
        lcd.print("PMS2.5(ug/m3): ");
        lcd.print(sensorData.pm2_5);
        lcd.setCursor(0,1);
        lcd.print("PMS10.0(ug/m3): ");
        lcd.print(sensorData.pm10_0);
        if(millis() - prevTime >= 4000)
        {
          displayState = displayState1;
          prevTime = millis();
          lcd.clear();
        }
        break;
    }
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
  vTaskDelay(pdMS_TO_TICKS(3000));
  static MNI mni(&Serial2);
  static SensorData_t sensorData;
  uint32_t prevTime = millis();

  while(1)
  {
    //Request for sensor data from node periodically
    if(millis() - prevTime >= 2500)
    {
      mni.EncodeData(MNI::QUERY,MNI::TxDataId::DATA_QUERY);
      mni.TransmitData();
      Serial.println("Query Sent");
      prevTime = millis();
    }
    //Decode data received from node
    if(mni.ReceivedData())
    {
      if(mni.DecodeData(MNI::RxDataId::DATA_ACK) == MNI::ACK)
      {
        Serial.println("--Received serial data from node\n");
        sensorData.temp = mni.DecodeData(MNI::RxDataId::TEMP) / 100.0;
        sensorData.hum = mni.DecodeData(MNI::RxDataId::HUM) / 100.0;
        sensorData.NO2 = mni.DecodeData(MNI::RxDataId::NO2);
        sensorData.NH3 = mni.DecodeData(MNI::RxDataId::NH3);
        sensorData.CO = mni.DecodeData(MNI::RxDataId::CO);
        sensorData.pinAState = mni.DecodeData(MNI::RxDataId::PIN_A_STATE);
        sensorData.pinBState = mni.DecodeData(MNI::RxDataId::PIN_B_STATE);
        sensorData.O3 = mni.DecodeData(MNI::RxDataId::O3) / 100.0;
        sensorData.pm2_5 = mni.DecodeData(MNI::RxDataId::PMS2_5);
        sensorData.pm10_0 = mni.DecodeData(MNI::RxDataId::PMS10_0);
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
        Serial.print("PM 2.5 (ug/m3): ");
        Serial.println(sensorData.pm2_5);
        Serial.print("PM 10.0 (ug/m3): ");
        Serial.println(sensorData.pm10_0);
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


void DataToCloudTask(void* pvParameters)
{
  static SensorData_t sensorData;
  static WiFiClient wifiClient;
  static PubSubClient mqttClient(wifiClient);
  ThingSpeak.begin(wifiClient);
  //Previously stored data in ESP32's flash
  char prevPubTopic[SIZE_TOPIC] = {0};
  char prevChannelId[SIZE_CHANNEL_ID] = {0};
  char prevApiKey[SIZE_API_KEY] = {0};
  
  const char *mqttBroker = "broker.hivemq.com";
  const uint16_t mqttPort = 1883;
  uint32_t prevUploadTime = millis();

  while(1)
  {
    if(WiFi.status() == WL_CONNECTED)
    {       
      if(!mqttClient.connected())
      {
        preferences.getBytes("0",prevPubTopic,SIZE_TOPIC);
        preferences.getBytes("1",prevChannelId,SIZE_CHANNEL_ID);
        preferences.getBytes("2",prevApiKey,SIZE_API_KEY);
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
          if(millis() - prevUploadTime >= 20000)
          {//Sends data to Thingspeak and MQTT every 20 seconds
            String dataToPublish = "TEMP: " + String(sensorData.temp) + " C\n" +
                                   "HUM: " + String(sensorData.hum) + " %\n" +
                                   "NO2 conc: " + String(sensorData.NO2) + " PPM\n" +
                                   "NH3 conc: " + String(sensorData.NH3) + " PPM\n" +
                                   "CO conc: " + String(sensorData.CO) + " PPM\n" +
                                   "O3 conc: " + String(sensorData.O3) + " PPM\n" +
                                   "PMS2.5: " + String(sensorData.pm2_5) + "ug/m3\n" +
                                   "PMS10.0: " + String(sensorData.pm10_0) + "ug/m3\n";
            mqttClient.publish(prevPubTopic,dataToPublish.c_str());
            //Encode data to be sent to Thingspeak
            ThingSpeak.setField(1,sensorData.temp);
            ThingSpeak.setField(2,sensorData.hum);
            ThingSpeak.setField(3,sensorData.NO2);
            ThingSpeak.setField(4,sensorData.NH3);
            ThingSpeak.setField(5,sensorData.CO);
            ThingSpeak.setField(6,sensorData.pm2_5);
            ThingSpeak.setField(7,sensorData.pm10_0);
            //Convert channel ID from string to Integer
            String idStr = String(prevChannelId);
            uint32_t idInt = idStr.toInt();
            if(ThingSpeak.writeFields(idInt,prevApiKey) == HTTP_CODE_OK)
            {
              Serial.println("SUCCESS: Data sent to ThingspeaK");
            }
            else
            {
              Serial.println("ERROR: Sending to Thingspeak failed");
            }
            prevUploadTime = millis();
          }
        }
      }
    }
  }
}

void WiFiManagerCallback(void)
{
  char prevPubTopic[SIZE_TOPIC] = {0};
  char prevChannelId[SIZE_CHANNEL_ID] = {0};
  char prevApiKey[SIZE_API_KEY] = {0};
  //Get data stored previously in flash memory
  preferences.getBytes("0",prevPubTopic,SIZE_TOPIC);
  preferences.getBytes("1",prevChannelId,SIZE_CHANNEL_ID);
  preferences.getBytes("2",prevApiKey,SIZE_API_KEY);
  //Store new data in flash memory if its different from the previously stored ones
  StoreNewFlashData("0",pubTopic.getValue(),prevPubTopic,SIZE_TOPIC);
  StoreNewFlashData("1",channelId.getValue(),prevChannelId,SIZE_CHANNEL_ID);
  StoreNewFlashData("2",apiKey.getValue(),prevApiKey,SIZE_API_KEY);
}
