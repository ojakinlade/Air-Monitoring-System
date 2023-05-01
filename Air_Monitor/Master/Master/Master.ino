#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ThingSpeak.h>
#include "MNI.h"
#include "AQI_Calc.h"
#include "numeric_lib.h"

//Maximum number of characters
#define SIZE_TOPIC      30
#define SIZE_CLIENT_ID  23
#define SIZE_CHANNEL_ID 30
#define SIZE_API_KEY    50

WiFiManagerParameter pubTopic("0","HiveMQ Publish topic","",SIZE_TOPIC);
WiFiManagerParameter channelId("1","Thingspeak Channel ID","",SIZE_CHANNEL_ID);
WiFiManagerParameter apiKey("2","Thingspeak API key","",SIZE_API_KEY);
WiFiManagerParameter clientId("3","MQTT Client ID","",SIZE_CLIENT_ID);
Preferences preferences; //for accessing ESP32 flash memory

//Type(s)
typedef struct
{
  float temp;
  float hum;
  float NO2;
  uint16_t NH3;
  float CO;
  uint16_t PM2_5;
  uint16_t PM10;
}SensorData_t;

typedef struct
{
  char temp[10];
  char hum[10];
  char NO2[10];
  char NH3[10];
  char CO[10];
  char PM2_5[10];
  char PM10[10];
}mqttData_t;

//RTOS Handle(s)
TaskHandle_t wifiTaskHandle;
TaskHandle_t nodeTaskHandle;
TaskHandle_t applicationTaskHandle;
TaskHandle_t dataToCloudTaskHandle;
QueueHandle_t nodeToAppQueue;
QueueHandle_t nodeToMqttQueue;

/**
 * @brief
 * @param
 * @param
*/
void ConvStrToInt(char* str, uint32_t* integer)
{
  for(uint32_t i = 0; str[i] != '\0'; i++)
  {
    *integer = *integer * 10 + (str[i] - 48);
  }
}

/**
 * @brief Store new data to specified location in ESP32's flash memory 
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

float GetMaximumAqi(float a,float b,float c,float d)
{
  float maximum = a;
  if(b > maximum)
  {
    maximum = b;
  }
  if(c > maximum)
  {
    maximum = c;
  }
  if(d > maximum)
  {
    maximum = d;
  }
  return maximum;
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
  xTaskCreatePinnedToCore(ApplicationTask,"",30000,NULL,1,&applicationTaskHandle,1);
  xTaskCreatePinnedToCore(NodeTask,"",30000,NULL,1,&nodeTaskHandle,1);
  xTaskCreatePinnedToCore(DataToCloudTask,"",7000,NULL,1,&dataToCloudTaskHandle,1);
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
  wm.addParameter(&clientId);
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
  const uint8_t buzzerPin = 13;
  pinMode(buzzerPin,OUTPUT);
  LiquidCrystal_I2C lcd(0x27,20,4);
  static SensorData_t sensorData;
  static AQI_Calc aqiCalc;
  bool isWifiTaskSuspended = false;
  float AQI_PM10;
  float AQI_PM2_5;
  float AQI_CO;
  float AQI_NO2;
  float maxAqi;

  //Startup message
  lcd.init();
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print(" AIR MONITOR");
  vTaskDelay(pdMS_TO_TICKS(1500));
  lcd.clear();
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
    //Compute AQI for the pollutants     
    AQI_PM10 = aqiCalc.ComputeIndex(P_PM10,sensorData.PM10);
    AQI_PM2_5 = aqiCalc.ComputeIndex(P_PM2_5,sensorData.PM2_5);
    AQI_CO = aqiCalc.ComputeIndex(P_CO,sensorData.CO);
    AQI_NO2 = aqiCalc.ComputeIndex(P_NO2,sensorData.NO2);
    maxAqi = GetMaximumAqi(AQI_PM10,AQI_PM2_5,
                           AQI_CO,AQI_NO2);
    //Turn on the Buzzer when the Maximum AQI exceeds acceptable limits
    if(maxAqi > 200.0)
    {
      digitalWrite(buzzerPin,HIGH);
    }
    else
    {
      digitalWrite(buzzerPin,LOW);
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
        lcd.print("CO conc: ");
        lcd.print(sensorData.CO);
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
        lcd.print("PM2.5(ug/m3): ");
        lcd.print(sensorData.PM2_5);
        lcd.setCursor(0,1);
        lcd.print("PM10(ug/m3): ");
        lcd.print(sensorData.PM10);
        lcd.setCursor(0,2);
        lcd.print("NO2 AQI: ");
        lcd.print(AQI_NO2);
        lcd.setCursor(0,3);
        lcd.print("CO AQI: ");
        lcd.print(AQI_CO);
        if(millis() - prevTime >= 4000)
        {
          displayState = displayState3;
          prevTime = millis();
          lcd.clear();
        }
        break;

      case displayState3:
        lcd.setCursor(0,0);
        lcd.print("PM2.5 AQI: ");
        lcd.print(AQI_PM2_5);
        lcd.setCursor(0,1);
        lcd.print("PM10 AQI: ");
        lcd.print(AQI_PM10);
        lcd.setCursor(0,2);
        lcd.print("Actual AQI: ");
        if(AQI_PM2_5 >= AQI_PM10 && AQI_PM2_5 >= AQI_CO)
        {
          lcd.print(AQI_PM2_5);
        }
        else if(AQI_PM10 >= AQI_PM2_5 && AQI_PM10 >= AQI_CO)
        {
          lcd.print(AQI_PM10);
        }
        else
        {
          lcd.print(AQI_CO);
        }
        lcd.setCursor(0,3);
        lcd.print("Remark:");
        if(maxAqi > 0 && maxAqi <= 50)
        {
          lcd.print("Good");
        }
        else if(maxAqi > 50 && maxAqi <= 100)
        {
          lcd.print("Moderate");
        }
        else if(maxAqi > 100 && maxAqi <= 150)
        {
          lcd.print("Unhealthy4SG");
        }
        else if(maxAqi > 150 && maxAqi <= 200)
        {
          lcd.print("Unhealthy");
        }
        else if(maxAqi > 200 && maxAqi <= 300)
        {
          lcd.print("Vry Unhealthy");
        }
        else if(maxAqi > 300)
        {
          lcd.print("Hazardous");
        }
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
    
    //Critical section
    vTaskSuspend(wifiTaskHandle);
    vTaskSuspend(applicationTaskHandle);
    vTaskSuspend(dataToCloudTaskHandle);
    //Decode data received from node
    if(mni.ReceivedData())
    {
      if(mni.DecodeData(MNI::RxDataId::DATA_ACK) == MNI::ACK)
      {
        Serial.println("--Received serial data from node\n");
        
        sensorData.temp = mni.DecodeData(MNI::RxDataId::TEMP) / 100.0;
        sensorData.hum = mni.DecodeData(MNI::RxDataId::HUM) / 100.0;
        sensorData.NO2 = mni.DecodeData(MNI::RxDataId::NO2) / 100.0;
        sensorData.NH3 = mni.DecodeData(MNI::RxDataId::NH3);
        sensorData.CO = mni.DecodeData(MNI::RxDataId::CO) / 100.0;
        sensorData.PM2_5 = mni.DecodeData(MNI::RxDataId::PM2_5);
        sensorData.PM10 = mni.DecodeData(MNI::RxDataId::PM10);
         
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
    vTaskResume(wifiTaskHandle);
    vTaskResume(applicationTaskHandle);
    vTaskResume(dataToCloudTaskHandle);
  }
}


void DataToCloudTask(void* pvParameters)
{
  static SensorData_t sensorData;
  static mqttData_t mqttData;
  static WiFiClient wifiClient;
  static PubSubClient mqttClient(wifiClient);
  ThingSpeak.begin(wifiClient);
  //Previously stored data in ESP32's flash
  char prevPubTopic[SIZE_TOPIC] = {0};
  char prevChannelId[SIZE_CHANNEL_ID] = {0};
  char prevApiKey[SIZE_API_KEY] = {0};
  char prevClientId[SIZE_CLIENT_ID] = {0};
  
  const char *mqttBroker = "broker.hivemq.com";
  const uint16_t mqttPort = 1883;

  char dataToPublish[250] = {0};
  uint32_t idInt = 0;
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
        preferences.getBytes("3",prevClientId,SIZE_CLIENT_ID);
        mqttClient.setServer(mqttBroker,mqttPort);
        while(!mqttClient.connected())
        {
          if(mqttClient.connect(prevClientId))
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
        }
        //Send data to Thingspeak and MQTT every 20 seconds
        if(millis() - prevUploadTime >= 20000)
        {
          //Encode data to send to MQTT
          FloatToString(sensorData.temp,mqttData.temp,2);
          FloatToString(sensorData.hum,mqttData.hum,2);
          FloatToString(sensorData.NO2,mqttData.NO2,2);
          IntegerToString(sensorData.NH3,mqttData.NH3);
          FloatToString(sensorData.CO,mqttData.CO,2);
          IntegerToString(sensorData.PM2_5,mqttData.PM2_5);
          IntegerToString(sensorData.PM10,mqttData.PM10);
          
          strcat(dataToPublish,"TEMP: ");
          strcat(dataToPublish,mqttData.temp);
          strcat(dataToPublish," C\n");
          strcat(dataToPublish,"HUM: ");
          strcat(dataToPublish,mqttData.hum);
          strcat(dataToPublish," %\n");
          strcat(dataToPublish,"NO2 conc: ");
          strcat(dataToPublish,mqttData.NO2);
          strcat(dataToPublish," PPM\n");
          strcat(dataToPublish,"NH3 conc: ");
          strcat(dataToPublish,mqttData.NH3);
          strcat(dataToPublish," PPM\n");
          strcat(dataToPublish,"CO conc: ");
          strcat(dataToPublish,mqttData.CO);
          strcat(dataToPublish," PPM\n");
          strcat(dataToPublish,"PM2.5: ");
          strcat(dataToPublish,mqttData.PM2_5);
          strcat(dataToPublish," ug/m3\n");
          strcat(dataToPublish,"PM10: ");
          strcat(dataToPublish,mqttData.PM10);
          strcat(dataToPublish," ug/m3\n");

          mqttClient.publish(prevPubTopic,dataToPublish);
          //Clear buffer after publishing
          memset(dataToPublish,'\0',strlen(dataToPublish));
          memset(mqttData.temp,'\0',strlen(mqttData.temp));
          memset(mqttData.hum,'\0',strlen(mqttData.hum));
          memset(mqttData.NO2,'\0',strlen(mqttData.NO2));
          memset(mqttData.NH3,'\0',strlen(mqttData.NH3));
          memset(mqttData.CO,'\0',strlen(mqttData.CO));
          memset(mqttData.PM2_5,'\0',strlen(mqttData.PM2_5));
          memset(mqttData.PM10,'\0',strlen(mqttData.PM10));
          //Encode data to be sent to Thingspeak
          ThingSpeak.setField(1,sensorData.temp);
          ThingSpeak.setField(2,sensorData.hum);
          ThingSpeak.setField(3,sensorData.NO2);
          ThingSpeak.setField(4,sensorData.NH3);
          ThingSpeak.setField(5,sensorData.CO);
          ThingSpeak.setField(6,sensorData.PM2_5);
          ThingSpeak.setField(7,sensorData.PM10);
          //Convert channel ID from string to Integer
          ConvStrToInt(prevChannelId,&idInt);
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

void WiFiManagerCallback(void)
{
  char prevPubTopic[SIZE_TOPIC] = {0};
  char prevChannelId[SIZE_CHANNEL_ID] = {0};
  char prevApiKey[SIZE_API_KEY] = {0};
  char prevClientId[SIZE_CLIENT_ID] = {0};
  //Get data stored previously in flash memory
  preferences.getBytes("0",prevPubTopic,SIZE_TOPIC);
  preferences.getBytes("1",prevChannelId,SIZE_CHANNEL_ID);
  preferences.getBytes("2",prevApiKey,SIZE_API_KEY);
  preferences.getBytes("3",prevClientId,SIZE_CLIENT_ID);
  //Store new data in flash memory if its different from the previously stored ones
  StoreNewFlashData("0",pubTopic.getValue(),prevPubTopic,SIZE_TOPIC);
  StoreNewFlashData("1",channelId.getValue(),prevChannelId,SIZE_CHANNEL_ID);
  StoreNewFlashData("2",apiKey.getValue(),prevApiKey,SIZE_API_KEY);
  StoreNewFlashData("3",clientId.getValue(),prevClientId,SIZE_CLIENT_ID);
}
