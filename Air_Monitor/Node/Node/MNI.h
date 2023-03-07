#pragma once

//MNI: Master-Node-Interface
//Handles Serial communication between Master(ESP32) and Node(Nano)
//Features: Bi-directional communication

class MNI
{
  private:
    enum BufferSize{TX = 20, RX = 2};
    SoftwareSerial* port;
    uint8_t rxDataCounter;
    uint8_t txBuffer[BufferSize::TX];
    uint8_t rxBuffer[BufferSize::RX];
    
  public:
    enum{QUERY = 0xAA, ACK = 0xBB};
    enum TxDataId
    {
      DATA_ACK = 0,
      TEMP = 2,
      HUM = 4,
      NO2 = 6,
      NH3 = 8,
      CO = 10,
      PIN_A_STATE = 12,
      PIN_B_STATE = 14,
      PM2_5 = 16,
      PM10_0 = 18
    };
    enum RxDataId {DATA_QUERY = 0};
    
    MNI(SoftwareSerial* serial, uint32_t baudRate = 9600);
    void EncodeData(uint16_t dataToEncode,TxDataId id);
    void TransmitData(void);
    bool ReceivedData(void);
    uint16_t DecodeData(RxDataId id);
};
