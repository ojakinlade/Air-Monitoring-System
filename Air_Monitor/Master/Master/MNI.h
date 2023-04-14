#pragma once

//MNI: Master-Node-Interface
//Handles Serial communication between Master(ESP32) and Node(Nano)
//Features: Bi-directional communication

class MNI
{
  private:
    enum BufferSize{TX = 2, RX = 16};
    HardwareSerial* port;
    uint8_t rxDataCounter;
    uint8_t txBuffer[BufferSize::TX];
    uint8_t rxBuffer[BufferSize::RX];
    
  public:
    enum{QUERY = 0xAA, ACK = 0xBB};
    enum TxDataId {DATA_QUERY = 0};
    enum RxDataId
    {
      DATA_ACK = 0,
      TEMP = 2,
      HUM = 4,
      NO2 = 6,
      NH3 = 8,
      CO = 10,
      PM2_5 = 12,
      PM10 = 14
    };
    
    MNI(HardwareSerial* serial, 
        uint32_t baudRate = 9600,
        int8_t serialRx = -1,
        int8_t SerialTx = -1);
    void EncodeData(uint16_t dataToEncode,TxDataId id);
    void TransmitData(void);
    bool ReceivedData(void);
    uint16_t DecodeData(RxDataId id);
};
