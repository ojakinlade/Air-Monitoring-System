#pragma once

//MNI: Master-Node-Interface
//Handles Serial communication between Master(ESP32) and Node(Nano)
//Features: Bi-directional communication

class MNI
{
  private:
    enum BufferSize{TX = 2, RX = 22};
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
      PIN_A_STATE = 12,
      PIN_B_STATE = 14,
      O3 = 16,
      PMS2_5 = 18,
      PMS10_0 = 20
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
