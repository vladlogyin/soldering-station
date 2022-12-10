#ifndef RC522_H
#define RC522_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>

#include <lib/systemutils.h>

// PCD register definitions
#define RC522_COMMAND 0x01
#define RC522_COMIEN 0x02
#define RC522_DIVIEN 0x03
#define RC522_COMIRQ 0x04
#define RC522_DIVIRQ 0x05
#define RC522_ERROR 0x06
#define RC522_STATUS1 0x07
#define RC522_STATUS2 0x08
#define RC522_FIFODATA 0x09
#define RC522_FIFOLEVEL 0x0A
#define RC522_WATERLEVEL 0x0B
#define RC522_CONTROL 0x0C
#define RC522_BITFRAMING 0x0D
#define RC522_COLL 0x0E

#define RC522_MODE 0x11
#define RC522_TXMODE 0x12
#define RC522_RXMODE 0x13
#define RC522_TXCONTROL 0x14
#define RC522_TXASK 0x15
#define RC522_TXSEL 0x16
#define RC522_RXSEL 0x17
#define RC522_RSTRESHOLD 0x18
#define RC522_DEMOD 0x19
#define RC522_MFTX 0x1C
#define RC522_MFRX 0x1D
#define RC522_SERIALSPEED 0x1F

#define RC522_CRCRESULTH 0x21
#define RC522_CRCRESULTL 0x22
#define RC522_MODWIDTH 0x24
#define RC522_RFCFG 0x26
#define RC522_GSN 0x27
#define RC522_CWGSP 0x28
#define RC522_MODGSP 0x29
#define RC522_TMODE 0x2A
#define RC522_TPRESCALER 0x2B
#define RC522_TRELOADH 0x2C
#define RC522_TRELOADL 0x2D
#define RC522_TCOUNTERVALUEH 0x2E
#define RC522_TCOUNTERVALUEL 0x2F

// TODO test register definitions

// PCD command definitions

#define RC522_IDLE 0x00
#define RC522_MEM 0x01
#define RC522_GENERATERANDOMID 0x02
#define RC522_CALCCRC 0x03
#define RC522_TRANSMIT 0x04
#define RC522_NOCMDCHANGE 0x07
#define RC522_RECEIVE 0x08
#define RC522_TRANSCEIVE 0x0C
#define RC522_MFAUTHENT 0x0E
#define RC522_SOFTRESET 0x0F

// Gain definitions

#define RC522_GAIN_18DB 0x00
#define RC522_GAIN_23DB 0x01
#define RC522_GAIN_18DB2 0x02
#define RC522_GAIN_23DB2 0x03
#define RC522_GAIN_33DB 0x04
#define RC522_GAIN_38DB 0x05
#define RC522_GAIN_43DB 0x06
#define RC522_GAIN_48DB 0x07
#define RC522_GAIN_MIN 0x00
#define RC522_GAIN_AVG 0x04
#define RC522_GAIN_MAX 0x07

// PICC command definition

#define RC522_PICC_REQA 0x26
#define RC522_PICC_WUPA 0x52
#define RC522_PICC_CT 0x88
#define RC522_PICC_SELCL1 0x93
#define RC522_PICC_SELCL2 0x95
#define RC522_PICC_SELCL3 0x97
#define RC522_PICC_HLTA 0x50
#define RC522_PICC_RATS 0xE0

#define RC522_PICC_MF_AUTHKEYA 0x60
#define RC522_PICC_MF_AUTHKEYB 0x61
#define RC522_PICC_MF_READ 0x30
#define RC522_PICC_MF_WRITE 0xA0
#define RC522_PICC_MF_RESTORE 0xC2
#define RC522_PICC_MF_TRANSFER 0xB0

#define RC522_PICC_UL_WRITE 0xA2

// Mifare constants

#define RC522_PICC_MF_ACK 0xA
#define RC522_PICC_MF_KEYSIZE 6 // Crypto1 key is 6 bytes long

// PICC types

#define RC522_PICC_TYPE_NOTCOMPLETE 0x04
#define RC522_PICC_TYPE_MFMINI 0x09
#define RC522_PICC_TYPE_MF1K 0x08
#define RC522_PICC_TYPE_MF4K 0x18
#define RC522_PICC_TYPE_MFUL 0x00
#define RC522_PICC_TYPE_MFPLUS1 0x10
#define RC522_PICC_TYPE_MFPLUS2 0x11
#define RC522_PICC_TYPE_TNP3XXX 0x01
#define RC522_PICC_TYPE_ISO14443_4 0x20
#define RC522_PICC_TYPE_ISO18092 0x40
#define RC522_PICC_TYPE_UNKNOWN 0xFF

// Status codes
#define RC522_STATUS_OK 0
#define RC522_STATUS_ERROR 1
#define RC522_STATUS_COLLISION 2
#define RC522_STATUS_TIMEOUT 3
#define RC522_STATUS_NO_ROOM 4
#define RC522_STATUS_INTERNAL_ERROR 5
#define RC522_STATUS_INVALID 6
#define RC522_STATUS_CRC_WRONG 7
#define RC522_STATUS_MIFARE_NACK 8




class rc522{
  public:
  
    rc522(uint32_t spi, uint32_t portCS, uint32_t pinCS, uint32_t portRST, uint32_t pinRST);
    void init();
    uint8_t getType(uint8_t sak);
    bool PICCIsNewCardPresent();
    bool PICCReadCardSerial();
    
    typedef struct {
      uint8_t size;			// Number of bytes in the UID. 4, 7 or 10.
      uint8_t uidByte[10];
      uint8_t sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
    } uid_t;
    typedef struct {
      uint8_t keyByte[RC522_PICC_MF_KEYSIZE];
    } key_t;
    
  //protected:
    uid_t uid;
    uint32_t spiDev;
    uint32_t portCS, pinCS;
    uint32_t portRST, pinRST;
    
    void PCDWriteRegister(uint8_t reg, uint8_t data);
    void PCDWriteRegister(uint8_t reg, uint8_t* data, uint8_t count);
    uint8_t PCDReadRegister(uint8_t reg);
    void PCDReadRegister(uint8_t reg, uint8_t* data, uint8_t count, uint8_t align);
    void PCDAntennaOn();
    uint8_t PICCRequestA( uint8_t* bufferATQA, uint8_t* size);
    uint8_t PICCWakeupA(uint8_t* bufferATQA, uint8_t* bufferSize);
    uint8_t PICCSelect(uid_t* u, uint8_t validBits = 0);
    void PCDSetRegisterBitMask(uint8_t reg, uint8_t mask);
    void PCDClearRegisterBitMask(uint8_t reg, uint8_t mask);
    uint8_t PCDCalculateCRC(uint8_t* data, uint8_t length, uint8_t* result);
    uint8_t PICCREQAorWUPA(uint8_t command, uint8_t* bufferATQA, uint8_t* size);
    uint8_t PCDTransceiveData(uint8_t* sendData, uint8_t sendLength, uint8_t* backData, uint8_t* backLength, uint8_t* validBits = nullptr, uint8_t rxAlign = 0, bool checkCRC = false);
    uint8_t PCDCommunicateWithPICC(uint8_t command, uint8_t waitIRQ, uint8_t* sendData, uint8_t sendLength, uint8_t* backData = nullptr, uint8_t* backLength = nullptr, uint8_t* validBits = nullptr, uint8_t rxAlign = 0, bool checkCRC = false);
    uint8_t PCDAuthenticate(uint8_t command, uint8_t blockAddr, key_t* k, uid_t* u);
    uint8_t PICCHaltA();
    bool PICCSetUID(uint8_t* newUID, uint8_t uidSize);
    uint8_t MifareRead(uint8_t blockAddr, uint8_t* buffer, uint8_t* bufferSize);
    uint8_t MifareWrite(uint8_t blockAddr, uint8_t* buffer, uint8_t bufferSize);
    uint8_t MifareTransceive(uint8_t* sendData, uint8_t sendLength, bool acceptTimeout = false);
    void PCDStopCrypto1();
    uint8_t OpenUIDBackdoor();
};


#endif
