#include <rc522/rc522.h>

uint8_t rc522::getType(uint8_t sak)
{
  return sak & 0x7F;
}


rc522::rc522(uint32_t spi, uint32_t portCS, uint32_t pinCS, uint32_t portRST, uint32_t pinRST)
{
  this->portCS=portCS; this->pinCS=pinCS;
  this->portRST=portRST; this->pinRST=pinRST;
  this->spiDev=spi;
}


void rc522::init()
{
  gpio_set_mode(portCS, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pinCS);
  gpio_set_mode(portRST, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pinRST);

  // Reset SPI, SPI_CR1 register cleared, SPI is disabled
  spi_reset(spiDev);
  
  spi_init_master(spiDev, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  //spi_set_standard_mode(spiDev,0);
  spi_enable_software_slave_management(spiDev);
  spi_set_nss_high(spiDev);
  
  // Enable spiDev peripheral
  spi_enable(spiDev);
  
  gpio_set(portRST, pinRST);
  robust_delay(50);
  gpio_clear(portRST, pinRST);
  robust_delay(2);
  gpio_set(portRST, pinRST);
  robust_delay(50);
  
  // Reset baud rates
  PCDWriteRegister(RC522_TXMODE, 0x00);
  PCDWriteRegister(RC522_RXMODE, 0x00);
  
  PCDWriteRegister(RC522_MODWIDTH, 0x26);
  // Set timeout
  PCDWriteRegister(RC522_TMODE, 0x80);
  PCDWriteRegister(RC522_TPRESCALER, 0xA9);
  PCDWriteRegister(RC522_TRELOADH, 0x03);
  PCDWriteRegister(RC522_TRELOADL, 0xE8);
  
  PCDWriteRegister(RC522_TXASK, 0x40);
  PCDWriteRegister(RC522_MODE, 0x3D);
  PCDAntennaOn();
  
}

void rc522::PCDWriteRegister(uint8_t reg, uint8_t data)
{
  gpio_clear(portCS, pinCS);
  
  spi_xfer(spiDev, (reg<<1));
  spi_xfer(spiDev, data);
  
  gpio_set(portCS, pinCS);
}
void rc522::PCDWriteRegister(uint8_t reg, uint8_t* data, uint8_t count)
{
  gpio_clear(portCS, pinCS);
  
  spi_xfer(spiDev, (reg<<1));
  for(uint8_t i = 0; i < count ; i++)
  {
    spi_xfer(spiDev, data[i]);
  }
  
  gpio_set(portCS, pinCS);
}

uint8_t rc522::PCDReadRegister(uint8_t reg)
{
  gpio_clear(portCS, pinCS);
  
  spi_xfer(spiDev, 0x80 | (reg<<1));
  
  uint8_t r = spi_xfer(spiDev, 0);
  
  gpio_set(portCS, pinCS);
  
  return r;
}

void rc522::PCDReadRegister(uint8_t reg, uint8_t* data, uint8_t count, uint8_t align)
{
  gpio_clear(portCS, pinCS);
  
  spi_xfer(spiDev, 0x80 | (reg<<1));
  count--;
  uint8_t i=0;
  if(align){
    uint8_t mask = 0xFF<<align;
    data[0] = (data[0] & ~mask) | (spi_xfer(spiDev, 0x80 | (reg<<1)) & mask);
    i++;
  }
  for(; i < count ; i++)
  {
    data[i] = spi_xfer(spiDev, 0x80 | (reg<<1));
  }
  data[count] = spi_xfer(spiDev, 0);
  
  gpio_set(portCS, pinCS);
  
}

void rc522::PCDAntennaOn()
{
  uint8_t r = PCDReadRegister(RC522_TXCONTROL);
  if((r&0x03) != 0x03)
  {
    PCDWriteRegister(RC522_TXCONTROL, r | 0x03);
  }
}

bool rc522::PICCIsNewCardPresent()
{
  uint8_t bufferATQA[2];
  
  // Reset baud rates
  PCDWriteRegister(RC522_TXMODE, 0x00);
  PCDWriteRegister(RC522_RXMODE, 0x00);
  PCDWriteRegister(RC522_MODWIDTH, 0x26);
  uint8_t len =2;
  uint8_t result = PICCRequestA(bufferATQA, &len);
  
  return (result == RC522_STATUS_OK || result == RC522_STATUS_COLLISION);
}

bool rc522::PICCReadCardSerial()
{
  return PICCSelect(&uid) == RC522_STATUS_OK;
}

uint8_t rc522::PICCRequestA( uint8_t* bufferATQA, uint8_t* size)
{
  return PICCREQAorWUPA(RC522_PICC_REQA, bufferATQA, size);
}

uint8_t rc522::PICCSelect(uid_t* u, uint8_t validBits)
{
  bool uidComplete = false;
  bool selectDone;
  bool useCascadeTag;
  
  uint8_t cascadeLevel = 1;
  uint8_t result;
  
  uint8_t count, checkBit, ind, uidIndex;
  uint8_t currentLevelKnownBits;
  uint8_t buffer[9];
  uint8_t bufferUsed;
  uint8_t rxAlign;
  uint8_t txLastBits;
  uint8_t *responseBuffer;
  uint8_t responseLength;
  
  if(validBits > 80)
  {
    return RC522_STATUS_INVALID;
  }
  
  PCDClearRegisterBitMask(RC522_COLL, 0x80);
  
  while(!uidComplete) {
    switch(cascadeLevel) {
      case 1:
        buffer[0] = RC522_PICC_SELCL1;
        uidIndex = 0;
        useCascadeTag = validBits && u->size > 4;
        break;
      case 2:
        buffer[0] = RC522_PICC_SELCL2;
        uidIndex = 3;
        useCascadeTag = validBits && u->size > 7;
        break;
      case 3:
        buffer[0] = RC522_PICC_SELCL3;
        uidIndex = 6;
        useCascadeTag = false;
        break;
      default:
        return RC522_STATUS_INTERNAL_ERROR;
    }
    
    currentLevelKnownBits = validBits - (uidIndex<<3);
    if (currentLevelKnownBits < 0)
    {
      currentLevelKnownBits = 0;
    }
    
    ind = 2;
    
    if(useCascadeTag) {
      buffer[ind++] = RC522_PICC_CT;
    }
    
    uint8_t bytesToCopy = (currentLevelKnownBits >>3) + ((currentLevelKnownBits & 0x7) ? 1 : 0);
    if(bytesToCopy)
    {
      uint8_t maxBytes = useCascadeTag ? 3 : 4;
      if(bytesToCopy > maxBytes) {
        bytesToCopy = maxBytes;
      }
      for(count = 0; count< bytesToCopy; count++)
      {
        buffer[ind++] = u->uidByte[uidIndex+count];
      }
    }
    
    if(useCascadeTag)
    {
      currentLevelKnownBits += 8;
    }
    
    selectDone = false;
    while(!selectDone) {
      if(currentLevelKnownBits >= 32)
      {
        buffer[1] = 0x70;
        
        buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
        
        result = PCDCalculateCRC(buffer, 7, &buffer[7]);
        if(result != RC522_STATUS_OK)
        {
          return result;
        }
        txLastBits = 0;
        bufferUsed = 9;
        responseBuffer = &buffer[6];
        responseLength = 3;
      }
      else
      {
        txLastBits = currentLevelKnownBits & 0xb111;
        count = currentLevelKnownBits >> 3;
        ind = 2 + count;
        buffer[1] = (ind<<4)+txLastBits;
        bufferUsed = ind + (txLastBits ? 1 : 0);
        responseBuffer = &buffer[ind];
        responseLength = sizeof(buffer) - ind;
      }
      
      rxAlign = txLastBits;
      PCDWriteRegister(RC522_BITFRAMING, (rxAlign <<4) + txLastBits);
      
      result = PCDTransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign);
      if(result == RC522_STATUS_COLLISION)
      {
        uint8_t valueOfCollReg = PCDReadRegister(RC522_COLL);
        if(valueOfCollReg & 0x20)
        {
          return RC522_STATUS_COLLISION;
        }
        uint8_t collisionPos = valueOfCollReg & 0x1F;
        if(collisionPos == 0)
        {
          collisionPos = 32;
        }
        if(collisionPos <= currentLevelKnownBits)
        {
          return RC522_STATUS_INTERNAL_ERROR;
        }
        
        currentLevelKnownBits = collisionPos;
        count = currentLevelKnownBits & 0x7;
        checkBit = (currentLevelKnownBits - 1) & 0x7;
        ind = 1 + (currentLevelKnownBits >> 3) + (count ? 1 : 0);
        buffer[ind] |= (1 << checkBit);
      }
      else if(result != RC522_STATUS_OK)
      {
        return result;
      }
      else
      {
        if(currentLevelKnownBits >= 32)
        {
          selectDone = true;
        }
        else
        {
          currentLevelKnownBits = 32;
        }
      }
      
    }
    
    ind = (buffer[2] == RC522_PICC_CT) ? 3 : 2;
    bytesToCopy = (buffer[2] == RC522_PICC_CT) ? 3 : 4;
    for(count = 0; count < bytesToCopy; count++)
    {
      u->uidByte[uidIndex + count] = buffer[ind++];
    }
    
    if(responseLength != 3 || txLastBits != 0) {
      return RC522_STATUS_ERROR;
    }
    result = PCDCalculateCRC(responseBuffer, 1, &buffer[2]);
    if((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
    {
      return RC522_STATUS_CRC_WRONG;
    }
    if(responseBuffer[0] & 0x04)
    {
      cascadeLevel++;
    }
    else
    {
      uidComplete = true;
      u->sak = responseBuffer[0];
    }
    
  }
  
  u->size = 3 * cascadeLevel + 1;
  
  return RC522_STATUS_OK;
  
}

void rc522::PCDSetRegisterBitMask(uint8_t reg, uint8_t mask)
{
  PCDWriteRegister(reg, PCDReadRegister(reg) | mask);
}

void rc522::PCDClearRegisterBitMask(uint8_t reg, uint8_t mask)
{
  PCDWriteRegister(reg, PCDReadRegister(reg) & (~mask));
}

uint8_t rc522::PCDCalculateCRC(uint8_t *data, uint8_t length, uint8_t *result)
{
  PCDWriteRegister(RC522_COMMAND, RC522_IDLE);
  PCDWriteRegister(RC522_DIVIRQ, 0x04);
  PCDWriteRegister(RC522_FIFOLEVEL, 0x80);
  PCDWriteRegister(RC522_FIFODATA, data, length);
  PCDWriteRegister(RC522_COMMAND, RC522_CALCCRC);
  
  for(uint8_t i = 255; i--;)
  {
    robust_us(350);
    if(PCDReadRegister(RC522_DIVIRQ) & 0x04)
    {
      PCDWriteRegister(RC522_COMMAND, RC522_IDLE);
      result[0] = PCDReadRegister(RC522_CRCRESULTL);
      result[1] = PCDReadRegister(RC522_CRCRESULTH);
      return RC522_STATUS_OK;
    }
  }
  
  return RC522_STATUS_TIMEOUT;
  
}

uint8_t rc522::PICCREQAorWUPA(uint8_t command, uint8_t* bufferATQA, uint8_t* size)
{
  uint8_t validBits = 7;
  if(bufferATQA == nullptr || *size < 2)
  {
    return RC522_STATUS_NO_ROOM;
  }
  
  PCDClearRegisterBitMask(RC522_COLL, 0x80);
  uint8_t status = PCDTransceiveData(&command, 1, bufferATQA, size, &validBits);
  
  if(!status)
  {
    return status;
  }
  
  if(*size != 2 || validBits != 0)
  {
    return RC522_STATUS_ERROR;
  }
  
  return RC522_STATUS_OK;
}

uint8_t rc522::PCDTransceiveData(uint8_t* sendData, uint8_t sendLength, uint8_t* backData, uint8_t* backLength, uint8_t* validBits, uint8_t rxAlign, bool checkCRC)
{
  uint8_t waitIRQ = 0x30;
  return PCDCommunicateWithPICC(RC522_TRANSCEIVE, waitIRQ, sendData, sendLength, backData, backLength, validBits, rxAlign, checkCRC);
}

uint8_t rc522::PCDCommunicateWithPICC(uint8_t command, uint8_t waitIRQ, uint8_t* sendData, uint8_t sendLength, uint8_t* backData, uint8_t* backLength, uint8_t* validBits, uint8_t rxAlign, bool checkCRC)
{
  uint8_t txLastBits = validBits ? *validBits : 0;
  uint8_t bitFraming = (rxAlign << 4) + txLastBits;
  
  PCDWriteRegister(RC522_COMMAND, RC522_IDLE);
  PCDWriteRegister(RC522_COMIRQ, 0x7F);
  PCDWriteRegister(RC522_FIFOLEVEL, 0x80);
  PCDWriteRegister(RC522_FIFODATA, sendData, sendLength);
  PCDWriteRegister(RC522_BITFRAMING, bitFraming);
  PCDWriteRegister(RC522_COMMAND, command);
  if(command == RC522_TRANSCEIVE) {
    PCDSetRegisterBitMask(RC522_BITFRAMING, 0x80);
  }
  
  uint16_t i;
  for(i = 2000; i>0;i--) {
    robust_us(100);
    uint8_t n = PCDReadRegister(RC522_COMIRQ);
    if(n & waitIRQ)
    {
      break;
    }
    if(n & 0x01)
    {
      return RC522_STATUS_TIMEOUT;
    }
  }
  if(i<=0){
    return RC522_STATUS_TIMEOUT;
  }
  
  uint8_t errorRegValue = PCDReadRegister(RC522_ERROR);
  
  if(errorRegValue & 0x13)
  {
    return RC522_STATUS_ERROR;
  }
  
  uint8_t _validBits = 0;
  
  if(backData && backLength)
  {
    uint8_t n = PCDReadRegister(RC522_FIFOLEVEL);
    if(n > *backLength)
    {
      return RC522_STATUS_NO_ROOM;
    }
    *backLength = n;
    PCDReadRegister(RC522_FIFODATA, backData, n, rxAlign);
    _validBits = PCDReadRegister(RC522_CONTROL) & 0x07;
    if(validBits) {
      *validBits = _validBits;
    }
  }
  
  if(errorRegValue & 0x08)
  {
    return RC522_STATUS_COLLISION;
  }
  
  if(backData && backLength && checkCRC)
  {
    if(*backLength == 1 && _validBits == 4)
    {
      return RC522_STATUS_CRC_WRONG;
    }
    
    uint8_t controlBuffer[2];
    uint8_t status = PCDCalculateCRC(&backData[0], *backLength - 2, &controlBuffer[0]);
    if(!status)
    {
      return status;
    }
    if((backData[*backLength - 2] != controlBuffer[0]) || (backData[*backLength - 1] != controlBuffer[1]))
    {
      return RC522_STATUS_CRC_WRONG;
    }
  }
  
  return RC522_STATUS_OK;
}

uint8_t rc522::PCDAuthenticate(uint8_t command, uint8_t blockAddr, key_t* k, uid_t* u)
{
  uint8_t waitIRQ = 0x10;
  
  uint8_t sendData[12];
  
  sendData[0] = command;
  sendData[1] = blockAddr;
  for(uint8_t i=0; i < RC522_PICC_MF_KEYSIZE; i++)
  {
    sendData[2+i] = k->keyByte[i];
  }
  for(uint8_t i=0; i < 4; i++) {
    sendData[8+i] = u->uidByte[i+u->size-4];
  }
  return PCDCommunicateWithPICC(RC522_MFAUTHENT, waitIRQ, &sendData[0], sizeof(sendData));
}

uint8_t rc522::PICCHaltA()
{
  uint8_t result;
  uint8_t buffer[4];
  
  buffer[0] = RC522_PICC_HLTA;
  buffer[1] = 0;
  
  result = PCDCalculateCRC(buffer, 2 ,&buffer[2]);
  if(result != RC522_STATUS_OK)
  {
    return result;
  }
  
  result = PCDTransceiveData(buffer, sizeof(buffer), nullptr, 0);
  if(result == RC522_STATUS_TIMEOUT)
  {
    return RC522_STATUS_OK;
  }
  if(result == RC522_STATUS_OK)
  {
    return RC522_STATUS_ERROR;
  }
  return result;
}
bool rc522::PICCSetUID(uint8_t* newUID, uint8_t uidSize)
{
  if(!newUID || !uidSize || uidSize > 15)
  {
    return false;
  }
  
  key_t k = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t status = PCDAuthenticate(RC522_PICC_MF_AUTHKEYA, 1, &k, &uid);
  if(status != RC522_STATUS_OK) {
  
  //printf("PCDAuthenticate failed\n\r");
   return false;
  }
  
  uint8_t block0_buffer[18];
  uint8_t byteCount = sizeof(block0_buffer);
  status = MifareRead(0, block0_buffer, &byteCount);
  if(status != RC522_STATUS_OK)
  {
    //printf("MifareRead failed\n\r");
    return false;
  }
  uint8_t bcc = 0;
  for (uint8_t i = 0; i < uidSize; i++) {
    block0_buffer[i] = newUID[i];
    bcc ^= newUID[i];
  }
  block0_buffer[uidSize] = bcc;
  
  PCDStopCrypto1();
  
  if(!OpenUIDBackdoor())
  {
    return false;
  }
  
  status = MifareWrite(0, block0_buffer, 16);
  if(status != RC522_STATUS_OK) {
    //printf("MifareWrite failed\n\r");
    return false;
  }
  uint8_t atqaAnswer[2];
  uint8_t atqaSize = 2;
  PICCWakeupA(atqaAnswer, &atqaSize);
  
  return true;
}

uint8_t rc522::MifareRead(uint8_t blockAddr, uint8_t* buffer, uint8_t* bufferSize)
{
  uint8_t result;
  
  if(buffer == nullptr || *bufferSize < 18)
  {
    return RC522_STATUS_NO_ROOM;
  }
  
  buffer[0] = RC522_PICC_MF_READ;
  buffer[1] = blockAddr;
  
  result = PCDCalculateCRC(buffer, 2, &buffer[2]);
  if(result != RC522_STATUS_OK)
  {
    return result;
  }
  
  return PCDTransceiveData(buffer, 4, buffer, bufferSize, nullptr, 0, true);
}

uint8_t rc522::MifareWrite(uint8_t blockAddr, uint8_t* buffer, uint8_t bufferSize)
{
  uint8_t result;
  
  if(buffer == nullptr || bufferSize < 16)
  {
    return RC522_STATUS_INVALID;
  }
  
  uint8_t cmdBuffer[2];
  cmdBuffer[0] = RC522_PICC_MF_WRITE;
  cmdBuffer[1] = blockAddr;
  result = MifareTransceive(cmdBuffer, 2);
  if(result != RC522_STATUS_OK)
  {
    return result;
  }
  result = MifareTransceive(buffer, bufferSize); // Adds CRC_A and checks that the response is MF_ACK.
  if (result != RC522_STATUS_OK)
  {
    return result;
  }
  return RC522_STATUS_OK;
	
}

uint8_t rc522::MifareTransceive(uint8_t* sendData, uint8_t sendLength, bool acceptTimeout)
{
  uint8_t result;
  uint8_t cmdBuffer[18];
  
  if(sendData == nullptr || sendLength > 16)
  {
    return RC522_STATUS_INVALID;
  }
  
  memcpy(cmdBuffer, sendData, sendLength);
  result = PCDCalculateCRC(cmdBuffer, sendLength, &cmdBuffer[sendLength]);
  if(result != RC522_STATUS_OK)
  {
    return result;
  }
  sendLength += 2;
  
  uint8_t waitIRQ = 0x30;
  uint8_t cmdBufferSize = sizeof(cmdBuffer);
  uint8_t validBits = 0;
  result = PCDCommunicateWithPICC(RC522_TRANSCEIVE, waitIRQ, cmdBuffer, sendLength, cmdBuffer, &cmdBufferSize, &validBits);
  if(acceptTimeout && result == RC522_STATUS_TIMEOUT) {
    return RC522_STATUS_OK;
  }
  if(result != RC522_STATUS_OK)
  {
    return result;
  }
  if(cmdBufferSize != 1 || validBits != 4)
  {
    return RC522_STATUS_ERROR;
  }
  if(cmdBuffer[0] != RC522_PICC_MF_ACK)
  {
    return RC522_STATUS_MIFARE_NACK;
  }
  return RC522_STATUS_OK;
  
}

void rc522::PCDStopCrypto1()
{
  PCDClearRegisterBitMask(RC522_STATUS2, 0x08);
}

uint8_t rc522::PICCWakeupA(uint8_t* bufferATQA, uint8_t* bufferSize)
{
  return PICCREQAorWUPA(RC522_PICC_WUPA, bufferATQA, bufferSize);
}

uint8_t rc522::OpenUIDBackdoor()
{
  PICCHaltA();
  
  uint8_t cmd = 0x40;
  uint8_t validBits = 7;
  uint8_t response[32];
  uint8_t received;
  uint8_t status = PCDTransceiveData(&cmd, 1, response, &received, &validBits, 0, false);
  if(status != RC522_STATUS_OK)
  {
    //printf("Card did not respond to 0x40 after HALT command. Are you sure it is a UID changeable one?");
    return false;
  }
  if(received != 1 || response[0] != 0x0A)
  {
    //printf("Got bad response on backdoor 0x40 command: ");
    return false;
  }
  
  cmd = 0x43;
  validBits = 8;
  status = PCDTransceiveData(&cmd, 1, response, &received, &validBits, 0, false);
  if(status != RC522_STATUS_OK)
  {
    //printf("Error in communication at command 0x43, after successfully executing 0x40");
    return false;
  }
  if(received != 1 || response[0] != 0x0A)
  {
    return false;
  }
  return true;
}
