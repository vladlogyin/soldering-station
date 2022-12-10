#include <htu21d/htu21d.h>

HTU21D::HTU21D(HTU21D_RESOLUTION sensorResolution)
{
  _resolution = sensorResolution;
}

bool HTU21D::begin(void) 
{

  //Wire.beginTransmission(HTU21D_ADDRESS);
  //if (Wire.endTransmission(true) != 0) return false; //safety check, make sure the sensor is connected

  setResolution(_resolution);
  setHeater(HTU21D_OFF);

  return true;
}
void HTU21D::setResolution(HTU21D_RESOLUTION sensorResolution)
{
  uint8_t userRegisterData = 0;

  userRegisterData  = read8(HTU21D_USER_REGISTER_READ); //reads current user register state
  userRegisterData &= 0x7E;                             //clears current resolution bits with 0
  userRegisterData |= sensorResolution;                 //adds new resolution bits to user register byte

  write8(HTU21D_USER_REGISTER_WRITE, userRegisterData); //writes updeted byte to the user register

  _resolution = sensorResolution;                       //updates private variable
}
void HTU21D::softReset(void)
{
  /*Wire.beginTransmission(HTU21D_ADDRESS);

  #if ARDUINO >= 100
  Wire.write(HTU21D_SOFT_RESET);
  #else
  Wire.send(HTU21D_SOFT_RESET);
  #endif

  Wire.endTransmission(true);
  */
  delay_ms(HTU21D_SOFT_RESET_DELAY);
}
bool HTU21D::batteryStatus(void)
{
  uint8_t userRegisterData = 0;
  
  userRegisterData  = read8(HTU21D_USER_REGISTER_READ);
  userRegisterData &= 0x40;

  return !userRegisterData;
}
void HTU21D::setHeater(HTU21D_HEATER_SWITCH heaterSwitch)
{
  uint8_t userRegisterData = 0;

  userRegisterData = read8(HTU21D_USER_REGISTER_READ);

  switch(heaterSwitch)
  {
    case HTU21D_ON:
      userRegisterData |= heaterSwitch;
      break;

    case HTU21D_OFF:
      userRegisterData &= heaterSwitch;
      break;
  }

  write8(HTU21D_USER_REGISTER_WRITE, userRegisterData);
}
float HTU21D::readHumidity(HTU21D_HUMD_OPERATION_MODE sensorOperationMode)
{
  uint16_t rawHumidity = 0;
  uint8_t  checksum    = 0;
  float    humidity    = 0;

  /* humidity measurement delay */
  switch(_resolution)
  {
    case HTU21D_RES_RH12_TEMP14:
      delay_ms(29);                                               //HTU21D - 14..16msec, Si7021 - 10..12msec, SHT21 - 22..29msec
      break;

    case HTU21D_RES_RH11_TEMP11:
      delay_ms(15);                                               //HTU21D - 7..8msec,   Si7021 - 6..7msec,   SHT21 - 12..15msec
      break;

    case HTU21D_RES_RH10_TEMP13:
      delay_ms(9);                                                //HTU21D - 4..5msec,   Si7021 - 4..5msec,   SHT21 - 7..9msec
      break;

    case HTU21D_RES_RH8_TEMP12:
      delay_ms(4);                                                //HTU21D - 2..3msec,   Si7021 - 3..4msec,   SHT21 - 3..4msec
      break;
  }

  uint8_t recv[3], sensop=(uint8_t)sensorOperationMode;
  i2c_transfer7(I2C2,HTU21D_ADDRESS,&sensop,1,recv,3); 


  rawHumidity  = (recv[0]<<8)|recv[1];
  checksum     = recv[2];

  if (checkCRC8(rawHumidity) != checksum) return HTU21D_ERROR; //error handler, checksum verification

  rawHumidity ^= 0x02;                                         //clear status bits, humidity always returns xxxxxx10 in the LSB field
  humidity     = (0.001907 * (float)rawHumidity - 6);
  
  if      (humidity < 0)   humidity = 0;                       //due to RH accuracy, measured value might be slightly less than 0 or more 100
  else if (humidity > 100) humidity = 100;

  return humidity;
}
float HTU21D::readTemperature(HTU21D_TEMP_OPERATION_MODE sensorOperationMode)
{
  int8_t   qntRequest     = 3;                                                                //3 bytes -> MSB, LSB byte & checksum or 2 bytes -> MSB, LSB byte
  uint16_t rawTemperature = 0;
  uint8_t  checksum       = 0;
 //i2c_write7_v1(I2C2,HTU21D_ADDRESS,&(uint8_t)sensorOperationMode,1);
  /* temperature measurement delay */
  if (sensorOperationMode != SI70xx_TEMP_READ_AFTER_RH_MEASURMENT)
  {
    switch(_resolution)
    {
      case HTU21D_RES_RH12_TEMP14:
        delay_ms(85);                                                                            //HTU21D - 44..50msec, Si7021 - 7..11msec, SHT21 - 66..85msec
        break;

      case HTU21D_RES_RH10_TEMP13:
        delay_ms(43);                                                                            //HTU21D - 22..25msec, Si7021 - 4..7msec,  SHT21 - 33..43msec
        break;

      case HTU21D_RES_RH8_TEMP12:
        delay_ms(22);                                                                            //HTU21D - 11..13msec, Si7021 - 3..4msec,  SHT21 - 17..22msec
        break;

      case HTU21D_RES_RH11_TEMP11:
        delay_ms(11);                                                                            //HTU21D - 6..7msec,   Si7021 - 2..3msec,  SHT21 - 9..11msec
        break;
    }
  }
  else qntRequest = 2;                                                                        //checksum is not available with "SI70xx_TEMP_READ_AFTER_RH_MEASURMENT"

  uint8_t recv[qntRequest], sensop=(uint8_t)sensorOperationMode;
  i2c_transfer7(I2C2,HTU21D_ADDRESS,&sensop,1,recv,qntRequest);
  //i2c_read7_v1(I2C2,HTU21D_ADDRESS,recv,qntRequest);
  /* reads MSB, LSB byte & checksum from "wire.h" rxBuffer */

  rawTemperature  = (recv[0]<<8)|recv[1];
  if (sensorOperationMode != SI70xx_TEMP_READ_AFTER_RH_MEASURMENT) checksum = recv[3];    //checksum is not available with "SI70xx_TEMP_READ_AFTER_RH_MEASURMENT"

  /* checksum is not available with "SI70xx_TEMP_READ_AFTER_RH_MEASURMENT" */
  if (sensorOperationMode != SI70xx_TEMP_READ_AFTER_RH_MEASURMENT && checkCRC8(rawTemperature) != checksum) return HTU21D_ERROR; //error handler, checksum verification

  return (0.002681 * (float)rawTemperature - 46.85);                                          //temperature always returns xxxxxx00 in the LSB field
}
uint16_t HTU21D::readDeviceID(void)
{
  uint16_t deviceID = 0;
  uint8_t  checksum = 0;
  uint8_t send[2]={HTU21D_SERIAL2_READ1,HTU21D_SERIAL2_READ2}, recv[3];
  i2c_transfer7(I2C2,HTU21D_ADDRESS,send,2,recv,3);

  deviceID  = recv[0] << 8;
  deviceID |= recv[1];
  checksum  = recv[2];

  if (checkCRC8(deviceID) != checksum) return HTU21D_ERROR; //error handler, checksum verification

  deviceID = deviceID >> 8;

  switch(deviceID)
  {
    case HTU21D_CHIPID:
      deviceID = 21;
      break;

    case SI7013_CHIPID:
      deviceID = 7013;
      break;

    case SI7020_CHIPID:
      deviceID = 7020;
      break;

    case SI7021_CHIPID:
      deviceID = 7021;
      break;

    default:
      deviceID = HTU21D_ERROR;
      break;
  }
  return deviceID;
}
uint8_t HTU21D::readFirmwareVersion(void)
{
  uint8_t firmwareVersion = 0, send[2]={HTU21D_FIRMWARE_READ1,HTU21D_FIRMWARE_READ2};
  i2c_transfer7(I2C2,HTU21D_ADDRESS,send,2,&firmwareVersion,1);



  switch(firmwareVersion)
  {
    case HTU21D_FIRMWARE_V1:
      firmwareVersion = 1;
      break;

    case HTU21D_FIRMWARE_V2:
      firmwareVersion = 2;
      break;

    default:
      firmwareVersion = HTU21D_ERROR;
      break;
  }
  return firmwareVersion;
}
bool HTU21D::write8(uint8_t reg, uint8_t value)
{
  uint8_t send[2]={reg,value};
  i2c_transfer7(I2C2,HTU21D_ADDRESS,send,2,NULL,0);
  return true;
}
uint8_t HTU21D::read8(uint8_t reg)
{
  uint8_t read;
  i2c_transfer7(I2C2,HTU21D_ADDRESS,&reg,1,&read,1);
	return read;
}
uint8_t HTU21D::checkCRC8(uint16_t data)
{
  for (uint8_t bit = 0; bit < 16; bit++)
  {
    if   (data & 0x8000) data = (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
    else data <<= 1;
  }
  return data >>= 8;
}
