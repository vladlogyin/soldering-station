#include <bme280/bme280.h>


bme280::bme280(uint32_t i2cDevice, uint8_t address)
{
  this->address = address;
  this->i2cDev = i2cDevice;
}

uint8_t bme280::init()
{
  
  //i2c_peripheral_enable(I2C2);
  
  int sID = read8(BME280_REGISTER_CHIPID);
  if(sID != 0x60)
  {
    return sID;
  }
  
  
  write8(BME280_REGISTER_SOFTRESET, 0xB6);
  robust_delay(10);
  
  while (isReadingCalibration())
  {
    robust_delay(10);
  }
  
  readCoefficients();
  
  setSampling();
  
  robust_delay(100);
  
  return 0;
  
  
}

float bme280::readHumidity() {
  readTemperature(); // must be done first to get t_fine

  int32_t adcH = read16(BME280_REGISTER_HUMIDDATA);
  if (adcH == 0x8000) // value in case humidity measurement was disabled
    return -1;

  int32_t v_x1_u32r;

  v_x1_u32r = (tFine - ((int32_t)76800));

  v_x1_u32r = (((((adcH << 14) - (((int32_t)digH4) << 20) -
                  (((int32_t)digH5) * v_x1_u32r)) +
                 ((int32_t)16384)) >>
                15) *
               (((((((v_x1_u32r * ((int32_t)digH6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)digH3)) >> 11) +
                     ((int32_t)32768))) >>
                   10) +
                  ((int32_t)2097152)) *
                     ((int32_t)digH2) +
                 8192) >>
                14));

  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                             ((int32_t)digH1)) >>
                            4));

  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  float h = (v_x1_u32r >> 12);
  return h / 1024.0;
}

float bme280::readPressure() {
  int64_t var1, var2, p;

  readTemperature(); // must be done first to get t_fine

  int32_t adcP = read24(BME280_REGISTER_PRESSUREDATA);
  if (adcP == 0x800000) // value in case pressure measurement was disabled
    return -1;
  adcP >>= 4;

  var1 = ((int64_t)tFine) - 128000;
  var2 = var1 * var1 * (int64_t)digP6;
  var2 = var2 + ((var1 * (int64_t)digP5) << 17);
  var2 = var2 + (((int64_t)digP4) << 35);
  var1 = ((var1 * var1 * (int64_t)digP3) >> 8) +
         ((var1 * (int64_t)digP2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)digP1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adcP;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)digP9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)digP8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)digP7) << 4);
  return (float)p / 256;
}

float bme280::readTemperature()
{
  int32_t var1, var2;
  int32_t adcT = read24(BME280_REGISTER_TEMPDATA);
  /*if(adcT == 0x800000)
  {
    return -1000;
  }*/
  adcT = adcT >> 4;
  
  var1 = ((((adcT >> 3) - ((int32_t)digT1 << 1))) *
          ((int32_t)digT2)) >>
         11;

  var2 = (((((adcT >> 4) - ((int32_t)digT1)) *
            ((adcT >> 4) - ((int32_t)digT1))) >>
           12) *
          ((int32_t)digT3)) >>
         14;
  tFine = var1 + var2 + tFineAdjust;
  float T = (tFine * 5 + 128) >> 8;
  return T/100;
         
         
}

void bme280::takeForcedMeasurement() {
    write8(BME280_REGISTER_CONTROL, (BME280_SAMPLING_X8 << 5) | (BME280_SAMPLING_X8 << 2) | BME280_MODE_FORCED);
    // wait until measurement has been completed, otherwise we would read
    // the values from the last measurement
    while (read8(BME280_REGISTER_STATUS) & 0x08)
    {
      robust_delay(1);
    }
}

void bme280::setSampling(uint8_t mode, uint8_t temperatureSampling, uint8_t pressureSampling, uint8_t humiditySampling, uint8_t filter, uint8_t duration)
{
  write8(BME280_REGISTER_CONTROL, BME280_MODE_SLEEP);
  
  write8(BME280_REGISTER_CONTROLHUMID, humiditySampling);
  write8(BME280_REGISTER_CONFIG, (duration << 5) | (filter << 2) | 1);
  write8(BME280_REGISTER_CONTROL, (temperatureSampling << 5) | (pressureSampling << 2) | mode);
}

void bme280::readCoefficients()
{
  digT1 = read16LE(BME280_REGISTER_DIG_T1);
  digT2 = (int16_t)read16LE(BME280_REGISTER_DIG_T2);
  digT3 = (int16_t)read16LE(BME280_REGISTER_DIG_T3);

  digP1 = read16LE(BME280_REGISTER_DIG_P1);
  digP2 = (int16_t)read16LE(BME280_REGISTER_DIG_P2);
  digP3 = (int16_t)read16LE(BME280_REGISTER_DIG_P3);
  digP4 = (int16_t)read16LE(BME280_REGISTER_DIG_P4);
  digP5 = (int16_t)read16LE(BME280_REGISTER_DIG_P5);
  digP6 = (int16_t)read16LE(BME280_REGISTER_DIG_P6);
  digP7 = (int16_t)read16LE(BME280_REGISTER_DIG_P7);
  digP8 = (int16_t)read16LE(BME280_REGISTER_DIG_P8);
  digP9 = (int16_t)read16LE(BME280_REGISTER_DIG_P9);

  digH1 = read8(BME280_REGISTER_DIG_H1);
  digH2 = (int16_t)read16LE(BME280_REGISTER_DIG_H2);
  digH3 = read8(BME280_REGISTER_DIG_H3);
  digH4 = ((int8_t)read8(BME280_REGISTER_DIG_H4) << 4) |
                         (read8(BME280_REGISTER_DIG_H4 + 1) & 0xF);
  digH5 = ((int8_t)read8(BME280_REGISTER_DIG_H5 + 1) << 4) |
                         (read8(BME280_REGISTER_DIG_H5) >> 4);
  digH6 = (int8_t)read8(BME280_REGISTER_DIG_H6);
}

bool bme280::isReadingCalibration()
{
  return (read8(BME280_REGISTER_STATUS) & 1);
}

uint32_t bme280::read24(uint8_t reg)
{
  uint8_t ret[3];
 i2c_transfer7(i2cDev, address, &reg, 1, ret, 3);
 uint32_t value;
 value = ret[0];
 value = ret[1] | (value << 8);
 value = ret[2] | (value << 8);
 
 return value;
}

uint16_t bme280::read16(uint8_t reg)
{
  uint8_t ret[2];
 i2c_transfer7(i2cDev, address, &reg, 1, ret, 2);
 uint16_t value;
 value = ret[0];
 value = ret[1] | (value << 8);
 return value;
}

uint16_t bme280::read16LE(uint8_t reg)
{
  uint8_t ret[2];
 i2c_transfer7(i2cDev, address, &reg, 1, ret, 2);
 uint16_t value;
 value = ret[1];
 value = ret[0] | (value << 8);
 return value;
}

uint8_t bme280::read8(uint8_t reg)
{
 uint8_t ret;
 i2c_transfer7(i2cDev, address, &reg, 1, &ret, 1);
 return ret;
}

void bme280::write8(uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = {reg, val};
  i2c_transfer7(i2cDev, address, buf, 2, &val, 0);
}
