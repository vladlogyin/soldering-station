#ifndef BME280_H
#define BME280_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>

#include <lib/systemutils.h>

#define BME280_ADDRESS 0x77
#define BME280_REGISTER_DIG_T1 0x88
#define BME280_REGISTER_DIG_T2 0x8A
#define BME280_REGISTER_DIG_T3 0x8C

#define BME280_REGISTER_DIG_P1 0x8E
#define BME280_REGISTER_DIG_P2 0x90
#define BME280_REGISTER_DIG_P3 0x92
#define BME280_REGISTER_DIG_P4 0x94
#define BME280_REGISTER_DIG_P5 0x96
#define BME280_REGISTER_DIG_P6 0x98
#define BME280_REGISTER_DIG_P7 0x9A
#define BME280_REGISTER_DIG_P8 0x9C
#define BME280_REGISTER_DIG_P9 0x9E

#define BME280_REGISTER_DIG_H1 0xA1
#define BME280_REGISTER_DIG_H2 0xE1
#define BME280_REGISTER_DIG_H3 0xE3
#define BME280_REGISTER_DIG_H4 0xE4
#define BME280_REGISTER_DIG_H5 0xE5
#define BME280_REGISTER_DIG_H6 0xE7

#define BME280_REGISTER_CHIPID 0xD0
#define BME280_REGISTER_VERSION 0xD1
#define BME280_REGISTER_SOFTRESET 0xE0

#define BME280_REGISTER_CAL26 0xE1 // R calibration stored in 0xE1-0xF0

#define BME280_REGISTER_CONTROLHUMID 0xF2
#define BME280_REGISTER_STATUS 0XF3
#define BME280_REGISTER_CONTROL 0xF4
#define BME280_REGISTER_CONFIG 0xF5
#define BME280_REGISTER_PRESSUREDATA 0xF7
#define BME280_REGISTER_TEMPDATA 0xFA
#define BME280_REGISTER_HUMIDDATA 0xFD

#define BME280_SAMPLING_NONE 0
#define BME280_SAMPLING_X1 1
#define BME280_SAMPLING_X2 2
#define BME280_SAMPLING_X4 3
#define BME280_SAMPLING_X8 4
#define BME280_SAMPLING_X16 5

#define BME280_MODE_SLEEP 0
#define BME280_MODE_FORCED 1
#define BME280_MODE_NORMAL 3

#define BME280_FILTER_OFF 0
#define BME280_FILTER_X2 1
#define BME280_FILTER_X4 2
#define BME280_FILTER_X8 3
#define BME280_FILTER_X16 4

#define BME280_STANDBY_0_5 0b000
#define BME280_STANDBY_10 0b110
#define BME280_STANDBY_20 0b111
#define BME280_STANDBY_62_5 0b001
#define BME280_STANDBY_125 0b010 
#define BME280_STANDBY_250 0b011
#define BME280_STANDBY_500 0b100
#define BME280_STANDBY_1000 0b101

class bme280
{
  public://protected:
  
  uint16_t digT1;
  int16_t digT2;
  int16_t digT3;
  
  
  uint16_t digP1;
  int16_t digP2;
  int16_t digP3;
  int16_t digP4;
  int16_t digP5;
  int16_t digP6;
  int16_t digP7;
  int16_t digP8;
  int16_t digP9;
  
  
  uint16_t digH1;
  int16_t digH2;
  int16_t digH3;
  int16_t digH4;
  int16_t digH5;
  int16_t digH6;
  
  int32_t tFine;
  int32_t tFineAdjust = 0;
  
  
  uint32_t i2cDev;
  uint8_t address;
  
  uint32_t read24(uint8_t reg);
  uint16_t read16LE(uint8_t reg);
  uint16_t read16(uint8_t reg);
  uint8_t read8(uint8_t reg);
  void write8(uint8_t reg, uint8_t val);
  bool isReadingCalibration();
  void readCoefficients();
  void takeForcedMeasurement();
  void setSampling(uint8_t mode = BME280_MODE_NORMAL, uint8_t temperatureSampling = BME280_SAMPLING_X16, uint8_t pressureSampling = BME280_SAMPLING_X16, uint8_t humiditySampling = BME280_SAMPLING_X16, uint8_t filter = BME280_FILTER_OFF, uint8_t duration = BME280_STANDBY_0_5);
  
  //public:
  
  bme280(uint32_t i2cDevice, uint8_t address = BME280_ADDRESS);
  uint8_t init();
  float readTemperature();
  float readPressure();
  float readHumidity();
  
};

#endif
