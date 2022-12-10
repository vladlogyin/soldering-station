
#ifndef LM75_H
#define LM75_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>

#include <lib/systemutils.h>

#define LM75_ADDRESS		0x48

#define LM75_TEMPERATURE 0x00
#define LM75_CONFIG 0x01
#define LM75_HYSTERESIS 0x02
#define LM75_OVERTEMPERATURE
#define LM75_OS_ACTIVEHIGH 1
#define LM75_OS_ACTIVELOW 0
#define LM75_OS_COMPARATOR 0
#define LM75_OS_INTERRUPT 1
#define LM75_SHUTDOWN 1
#define LM75_NORMAL 0
void lm75_write_config(uint32_t i2c, uint8_t sensor);
void lm75_write_temp_os(uint32_t i2c, uint8_t sensor, uint16_t temp_os);
void lm75_write_temp_hyst(uint32_t i2c, uint8_t sensor, uint16_t temp_hyst);
uint16_t lm75_read_temperature(uint32_t i2c, uint8_t sensor);
uint16_t lm75temp();

class lm75{
  public: //protected:
  
  uint32_t i2cDev;
  uint8_t address;
  
  //public:
  
  lm75(uint32_t i2cDev, uint8_t address = LM75_ADDRESS);
  uint8_t init();
  float readTemperature(uint8_t keep=0b11111111);
  void setOS(float temperature,float hysteresis);
};
#endif
