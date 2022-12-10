#include <ds3231/ds3231.h>

ds3231::ds3231()
{

}

int ds3231::getSec()
{
  uint8_t rawsec = readReg(0x00);
  uint8_t sec = ((rawsec>>4)*10)+(rawsec&0xF);
  return sec;
}
int ds3231::getMin()
{
  uint8_t rawmin = readReg(0x01);
  uint8_t min = ((rawmin>>4)*10)+(rawmin&0xF);
  return min;
}

uint8_t ds3231::readReg(uint8_t reg)
{
	uint8_t val;
	 i2c_transfer7(I2C2,DS3231_ADDRESS,&reg,1,&val,1);
	return val;
}
