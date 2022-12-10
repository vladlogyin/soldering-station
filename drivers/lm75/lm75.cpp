#include <libopencm3/stm32/i2c.h>
#include <lm75/lm75.h>
void lm75_write_config(uint32_t i2c, uint8_t sensor)
{
	uint32_t reg32 __attribute__((unused));

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Send destination address. */
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Sending the data. */
	i2c_send_data(i2c, 0x1); /* lm75 config register */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF)); /* Await ByteTransferedFlag. */
	/* Polarity reverse - LED glows if temp is below Tos/Thyst. */
	i2c_send_data(i2c, 0x4);
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
}

void lm75_write_temp_os(uint32_t i2c, uint8_t sensor, uint16_t temp_os)
{
	uint32_t reg32 __attribute__((unused));

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Send destination address. */
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Sending the data. */
	i2c_send_data(i2c, 0x3); /* OvertemperatureShutdown register */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, (uint8_t)(temp_os >> 8)); /* MSB */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, (uint8_t)(temp_os & 0xff00)); /* LSB */
	/* After the last byte we have to wait for TxE too. */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
}

void lm75_write_temp_hyst(uint32_t i2c, uint8_t sensor, uint16_t temp_hyst)
{
	uint32_t reg32 __attribute__((unused));

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and therefore switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Sending the data. */
	i2c_send_data(i2c, 0x2); /* TemperatureHysteresis register */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, (uint8_t)(temp_hyst >> 8)); /* MSB */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	i2c_send_data(i2c, (uint8_t)(temp_hyst & 0xff00)); /* LSB */
	/* After the last byte we have to wait for TxE too. */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/* Send STOP condition. */
	i2c_send_stop(i2c);
}
uint16_t lm75temp()
{
	uint8_t asd[2];
	uint8_t reg = 0x0;
	//i2c_read7_v1(I2C2,0x48,asd,2);
	i2c_transfer7(I2C2,0b1001111,&reg,1,asd,2);
	return (uint16_t)(asd[0]<<8)|asd[1];
}
uint16_t lm75_read_temperature(uint32_t i2c, uint8_t sensor)
{
	uint32_t reg32 __attribute__((unused));
	uint16_t temperature;

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	/* Yes, WRITE is correct - for selecting register in lm75. */
	i2c_send_7bit_address(i2c, sensor, I2C_WRITE);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	i2c_send_data(i2c, 0x0); /* temperature register */
	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

	/*
	 * Now we transferred that we want to ACCESS the temperature register.
	 * Now we send another START condition (repeated START) and then
	 * transfer the destination but with flag READ.
	 */

	/* Send START condition. */
	i2c_send_start(i2c);

	/* Waiting for START is send and switched to master mode. */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	/* Say to what address we want to talk to. */
	i2c_send_7bit_address(i2c, sensor, I2C_READ); 

	/* 2-byte receive is a special case. See datasheet POS bit. */
	I2C_CR1(i2c) |= (I2C_CR1_POS | I2C_CR1_ACK);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));

	/* Cleaning ADDR condition sequence. */
	reg32 = I2C_SR2(i2c);

	/* Cleaning I2C_SR1_ACK. */
	I2C_CR1(i2c) &= ~I2C_CR1_ACK;

	/* Now the slave should begin to send us the first byte. Await BTF. */
	while (!(I2C_SR1(i2c) & I2C_SR1_BTF));
	temperature = (uint16_t)(I2C_DR(i2c) << 8); /* MSB */

	/*
	 * Yes they mean it: we have to generate the STOP condition before
	 * saving the 1st byte.
	 */
	I2C_CR1(i2c) |= I2C_CR1_STOP;

	temperature |= I2C_DR(i2c); /* LSB */

	/* Original state. */
	I2C_CR1(i2c) &= ~I2C_CR1_POS;

	return temperature;
}

lm75::lm75(uint32_t i2cDev, uint8_t address)
{
  this->i2cDev = i2cDev;
  this->address = address;
}

uint8_t lm75::init()
{
  uint8_t data[2] = {LM75_CONFIG, 0x4};
}
float lm75::readTemperature(uint8_t keep)
{
  uint8_t data[2];
  uint8_t reg = 0x0;
  i2c_transfer7(i2cDev,address,&reg,1,data,2);
  return ((int16_t)((data[0]<<8)|(data[1]&keep)))/256.0;
}
void setOS(float temperature,float hysteresis)
{
  
}
