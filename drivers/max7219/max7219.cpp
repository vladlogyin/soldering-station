#include <max7219/max7219.h>

//const uint8_t max7219::digitFont[10] = {0b00000001,0b00000010,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b10000000,0b00000000,0b00000000};
const uint8_t max7219::digitFont[16] = {0b01111110,0b00110000,0b01101101,0b01111001,0b00110011,0b01011011,0b01011111,0b01110000,0b01111111,0b01111011,0b01110111,0b00011111,0b01001110,0b00111101,0b01001111,0b01000111};

max7219::max7219(uint32_t spiDev, uint32_t portCS, uint32_t pinCS)
{
  this->spiDev=spiDev;
  this->portCS=portCS;
  this->pinCS=pinCS;
}

void max7219::init()
{
  gpio_set_mode(portCS, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pinCS);
  
  spi_init_master(spiDev, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
  spi_enable_software_slave_management(spiDev);
  spi_set_nss_high(spiDev);
  
    // Enable spiDev peripheral
  spi_enable(spiDev);
  
  spiTransfer(0, MAX7219_DISPLAYTEST, 0);
  spiTransfer(0, MAX7219_SCANLIMIT, 7);
  spiTransfer(0, MAX7219_DECODEMODE, 0);
  
  
  spiTransfer(0, MAX7219_SHUTDOWN, 1);
  
}

void max7219::printInt(uint8_t address, int64_t num, uint8_t pos, uint8_t minlength)
{
  bool hasSign = num<0;
  num = abs(num);
  uint8_t i;
  for(i = 0; num | i < minlength; num /= 10)
  {
    setRow(address, pos + (i++), digitFont[num % 10]);
  }
  if(hasSign)
  {
    setRow(address, pos + i, 0b00000001);
  }
}

void max7219::printFloat(uint8_t address, float num, uint8_t pos, uint8_t decimals)
{
  bool hasSign = num<0;
  for(uint8_t j = decimals; j;j--)
  {
    num = num * 10;
  }
  num = abs(num);
  int64_t n = (int64_t)num;
  uint8_t i;
  for(i = 0; n; n /= 10)
  {
    setRow(address, pos + (i++), digitFont[n % 10] | (i-1 != decimals ? 0 : 0b10000000));
  }
  if(hasSign)
  {
    setRow(address, pos + i, 0b00000001);
  }
}

void max7219::printHex(uint8_t address, uint64_t num, uint8_t pos, uint8_t minlength)
{
  for(int i = 0; num | i < minlength; num = num >> 4)
  {
    setRow(address,pos + (i++), digitFont[num & 0xF]);
  }
}

void max7219::setRow(uint8_t address, uint8_t row, uint8_t value)
{
  spiTransfer(address, MAX7219_DIGIT0+row, value);
}

void max7219::setColumn(uint8_t address, uint8_t row, uint8_t value)
{
  //TODO
}

void max7219::setScanLimit(uint8_t address, uint8_t limit)
{
  spiTransfer(address, MAX7219_SCANLIMIT, limit);
}

void max7219::setIntensity(uint8_t address, uint8_t intensity)
{
  spiTransfer(address, MAX7219_INTENSITY, intensity);
}

void max7219::clearDisplay(uint8_t address)
{
  for(uint8_t i = 7; i; i--)
  {
    setRow(address,i,0);
  }
}

void max7219::spiTransfer(uint8_t address, uint8_t opcode, uint8_t data)
{
  gpio_clear(portCS, pinCS);
  spi_xfer(spiDev, opcode);
  spi_xfer(spiDev, data);
  //while (!(SPI_SR(spiDev) & SPI_SR_TXE));
  gpio_set(portCS, pinCS);
}

