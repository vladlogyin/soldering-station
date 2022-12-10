#ifndef MAX7219_H
#define MAX7219_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>

#include <lib/systemutils.h>

#define MAX7219_NOOP   0
#define MAX7219_DIGIT0 1
#define MAX7219_DIGIT1 2
#define MAX7219_DIGIT2 3
#define MAX7219_DIGIT3 4
#define MAX7219_DIGIT4 5
#define MAX7219_DIGIT5 6
#define MAX7219_DIGIT6 7
#define MAX7219_DIGIT7 8
#define MAX7219_DECODEMODE  9
#define MAX7219_INTENSITY   10
#define MAX7219_SCANLIMIT   11
#define MAX7219_SHUTDOWN    12
#define MAX7219_DISPLAYTEST 15

class max7219
{
  protected:
    const static uint8_t digitFont[16];
  
    uint32_t portCS;
    uint32_t pinCS;
    uint32_t spiDev;
    void spiTransfer(uint8_t address, uint8_t opcode, uint8_t data);
  
  public:
    max7219(uint32_t spiDev, uint32_t portCS, uint32_t pinCS);
    void init();
    void setRow(uint8_t address, uint8_t row, uint8_t value);
    void setColumn(uint8_t address, uint8_t row, uint8_t value);
    void setIntensity(uint8_t address, uint8_t intensity);
    void setScanLimit(uint8_t address, uint8_t limit);
    void clearDisplay(uint8_t address);
    void printInt(uint8_t address, int64_t num, uint8_t pos = 0, uint8_t minlength = 0);
    void printHex(uint8_t address, uint64_t num, uint8_t pos = 0, uint8_t minlength = 0);
    void printFloat(uint8_t address, float num, uint8_t pos = 0, uint8_t decimals = 2);
};

#endif
