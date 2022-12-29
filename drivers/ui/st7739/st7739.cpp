
#include "st7739.h"

#include "../../delay.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include <vector>


#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_RDDID 0x04
#define ST77XX_RDDST 0x09

#define ST77XX_SLPIN 0x10
#define ST77XX_SLPOUT 0x11
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_RAMRD 0x2E

#define ST77XX_PTLAR 0x30
#define ST77XX_VSCRDEF 0x33
#define ST77XX_TEOFF 0x34
#define ST77XX_TEON 0x35
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#define ST77XX_RAMCTRL 0xB0

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

class io {
public:
	constexpr io(uint32_t port,uint32_t pin):port(port),pin(pin){}

	inline void config(const uint32_t mode,const  uint32_t pupd)
	{
		gpio_mode_setup(port,mode,pupd,pin);
	}

	inline void set()
	{
		gpio_set(port,pin);
	}

	inline void clear()
	{
		gpio_clear(port,pin);
	}

	inline void set_af(uint32_t af)
	{
		gpio_set_af(port, af, pin);
	}

	const uint32_t port;
	const uint32_t pin;
};

// ST7735V - breadboard
// pinout:
// SCK  - 11    - PA5   - SPI0 AF0
// MOSI - 13    - PA7   - SPI0 AF0
// RST  - 3     - PF1   - OUT
// D/C  - 2     - PF0   - OUT
// CS   - GND
//

// ST7735V - pcb
// pinout:
// SCK  - 11    - PA5   - SPI0 AF0
// MOSI - 13    - PA7   - SPI0 AF0
// RST  - 3     - PF1   - OUT
// D/C  - 2     - PF0   - OUT
// CS   - GND
//

io		SCK(GPIOA,GPIO5),
MOSI(GPIOA,GPIO7),
RST(GPIOF,GPIO1),
DC(GPIOF,GPIO0);

st7739::st7739()
{

}

void st7739::init()
{

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_SPI0);

	SCK.config(GPIO_MODE_AF,GPIO_PUPD_NONE);
	SCK.set_af(GPIO_AF0);
	MOSI.config(GPIO_MODE_AF,GPIO_PUPD_NONE);
	MOSI.set_af(GPIO_AF0);
	DC.config(GPIO_MODE_OUTPUT,GPIO_PUPD_NONE);
	RST.config(GPIO_MODE_OUTPUT,GPIO_PUPD_NONE);

	spi_disable(SPI0);
	spi_init_master(SPI0,SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,SPI_CR1_CPHA_CLK_TRANSITION_1,SPI_CR1_MSBFIRST);
	spi_enable(SPI0);

	// Reset the display
	RST.set();
	delay(20000);
	RST.clear();
	delay(1000);
	RST.set();
	delay(20000);

	// static constexpr uint8_t _initCommands[] =
	// {
	// // command, parameter count, parameters...
	// 	ST77XX_SLPOUT, 0,
	// 	ST77XX_COLMOD, 1, 0b101,
	// 	ST77XX_MADCTL, 1, 0b1000,
	// 	ST77XX_CASET, 4, 0x00, 0x00, 0x00, 240,
	// 	ST77XX_RASET, 4, 0x00, 0x00, 320>>8, 320&0xFF,
	// 	ST77XX_NORON, 0,
	// 	ST77XX_DISPON, 0,
	// };
#define ST_CMD_DELAY_150 0x40
#define ST_CMD_DELAY_300 0x80
#define ST_CMD_DELAY_450 0xC0

	// static constexpr uint8_t _initCommands[] =
	// {
	// // command, parameter count, parameters...
 //    ST77XX_SWRESET,   ST_CMD_DELAY_300, //  1: Software reset, no args, w/delay
 //    ST77XX_SLPOUT,   ST_CMD_DELAY_300, //  2: Out of sleep mode, no args, w/delay
 //    ST77XX_VSCRDEF, 6, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00,
	// ST77XX_NORON  ,   ST_CMD_DELAY_150, //  8: Normal display on, no args, w/delay
	// ST77XX_INVON  ,   0,  //  7: hack
 //    ST77XX_MADCTL, 1,  0x00/*0x08*/,  //Row/col addr, bottom-top refresh
	// ST77XX_COLMOD, 1|ST_CMD_DELAY_150, 0x55,//  16-bit color
 //   ST77XX_DISPON ,   ST_CMD_DELAY_300,
 //    ST77XX_CASET, 4,     //  5: Column addr set, 4 args, no delay:
 //      0x00,
 //      0,   //     XSTART = 0
 //      0,
 //      240,  //     XEND = 240
 //    ST77XX_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
 //      0x00,
 //      0,             //     YSTART = 0
 //      320>>8,
 //      320&0xFF,  //     YEND = 320
	// };

		static constexpr uint8_t _initCommands[] =
	{
	// command, parameter count, parameters...
    ST77XX_SLPOUT,   ST_CMD_DELAY_300, //  2: Out of sleep mode, no args, w/delay
	ST77XX_COLMOD, 1|ST_CMD_DELAY_150, 0x55,//  16-bit color
    ST77XX_MADCTL, 1,  0x00/*0xa0*/,  //Row/col addr, bottom-top refresh
	ST77XX_INVON  ,   0,  //  7: hack
	ST77XX_NORON  ,   ST_CMD_DELAY_150, //  8: Normal display on, no args, w/delay
	ST77XX_DISPON ,   ST_CMD_DELAY_300,
    //ST77XX_VSCRDEF, 6, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00,
    ST77XX_CASET, 4,     //  5: Column addr set, 4 args, no delay:
      0x00,
      0,   //     XSTART = 0
      0,
      240,  //     XEND = 240
    ST77XX_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
      0x00,
      0,             //     YSTART = 0
      320>>8,
      320&0xFF,  //     YEND = 320
	};

	// Run init commands
	for(int i=0;i<std::size(_initCommands);)
	{
		const uint8_t cmd = _initCommands[i];
		const uint8_t paramCount = _initCommands[i+1] &0x3F;
		const uint8_t delayCount = _initCommands[i+1] >> 6;
		// Send command
		DC.clear();
		delay(1);
		spi_xfer(SPI0,cmd);
		delay(1);
		DC.set();
		delay(1);
		// Send parameters
		for(int j=0;j<paramCount;j++)
		{
			const uint8_t param = _initCommands[i+j+2];
			spi_xfer(SPI0,param);
			delay(10);
		}
		for(int d = 0;d<delayCount;d++)
			delay(150000);
		i+=paramCount+2;
	}
	//setWindow(0,320,0,240);

	DC.clear();
		delay(1);
	spi_xfer(SPI0,ST77XX_RAMWR);
		delay(1);
	DC.set();

	for(int y=0;y<320;y++)
		for(int x=0;x<240;x++)
		{
			uint32_t r = y *31/320;
			uint32_t b = x *31/240;
			spi_xfer(SPI0,(r&0x1F)<<3);
			spi_xfer(SPI0,b&0x1F);
		}
}

void st7739::setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	DC.clear();
		delay(100);
	spi_xfer(SPI0,ST77XX_CASET);
		delay(100);
	DC.set();
		delay(100);
	spi_xfer(SPI0,x1>>8);
	spi_xfer(SPI0,x1&0xFF);
	spi_xfer(SPI0,x2>>8);
	spi_xfer(SPI0,x2&0xFF);
		delay(100);
	DC.clear();
		delay(100);
	spi_xfer(SPI0,ST77XX_RASET);
		delay(100);
	DC.set();
		delay(100);
	spi_xfer(SPI0,y1>>8);
	spi_xfer(SPI0,y1&0xFF);
	spi_xfer(SPI0,y2>>8);
	spi_xfer(SPI0,y2&0xFF);
		delay(10);
}

void st7739::update()
{

}

