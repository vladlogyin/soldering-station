
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
#define ST77XX_VSCSAD 0x37
#define ST77XX_COLMOD 0x3A

#define ST77XX_RAMCTRL 0xB0

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

void sendCommand ( const uint8_t command );
void sendData ( const uint8_t data[], const uint16_t len );
void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void setScrollArea(uint16_t by, uint16_t sy, uint16_t ty);
void setScroll(uint16_t y);
void clearScreen(uint16_t color);

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

#define ST_CMD_DELAY_150 0x40
#define ST_CMD_DELAY_300 0x80
#define ST_CMD_DELAY_450 0xC0


		static constexpr uint8_t _initCommands[] =
	{
	// command, parameter count, parameters...
    ST77XX_SLPOUT,   ST_CMD_DELAY_300,
	ST77XX_COLMOD, 1|ST_CMD_DELAY_150, 0x55,
    ST77XX_MADCTL, 1,  0xa0/*0xa0*/,
	ST77XX_INVON, 0,
	ST77XX_NORON, ST_CMD_DELAY_150,
	ST77XX_DISPON, ST_CMD_DELAY_300,
    //ST77XX_VSCRDEF, 6, 0x00, 0x00, 0x01, 0x40, 0x00, 0x00,
    ST77XX_RASET, 4,
      0,
      0,   //     XSTART = 0
      0,
      240,  //     XEND = 240
    ST77XX_CASET  , 4,
      0x00,
      0,
      320>>8,
      320&0xFF,
	};

	// Run init commands
	for(int i=0;i<std::size(_initCommands);)
	{
		const uint8_t cmd = _initCommands[i];
		const uint8_t paramCount = _initCommands[i+1] &0x3F;
		const uint8_t delayCount = _initCommands[i+1] >> 6;
		// Send command
		sendCommand(cmd);

		DC.set();
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
	clearScreen(0xFFFF);
	setWindow(10,10,310,210);
	sendCommand(ST77XX_RAMWR);


	DC.set();
	for(int y=10;y<230;y++)
		for(int x=10;x<310;x++)
		{
			uint32_t r = y *64/240;
			uint32_t b = x *64/320;
			spi_xfer(SPI0,(r&0x1F)<<3);
			spi_xfer(SPI0,b&0x1F);
		}

	setScrollArea(50,220,50);

	for(int y=50;y<271;y++)
	{
		setScroll(y);
		delay(5000);
	}

}

void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
	x2--;y2--;
	 const uint8_t columnData[4]={
		 uint8_t(x1>>8),uint8_t(x1&0xFF),
		 uint8_t(x2>>8),uint8_t(x2&0xFF)
	};
	 const uint8_t rowData[4]={
		 uint8_t(y1>>8),uint8_t(y1&0xFF),
		 uint8_t(y2>>8),uint8_t(y2&0xFF)
	};

	sendCommand(ST77XX_CASET);
	sendData(columnData,std::size(columnData));
	sendCommand(ST77XX_RASET);
	sendData(rowData,std::size(rowData));
}
void setScrollArea(uint16_t by, uint16_t sy, uint16_t ty)
{
	 const uint8_t data[6]={
		 uint8_t(ty>>8),uint8_t(ty&0xFF),
		 uint8_t(sy>>8),uint8_t(sy&0xFF),
		 uint8_t(by>>8),uint8_t(by&0xFF)
	};

	 sendCommand(ST77XX_VSCRDEF);
	 sendData(data,std::size(data));

}

void setScroll(uint16_t y)
{
	sendCommand(ST77XX_VSCSAD);
	DC.set();
	spi_xfer(SPI0,y>>8); spi_xfer(SPI0,y&0xFF);

}

void st7739::update()
{

}

void sendCommand ( const uint8_t command )
{
	DC.clear();
	spi_xfer(SPI0,command);
}

void sendData ( const uint8_t data[], const uint16_t len)
{
	DC.set();
	for(uint16_t i=0;i<len;i++)
	spi_xfer(SPI0,data[i]);


}

void clearScreen(uint16_t color)
{
	setWindow(0,0,320,240);
	sendCommand(ST77XX_RAMWR);
	DC.set();
	for(uint32_t i=0;i<240*320;i++)
	{
		spi_xfer(SPI0,color>>8);
		spi_xfer(SPI0,color&0xFF);
	}
}



