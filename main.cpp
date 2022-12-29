#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include <drivers/ui/st7739/st7739.h>
#include <drivers/delay.h>
int main()
{

    rcc_clock_setup_pll(&rcc_hsi8_configs[RCC_CLOCK_HSI8_72MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_enable(RCC_SPI0);

    st7739 disp;
    while(true)
    {
        for(int i=0;i<1000;i++)
            delay(1000);
        disp.init();
    }

    return 0;
}
