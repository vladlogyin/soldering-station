#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include <drivers/ui/st7739/st7739.h>
#include <drivers/delay.h>
#include <drivers/input/encoder/encoder.h>
int main()
{

    rcc_clock_setup_pll(&rcc_hsi8_configs[RCC_CLOCK_HSI8_72MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_enable(RCC_SPI0);

    st7739& disp = st7739::getInstance();
    encoder& enc = encoder::getInstance();
    disp.init();
    enc.init();

    const static int maxTemp=250;
    const static int minTemp=0;
    const static int scrollLength=maxTemp-minTemp;

    int targetTemp=minTemp;
    while(true)
    {
        while(!enc.poll())
            delay(10000);
        auto delta = enc.getDelta();
        targetTemp+=delta;
        if(targetTemp>maxTemp)
            targetTemp=maxTemp;
        if(targetTemp<minTemp)
            targetTemp=minTemp;

        disp.setTargetTemp(targetTemp);
        disp.update();
    }


    return 0;
}
