#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>

#include <drivers/ui/st7739/st7739.h>
#include <drivers/delay.h>
#include <drivers/input/encoder/encoder.h>
#include <drivers/temp/type_c/type_c.h>
#include <drivers/output/pwm/pwm.h>
int main()
{
    type_c thermo;
    pwm output;
    rcc_clock_setup_pll(&rcc_hsi8_configs[RCC_CLOCK_HSI8_72MHZ]);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOF);
    rcc_periph_clock_enable(RCC_SPI0);

    st7739& disp = st7739::getInstance();
    encoder& enc = encoder::getInstance();
    disp.init();
    enc.init();
    thermo.init();
    output.init();

    const static int maxTemp=380;
    const static int minTemp=0;
    const static int scrollLength=maxTemp-minTemp;

    int targetTemp=minTemp;
    float currentTemp=0;
    while(true)
    {
        /*while(!enc.poll())
            delay(10000);
        */
        auto delta = enc.getDelta();
        targetTemp+=delta;
        if(targetTemp>maxTemp)
            targetTemp=maxTemp;
        if(targetTemp<minTemp)
            targetTemp=minTemp;

        if(targetTemp>currentTemp)
            output.on(0xE0); //50% duty
        delay(44E3);
        output.off();
        delay(2E3);
        output.highZ();
        delay(2E3);
        currentTemp= thermo.read();

        disp.setTargetTemp(targetTemp);
        disp.setCurrentTemp(currentTemp);
        disp.update();
    }


    return 0;
}
