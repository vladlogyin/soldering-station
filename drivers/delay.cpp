
#include "delay.h"

void delay(const uint32_t us)
{
    for(uint32_t i=0;i<us*DELAY_CALIB;i++)
        asm("nop");
}

