
#include "delay.h"

void delay(const uint32_t us)
{
    uint32_t looptyloops = us*DELAY_CALIB;
    for(uint32_t i=0;i<looptyloops;i++)
        asm("nop");
}

