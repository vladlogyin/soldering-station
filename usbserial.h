#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>

#include <lib/systemutils.h>
#include <cdcacm/cdcacm.h>

/**
 * Sets up PLL and alternate function clocksetup
 */
void clocksetup(void);

/**
 * Sets up USB driver and polling interrupt
 */
void usbsetup(void);
