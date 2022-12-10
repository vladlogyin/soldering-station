#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>

#include <lib/systemutils.h>
#include <stdlib.h>

/**
 * Sets up PLL and alternate function clocksetup
 */
void clocksetup(void);

/**
 * Sets up PB12 along with a timer base interrupt to toggle it
 */
void ledsetup(void);

/**
 * Sets up USART1 for transmission
 */
void usartsetup(void);

/**
 * Prints null terminated string to USART1
 *
 * @param str string to be printed
 */
int usart_print(const char* str);
