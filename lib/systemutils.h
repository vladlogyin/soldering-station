#ifndef SYSTEMUTILS_H
#define SYSTEMUTILS_H
#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <algorithm>
#include <cstring>

/**
 * Context-safe millisecond delay
 *
 * Don't use this as it is not accurate!
 *
 * @param m pseudo-milliseconds
 */
volatile void robust_delay(uint32_t m);

/**
 * Context-safe microsecond delay
 *
 * Don't use this as it is not accurate!
 *
 * @param u pseudo-microseconds
 */
volatile void robust_us(uint32_t u);

/**
 * Base address of the delay timer
 */
#define TIMDELAY TIM4

/**
 * RCC base address of the delay timer
 */
#define RCC_TIMDELAY RCC_TIM4

/**
 * Enables delay timer
 *
 * Run this before calling delay_us or delay_ms
 */
void delay_setup(void);

/**
 * Microsecond blocking delay
 *
 * @param u microseconds
 */
void delay_us(uint16_t u);

/**
 * Millisecond blocking delay
 *
 * @param m milliseconds
 */
void delay_ms(uint16_t m);

#endif
