
#include "pwm.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

void pwm::init()
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_TIM0);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);

	gpio_set_af(GPIOA, GPIO_AF2, GPIO10);
	gpio_set_af(GPIOB, GPIO_AF2, GPIO1);

	timer_disable_counter(TIM0);
	timer_set_mode(TIM0,TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_enable_preload(TIM0);
	timer_continuous_mode(TIM0);
	timer_set_prescaler(TIM0,0);
	timer_set_period(TIM0,0xff);
	timer_set_enabled_off_state_in_idle_mode(TIM0);
	timer_set_enabled_off_state_in_run_mode(TIM0);
	timer_disable_break(TIM0);
	timer_disable_oc_output(TIM0,TIM_OC2);
	timer_disable_oc_output(TIM0,TIM_OC2N);
	timer_set_oc_mode(TIM0, TIM_OC2,TIM_OCM_PWM1);
	timer_set_oc_mode(TIM0, TIM_OC2,TIM_OCM_PWM1);
	timer_set_oc_polarity_low(TIM0, TIM_OC2N);
	timer_reset_output_idle_state(TIM0, TIM_OC2); // DOES NOTHING ATM
	timer_set_output_idle_state(TIM0, TIM_OC2N);  // ALSO DOES NOTHING ATM
	timer_set_deadtime(TIM0, 0xC);
	/* RE version has another set of output disables */
	timer_enable_break_main_output(TIM0);
	timer_enable_counter(TIM0);
}

void pwm::highZ()
{
	// float output
	timer_disable_oc_output(TIM0, TIM_OC2);
	timer_disable_oc_output(TIM0, TIM_OC2N);
}

void pwm::on(uint32_t duty)
{
	// set value to duty
	timer_set_oc_value(TIM0,TIM_OC2,duty);
	timer_enable_oc_output(TIM0, TIM_OC2);
	timer_enable_oc_output(TIM0, TIM_OC2N);
}

void pwm::off()
{
	// set value to 0
	timer_set_oc_value(TIM0,TIM_OC2, 0);
}
