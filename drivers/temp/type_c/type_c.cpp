#include "type_c.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>

uint32_t readChannel(uint8_t ch);

void type_c::init()
{
	// TODO move to injected channels
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	rcc_osc_on(RCC_HSI28);
	rcc_wait_for_osc_ready(RCC_HSI28);
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_reset_pulse(RST_ADC);

	adc_power_off(ADC);
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC,ADC_SMPR_SMP_13DOT5CYC);
	adc_enable_oversampling(ADC, ADC_OSCR_OSRATIO_128, ADC_OSCR_OSSHIFT_4, ADC_OSCR_TOS_ONCE);
	adc_disable_scan_mode(ADC);
	adc_set_single_conversion_mode(ADC);
	adc_enable_external_trigger_regular(ADC,ADC_CR2_EXTSEL_SWSTART);
	adc_power_on(ADC);
	adc_reset_calibration(ADC);
	adc_calibrate(ADC);

}

float type_c::read()
{
	//read vref
	uint32_t ref = readChannel(ADC_CHANNEL_VREF);
	//read ADC0 (thermo input)
	uint32_t thermo = readChannel(ADC_CHANNEL0);
	//calc

	constexpr float vref = 1200000.0f;
	constexpr float gain = 220000.0f/(750.0f*3);
	constexpr float offset = 0.0f; // C
	constexpr float tempco = 14.51f ;// uV/C

	return float(thermo)/float(ref)*vref/gain/tempco + offset;
}

uint32_t readChannel(uint8_t ch)
{
	adc_set_regular_sequence(ADC,1,&ch);
	adc_start_conversion_regular(ADC);
	while(!adc_eoc(ADC));
	return adc_read_regular(ADC);
}
