#include "encoder.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <vector>

// encoder::encoder()
// {
//
// }
//
//
// encoder::~encoder()
// {
//
// }


void encoder::init()
{
	//PINS
	// A	- PA6
	// B	- PA9
	// BTN	- ???
	rcc_periph_clock_enable(RCC_GPIOA);
	nvic_enable_irq(NVIC_EXTI4_15_IRQ);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO6 | GPIO9);
	exti_select_source(EXTI6|EXTI9,GPIOA);
	exti_enable_request(EXTI6|EXTI9);
	exti_set_trigger(EXTI6|EXTI9, EXTI_TRIGGER_BOTH);

	_lastState = getState();
}

int encoder::getDelta()
{
	auto rv = _delta;
	_delta=0;
	return rv;
}

const uint8_t encoder::_previousStates[] = {
	0b01, //0b00
	0b11, //0b01
	0b00, //0b10
	0b10, //0b11
};
const uint8_t encoder::_nextStates[] = {
	0b10, //0b00
	0b00, //0b01
	0b11, //0b10
	0b01  //0b11
};

uint8_t encoder::getState()
{
	const uint8_t currentState =	(gpio_get(GPIOA, GPIO6)?0b10:0b00)|
									(gpio_get(GPIOA, GPIO9)?0b01:0b00);
	return currentState;
}

bool encoder::poll(){
	return _delta!=0;
}

encoder& encoder::getInstance()
{
	static encoder singleton;
	return singleton;
}

int encoder::updateEncoder()
{
	auto currentState = getState();
	if(currentState==_nextStates[_lastState])
	{
		_delta++;
	}
	else if(currentState==_previousStates[_lastState])
	{
		_delta--;
	}
		_lastState=currentState;
	return _delta;
}

#include <drivers/ui/st7739/st7739.h>

void exti4_15_isr()
{
	auto val = encoder::getInstance().updateEncoder();
	//st7739::getInstance().displayEncoder(val);
	exti_reset_request(GPIO6|GPIO9);
}
