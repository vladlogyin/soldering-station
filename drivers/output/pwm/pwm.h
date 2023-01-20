#ifndef PWM_H_INCLUDED
#define PWM_H_INCLUDED

#include <cstdint>

class pwm {
public:
	void init();
	void on(uint32_t duty);
	void off();
	void highZ();
};

#endif // PWM_H_INCLUDED
