#include <lib/systemutils.h>

volatile void robust_delay(uint32_t gib)
{
  int a = 0, b = 0;

  for(volatile int i = 0; i < gib * 3000; i++) {a += i;}

  b = a;
  a = b;
}
volatile void robust_us(uint32_t gib)
{
  int a = 0, b = 0;

  for(volatile int i = 0; i < gib * 3; i++) {a += i;}

  b = a;
  a = b;
}
void delay_setup()
{
  rcc_periph_clock_enable(RCC_TIMDELAY);
}

void delay_us(uint16_t us)
{

  timer_set_prescaler(TIMDELAY, rcc_apb1_frequency / 50000 - 1);
  timer_set_period(TIMDELAY, 10*us);
  timer_one_shot_mode(TIMDELAY);
  timer_generate_event(TIMDELAY, TIM_EGR_UG);
  timer_enable_counter(TIMDELAY);

  while(TIM_CR1(TIMDELAY) & TIM_CR1_CEN);
}

void delay_ms(uint16_t ms)
{
  timer_set_prescaler(TIMDELAY, rcc_apb1_frequency / 5000 - 1);
  timer_one_shot_mode(TIMDELAY);
  timer_set_period(TIMDELAY, 10*ms);
  timer_generate_event(TIMDELAY, TIM_EGR_UG);
  timer_enable_counter(TIMDELAY);

  while(TIM_CR1(TIMDELAY) & TIM_CR1_CEN);
}
