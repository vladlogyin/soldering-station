#include <main.h>

/**
 * Interrupt-based blinky with extra serial hello world goodness
 */
int main()
{
  clocksetup();
  usartsetup();
  ledsetup();

  char str[] = "Hello World!\n\r";
  usart_print(str);

  while(true);

  return 0;
}

// This isr gets called everytime TIM3 overflows
void tim3_isr()
{
  // Toggle PB12
  gpio_toggle(GPIOB, GPIO12);
  // Clear interrupt flag
  TIM_SR(TIM3) &= ~TIM_SR_UIF;
}

void clocksetup()
{
  // Set up the PLL to generate 72MHz from the external oscillator
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  // Enable the alternate function clock
  rcc_periph_clock_enable(RCC_AFIO);

}

void ledsetup()
{
  // RCC_GPIOB needs to be enabled in order to do anything
  rcc_periph_clock_enable(RCC_GPIOB);
  // Configure PB12 as a push-pull output
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

  // Enable TIM3 clock
  rcc_periph_clock_enable(RCC_TIM3);
  // Set prescaler to 1kHz
  timer_set_prescaler(TIM3, rcc_apb1_frequency / 1000 - 1);
  // Set timer period to 500ms
  timer_set_period(TIM3, 500);
  timer_continuous_mode(TIM3);
  // Configure TIM3 to call an interrupt
  timer_enable_irq(TIM3, TIM_DIER_UIE);
  // Enable the interrupt source and start the timer
  nvic_enable_irq(NVIC_TIM3_IRQ);
  timer_enable_counter(TIM3);
}

void usartsetup(void)
{
  // Enable USART1 clock
  rcc_periph_clock_enable(RCC_USART1);

  // Setup GPIO pin GPIO_USART1_TX on GPIO port A for transmit.
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  // Setup UART parameters.
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART1, USART_MODE_TX);

  // Finally enable the USART
  usart_enable(USART1);
}

int usart_print(const char* str)
{
  int i = 0;

  while(str[i] != '\0')
  {
    usart_send_blocking(USART1, str[i++]);
  }

  return i;
}
