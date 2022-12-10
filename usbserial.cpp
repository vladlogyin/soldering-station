#include <usbserial.h>

const char hello[] = "Hello World!\n\r";

/**
 * CDC ACM hello world example
 */
int main()
{
  clocksetup();
  usbsetup();
  delay_setup();

  while(true)
  {
    usbd_ep_write_packet(usbd_dev, 0x82, hello, sizeof(hello));
    delay_ms(1000);
  }

  return 0;
}

void clocksetup()
{
  // Set up the PLL to generate 72MHz from the external oscillator
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

}

void usbsetup()
{
  // Enable the USB clock
  rcc_periph_clock_enable(RCC_USB);

  // Initialize the USB driver
  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

  // Enable the low priority USB interrupt
  nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

void usb_lp_can_rx0_isr()
{
  usbd_poll(usbd_dev);
}
