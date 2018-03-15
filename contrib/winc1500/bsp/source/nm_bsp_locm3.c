// board support for winc1500

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "driver/include/m2m_wifi.h"
#include "bsp/include/nm_bsp.h"

#include "../../src/debug.h"
#include "../../src/clock.h"

#define ISR_EXTI_PORT GPIOB
#define ISR_EXTI_RCC  RCC_GPIOB
#define ISR_EXTI_IRQ  NVIC_EXTI15_10_IRQ
#define ISR_EXTI_LINE EXTI14

#define WINC_GPIO_RCC    RCC_GPIOB
#define WINC_GPIO_PORT   GPIOB
#define WINC_ENABLE_PIN  GPIO13
#define WINC_RESET_PIN   GPIO12
#define WINC_IRQ_PIN     GPIO14

sint8 nm_bsp_init() {
#ifdef STM32F3
  // setup gpios
  rcc_periph_clock_enable(WINC_GPIO_RCC);
  gpio_mode_setup(WINC_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, WINC_ENABLE_PIN | WINC_RESET_PIN);

  // setup IRQ line
  gpio_mode_setup(WINC_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, WINC_IRQ_PIN);
  nvic_enable_irq(ISR_EXTI_IRQ);
  exti_select_source(ISR_EXTI_LINE, ISR_EXTI_PORT);
  exti_set_trigger(ISR_EXTI_LINE, EXTI_TRIGGER_FALLING);
  exti_enable_request(ISR_EXTI_LINE);
#else
  gpio_set_mode(WINC_GPIO_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, WINC_ENABLE_PIN | WINC_RESET_PIN);

  gpio_set_mode(WINC_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, WINC_IRQ_PIN);
  nvic_enable_irq(ISR_EXTI_IRQ);
  exti_select_source(ISR_EXTI_LINE, ISR_EXTI_PORT);
  exti_set_trigger(ISR_EXTI_LINE, EXTI_TRIGGER_FALLING);
  exti_enable_request(ISR_EXTI_LINE);
#endif

  // reset the module
  nm_bsp_reset();

  return M2M_SUCCESS;
}

sint8 nm_bsp_deinit() {
  gpio_clear(WINC_GPIO_PORT, WINC_ENABLE_PIN | WINC_RESET_PIN);

  return M2M_SUCCESS;
}

void nm_bsp_reset() {
  gpio_clear(WINC_GPIO_PORT, WINC_ENABLE_PIN | WINC_RESET_PIN);
  nm_bsp_sleep(1);
  gpio_set(WINC_GPIO_PORT, WINC_ENABLE_PIN);
  nm_bsp_sleep(10);
  gpio_set(WINC_GPIO_PORT, WINC_RESET_PIN);
}

void nm_bsp_sleep(uint32 time_msec) {
  clock_delay_ms(time_msec);
}

volatile tpfNmBspIsr bsp_isr;

void nm_bsp_register_isr(tpfNmBspIsr pfIsr) {
  bsp_isr = pfIsr;
}

void nm_bsp_interrupt_ctrl(uint8 enable) {
  if (enable) {
  //DBG("wifi_bsp: enabled irq");
    exti_enable_request(ISR_EXTI_LINE);
  } else {
  //  DBG("wifi_bsp: disabled irq");
    exti_disable_request(ISR_EXTI_LINE);
  }
}

void exti15_10_isr() {
  if (exti_get_flag_status(ISR_EXTI_LINE)) {
    exti_reset_request(ISR_EXTI_LINE);

    DBG("wifi_bsp: isr!");

    if (bsp_isr) {
      bsp_isr();
    }

    //DBG("wifi_bsp: isr done!");
  } else {
    DBG("other exti15_10 isr?");
  }
}
