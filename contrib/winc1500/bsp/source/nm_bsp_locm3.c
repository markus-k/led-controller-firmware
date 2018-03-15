// board support for winc1500

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#include "driver/include/m2m_wifi.h"
#include "bsp/include/nm_bsp.h"

#include "../../src/board.h"
#include "../../src/debug.h"
#include "../../src/clock.h"


static volatile tpfNmBspIsr bsp_isr;

static void winc_irq() {
  DBG("wifi_bsp: isr!");

  if (bsp_isr) {
    bsp_isr();
  }
}

sint8 nm_bsp_init() {
  // GPIOs are already set up for us in board.c

  // set and enable irq
  board_register_winc_irq(winc_irq);
  nm_bsp_interrupt_ctrl(1);

  // reset the module
  nm_bsp_reset();

  return M2M_SUCCESS;
}

sint8 nm_bsp_deinit() {
  gpio_clear(BOARD_WINC_PORT, BOARD_WINC_ENABLE_PIN | BOARD_WINC_RESET_PIN);

  return M2M_SUCCESS;
}

void nm_bsp_reset() {
  gpio_clear(BOARD_WINC_PORT, BOARD_WINC_ENABLE_PIN | BOARD_WINC_RESET_PIN);
  nm_bsp_sleep(1);
  gpio_set(BOARD_WINC_PORT, BOARD_WINC_ENABLE_PIN);
  nm_bsp_sleep(10);
  gpio_set(BOARD_WINC_PORT, BOARD_WINC_RESET_PIN);
}

void nm_bsp_sleep(uint32 time_msec) {
  clock_delay_ms(time_msec);
}

void nm_bsp_register_isr(tpfNmBspIsr pfIsr) {
  bsp_isr = pfIsr;
}

void nm_bsp_interrupt_ctrl(uint8 enable) {
  if (enable) {
    //DBG("wifi_bsp: enabled irq");
    exti_enable_request(BOARD_WINC_IRQ_EXTI);
  } else {
    //DBG("wifi_bsp: disabled irq");
    exti_disable_request(BOARD_WINC_IRQ_EXTI);
  }
}
