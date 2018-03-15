/*
 *  Copyright (C) 2018  Markus Kasten
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

#include "clock.h"
#include "debug.h"
#include "states.h"
#include "led.h"
#include "mqtt.h"
#include "wifi.h"

#define LED_GPIO_MASK	(GPIO8  | GPIO9  | GPIO10 | GPIO11 | \
			 GPIO12 | GPIO13 | GPIO14 | GPIO15)

static void gpio_setup(void) {
#if defined(STM32F3)
  /* Enable GPIOE clock. */
  rcc_periph_clock_enable(RCC_GPIOE);

  /* Set LEDs (in GPIO port E) to 'output push-pull'. */
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_GPIO_MASK);

  // MEMS CS high
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
  gpio_set(GPIOE, GPIO3);

  // User button interrupt
  rcc_periph_clock_enable(RCC_GPIOA);
  nvic_enable_irq(NVIC_EXTI0_IRQ);
  exti_select_source(EXTI0, GPIOA);
  exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI0);

  // LED row driver
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);
#elif defined(STM32F1)
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_AFIO);

  AFIO_MAPR = AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON | AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP;

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO14 | GPIO15);
#endif
}

static void rcc_setup() {
#if defined(STM32F3)
  // HSE input is 8MHZ
  // PLL set to 9x = 72MHz
  // USB prescaler set to 1.5 = 48MHz

  // set up mco
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO8);
  rcc_set_mco(RCC_CFGR_MCO_HSI);

  // enable the HSE oscillator and wait to get ready
  rcc_osc_bypass_enable(RCC_HSE);
  rcc_osc_on(RCC_HSE);
  rcc_wait_for_osc_ready(RCC_HSE);

  // shut down PLL, configure it and start it again
  rcc_osc_off(RCC_PLL);
  rcc_wait_for_osc_not_ready(RCC_PLL);
  rcc_set_prediv(RCC_CFGR2_PREDIV_DIV2);
  rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_PREDIV);
  rcc_set_pll_multiplier(RCC_CFGR_PLLMUL_PLL_IN_CLK_X9);
  rcc_osc_on(RCC_PLL);
  rcc_wait_for_osc_ready(RCC_PLL);

  // set PLL/2 to mco
  rcc_set_mco(RCC_CFGR_MCO_PLL);

  // set up peripheral prescalers
  rcc_usb_prescale_1_5();
  rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);
  rcc_set_ppre1(RCC_CFGR_PPRE1_DIV_2);
  rcc_set_ppre2(RCC_CFGR_PPRE2_DIV_NONE);

  // flash needs two wait states at sysclk >= 48MHz
  flash_set_ws(FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2WS);

  // set sysclk source to PLL
  rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
  rcc_wait_for_sysclk_status(RCC_PLL);

  // set SYSCLK to mco
  rcc_set_mco(RCC_CFGR_MCO_SYSCLK);

  rcc_periph_clock_enable(RCC_SYSCFG);

  rcc_ahb_frequency = 36000000;
  rcc_apb1_frequency = 18000000;
  rcc_apb2_frequency = 36000000;
#elif defined(STM32F1)
  //rcc_clock_setup_in_hsi_out_24mhz();
  /* Enable internal high-speed oscillator. */
  rcc_osc_on(RCC_HSI);
  rcc_wait_for_osc_ready(RCC_HSI);

  /* Select HSI as SYSCLK source. */
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_HSICLK);

  /*
   * Set prescalers for AHB, ADC, ABP1, ABP2.
   * Do this before touching the PLL (TODO: why?).
   */
  rcc_set_hpre(RCC_CFGR_HPRE_SYSCLK_NODIV);   /* Set. 36MHz Max. 36MHz */
  rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV2); /* Set. 18MHz Max. 36MHz */
  rcc_set_ppre1(RCC_CFGR_PPRE1_HCLK_NODIV);   /* Set. 36MHz Max. 36MHz */
  rcc_set_ppre2(RCC_CFGR_PPRE2_HCLK_NODIV);   /* Set. 36MHz Max. 36MHz */

  /*
   * Sysclk is running with 64MHz -> 2 waitstates.
   * 0WS from 0-24MHz
   * 1WS from 24-48MHz
   * 2WS from 48-72MHz
   */
  flash_set_ws(FLASH_ACR_LATENCY_1WS);

  /*
   * Set the PLL multiplication factor to 9.
   * 8MHz (internal) * 9 (multiplier) / 2 (PLLSRC_HSI_CLK_DIV2) = 36MHz
   */
  rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_PLL_CLK_MUL9);

  /* Select HSI/2 as PLL source. */
  rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);

	/* Enable PLL oscillator and wait for it to stabilize. */
  rcc_osc_on(RCC_PLL);
  rcc_wait_for_osc_ready(RCC_PLL);

  /* Select PLL as SYSCLK source. */
  rcc_set_sysclk_source(RCC_CFGR_SW_SYSCLKSEL_PLLCLK);

  /* Set the peripheral clock frequencies used */
  rcc_ahb_frequency = 36000000;
  rcc_apb1_frequency = 36000000;
  rcc_apb2_frequency = 36000000;
#endif
}

void exti0_isr() {
  if (exti_get_flag_status(EXTI0)) {
    exti_reset_request(EXTI0);

    gpio_toggle(GPIOE, GPIO9);
  }
}

void demo_fading() {
  static uint8_t pre = 0;
  static uint8_t dir = 0;

  pre++;
  if (pre > 50) {
    if (dir == 0) {
      led_channels[3].value++;
    } else {
      led_channels[3].value--;
    }

    if (led_channels[3].value == 0 || led_channels[3].value == 0xFF) {
      dir = !dir;
    }
    pre = 0;
  }
}

const static struct state_transition state_transition_table[] = {
  { STATE_INIT,            STATE_WIFI_CONNECTING, EVENT_WIFI_CONNECTING },
  { STATE_WIFI_CONNECTING, STATE_WIFI_CONNECTED,  EVENT_WIFI_CONNECTED },
  { STATE_WIFI_CONNECTED,  STATE_INIT,            EVENT_WIFI_DISCONNECTED },
  { STATE_WIFI_CONNECTED,  STATE_MQTT_CONNECTING, EVENT_MQTT_CONNECTING },
  { STATE_MQTT_CONNECTING, STATE_IDLE,            EVENT_MQTT_CONNECTED },

  // mqtt failures
  { STATE_MQTT_CONNECTING, STATE_WIFI_CONNECTED,  EVENT_MQTT_DISCONNECTED },
  { STATE_IDLE,            STATE_WIFI_CONNECTED,  EVENT_MQTT_DISCONNECTED }
};
volatile struct state_machine main_state;

void signal_event(event_t event) {
  DBG("got event %u", event);

  // search cur_state+event combo and update current state
  for (int i = 0; i < sizeof(state_transition_table) / sizeof(state_transition_table[0]); i++) {
    const struct state_transition *trans = &state_transition_table[i];
    if (trans->event == event && trans->from == main_state.cur_state) {
      DBG("transition to state %u", trans->to);
      main_state.cur_state = trans->to;
      return;
    }
  }

  DBG("no transition for state %u and event %u", main_state.cur_state, event);
}

struct mqtt_context mqtt_ctx;

void handle_states() {
  state_t state = main_state.cur_state;

  switch (state) {
  case STATE_INIT:
    wifi_connect();
    break;
  case STATE_WIFI_CONNECTING:
    break;
  case STATE_WIFI_CONNECTED:
    DBG("ready to connect mqtt");
    signal_event(EVENT_MQTT_CONNECTING);
    mqtt_init(&mqtt_ctx);
    break;
  case STATE_MQTT_CONNECTING:
    mqtt_connect(&mqtt_ctx);
    break;
  case STATE_IDLE:
    break;
  default:
    DBG("err: unkown state %u");
    break;
  }
}

int main() {
  // enable debugging during sleep
  DBGMCU_CR |= DBGMCU_CR_SLEEP;

  rcc_setup();
  gpio_setup();
  debug_init();
  DBG("Hello World.");

  clock_init();

  led_init();

  gpio_set(GPIOA, GPIO15);
  gpio_set(GPIOC, GPIO14);

  wifi_init();


  main_state.cur_state = STATE_INIT;

  while (1) {
    wifi_poll();
    mqtt_poll(&mqtt_ctx);

    handle_states();

    demo_fading();

    __asm__ volatile("wfi");
  }

  return 0;
}
