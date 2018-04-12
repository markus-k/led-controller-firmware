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
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

#include "board.h"

static volatile board_exti_func_t winc_exti_func = 0;
static volatile board_blinkrate_t blinkrates[BOARD_LED_NUM] = {
  BOARD_BLINKRATE_SLOW,
  BOARD_BLINKRATE_ON,
  BOARD_BLINKRATE_OFF
};

static const uint16_t led_gpio_pins[BOARD_LED_NUM] = {
  BOARD_LED_ONBOARD_PIN,
  BOARD_LED_FRONT_GREEN_PIN,
  BOARD_LED_FRONT_RED_PIN
};

static const uint32_t led_gpio_ports[BOARD_LED_NUM] = {
  BOARD_LED_ONBOARD_PORT,
  BOARD_LED_FRONT_PORT,
  BOARD_LED_FRONT_PORT
};

/**
 * set up rcc to 36MHz sysclk from HSI
 *
 * this is a modified version of rcc_clock_setup_in_hsi_out_24mhz()
 */
static void setup_rcc() {
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
}

/**
 * enable all required peripheral clocks
 */
static void enable_periph_clocks() {
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_I2C2);
  rcc_periph_clock_enable(RCC_CRC);
}

/**
 * setup gpios
 */
static void setup_gpio() {
  // enable Pin A15 as GPIO and remap TIM3 so SPI1 is usable
  AFIO_MAPR = AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON | AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP;

  // setup led gpios
  gpio_set_mode(BOARD_LED_ONBOARD_PORT,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		BOARD_LED_ONBOARD_PIN);
  gpio_set_mode(BOARD_LED_FRONT_PORT,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		BOARD_LED_FRONT_RED_PIN | BOARD_LED_FRONT_GREEN_PIN);

  // setup button inputs
  gpio_set_mode(BOARD_BTN_PORT,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN,
		BOARD_BTN_ONBOARD_PIN);
  gpio_set_mode(BOARD_BTN_PORT,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,
		BOARD_BTN_EXTERNAL_PIN);

  // setup winc gpios
  gpio_set_mode(BOARD_WINC_PORT,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		BOARD_WINC_ENABLE_PIN | BOARD_WINC_RESET_PIN);
  gpio_set_mode(BOARD_WINC_PORT,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,
		BOARD_WINC_IRQ_PIN);
}

static void setup_exti() {
  nvic_enable_irq(NVIC_EXTI15_10_IRQ);

  // onboard button
  exti_select_source(BOARD_BTN_ONBOARD_PIN | BOARD_BTN_EXTERNAL_EXTI,
		     BOARD_BTN_PORT);
  exti_set_trigger(BOARD_BTN_ONBOARD_PIN | BOARD_BTN_EXTERNAL_EXTI,
		   EXTI_TRIGGER_FALLING);
  exti_enable_request(BOARD_BTN_ONBOARD_EXTI | BOARD_BTN_EXTERNAL_EXTI);

  // winc irq
  exti_select_source(BOARD_WINC_IRQ_EXTI, BOARD_WINC_PORT);
  exti_set_trigger(BOARD_WINC_IRQ_EXTI, EXTI_TRIGGER_FALLING);
  // don't enable this exti line yet, controlled by winc bsp
}

void exti15_10_isr() {
  if (exti_get_flag_status(BOARD_BTN_ONBOARD_EXTI)) {
    exti_reset_request(BOARD_BTN_ONBOARD_EXTI);

    // todo
  }

  if (exti_get_flag_status(BOARD_BTN_EXTERNAL_EXTI)) {
    exti_reset_request(BOARD_BTN_EXTERNAL_EXTI);

    // todo
  }

  if (exti_get_flag_status(BOARD_WINC_IRQ_EXTI)) {
    exti_reset_request(BOARD_WINC_IRQ_EXTI);

    if (winc_exti_func) {
      winc_exti_func();
    }
  }
}

static void setup_i2c() {
  gpio_set_mode(BOARD_I2C_PORT,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
		BOARD_I2C_SDA_PIN | BOARD_I2C_SCL_PIN);

  i2c_reset(BOARD_I2C);
  i2c_peripheral_disable(BOARD_I2C);
  i2c_set_clock_frequency(BOARD_I2C, I2C_CR2_FREQ_36MHZ);
  i2c_set_fast_mode(BOARD_I2C);
  /*
   * fclock for I2C is 36MHz APB2 -> cycle time 28ns, low time at 400kHz
   * incl trise -> Thigh = 1600ns; CCR = tlow/tcycle = 0x1C,9;
   * Datasheet suggests 0x1e.
   */
  i2c_set_ccr(I2C2, 0x1e);

  /*
   * fclock for I2C is 36MHz -> cycle time 28ns, rise time for
   * 400kHz => 300ns and 100kHz => 1000ns; 300ns/28ns = 10;
   * Incremented by 1 -> 11.
   */
  i2c_set_trise(I2C2, 0x0b);
  i2c_peripheral_enable(BOARD_I2C);
}

void board_register_winc_irq(board_exti_func_t func) {
  winc_exti_func = func;
}

static void update_status_leds() {
  static uint16_t prescaler_slow = 0;
  static uint16_t prescaler_fast = 0;
  uint8_t toggle_slow = 0;
  uint8_t toggle_fast = 0;

  if (prescaler_slow == BOARD_BLINKRATE_PRESCALER_SLOW) {
    prescaler_slow = 0;
    toggle_slow = 1;
  }
  if (prescaler_fast == BOARD_BLINKRATE_PRESCALER_FAST) {
    prescaler_fast = 0;
    toggle_fast = 1;
  }

  for (int led = 0; led < BOARD_LED_NUM; led++) {
    uint32_t port = led_gpio_ports[led];
    uint16_t pin = led_gpio_pins[led];

    switch (blinkrates[led]) {
    case BOARD_BLINKRATE_OFF:
      gpio_clear(port, pin);
      break;
    case BOARD_BLINKRATE_ON:
      gpio_set(port, pin);
      break;
    case BOARD_BLINKRATE_SLOW:
      if (toggle_slow) {
	gpio_toggle(port, pin);
      }
      break;
    case BOARD_BLINKRATE_FAST:
      if (toggle_fast) {
	gpio_toggle(port, pin);
      }
      break;
    }
  }

  prescaler_fast++;
  prescaler_slow++;
}

/**
 * called every 1ms by clock.c
 */
void board_tick() {
  update_status_leds();
}

void board_set_blinkrate(board_led_t led, board_blinkrate_t blinkrate) {
  blinkrates[led] = blinkrate;
}

void board_init() {
  setup_rcc();
  enable_periph_clocks();
  setup_gpio();
  setup_exti();
  setup_i2c();

  // enable debugging during sleep
  DBGMCU_CR |= DBGMCU_CR_SLEEP;

  // enable additional faults
  SCB_SHCSR |= SCB_SHCSR_USGFAULTENA | SCB_SHCSR_BUSFAULTENA | SCB_SHCSR_MEMFAULTENA;
}
