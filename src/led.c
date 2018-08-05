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

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "debug.h"
#include "eeprom.h"
#include "led.h"
#include "gamma-lut.h"

volatile struct led_channel led_channels[LED_CHANNEL_NUM];
volatile struct led_group led_groups[LED_GROUP_NUM];
static volatile uint8_t all_ch_override = 1;

static void setup_pwm(uint32_t timer_periph) {
  timer_set_mode(timer_periph, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  timer_enable_oc_output(timer_periph, TIM_OC1);

  const uint32_t pwm_mode = TIM_OCM_PWM1;
  timer_set_oc_mode(timer_periph, TIM_OC1, pwm_mode);
  timer_set_oc_mode(timer_periph, TIM_OC2, pwm_mode);
  timer_set_oc_mode(timer_periph, TIM_OC3, pwm_mode);
  timer_set_oc_mode(timer_periph, TIM_OC4, pwm_mode);
  timer_set_prescaler(timer_periph, 0);
  //timer_set_period(timer_periph, 65535);
  //timer_set_period(timer_periph, 8191); // 13 bit
  timer_set_period(timer_periph, 1023); // 10 bit

  timer_enable_oc_output(timer_periph, TIM_OC1);
  timer_enable_oc_output(timer_periph, TIM_OC2);
  timer_enable_oc_output(timer_periph, TIM_OC3);
  timer_enable_oc_output(timer_periph, TIM_OC4);

  timer_enable_break_main_output(timer_periph);
  timer_enable_counter(timer_periph);
}

void led_load_config(struct eeprom_config *config) {
  int i;

  for (i = 0; i < LED_CHANNEL_NUM; i++) {
    led_channels[i].value = config->channels[i].value;
    led_channels[i].mode = config->channels[i].mode;
  }

  for (i = 0; i < LED_GROUP_NUM; i++) {
    led_groups[i].brightness = config->groups[i].brightness;
  }
}

void led_store_config(struct eeprom_config *config) {
  int i;

  for (i = 0; i < LED_CHANNEL_NUM; i++) {
    config->channels[i].value = led_channels[i].value;
    config->channels[i].mode = led_channels[i].mode;
  }

  for (i = 0; i < LED_GROUP_NUM; i++) {
    config->groups[i].brightness = led_groups[i].brightness;
  }
}

void led_init() {
  int i;

  for (i = 0; i < LED_GROUP_NUM; i++) {
    led_groups[i].brightness = 255;
  }

  for (i = 0; i < LED_CHANNEL_NUM; i++) {
    led_channels[i].value = 1;
    led_channels[i].mode = LED_CHANNEL_MODE_OFF;
    led_channels[i].group = &led_groups[i / (LED_CHANNEL_NUM / LED_GROUP_NUM)];
    led_channels[i].pwm_reg = 0;
  }

  const uint32_t gpioa_mask = GPIO0 | GPIO1 | GPIO2 | GPIO3;
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, gpioa_mask);

  const uint32_t gpiob_mask = GPIO0 | GPIO1 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9;
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, gpiob_mask);

  rcc_periph_clock_enable(RCC_TIM2);
  rcc_periph_clock_enable(RCC_TIM3);
  rcc_periph_clock_enable(RCC_TIM4);
  setup_pwm(TIM2);
  setup_pwm(TIM3);
  setup_pwm(TIM4);

  led_channels[0].pwm_reg = (volatile uint16_t *)&TIM_CCR1(TIM2);
  led_channels[1].pwm_reg = (volatile uint16_t *)&TIM_CCR2(TIM2);
  led_channels[2].pwm_reg = (volatile uint16_t *)&TIM_CCR3(TIM2);
  led_channels[3].pwm_reg = (volatile uint16_t *)&TIM_CCR4(TIM2);

  led_channels[4].pwm_reg = (volatile uint16_t *)&TIM_CCR1(TIM3);
  led_channels[5].pwm_reg = (volatile uint16_t *)&TIM_CCR2(TIM3);
  led_channels[6].pwm_reg = (volatile uint16_t *)&TIM_CCR3(TIM3);
  led_channels[7].pwm_reg = (volatile uint16_t *)&TIM_CCR4(TIM3);

  led_channels[8].pwm_reg = (volatile uint16_t *)&TIM_CCR1(TIM4);
  led_channels[9].pwm_reg = (volatile uint16_t *)&TIM_CCR2(TIM4);
  led_channels[10].pwm_reg = (volatile uint16_t *)&TIM_CCR3(TIM4);
  led_channels[11].pwm_reg = (volatile uint16_t *)&TIM_CCR4(TIM4);
}

void led_ch_set_brightness(int ch, uint8_t value) {
  led_channels[ch].value = value;
}

void led_ch_set_mode(int ch, uint8_t mode) {
  led_channels[ch].mode = mode;
}

void led_grp_set_brightness(int grp, uint8_t value) {
  led_groups[grp].brightness = value;
}

void led_set_all_ch_override(uint8_t val) {
  all_ch_override = val;
}

void led_ch_update(volatile struct led_channel *ch) {
  uint16_t value = ch->value;

  // group brightness
  value = (uint16_t)value * (uint16_t)ch->group->brightness;
  value = (value + 1 + (value >> 8)) >> 8; // /255

  // gamma correction
  value = gamma_lut[value];

  if (all_ch_override == 0 || ch->mode == LED_CHANNEL_MODE_OFF) {
    *ch->pwm_reg = 0;
  } else {
    *ch->pwm_reg = value;
  }
}

void led_update_all_channels() {
  for (int i = 0; i < LED_CHANNEL_NUM; i++) {
    led_ch_update(&led_channels[i]);
  }
}

void led_debug_status() {
  for (int i = 0; i < LED_CHANNEL_NUM; i++) {
    DBG("led_channels[%d] = { .value = %d, .mode = %d }", i, led_channels[i].value, led_channels[i].mode);
  }

  for (int i = 0; i < LED_GROUP_NUM; i++) {
    DBG("led_groups[%d] = { .brightness = %d }", i, led_groups[i].brightness);
  }
}
