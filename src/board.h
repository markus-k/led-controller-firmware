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

#ifndef __BOARD_H
#define __BOARD_H

#include <libopencm3/stm32/gpio.h>

// LED GPIOs
#define BOARD_LED_ONBOARD_PIN       GPIO15
#define BOARD_LED_ONBOARD_PORT      GPIOA
#define BOARD_LED_FRONT_RED_PIN     GPIO15
#define BOARD_LED_FRONT_GREEN_PIN   GPIO14
#define BOARD_LED_FRONT_PORT        GPIOC

// button GPIOs
#define BOARD_BTN_ONBOARD_PIN       GPIO11
#define BOARD_BTN_ONBOARD_EXTI      EXTI11
#define BOARD_BTN_EXTERNAL_PIN      GPIO12
#define BOARD_BTN_EXTERNAL_EXTI     EXTI12
#define BOARD_BTN_PORT              GPIOA

// winc GPIOs
#define BOARD_WINC_IRQ_PIN          GPIO14
#define BOARD_WINC_IRQ_EXTI         EXTI14
#define BOARD_WINC_ENABLE_PIN       GPIO13
#define BOARD_WINC_RESET_PIN        GPIO12
#define BOARD_WINC_WAKE_PIN         GPIO15
#define BOARD_WINC_PORT             GPIOB

// eeprom GPIOs
#define BOARD_I2C                   I2C2
#define BOARD_I2C_PORT              GPIOB
#define BOARD_I2C_SCL_PIN           GPIO10
#define BOARD_I2C_SDA_PIN           GPIO11

#define BOARD_EEPROM_I2C            BOARD_I2C

typedef void (*board_exti_func_t)();

typedef enum {
  BOARD_LED_ONBOARD = 0,
  BOARD_LED_FRONT_RED,
  BOARD_LED_FRONT_GREEN,
  BOARD_LED_NUM,
} board_led_t;

#define BOARD_BLINKRATE_PRESCALER_SLOW 1000
#define BOARD_BLINKRATE_PRESCALER_FAST 200

typedef enum {
  BOARD_BLINKRATE_OFF = 0,
  BOARD_BLINKRATE_ON,
  BOARD_BLINKRATE_SLOW,
  BOARD_BLINKRATE_FAST
} board_blinkrate_t;

void board_init();
void board_register_winc_irq(board_exti_func_t func);
void board_tick();

#endif
