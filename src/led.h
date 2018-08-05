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

#ifndef __LED_H
#define __LED_H

#include <stdint.h>

typedef enum {
  LED_CHANNEL_MODE_OFF = 0,
  LED_CHANNEL_MODE_ON
} led_channel_mode_t;

struct led_group {
  uint8_t brightness;
};

struct led_channel {
  uint8_t value;
  led_channel_mode_t mode;
  volatile struct led_group *group;

  volatile uint16_t *pwm_reg;
};

#define LED_CHANNEL_NUM                 12
#define LED_GROUP_NUM                   4

//extern volatile struct led_channel led_channels[];
//extern volatile struct led_group led_groups[];

#include "eeprom.h"

void led_init();

void led_load_config(struct eeprom_config *config);
void led_store_config(struct eeprom_config *config);

void led_ch_set_brightness(int ch, uint8_t value);
void led_ch_set_mode(int ch, uint8_t mode);
void led_grp_set_brightness(int grp, uint8_t value);
void led_set_all_ch_override(uint8_t val);

void led_ch_update(volatile struct led_channel *ch);
void led_update_all_channels();

void led_debug_status();

#endif
