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

#ifndef __EEPROM_H
#define __EEPROM_H

#include <stdint.h>

#define EEPROM_I2C_ADDRESS(block)       (0b1010000 | ((block) & 0x7))
#define EEPROM_PAGE_SIZE                16

struct eeprom_ch_config {
  uint8_t value;
  uint8_t mode;
};

struct eeprom_grp_config {
  uint8_t brightness;
};

struct eeprom_config {
  uint32_t chksum;

  char wifi_ssid[64];
  char wifi_psk[64];

  char mqtt_hostname[32];
  uint16_t mqtt_port;
  char mqtt_username[32];
  char mqtt_password[32];

  struct eeprom_ch_config channels[12];
  struct eeprom_grp_config groups[4];
};

extern volatile struct eeprom_config global_config;

void eeprom_init();
void eeprom_read_page(uint8_t block, uint8_t address, uint8_t *data);
void eeprom_write_page(uint8_t block, uint8_t address, uint8_t *data);
void eeprom_read_config(struct eeprom_config *config);
void eeprom_write_config(struct eeprom_config *config);

#endif