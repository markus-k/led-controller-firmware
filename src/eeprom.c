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

#include <libopencm3/stm32/i2c.h>
#include <string.h>

#include "debug.h"
#include "board.h"
#include "eeprom.h"

volatile struct eeprom_config global_config;

void eeprom_init() {
  // nothing
}

void eeprom_read_page(uint8_t block, uint8_t address, uint8_t *data) {
  uint8_t bus_addr = EEPROM_I2C_ADDRESS(block);
  uint8_t wbuf[1];

  wbuf[0] = address;

  i2c_transfer7(BOARD_EEPROM_I2C, bus_addr, wbuf, sizeof(wbuf), data, EEPROM_PAGE_SIZE);
}

void eeprom_write_page(uint8_t block, uint8_t address, uint8_t *data) {
  uint8_t bus_addr = EEPROM_I2C_ADDRESS(block);
  uint8_t buf[EEPROM_PAGE_SIZE + 1];

  buf[0] = address;
  memcpy(buf + 1, data, EEPROM_PAGE_SIZE);

  i2c_transfer7(BOARD_EEPROM_I2C, bus_addr, buf, sizeof(buf), NULL, 0);
}

void eeprom_read_config(struct eeprom_config *config) {
  uint8_t buf[EEPROM_PAGE_SIZE];

  eeprom_read_page(0, 0, buf);

  for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
    DBG("eeprom: 0x%x", buf[i]);
  }
}

void eeprom_write_config(struct eeprom_config *config) {
  uint8_t buf[EEPROM_PAGE_SIZE];

  eeprom_write_page(0, 0, buf);
}
