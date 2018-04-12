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

#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/i2c.h>
#include <string.h>

#include "debug.h"
#include "board.h"
#include "clock.h"
#include "eeprom.h"

volatile struct eeprom_config global_config;

void eeprom_init() {
  // nothing
}

void eeprom_read_page(uint8_t page, uint8_t *data) {
  uint8_t block = page / 16;
  uint8_t bus_addr = EEPROM_I2C_ADDRESS(block);
  uint8_t wbuf[1];

  wbuf[0] = page * EEPROM_PAGE_SIZE;

  i2c_transfer7(BOARD_EEPROM_I2C, bus_addr, wbuf, sizeof(wbuf), data, EEPROM_PAGE_SIZE);
}

void eeprom_write_page(uint8_t page, uint8_t *data) {
  uint8_t block = page / 16;
  uint8_t bus_addr = EEPROM_I2C_ADDRESS(block);
  uint8_t buf[EEPROM_PAGE_SIZE + 1];

  buf[0] = page * EEPROM_PAGE_SIZE;
  memcpy(buf + 1, data, EEPROM_PAGE_SIZE);

  i2c_transfer7(BOARD_EEPROM_I2C, bus_addr, buf, sizeof(buf), NULL, 0);

  clock_delay_ms(5);
}

static uint32_t crc_page(uint32_t *page) {
  return crc_calculate_block(page, EEPROM_PAGE_SIZE / sizeof(uint32_t));
}

void eeprom_read_config(struct eeprom_config *config) {
  uint8_t buf[EEPROM_PAGE_SIZE];
  uint32_t crc, crc_stored;
  uint8_t page = 0;
  int i, j;

  crc_reset();

  eeprom_read_page(page++, buf);

  crc_stored = buf[0] << 24;
  crc_stored |= buf[1] << 16;
  crc_stored |= buf[2] << 8;
  crc_stored |= buf[3];

  // clear crc in first page for crc calculation
  memset(buf, 0, 4);
  crc_page((uint32_t *)buf);

  memcpy(config->reserved1, buf + 4, 12);

  // read 6x 2 channels of 8 bytes each
  for (i = 0; i < 6; i++) {
    eeprom_read_page(page++, buf);
    crc_page((uint32_t *)buf);

    for (j = 0; j < 2; j++) {
      struct eeprom_ch_config *ch = &config->channels[2 * i + j];
      ch->value = buf[8 * j + 0];
      ch->mode = buf[8 * j + 1];
      memcpy(ch->reserved1, buf + 8 * i + 2, 6);
    }
  }

  // read 2x 2 groups of 8 bytes each
  for (i = 0; i < 2; i++) {
    eeprom_read_page(page++, buf);
    crc_page((uint32_t *)buf);

    for (j = 0; j < 2; j++) {
      struct eeprom_grp_config *grp = &config->groups[2 * i + j];
      grp->brightness = buf[8 * j + 0];
      memcpy(grp->reserved1, buf + 8 * j + 1, 7);
    }
  }

  // wifi ssid
  for (i = 0; i < 4; i++) {
    eeprom_read_page(page++, buf);
    crc_page((uint32_t *)buf);

    memcpy(config->wifi_ssid + i * 16, buf, 16);
  }

  // wifi psk
  for (i = 0; i < 4; i++) {
    eeprom_read_page(page++, buf);
    crc_page((uint32_t *)buf);

    memcpy(config->wifi_psk + i * 16, buf, 16);
  }

  // wifi reserved space
  eeprom_read_page(page++, buf);
  crc_page((uint32_t *)buf);
  memcpy(config->reserved2, buf, 16);

  // mqtt hostname
  for (i = 0; i < 2; i++) {
    eeprom_read_page(page++, buf);
    crc_page((uint32_t *)buf);
    memcpy(config->mqtt_hostname + i * 16, buf, 16);
  }

  eeprom_read_page(page++, buf);
  crc_page((uint32_t *)buf);
  config->mqtt_port = buf[0] << 8;
  config->mqtt_port |= buf[1];
  memcpy(config->reserved3, buf + 2, 14);

  // mqtt username
  for (i = 0; i < 2; i++) {
    eeprom_read_page(page++, buf);
    crc_page((uint32_t *)buf);

    memcpy(config->mqtt_username + i * 16, buf, 16);
  }

  // mqtt password
  for (i = 0; i < 2; i++) {
    eeprom_read_page(page++, buf);
    crc = crc_page((uint32_t *)buf);

    memcpy(config->mqtt_password + i * 16, buf, 16);
  }

  if (crc != crc_stored) {
    // CRC mismatch
    DBG("eeprom: CRC mismatch while reading, read: 0x%x, calculated: 0x%x", crc_stored, crc);
  }
}

void eeprom_write_config(struct eeprom_config *config) {
  uint8_t buf[EEPROM_PAGE_SIZE];
  uint32_t crc;
  uint8_t page = 0;
  int i, j;

  crc_reset();

  // clear checksum
  memset(buf, 0, 4);
  memcpy(buf + 4, config->reserved1, 12);
  crc_page((uint32_t *)buf);

  // dont write this page yet
  //eeprom_write_page(page++, buf);
  page++;

  // write 6x 2 channels of 8 bytes each
  for (i = 0; i < 6; i++) {
    for (j = 0; j < 2; j++) {
      struct eeprom_ch_config *ch = &config->channels[2 * i + j];
      buf[8 * j + 0] = ch->value;
      buf[8 * j + 1] = ch->mode;
      memcpy(buf + 8 * j + 2, ch->reserved1, 6);
    }

    crc_page((uint32_t *)buf);
    eeprom_write_page(page++, buf);
  }

  // write 2x 2 groups of 8 bytes each
  for (i = 0; i < 2; i++) {
    for (j = 0; j < 2; j++) {
      struct eeprom_grp_config *grp = &config->groups[2 * i + j];
      buf[8 * j + 0] = grp->brightness;
      memcpy(buf + 8 * j + 1, grp->reserved1, 7);
    }

    crc_page((uint32_t *)buf);
    eeprom_write_page(page++, buf);
  }

  // wifi ssid
  for (i = 0; i < 4; i++) {
    memcpy(buf, config->wifi_ssid + i * 16, 16);
    crc_page((uint32_t *)buf);
    eeprom_write_page(page++, buf);
  }

  // wifi psk
  for (i = 0; i < 4; i++) {
    memcpy(buf, config->wifi_psk + i * 16, 16);
    crc_page((uint32_t *)buf);
    eeprom_write_page(page++, buf);
  }

  // wifi reserved space
  memcpy(buf, config->reserved2, 16);
  crc_page((uint32_t *)buf);
  eeprom_write_page(page++, buf);

  // mqtt hostname
  for (i = 0; i < 2; i++) {
    memcpy(buf, config->mqtt_hostname + i * 16, 16);
    crc_page((uint32_t *)buf);
    eeprom_write_page(page++, buf);
  }

  buf[0] = config->mqtt_port >> 8;
  buf[1] = config->mqtt_port & 0xFF;
  memcpy(buf + 2, config->reserved3, 14);
  crc_page((uint32_t *)buf);
  eeprom_write_page(page++, buf);

  // mqtt username
  for (i = 0; i < 2; i++) {
    memcpy(buf, config->mqtt_username + i * 16, 16);
    crc_page((uint32_t *)buf);
    eeprom_write_page(page++, buf);
  }

  // mqtt password
  for (i = 0; i < 2; i++) {
    memcpy(buf, config->mqtt_password + i * 16, 16);
    crc = crc_page((uint32_t *)buf);
    eeprom_write_page(page++, buf);
  }

  // write first block with valid crc
  buf[0] = (crc >> 24) & 0xFF;
  buf[1] = (crc >> 16) & 0xFF;
  buf[2] = (crc >> 8) & 0xFF;
  buf[3] = crc & 0xFF;
  memcpy(buf + 4, config->reserved1, 12);
  eeprom_write_page(0, buf);

  DBG("eeprom: written up to addr %u with crc 0x%x", page, crc);
}
