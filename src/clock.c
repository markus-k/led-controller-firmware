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

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>

#include "clock.h"
#include "board.h"
#include "led.h"

volatile uint64_t clock_ticks;

void sys_tick_handler() {
  clock_ticks++;

  board_tick();

  led_update_all_channels();
}

void clock_delay_ms(uint32_t ms) {
  uint64_t start = clock_ticks;
  uint64_t end = start + ms;
  while (clock_ticks < end) ;
}

void clock_init() {
  clock_ticks = 0;

  // 36MHz / 36000 = 1KHz
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_set_reload(35999);
  systick_interrupt_enable();
  systick_counter_enable();
}
