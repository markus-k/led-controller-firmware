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

#ifndef __DEBUG_H
#define __DEBUG_H

#include <stdint.h>
#include "clock.h"

void debug_init();
void debug_putc(char c);
void debug_puts(char *s);
void debug_puti(uint32_t num, uint8_t base);
void debug_printf(const char *format, ...);

// dbg_printf gets removed in non-debug builds
#ifdef DEBUG
#define DBG(format, ...) do { debug_printf("[%u] ", (uint32_t)clock_ticks); debug_printf(format, ##__VA_ARGS__); debug_puts("\r\n"); } while(0)
#define DBG_NLF(format, ...) do { debug_printf(format, ##__VA_ARGS__); } while(0)
#else
#define DBG(format, ...)
#define DBG_NLF(format, ...)
#endif

#endif
