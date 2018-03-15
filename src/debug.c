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
#include <libopencm3/stm32/usart.h>

#include <stdarg.h>

#include "debug.h"

void debug_init() {
#if defined(STM32F3)
  rcc_periph_clock_enable(RCC_GPIOC);
  rcc_periph_clock_enable(RCC_USART1);

  gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5);
  gpio_set_af(GPIOC, GPIO_AF7, GPIO4 | GPIO5);
#elif defined(STM32F1)
  rcc_periph_clock_enable(RCC_USART1);

  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
#else
#  error "Unknown platform for debug"
#endif

  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  usart_enable(USART1);
}

void debug_putc(char c) {
  usart_send_blocking(USART1, c);
}

void debug_puts(char *s) {
  while (*s) {
    debug_putc(*s++);
  }
}

 void debug_puti(uint32_t val, uint8_t base) {
   static const char *chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
   int i = 0;
   char buf[32];

   if (val == 0) {
     buf[0] = chars[0];
     i++;
   } else {
     while (val) {
       uint32_t rem = val % base;
       buf[i] = chars[rem];
       val = val / base;
       i++;
     }
   }

   while (i--) {
     debug_putc(buf[i]);
   }
}

void debug_printf(const char *str, ...) {
  va_list ap;
  int i;

  va_start(ap, str);

  for (i = 0; str[i] != '\0'; i++) {
    char c = str[i];

    if (c == '%' && str[i + 1] != '\0') {
      char nextc = str[i + 1];

      if (nextc == 'l' || (nextc >= '0' && nextc <= '9')) {
	do {
	  // ignore any attributes
	  nextc = str[i + 2];
	  i++;
	} while ((nextc >= '0' && nextc <= '9'));
      }

      switch (nextc) {
      case '%':
	debug_putc('%');
	break;
      case 'c':
	debug_putc((char)va_arg(ap, int));
	break;
      case 's':
	debug_puts(va_arg(ap, char *));
	break;
      case 'u':
	debug_puti(va_arg(ap, int), 10);
	break;
      case 'd':
	{
	  int val = va_arg(ap, int);
	  int mask = val >> 31;
	  val = val ^ mask;
	  val = val - mask;
	  if (mask) {
	    debug_putc('-');
	  }
	  debug_puti(val, 10);
	}
	break;
      case 'x':
	debug_puti(va_arg(ap, int), 16);
	break;
      default:
	debug_putc('%');
	debug_putc(nextc);
      }
      i++; // we already read the next char, so i++
    } else {
      debug_putc(c);
    }
  }

  va_end(ap);
}
