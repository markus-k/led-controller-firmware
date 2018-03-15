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

#ifndef __CONFIG_H
#define __CONFIG_H

#include "debug.h"

#define WIFI_SSID "test"
#define WIFI_PSK  "test"

/* wolfmqtt configuration */
//#define WOLFMQTT_NO_STDIO
#define WOLFMQTT_DEBUG_CLIENT
#define WOLFMQTT_CUSTOM_PRINTF
#define PRINTF(format, ...)  do{DBG((format), ##__VA_ARGS__);}while(0)

#endif
