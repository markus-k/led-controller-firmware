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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dbgmcu.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>

#include "board.h"
#include "clock.h"
#include "debug.h"
#include "states.h"
#include "led.h"
#include "mqtt.h"
#include "wifi.h"

static void debug_fault() {
  DBG("SCB_CFSR:  0x%x", SCB_CFSR);
  DBG("SCB_HFSR:  0x%x", SCB_HFSR);

  if (SCB_CFSR & SCB_CFSR_MMARVALID) {
    DBG("SCB_MMFAR: 0x%x", SCB_MMFAR);
  }
  if (SCB_CFSR & SCB_CFSR_BFARVALID) {
    DBG("SCB_BFAR:  0x%x", SCB_BFAR);
  }
  while (1);
}

void bus_fault_handler() {
  DBG("!! BUS FAULT !!");
  debug_fault();
}

void usage_fault_handler() {
  DBG("!! USAGE FAULT !!");
  debug_fault();
}

void hard_fault_handler() {
  DBG("!! HARD FAULT !!");
  debug_fault();
}

const static struct state_transition state_transition_table[] = {
  { STATE_INIT,            STATE_WIFI_CONNECTING, EVENT_WIFI_CONNECTING },
  { STATE_WIFI_CONNECTING, STATE_WIFI_CONNECTED,  EVENT_WIFI_CONNECTED },
  { STATE_WIFI_CONNECTED,  STATE_INIT,            EVENT_WIFI_DISCONNECTED },
  { STATE_WIFI_CONNECTED,  STATE_MQTT_CONNECTING, EVENT_MQTT_CONNECTING },
  { STATE_MQTT_CONNECTING, STATE_IDLE,            EVENT_MQTT_CONNECTED },

  // mqtt failures
  { STATE_MQTT_CONNECTING, STATE_WIFI_CONNECTED,  EVENT_MQTT_DISCONNECTED },
  { STATE_IDLE,            STATE_WIFI_CONNECTED,  EVENT_MQTT_DISCONNECTED }
};
volatile struct state_machine main_state;

void signal_event(event_t event) {
  DBG("got event %u", event);

  // search cur_state+event combo and update current state
  for (int i = 0; i < sizeof(state_transition_table) / sizeof(state_transition_table[0]); i++) {
    const struct state_transition *trans = &state_transition_table[i];
    if (trans->event == event && trans->from == main_state.cur_state) {
      DBG("transition to state %u", trans->to);
      main_state.cur_state = trans->to;
      return;
    }
  }

  DBG("no transition for state %u and event %u", main_state.cur_state, event);
}

struct mqtt_context mqtt_ctx;

void handle_states() {
  state_t state = main_state.cur_state;

  switch (state) {
  case STATE_INIT:
    wifi_connect();
    break;
  case STATE_WIFI_CONNECTING:
    break;
  case STATE_WIFI_CONNECTED:
    DBG("ready to connect mqtt");
    signal_event(EVENT_MQTT_CONNECTING);
    mqtt_init(&mqtt_ctx);
    mqtt_connect(&mqtt_ctx);
    break;
  case STATE_MQTT_CONNECTING:
    break;
  case STATE_IDLE:
    break;
  default:
    DBG("err: unkown state %u");
    break;
  }
}

int main() {
  board_init();

  debug_init();

  DBG("Hello World.");

  clock_init();

  led_init();

  wifi_init();

  main_state.cur_state = STATE_INIT;

  while (1) {
    wifi_poll();
    mqtt_poll(&mqtt_ctx);

    handle_states();

    __asm__ volatile("wfi");
  }

  return 0;
}
