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

#ifndef __STATES_H
#define __STATES_H

typedef enum {
  /**
   * WiFi disconnected
   */
  STATE_INIT,

  /**
   * currently connecting to WiFi
   */
  STATE_WIFI_CONNECTING,

  /**
   * WiFi connection up, waiting for MQTT to connect
   */
  STATE_WIFI_CONNECTED,

  /**
   * MQTT connecting to server
   */
  STATE_MQTT_CONNECTING,

  /**
   * WiFi & MQTT ready
   */
  STATE_IDLE,

  NUM_STATES
} state_t;

typedef enum {
  EVENT_WIFI_CONNECTING,
  EVENT_WIFI_CONNECTED,
  EVENT_WIFI_DISCONNECTED,
  EVENT_MQTT_CONNECTING,
  EVENT_MQTT_CONNECTED,
  EVENT_MQTT_DISCONNECTED,
  EVENT_BUTTON_PRESSED,
  NUM_EVENTS
} event_t;

struct state_transition {
  state_t from;
  state_t to;
  event_t event;
};

struct state_machine {
  state_t cur_state;
};

void signal_event(event_t event);

#endif
