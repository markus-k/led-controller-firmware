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

#ifndef __MQTT_H
#define __MQTT_H

#include <stdint.h>
#include <wolfmqtt/mqtt_client.h>
#include <winc1500/socket/include/socket.h>

#define MQTT_TX_BUF_SIZE 64
#define MQTT_RX_BUF_SIZE 64
#define MQTT_RX_OVF_BUF_SIZE (MQTT_RX_BUF_SIZE)

typedef enum {
  MQTT_SOCK_STATE_INIT = 0,
  MQTT_SOCK_STATE_CONNECTING,
  MQTT_SOCK_STATE_CONNECT_ERR,
  MQTT_SOCK_STATE_CONNECTED,
  MQTT_SOCK_STATE_READING,
  MQTT_SOCK_STATE_READ_DONE,
  MQTT_SOCK_STATE_READ_ERR,
  MQTT_SOCK_STATE_WRITING,
  MQTT_SOCK_STATE_WRITE_DONE,
  MQTT_SOCK_STATE_WRITE_ERR
} mqtt_sock_state_t;

typedef enum {
  MQTT_CONN_STATE_INIT = 0,
  MQTT_CONN_STATE_NETCONNECT,
  MQTT_CONN_STATE_CONNECT,
  MQTT_CONN_STATE_CONNECTED,
  MQTT_CONN_STATE_FAILED
} mqtt_conn_state_t;

struct mqtt_sock_context {
  SOCKET sock;
  struct sockaddr_in addr;

  mqtt_sock_state_t state;

  /**
   * temporary buffer for storing recieved data
   */
  uint8_t rx_tmp_buf[MQTT_RX_OVF_BUF_SIZE];

  /**
   * recv buffer
   */
  uint8_t rx_ovf_buf[MQTT_RX_OVF_BUF_SIZE];

  /**
   * current read position
   */
  uint16_t rx_ovf_pos;

  /**
   * total buf length
   */
  uint16_t rx_ovf_len;
};

struct mqtt_context {
  struct mqtt_sock_context sock_context;
  MqttClient client;
  MqttConnect connect;
  MqttNet net;

  uint8_t tx_buf[MQTT_TX_BUF_SIZE];
  uint8_t rx_buf[MQTT_RX_BUF_SIZE];

  uint16_t packet_id;
  mqtt_conn_state_t conn_state;
};

void mqtt_socket_handler(SOCKET sock, uint8_t msg_type, void *msg);
void mqtt_init(struct mqtt_context *context);
void mqtt_connect(struct mqtt_context *context);
void mqtt_poll(struct mqtt_context *context);

#endif
