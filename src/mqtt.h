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

/*
 * mqtt topic structure overview:
 *
 * subscribed topics:
 * - <base>/all/set: turn all channels on/off, values 0 or 1
 * - <base>/ch[1-12]/brightness/set: channel brightness, values 0 to 255
 * - <base>/grp[1-4]/rgb/set: rgb color, three 0-255 values, comma separated
 * - <base>/grp[1-4]/brightness/set: additional dimming factor for group: 0-255
 *
 * published topics:
 * - <base>/all/get: current status, 0 or 1
 * - <base>/ch[1-12]/brightness/get: channel brightness, values 0 to 255
 * - <base>/grp[1-4]/rgb/get: rgb color, three 0-255 values, comma separated
 * - <base>/grp[1-4]/brightness/get: additional dimming factor for group: 0-255
 */

#define MQTT_TOPIC_PREFIX               "led-controller/"
#define MQTT_TOPIC_ALL_SET              MQTT_TOPIC_PREFIX "all/set"
#define MQTT_TOPIC_ALL_GET              MQTT_TOPIC_PREFIX "all/get"
#define MQTT_TOPIC_CH_BR_SET(ch)        MQTT_TOPIC_PREFIX "ch" #ch "/brightness/set"
#define MQTT_TOPIC_CH_BR_GET(ch)        MQTT_TOPIC_PREFIX "ch" #ch "/brightness/get"
#define MQTT_TOPIC_GRP_RGB_SET(grp)     MQTT_TOPIC_PREFIX "grp" #grp "/rgb/set"
#define MQTT_TOPIC_GRP_RGB_GET(grp)     MQTT_TOPIC_PREFIX "grp" #grp "/rgb/get"

#define MQTT_TX_BUF_SIZE 256
#define MQTT_RX_BUF_SIZE 256
#define MQTT_RX_OVF_BUF_SIZE (MQTT_RX_BUF_SIZE)

#define MQTT_MAX_TOPIC_LEN 64

#define MQTT_PUBLISH_PAYLOAD_LEN 32
#define MQTT_PUBLISH_QUEUE_LEN 8

//#define MQTT_VERBOSE_DEBUG

typedef enum {
  MQTT_SOCK_STATE_INIT = 0,
  MQTT_SOCK_STATE_CONNECTING,
  MQTT_SOCK_STATE_CONNECT_ERR,
  MQTT_SOCK_STATE_CONNECTED
} mqtt_sock_state_t;

typedef enum {
  MQTT_WRITE_STATE_IDLE = 0,
  MQTT_WRITE_STATE_WRITING,
  MQTT_WRITE_STATE_WRITE_DONE,
  MQTT_WRITE_STATE_WRITE_ERR
} mqtt_write_state_t;

typedef enum {
  MQTT_READ_STATE_IDLE = 0,
  MQTT_READ_STATE_READING,
  MQTT_READ_STATE_READ_DONE,
  MQTT_READ_STATE_READ_TIMEOUT,
  MQTT_READ_STATE_READ_ERR
} mqtt_read_state_t;

typedef enum {
  MQTT_CONN_STATE_INIT = 0,
  MQTT_CONN_STATE_NETCONNECT,
  MQTT_CONN_STATE_CONNECT,
  MQTT_CONN_STATE_CONNECTED,
  MQTT_CONN_STATE_SUBSCRIBE,
  MQTT_CONN_STATE_WAIT,
  MQTT_CONN_STATE_WAIT_PING,
  MQTT_CONN_STATE_PUBLISH,
  MQTT_CONN_STATE_FAILED
} mqtt_conn_state_t;

struct mqtt_sock_context {
  SOCKET sock;
  struct sockaddr_in addr;

  mqtt_sock_state_t state;
  mqtt_read_state_t read_state;
  mqtt_write_state_t write_state;

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

struct mqtt_pub_msg {
  MqttPublish pub;
  uint8_t payload[MQTT_PUBLISH_PAYLOAD_LEN];
};

/**
 * ring buffer for queueing publish messages
 */
struct mqtt_pub_buffer {
  struct mqtt_pub_msg buffer[MQTT_PUBLISH_QUEUE_LEN];
  uint8_t start;
  uint8_t end;
  uint8_t filled;
};

struct mqtt_context {
  struct mqtt_sock_context sock_context;
  MqttClient client;
  MqttConnect connect;
  MqttNet net;
  MqttTopic topics[4];
  MqttSubscribe sub;

  uint8_t tx_buf[MQTT_TX_BUF_SIZE];
  uint8_t rx_buf[MQTT_RX_BUF_SIZE];

  uint16_t packet_id;
  uint64_t last_ping;
  mqtt_conn_state_t conn_state;

  struct mqtt_pub_buffer pub_queue;
};

void mqtt_socket_handler(SOCKET sock, uint8_t msg_type, void *msg);
void mqtt_init(struct mqtt_context *context);
void mqtt_connect(struct mqtt_context *context);
void mqtt_publish(struct mqtt_context *context, const char *topic, uint8_t *payload);
void mqtt_poll(struct mqtt_context *context);

#endif
