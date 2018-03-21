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

#include <string.h>
#include <stdlib.h>

#include "mqtt.h"
#include "debug.h"
#include "states.h"
#include "led.h"

#include <wolfmqtt/mqtt_client.h>
#include <winc1500/socket/include/socket.h>

#define MQTT_TIMEOUT 1000

static const char *sock_state_strings[] = {
  "MQTT_SOCK_STATE_INIT",
  "MQTT_SOCK_STATE_CONNECTING",
  "MQTT_SOCK_STATE_CONNECT_ERR",
  "MQTT_SOCK_STATE_CONNECTED"
};

static const char *sock_read_state_strings[] = {
  "MQTT_READ_STATE_IDLE",
  "MQTT_READ_STATE_READING",
  "MQTT_READ_STATE_READ_DONE",
  "MQTT_READ_STATE_READ_TIMEOUT",
  "MQTT_READ_STATE_READ_ERR"
};

static const char *sock_write_state_strings[] = {
  "MQTT_WRITE_STATE_IDLE",
  "MQTT_SOCK_STATE_WRITING",
  "MQTT_SOCK_STATE_WRITE_DONE",
  "MQTT_SOCK_STATE_WRITE_ERR"
};

struct socket_context_map {
  SOCKET sock;
  struct mqtt_sock_context *context;
};

static struct socket_context_map context_map[1] = {
  { 0, 0 }
};

static void get_topic_name(char *topic) {
  uint8_t len = 0;

  const char *topic_base = "led-controller";

  strcpy(topic, topic_base);
  len += strlen(topic_base);

  // null-terminate topic
  topic[len] = 0;
}

/* ---------- mqtt net callbacks ---------- */

static int mqtt_net_connect(void *context, const char *host, uint16_t port, int timeout) {
  struct mqtt_sock_context *sock_ctx = (struct mqtt_sock_context *)context;
  int ret;
  int rc;

  DBG("mqtt_net: connect (%s)", sock_state_strings[sock_ctx->state]);

  switch (sock_ctx->state) {
  case MQTT_SOCK_STATE_INIT:
    {
      uint32_t ip = nmi_inet_addr((char *)host);
      sock_ctx->addr.sin_addr.s_addr = ip;
      sock_ctx->addr.sin_family = AF_INET;
      sock_ctx->addr.sin_port = _htons(port);
      sock_ctx->rx_ovf_pos = 0;
      sock_ctx->rx_ovf_len = 0;

      sock_ctx->sock = socket(AF_INET, SOCK_STREAM, 0);
      if (sock_ctx->sock < 0) {
	DBG("mqtt_net: socket creation failed");
	return MQTT_CODE_ERROR_NETWORK;
      } else {
	DBG("mqtt_net: socket created: %u", sock_ctx->sock);
      }

      context_map[0].sock = sock_ctx->sock;
      context_map[0].context = sock_ctx;

      sock_ctx->state = MQTT_SOCK_STATE_CONNECTING;
      ret = connect(sock_ctx->sock, (struct sockaddr *)&sock_ctx->addr, sizeof(struct sockaddr_in));
      if (ret < 0) {
	DBG("mqtt_net: connect failed");
	sock_ctx->state = MQTT_SOCK_STATE_CONNECT_ERR;
	close(sock_ctx->sock);
	rc = MQTT_CODE_ERROR_NETWORK;
      } else {
	rc = MQTT_CODE_CONTINUE;
      }
    }
    break;

  case MQTT_SOCK_STATE_CONNECTING:
    rc = MQTT_CODE_CONTINUE;
    break;

  case MQTT_SOCK_STATE_CONNECTED:
    rc = MQTT_CODE_SUCCESS;
    break;

  case MQTT_SOCK_STATE_CONNECT_ERR:
    rc = MQTT_CODE_ERROR_NETWORK;
    break;
  }

  return rc;
}

/**
 * wrapper around recv to handle buffering
 *
 * returns 0 when data was requested from the network, a positive number for
 * instant responses, negative numbers for errors
 */
static int16_t recv_wrapper(struct mqtt_sock_context *sock_ctx, uint8_t *buf, uint16_t len, uint32_t timeout) {
  uint16_t ovf_len = sock_ctx->rx_ovf_len - sock_ctx->rx_ovf_pos;
  int ret;
  int rc;

  if (ovf_len > 0) {
    if (ovf_len >= len) {
      // all requested data is in buffer
      memcpy(buf, sock_ctx->rx_ovf_buf + sock_ctx->rx_ovf_pos, len);
      sock_ctx->rx_ovf_pos += len;

      if (sock_ctx->rx_ovf_pos == sock_ctx->rx_ovf_len) {
	// buffer is empty now
	sock_ctx->rx_ovf_pos = 0;
	sock_ctx->rx_ovf_len = 0;
      }

      rc = len;
    } else {
      // partial data is available

      // ??
      DBG("recv_wrapper: partial recv TODO!");
      rc = -1;
    }
  } else {
    ret = recv(sock_ctx->sock, sock_ctx->rx_tmp_buf, len, timeout);
    if (ret != SOCK_ERR_NO_ERROR) {
      // ...
      return -1;
    }

    rc = 0;
  }

  return rc;
}

static int mqtt_net_read(void *context, uint8_t *buf, int len, int timeout) {
  struct mqtt_sock_context *sock_ctx = (struct mqtt_sock_context *)context;
  int ret;
  int rc = 0;

#ifdef MQTT_VERBOSE_DEBUG
  DBG("mqtt_net: read (%s, %u bytes)", sock_read_state_strings[sock_ctx->read_state], len);
#endif

  switch (sock_ctx->read_state) {
  case MQTT_READ_STATE_IDLE:
    ret = recv_wrapper(sock_ctx, buf, len, timeout);
    if (ret > 0) {
      // done, no change to state
      rc = ret;
    } else if (ret < 0) {
      DBG("mqtt_net: read error");
      rc = MQTT_CODE_ERROR_NETWORK;
    } else {
      DBG("mqtt_net: waiting for read");
      sock_ctx->read_state = MQTT_READ_STATE_READING;
    }
    break;

  case MQTT_READ_STATE_READING:
    rc = MQTT_CODE_CONTINUE;
    break;

  case MQTT_READ_STATE_READ_DONE:
     ret = recv_wrapper(sock_ctx, buf, len, 0);
     if (ret > 0) {
       sock_ctx->read_state = MQTT_READ_STATE_IDLE;

#ifdef MQTT_VERBOSE_DEBUG
       for (int i = 0; i < len; i++) {
	 DBG("recv data: 0x%x", buf[i]);
       }
#endif

       rc = ret;
     } else if (ret < 0) {
       DBG("mqtt_net: read error");
       rc = MQTT_CODE_ERROR_NETWORK;
     } else {
       DBG("mqtt_net: still waiting for read");
       sock_ctx->read_state = MQTT_READ_STATE_READING;
     }

     break;

  case MQTT_READ_STATE_READ_TIMEOUT:
    rc = MQTT_CODE_ERROR_TIMEOUT;
    sock_ctx->read_state = MQTT_READ_STATE_IDLE;
    break;

  case MQTT_READ_STATE_READ_ERR:
    rc = MQTT_CODE_ERROR_NETWORK;
    break;

  default:
    DBG("mqtt_net: read in invalid socket state: %s", sock_read_state_strings[sock_ctx->read_state]);
    rc = 0;
    break;
  }

  return rc;
}

static int mqtt_net_write(void *context, const uint8_t *buf, int len, int timeout) {
  struct mqtt_sock_context *sock_ctx = (struct mqtt_sock_context *)context;
  int ret;
  int rc;

#ifdef MQTT_VERBOSE_DEBUG
  DBG("mqtt_net: write (%s, %u bytes)", sock_write_state_strings[sock_ctx->write_state], len);
#endif

  switch (sock_ctx->write_state) {
  case MQTT_WRITE_STATE_IDLE:
    // timeout?
    sock_ctx->write_state = MQTT_WRITE_STATE_WRITING;
    ret = send(sock_ctx->sock, (uint8_t *)buf, len, 0);
    if (ret != SOCK_ERR_NO_ERROR) {
      DBG("mqtt_net: write failed: %d", ret);
      rc = MQTT_CODE_ERROR_NETWORK;
    } else {
      rc = MQTT_CODE_CONTINUE;
    }
    break;

  case MQTT_WRITE_STATE_WRITING:
    rc = MQTT_CODE_CONTINUE;
    break;

  case MQTT_WRITE_STATE_WRITE_DONE:
    sock_ctx->write_state = MQTT_WRITE_STATE_IDLE;
    rc = len;
    break;

  case MQTT_WRITE_STATE_WRITE_ERR:
    rc = MQTT_CODE_ERROR_NETWORK;
    break;

  default:
    DBG("mqtt_net: write in invalid socket state: %s", sock_write_state_strings[sock_ctx->write_state]);
    rc = 0;
    break;
  }

  return rc;
}

static int mqtt_net_disconnect(void *context) {
  struct mqtt_sock_context *sock_ctx = (struct mqtt_sock_context *)context;

  DBG("mqtt_net: disconnect");

  close(sock_ctx->sock);

  return MQTT_CODE_SUCCESS;
}

/* ---------- socket handling ---------- */

static void handle_socket_connect(struct mqtt_sock_context *sock_ctx, tstrSocketConnectMsg *conn_msg) {
  if (conn_msg->s8Error) {
    DBG("sock: connect err: %d", conn_msg->s8Error);
    sock_ctx->state = MQTT_SOCK_STATE_CONNECT_ERR;
  } else {
    DBG("sock: connected");
    sock_ctx->state = MQTT_SOCK_STATE_CONNECTED;
  }
}

static void handle_socket_recv(struct mqtt_sock_context *sock_ctx, tstrSocketRecvMsg *recv_msg) {
  int16_t buf_len = recv_msg->s16BufferSize;
  if (recv_msg->s16BufferSize > 0) {
#ifdef MQTT_VERBOSE_DEBUG
    DBG("sock: recv %u, rem %u, buf: 0x%x", buf_len, recv_msg->u16RemainingSize, recv_msg->pu8Buffer);

    for (int i = 0; i < buf_len; i++) {
      DBG("data: 0x%x", recv_msg->pu8Buffer[i]);
    }

    DBG("sock: copying %u bytes to ovf_buf, len: %u", buf_len, sock_ctx->rx_ovf_len);
#endif

    if (buf_len + sock_ctx->rx_ovf_len > MQTT_RX_OVF_BUF_SIZE) {
      DBG("sock: ERROR: ovf buffer full, discarding data!");
      return;
    }

    // copy data into overflow buffer
    memcpy(sock_ctx->rx_ovf_buf + sock_ctx->rx_ovf_len, recv_msg->pu8Buffer, buf_len);
    sock_ctx->rx_ovf_len += buf_len;

    if (recv_msg->u16RemainingSize > 0) {
      sock_ctx->read_state = MQTT_READ_STATE_READING;
    } else {
      sock_ctx->read_state = MQTT_READ_STATE_READ_DONE;
    }
  } else if (buf_len == 0) {
    DBG("sock: socket closed");
    // we've been disconnected ? TODO
    sock_ctx->read_state = MQTT_READ_STATE_READ_ERR;
  } else {
    if (buf_len == SOCK_ERR_TIMEOUT) {
      DBG("sock: recv timeout");
      sock_ctx->read_state = MQTT_READ_STATE_READ_TIMEOUT;

      // else if (buf_len == SOCK_ERR_CONN_ABORTED) {}
    } else {
      DBG("sock: recv err: %d", buf_len);
      sock_ctx->read_state = MQTT_READ_STATE_READ_ERR;
    }
  }
}

static void handle_socket_send(struct mqtt_sock_context *sock_ctx) {
  DBG("sock: send");

  sock_ctx->write_state = MQTT_WRITE_STATE_WRITE_DONE;
}

void mqtt_socket_handler(SOCKET sock, uint8_t msg_type, void *msg) {
  struct mqtt_sock_context *context = NULL;

  for (int i = 0; i < sizeof(context_map) / sizeof(context_map[0]); i++) {
    if (context_map[i].sock == sock) {
      context = context_map[i].context;
      break;
    }
  }

  if (context == NULL) {
    // not ours I guess
    return;
  }

  switch (msg_type) {
  case SOCKET_MSG_CONNECT:
    handle_socket_connect(context, (tstrSocketConnectMsg *)msg);
    break;
  case SOCKET_MSG_RECV:
    handle_socket_recv(context, (tstrSocketRecvMsg *)msg);
    break;
  case SOCKET_MSG_SEND:
    handle_socket_send(context);
    break;
  default:
    DBG("mqtt_net: unhandled socket message type %u", msg_type);
    break;
  }
}

/* ---------- mqtt functions ---------- */

static int compare_topic(MqttMessage *msg, const char *topic) {
  return memcmp(msg->topic_name, topic, msg->topic_name_len) == 0;
}

static void msg2str(MqttMessage *msg, char *str) {
  memcpy(str, msg->buffer, msg->buffer_len);
  str[msg->buffer_len] = 0; // null-terminate string
}

static int mqtt_cb(MqttClient *client, MqttMessage *msg, uint8_t msg_new, uint8_t msg_done) {
  struct mqtt_context *context = (struct mqtt_context *)client->ctx;
  char topic_name[64];
  char msg_str[16];

  // copy topic name and null-terminate it
  memcpy(topic_name, msg->topic_name, msg->topic_name_len);
  topic_name[msg->topic_name_len] = 0;

  DBG("mqtt: cb: topic: %s, new: %u, done: %u", topic_name, msg_new, msg_done);

  if (msg_new) {
    for (int i = 0; i < msg->buffer_len; i++) {
      DBG("mqtt: msg 0x%x", msg->buffer[i]);
    }

    if (compare_topic(msg, MQTT_TOPIC_ALL_SET)) {
      int val = msg->buffer[0] == '0' ? 0 : 1;
      DBG("mqtt: got set all to %d", val);
      led_set_all_ch_override(val);

      mqtt_publish(context, MQTT_TOPIC_ALL_GET, val == 0 ? "0" : "1");
    } else if (compare_topic(msg, MQTT_TOPIC_CH_BR_SET(1))) {
      uint8_t val;
      msg2str(msg, msg_str);
      val = strtoul(msg_str, NULL, 10);
      led_channels[0].value = val;
    } else if (compare_topic(msg, MQTT_TOPIC_CH_BR_SET(2))) {
      uint8_t val;
      msg2str(msg, msg_str);
      val = strtoul(msg_str, NULL, 10);
      led_channels[1].value = val;
    } else if (compare_topic(msg, MQTT_TOPIC_GRP_RGB_SET(1))) {
      uint8_t r, g, b;
      char *end = msg_str;
      msg2str(msg, msg_str);
      r = strtoul(end, &end, 10); end++;
      g = strtoul(end, &end, 10); end++;
      b = strtoul(end, &end, 10);
      led_channels[0].value = r;
      led_channels[1].value = g;
      led_channels[2].value = b;
    }
  }

  return MQTT_CODE_SUCCESS;
}

void mqtt_init(struct mqtt_context *context) {
  int ret;

  context->net.connect = mqtt_net_connect;
  context->net.disconnect = mqtt_net_disconnect;
  context->net.read = mqtt_net_read;
  context->net.write = mqtt_net_write;
  context->net.context = &context->sock_context;
  context->conn_state = MQTT_CONN_STATE_INIT;

  ret = MqttClient_Init(&context->client,
			&context->net,
			mqtt_cb,
			context->tx_buf, MQTT_TX_BUF_SIZE,
			context->rx_buf, MQTT_RX_BUF_SIZE,
			MQTT_TIMEOUT);
  if (ret != MQTT_CODE_SUCCESS) {
    const char *code_str = MqttClient_ReturnCodeToString(ret);
    DBG("mqtt: init failed: %s (%d)", code_str, ret);
  }

  // set the client context after initialization
  context->client.ctx = context;
}

void mqtt_connect(struct mqtt_context *context) {
  int ret;

  context->conn_state = MQTT_CONN_STATE_NETCONNECT;
}

/* ---------- mqtt publish buffering ---------- */

static int queue_check_full(struct mqtt_pub_buffer *queue) {
  return queue->start == queue->end && queue->filled;
}

static int queue_check_empty(struct mqtt_pub_buffer *queue) {
  return queue->start == queue->end && !queue->filled;
}

static int enqueue_message(struct mqtt_pub_buffer *queue, struct mqtt_pub_msg **pub) {
  int pos;

  if (queue_check_full(queue)) {
    return -1;
  }

  pos = queue->end++;

  if (queue->end >= MQTT_PUBLISH_QUEUE_LEN) {
    queue->end = 0;
  }

  queue->filled = 1;

  *pub = &queue->buffer[pos];

  return 0;
}

static struct mqtt_pub_msg *dequeue_message(struct mqtt_pub_buffer *queue) {
  struct mqtt_pub_msg *pub;

  if (queue_check_empty(queue)) {
    return NULL;
  }

  pub = &queue->buffer[queue->start++];

  if (queue->start >= MQTT_PUBLISH_QUEUE_LEN) {
    queue->start = 0;
  }

  if (queue->start == queue->end) {
    queue->filled = 0;
  }

  return pub;
}

static struct mqtt_pub_msg *queue_peek_message(struct mqtt_pub_buffer *queue) {
  if (queue_check_empty(queue)) {
    return NULL;
  }

  return &queue->buffer[queue->start];
}

static void mqtt_publish_poll(struct mqtt_context *context) {
  struct mqtt_pub_msg *msg;
  int ret;

  msg = queue_peek_message(&context->pub_queue);

  if (!msg) {
    // this shouldn't happen though
    DBG("mqtt: publish message gone?");
    return;
  }

  ret = MqttClient_Publish(&context->client, &msg->pub);
  if (ret == MQTT_CODE_SUCCESS) {
    DBG("mqtt: publish successful");

    // remove message from queue
    dequeue_message(&context->pub_queue);

    context->conn_state = MQTT_CONN_STATE_WAIT;
  } else if (ret != MQTT_CODE_CONTINUE) {
    const char *code_str = MqttClient_ReturnCodeToString(ret);
    DBG("mqtt: netconnect failed: %s (%d)", code_str, ret);

    context->conn_state = MQTT_CONN_STATE_WAIT;
  }
}

void mqtt_publish(struct mqtt_context *context, const char *topic, uint8_t *payload) {
  int ret;
  struct mqtt_pub_msg *msg;
  size_t len = strlen(payload);

  if (enqueue_message(&context->pub_queue, &msg)) {
    DBG("mqtt: publish queue full!");
    return;
  }

  memset(msg, 0, sizeof(MqttPublish));

  if (len > MQTT_PUBLISH_PAYLOAD_LEN) {
    memcpy(msg->payload, payload, MQTT_PUBLISH_PAYLOAD_LEN);
  } else {
    strcpy(msg->payload, payload);
  }

  msg->pub.buffer = msg->payload;
  msg->pub.total_len = len;
  msg->pub.topic_name = topic;
  msg->pub.topic_name_len = strlen(topic);
  msg->pub.packet_id = ++context->packet_id;
  msg->pub.duplicate = 0;
  msg->pub.retain = 0;
}

void mqtt_poll(struct mqtt_context *context) {
  static uint64_t last_ping = 0;
  int ret;

  switch (context->conn_state) {
  case MQTT_CONN_STATE_INIT:
    memset(&context->connect, 0, sizeof(MqttConnect));
    memset(&context->pub_queue, 0, sizeof(struct mqtt_pub_buffer));

    context->connect.client_id = "led-controller";
    context->connect.keep_alive_sec = 10;

    break;

  case MQTT_CONN_STATE_NETCONNECT:
    ret = MqttClient_NetConnect(&context->client,
				"192.168.2.8",
				1883,
				MQTT_TIMEOUT,
				0, // no tils
				0);
    if(ret == MQTT_CODE_SUCCESS) {
      DBG("mqtt: netconnect successful");

      context->conn_state = MQTT_CONN_STATE_CONNECT;
    } else if (ret == MQTT_CODE_CONTINUE) {
      /* nop */
    } else {
      const char *code_str = MqttClient_ReturnCodeToString(ret);
      DBG("mqtt: netconnect failed: %s (%d)", code_str, ret);

      context->conn_state = MQTT_CONN_STATE_FAILED;
      return;
    }
    break;

  case MQTT_CONN_STATE_CONNECT:
    ret = MqttClient_Connect(&context->client, &context->connect);
    if (ret == MQTT_CODE_SUCCESS) {
      DBG("mqtt: connect successful");

      context->conn_state = MQTT_CONN_STATE_CONNECTED;
    } else if (ret == MQTT_CODE_CONTINUE) {
      /* nop */
    } else {
      const char *code_str = MqttClient_ReturnCodeToString(ret);
      DBG("mqtt: connect failed: %s (%d)", code_str, ret);

      context->conn_state = MQTT_CONN_STATE_FAILED;
    }
    break;

  case MQTT_CONN_STATE_CONNECTED:
    memset(context->topics, 0, sizeof(context->topics));
    context->topics[0].topic_filter = MQTT_TOPIC_ALL_SET;
    context->topics[1].topic_filter = MQTT_TOPIC_CH_BR_SET(1);
    context->topics[2].topic_filter = MQTT_TOPIC_CH_BR_SET(2);
    context->topics[3].topic_filter = MQTT_TOPIC_GRP_RGB_SET(1);

    memset(&context->sub, 0, sizeof(MqttSubscribe));
    context->sub.topics = context->topics;
    context->sub.topic_count = sizeof(context->topics) / sizeof(context->topics[0]);
    context->sub.packet_id = ++context->packet_id;

    context->conn_state = MQTT_CONN_STATE_SUBSCRIBE;

    // fallthrough
    //break;

  case MQTT_CONN_STATE_SUBSCRIBE:
    ret = MqttClient_Subscribe(&context->client, &context->sub);
    if (ret == MQTT_CODE_SUCCESS) {
      context->conn_state = MQTT_CONN_STATE_WAIT;
      signal_event(EVENT_MQTT_CONNECTED);
    } else if (ret == MQTT_CODE_CONTINUE) {
      /* nop */
    } else {
      const char *code_str = MqttClient_ReturnCodeToString(ret);
      DBG("mqtt: subscribe failed: %s (%d)", code_str, ret);
    }
    break;

  case MQTT_CONN_STATE_WAIT:
    ret = MqttClient_WaitMessage(&context->client, 5000);

    if (ret == MQTT_CODE_ERROR_TIMEOUT ||
	(ret == MQTT_CODE_SUCCESS && clock_ticks - context->last_ping > 5000)) {
      // send ping on timeout or if haven't send a ping in 5 seconds
      context->conn_state = MQTT_CONN_STATE_WAIT_PING;
    } else if (!(ret == MQTT_CODE_CONTINUE || ret == MQTT_CODE_SUCCESS)) {
      const char *code_str = MqttClient_ReturnCodeToString(ret);
      DBG("mqtt: waitmessage failed: %s (%d)", code_str, ret);
    }

    if (!queue_check_empty(&context->pub_queue)) {
      context->conn_state = MQTT_CONN_STATE_PUBLISH;
    }
    break;

  case MQTT_CONN_STATE_WAIT_PING:
    ret = MqttClient_Ping(&context->client);
    if (ret == MQTT_CODE_SUCCESS) {
      DBG("mqtt: ping successful");
      context->last_ping = clock_ticks;
      context->conn_state = MQTT_CONN_STATE_WAIT;
    } else if (ret != MQTT_CODE_CONTINUE) {
      const char *code_str = MqttClient_ReturnCodeToString(ret);
      DBG("mqtt: ping failed: %s (%d)", code_str, ret);
    }
    break;

  case MQTT_CONN_STATE_PUBLISH:
    mqtt_publish_poll(context);
    break;

  case MQTT_CONN_STATE_FAILED:
    signal_event(EVENT_MQTT_DISCONNECTED);
    break;

  default:
    DBG("mqtt:unknown mqtt connection state: %d", context->conn_state);
    break;
  }
}
