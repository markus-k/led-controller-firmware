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

#include "mqtt.h"
#include "debug.h"
#include "states.h"

#include <wolfmqtt/mqtt_client.h>
#include <winc1500/socket/include/socket.h>

#define MQTT_TIMEOUT 1000

static const char *sock_state_strings[] = {
  "MQTT_SOCK_STATE_INIT",
  "MQTT_SOCK_STATE_CONNECTING",
  "MQTT_SOCK_STATE_CONNECT_ERR",
  "MQTT_SOCK_STATE_CONNECTED",
  "MQTT_SOCK_STATE_READING",
  "MQTT_SOCK_STATE_READ_DONE",
  "MQTT_SOCK_STATE_READ_ERR",
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

int mqtt_topic_cmp(MqttMessage *msg, const char *topic) {
  int len = msg->topic_name_len;
  const char *str = msg->topic_name;
  while (len--) {
    if (*str++ != *topic++) {
      return 0;
    }
  }
  return 1;
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

  DBG("mqtt_net: read (%s, %u bytes)", sock_state_strings[sock_ctx->state], len);

  switch (sock_ctx->state) {
  case MQTT_SOCK_STATE_CONNECTED:
    ret = recv_wrapper(sock_ctx, buf, len, timeout);
    if (ret > 0) {
      // done, no change to state
      rc = ret;
    } else if (ret < 0) {
      DBG("mqtt_net: read error");
      rc = MQTT_CODE_ERROR_NETWORK;
    } else {
      DBG("mqtt_net: waiting for read");
      sock_ctx->state = MQTT_SOCK_STATE_READING;
    }

    /*if (sock_ctx->rx_ovf_len > 0) {
      // there is data left in the overflow buffer
      uint8_t ovf_len;

      if (len > sock_ctx->rx_ovf_len - sock_ctx->rx_ovf_pos) {
	ovf_len = sock_ctx->rx_ovf_len - sock_ctx->rx_ovf_pos;
      } else {
	ovf_len = len;
      }

      DBG("mqtt_net: reading %u bytes from ovf buffer", ovf_len);

      memcpy(buf, sock_ctx->rx_ovf_buf + sock_ctx->rx_ovf_pos, ovf_len);
      sock_ctx->rx_ovf_pos += ovf_len;

      if (sock_ctx->rx_ovf_pos >= sock_ctx->rx_ovf_len) {
	// buffer is empty now
	sock_ctx->rx_ovf_pos = 0;
	sock_ctx->rx_ovf_len = 0;
      }

      // return the number of bytes read from the ovf buf
      rc = ovf_len;

      if (ovf_len == len) {
	// we are done
	sock_ctx->state = MQTT_SOCK_STATE_CONNECTED;
      }
    } else {
      DBG("mqtt_net: using recv");
      ret = recv(sock_ctx->sock, buf, len, timeout);
      if (ret != SOCK_ERR_NO_ERROR) {
	DBG("mqtt_net: read failed: %d", ret);
	rc = MQTT_CODE_ERROR_NETWORK;
      } else {
	rc = MQTT_CODE_CONTINUE;
      }
      }*/
    break;

  case MQTT_SOCK_STATE_READING:
    rc = MQTT_CODE_CONTINUE;
    break;

  case MQTT_SOCK_STATE_READ_DONE:


     /*
       if (sock_ctx->rx_ovf_len > 0) {
      // there is data left in the overflow buffer
      uint8_t ovf_len;

      if (len > sock_ctx->rx_ovf_len - sock_ctx->rx_ovf_pos) {
	ovf_len = sock_ctx->rx_ovf_len - sock_ctx->rx_ovf_pos;
      } else {
	ovf_len = len;
      }

      DBG("mqtt_net: copying %u bytes from ovf buffer", ovf_len);

      memcpy(buf, sock_ctx->rx_ovf_buf + sock_ctx->rx_ovf_pos, ovf_len);
      sock_ctx->rx_ovf_pos += ovf_len;

      if (sock_ctx->rx_ovf_pos >= sock_ctx->rx_ovf_len) {
	// buffer is empty now
	sock_ctx->rx_ovf_pos = 0;
	sock_ctx->rx_ovf_len = 0;
      }

      // return the number of bytes read from the ovf buf
      rc = ovf_len;

      if (ovf_len == len) {
	// we are done
	sock_ctx->state = MQTT_SOCK_STATE_CONNECTED;
      }
    }
    */

     ret = recv_wrapper(sock_ctx, buf, len, 0);
     if (ret > 0) {
       sock_ctx->state = MQTT_SOCK_STATE_CONNECTED;

       for (int i = 0; i < len; i++) {
	 DBG("recv data: 0x%x", buf[i]);
       }

       rc = ret;
     } else if (ret < 0) {
       DBG("mqtt_net: read error");
       rc = MQTT_CODE_ERROR_NETWORK;
     } else {
       DBG("mqtt_net: still waiting for read");
       sock_ctx->state = MQTT_SOCK_STATE_READING;
     }

     break;

  case MQTT_SOCK_STATE_READ_ERR:
    rc = MQTT_CODE_ERROR_NETWORK;
    break;

  default:
    DBG("mqtt_net: read in invalid socket state: %s", sock_state_strings[sock_ctx->state]);
    rc = 0;
    break;
  }

  return rc;
}

static int mqtt_net_write(void *context, const uint8_t *buf, int len, int timeout) {
  struct mqtt_sock_context *sock_ctx = (struct mqtt_sock_context *)context;
  int ret;
  int rc;

  DBG("mqtt_net: write (%s, %u bytes)", sock_state_strings[sock_ctx->state], len);

  switch (sock_ctx->state) {
  case MQTT_SOCK_STATE_CONNECTED:
    // timeout?
    sock_ctx->state = MQTT_SOCK_STATE_WRITING;
    ret = send(sock_ctx->sock, (uint8_t *)buf, len, 0);
    if (ret != SOCK_ERR_NO_ERROR) {
      DBG("mqtt_net: write failed: %d", ret);
      rc = MQTT_CODE_ERROR_NETWORK;
    } else {
      rc = MQTT_CODE_CONTINUE;
    }
    break;

  case MQTT_SOCK_STATE_WRITING:
    rc = MQTT_CODE_CONTINUE;
    break;

  case MQTT_SOCK_STATE_WRITE_DONE:
    sock_ctx->state = MQTT_SOCK_STATE_CONNECTED;
    rc = len;
    break;

  case MQTT_SOCK_STATE_WRITE_ERR:
    rc = MQTT_CODE_ERROR_NETWORK;
    break;

  default:
    DBG("mqtt_net: write in invalid socket state: %s", sock_state_strings[sock_ctx->state]);
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

static int mqtt_cb(MqttClient *client, MqttMessage *msg, uint8_t msg_new, uint8_t msg_done) {
  DBG("mqtt: cb called");
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
  if (recv_msg->s16BufferSize > 0) {
    int16_t buf_len = recv_msg->s16BufferSize;

    DBG("sock: recv %u, rem %u, buf: 0x%x", buf_len, recv_msg->u16RemainingSize, recv_msg->pu8Buffer);

    for (int i = 0; i < buf_len; i++) {
      DBG("data: 0x%x", recv_msg->pu8Buffer[i]);
    }

    //if (recv_msg->u16RemainingSize > 0) {

    DBG("sock: copying %u bytes to ovf_buf, len: %u", buf_len, sock_ctx->rx_ovf_len);

    if (buf_len + sock_ctx->rx_ovf_len > MQTT_RX_OVF_BUF_SIZE) {
      DBG("sock: ERROR: ovf buffer full, discarding data!");
      return;
    }

    // copy data into overflow buffer
    memcpy(sock_ctx->rx_ovf_buf + sock_ctx->rx_ovf_len, recv_msg->pu8Buffer, buf_len);
    sock_ctx->rx_ovf_len += buf_len;

    if (recv_msg->u16RemainingSize > 0) {
      sock_ctx->state = MQTT_SOCK_STATE_READING;
    } else {
      sock_ctx->state = MQTT_SOCK_STATE_READ_DONE;
    }
  } else if (recv_msg->s16BufferSize == 0) {
    DBG("sock: socket closed");
    // we've been disconnected ?
    sock_ctx->state = MQTT_SOCK_STATE_READ_ERR;
  } else {
    DBG("sock: recv err: %d", recv_msg->s16BufferSize);
    sock_ctx->state = MQTT_SOCK_STATE_READ_ERR;
  }
}

static void handle_socket_send(struct mqtt_sock_context *sock_ctx) {
  DBG("sock: send");

  sock_ctx->state = MQTT_SOCK_STATE_WRITE_DONE;
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
}

void mqtt_connect(struct mqtt_context *context) {
  int ret;

  switch (context->conn_state) {
  case MQTT_CONN_STATE_INIT:
    memset(&context->connect, 0, sizeof(MqttConnect));

    context->connect.client_id = "led-controller";
    context->connect.keep_alive_sec = 10;

    context->conn_state = MQTT_CONN_STATE_NETCONNECT;

    /* fallthrough */
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
    signal_event(EVENT_MQTT_CONNECTED);
    break;
  case MQTT_CONN_STATE_FAILED:
    signal_event(EVENT_MQTT_DISCONNECTED);
    break;
  }

  /*
  topic.topic_filter = "led/status";
  sub.topics = &topic;
  sub.topic_count = 1;

  ret = MqttClient_Subscribe(&context->client, &sub);
  if (ret != MQTT_CODE_SUCCESS) {
    const char *code_str = MqttClient_ReturnCodeToString(ret);
    DBG("mqtt: subscribe failed: %s (%d)", code_str, ret);
  } else {
    DBG("mqtt: subscribe successful");
    }*/
}

void mqtt_publish(struct mqtt_context *context, const char *topic, const char *msg) {
  int ret;
  MqttPublish pub;

  memset(&pub, 0, sizeof(MqttPublish));

  pub.buffer = msg;
  pub.total_len = strlen(msg);
  pub.topic_name = topic;
  pub.packet_id = ++context->packet_id;
  pub.duplicate = 0;
  pub.retain = 0;

  ret = MqttClient_Publish(&context->client, &pub);
  if (ret != MQTT_CODE_SUCCESS) {
    const char *code_str = MqttClient_ReturnCodeToString(ret);
    DBG("mqtt: publish failed: %s (%d)", code_str, ret);
  } else {
    DBG("mqtt: publish successful");
  }
}

void mqtt_poll(struct mqtt_context *context) {
  static uint64_t last_ping = 0;
  int ret;

  if (clock_ticks - last_ping > 5000) {
    if (context->conn_state == MQTT_CONN_STATE_CONNECTED) {
      DBG("mqtt: ping");
      ret = MqttClient_Ping(&context->client);
      if (ret != MQTT_CODE_CONTINUE) {
	last_ping = clock_ticks;
      }
    }

    //mqtt_publish(context, "led-controller/status", "alive");
  }
}
