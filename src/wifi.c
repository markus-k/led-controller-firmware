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

#include <stdint.h>

#include <libopencm3/stm32/gpio.h>

#include <winc1500/bsp/include/nm_bsp.h>
#include <winc1500/driver/include/m2m_wifi.h>
#include <winc1500/socket/include/socket.h>
#include <winc1500/spi_flash/include/spi_flash.h>
//#include <MQTTClient.h>
#include <wolfmqtt/mqtt_client.h>

#include "config.h"

#include "board.h"
#include "debug.h"
#include "wifi.h"
#include "mqtt.h"
#include "states.h"

static void socket_event_handler(SOCKET sock, uint8_t msg_type, void *msg) {
  //DBG("socket: message %u", msg_type);

  mqtt_socket_handler(sock, msg_type, msg);
}

static void socket_resolve_handler(uint8_t *domain_name, uint32_t ip) {
}

static void wifi_cb(uint8 msg_type, void *msg) {
  /* Type of notifications. Possible types are:
				/ref M2M_WIFI_RESP_CON_STATE_CHANGED
				/ref M2M_WIFI_RESP_CONN_INFO
				/ref M2M_WIFI_REQ_DHCP_CONF
				/ref M2M_WIFI_REQ_WPS
				/ref M2M_WIFI_RESP_IP_CONFLICT
				/ref M2M_WIFI_RESP_SCAN_DONE
				/ref M2M_WIFI_RESP_SCAN_RESULT
				/ref M2M_WIFI_RESP_CURRENT_RSSI
				/ref M2M_WIFI_RESP_CLIENT_INFO
				/ref M2M_WIFI_RESP_PROVISION_INFO
				/ref M2M_WIFI_RESP_DEFAULT_CONNECT
  */
  tstrM2mWifiStateChanged *wifi_state;
  uint8_t *ip_addr;

  switch (msg_type) {
  case M2M_WIFI_RESP_CON_STATE_CHANGED:
    wifi_state = (tstrM2mWifiStateChanged *)msg;
    if (wifi_state->u8CurrState == M2M_WIFI_CONNECTED) {
      DBG("wifi: connected");
      // grab an IP from the dhcp server once connected
      m2m_wifi_request_dhcp_client();

      //signal_event(EVENT_WIFI_CONNECTED);
    } else if (wifi_state->u8CurrState == M2M_WIFI_DISCONNECTED) {
      DBG("wifi: disconnected");

      signal_event(EVENT_WIFI_DISCONNECTED);
    }
    break;
  case M2M_WIFI_REQ_DHCP_CONF:
    ip_addr = (uint8_t *)msg;
    DBG("wifi: got dhcp response, ip: %d.%d.%d.%d", ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
    signal_event(EVENT_WIFI_CONNECTED);

    //mqtt_init(0);
    break;
  case M2M_WIFI_RESP_SCAN_DONE:
    DBG("wifi: scan done");
    break;
  case M2M_WIFI_RESP_SCAN_RESULT:
    {
      tstrM2mWifiscanResult *result = (tstrM2mWifiscanResult *)msg;
      DBG("wifi: scan result: %s", result->au8SSID);
    }
    break;
  default:
    DBG("wifi: got unhandled msg_type 0x%x", msg_type);
    break;
  }
}

void wifi_init() {
  tstrWifiInitParam init_param;
  int8_t ret;

  m2m_memset((uint8 *)&init_param, 0, sizeof(tstrWifiInitParam));
  init_param.pfAppWifiCb = wifi_cb;

  DBG("wifi: init");

  nm_bsp_init();
  DBG("wifi: bsp init done");

  if (gpio_get(BOARD_BTN_PORT, BOARD_BTN_ONBOARD_PIN)) {
    // wifi bootloader mode
    gpio_set(GPIOE, GPIO15);
    nm_bsp_reset();

    DBG("wifi: now in bootloader mode");

    while (1);
  }

  ret = m2m_wifi_init(&init_param);
  if (ret != M2M_SUCCESS) {
    DBG("wifi: m2m_wifi_init() failed with ret %d", ret);
    return;
  } else {
    DBG("wifi: m2m_wifi_init() successful");
  }

  socketInit();
  registerSocketCallback(socket_event_handler, socket_resolve_handler);

  uint8_t mac[6];
  ret = m2m_wifi_get_mac_address(mac);
  if (ret != M2M_SUCCESS) {
    DBG("wifi: getting mac address failed: %d", ret);
    return;
  } else {
    DBG("wifi: mac address: %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  DBG("wifi: init done");
}

void wifi_connect() {
  int8_t ret;

  ret = m2m_wifi_connect(WIFI_SSID, sizeof(WIFI_SSID), M2M_WIFI_SEC_WPA_PSK, WIFI_PSK, M2M_WIFI_CH_ALL);
  if (ret != M2M_SUCCESS) {
    DBG("wifi: m2m_wifi_connect() failed with ret %d", ret);
    return;
  } else {
    DBG("wifi: m2m_wifi_connect() successful");
  }

  signal_event(EVENT_WIFI_CONNECTING);
}

void wifi_poll() {
  m2m_wifi_handle_events(0);
}
