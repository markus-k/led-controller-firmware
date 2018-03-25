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
#include <stdlib.h>

#include "led.h"
#include "mqtt.h"
#include "mqtt_device.h"

/* ---------- message parsing ---------- */

static int parse_topic(const char *topic, const char *tmpl, uint32_t *val) {
  const char *offset = strchr(tmpl, '%');

  if (offset && val) {
    int before_len = offset - tmpl;
    if (offset[1] == 'd' &&
        memcmp(topic, tmpl, before_len) == 0) {
      char *end;
      *val = strtoul(topic + before_len, &end, 10);
      return strcmp(end, offset + 2) == 0;
    }
  }

  return strcmp(topic, tmpl) == 0;
}

static int ultostr(char *dst, unsigned int val) {
  const char *chars = "0123456789";
  int i = 0;
  int ret;
  char buf[8];

  if (val == 0) {
    buf[0] = chars[0];
    i++;
  } else {
    while (val) {
      uint32_t rem = val % 10;
      buf[i] = chars[rem];
      val = val / 10;
      i++;
    }
  }

  ret = i;

  while (i--) {
    *dst++ = buf[i];
  }

  // null-terminate
  *dst = 0;

  return ret;
}

static void print_topic(const char *tmpl, char *buf, int val) {
  const char *offset = strchr(tmpl, '%');

  if (offset) {
    int before_len = offset - tmpl;
    if (offset[1] == 'd') {
      int s;

      memcpy(buf, tmpl, before_len);
      buf += before_len;
      s = ultostr(buf, val);
      buf += s;
      strcpy(buf, offset + 2);
    }
  }
}

void mqtt_dev_parse_message(struct mqtt_context *context, const char *topic, const char *msg) {
  uint32_t placeholder;

  if (parse_topic(topic, MQTT_TOPIC_ALL_SET, NULL)) {
    uint8_t val = msg[0] != '0';
    mqtt_dev_all_set(context, val);
  } else if (parse_topic(topic, MQTT_TOPIC_CH_BR_SET_TMPL, &placeholder)) {
    uint8_t val = strtoul(msg, NULL, 10);
    mqtt_dev_ch_set_br(context, placeholder, val);
  } else if (parse_topic(topic, MQTT_TOPIC_GRP_RGB_SET_TMPL, &placeholder)) {
    uint8_t r, g, b;
    char *end = (char *)msg;
    r = strtoul(end, &end, 10); end++;
    g = strtoul(end, &end, 10); end++;
    b = strtoul(end, &end, 10);
    mqtt_dev_grp_set_rgb(context, placeholder, r, g, b);
  }
}

/* ---------- mqtt device functions ---------- */

void mqtt_dev_all_set(struct mqtt_context *context, uint8_t val) {
  char buf[4];

  led_set_all_ch_override(val);

  ultostr(buf, val);
  mqtt_publish(context, MQTT_TOPIC_ALL_GET, buf);
}

void mqtt_dev_ch_set_br(struct mqtt_context *context, int ch, uint8_t val) {
  char buf[4];
  char topic[MQTT_MAX_TOPIC_LEN];

  led_channels[ch - 1].value = val;

  ultostr(buf, val);
  print_topic(MQTT_TOPIC_CH_BR_GET_TMPL, topic, ch);
  mqtt_publish(context, topic, buf);
}

void mqtt_dev_grp_set_rgb(struct mqtt_context *context, int grp, uint8_t r, uint8_t g, uint8_t b) {
  char buf[12];
  char topic[MQTT_MAX_TOPIC_LEN];
  int s;
  int ch_offset = (grp - 1) * 3;

  mqtt_dev_ch_set_br(context, ch_offset + 1, r);
  mqtt_dev_ch_set_br(context, ch_offset + 2, g);
  mqtt_dev_ch_set_br(context, ch_offset + 3, b);

  s = ultostr(buf, r);
  buf[s++] = ',';
  s += ultostr(buf + s, g);
  buf[s++] = ',';
  ultostr(buf + s, b);
  print_topic(MQTT_TOPIC_GRP_RGB_GET_TMPL, topic, grp);
  mqtt_publish(context, topic, buf);
}
