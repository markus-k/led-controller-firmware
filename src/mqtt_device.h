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

#ifndef __MQTT_DEVICE_H
#define __MQTT_DEVICE_H

#include <stdint.h>
#include "mqtt.h"

/*
 * mqtt topic structure overview:
 *
 * subscribed topics:
 * - <base>/all/set: turn all channels on/off, values 0 or 1
 * - <base>/ch/[1-12]/brightness/set: channel brightness, values 0 to 255
 * - <base>/ch/[1-12]/switch/set: channel switch
 * - <base>/grp/[1-4]/rgb/set: rgb color, three 0-255 values, comma separated
 * - <base>/grp/[1-4]/brightness/set: additional dimming factor for group: 0-255
 * - <base>/grp/[1-4]/switch/set: group switch
 *
 * published topics:
 * - <base>/all/get: current status, 0 or 1
 * - <base>/ch/[1-12]/brightness/get: channel brightness, values 0 to 255
 * - <base>/grp/[1-4]/rgb/get: rgb color, three 0-255 values, comma separated
 * - <base>/grp/[1-4]/brightness/get: additional dimming factor for group: 0-255
 */

#define MQTT_TOPIC_PREFIX               "led-controller/"

#define MQTT_TOPIC_ALL_SET              MQTT_TOPIC_PREFIX "all/set"
#define MQTT_TOPIC_ALL_GET              MQTT_TOPIC_PREFIX "all/get"

#define MQTT_TOPIC_CH_BR_SET_TMPL       MQTT_TOPIC_PREFIX "ch/%d/brightness/set"
#define MQTT_TOPIC_CH_BR_GET_TMPL       MQTT_TOPIC_PREFIX "ch/%d/brightness/get"
#define MQTT_TOPIC_CH_SW_SET_TMPL       MQTT_TOPIC_PREFIX "ch/%d/switch/set"
#define MQTT_TOPIC_CH_SW_GET_TMPL       MQTT_TOPIC_PREFIX "ch/%d/switch/get"

#define MQTT_TOPIC_GRP_RGB_SET_TMPL     MQTT_TOPIC_PREFIX "grp/%d/rgb/set"
#define MQTT_TOPIC_GRP_RGB_GET_TMPL     MQTT_TOPIC_PREFIX "grp/%d/rgb/get"
#define MQTT_TOPIC_GRP_BR_SET_TMPL      MQTT_TOPIC_PREFIX "grp/%d/brightness/set"
#define MQTT_TOPIC_GRP_BR_GET_TMPL      MQTT_TOPIC_PREFIX "grp/%d/brightness/get"
#define MQTT_TOPIC_GRP_SW_SET_TMPL      MQTT_TOPIC_PREFIX "grp/%d/switch/set"
#define MQTT_TOPIC_GRP_SW_GET_TMPL      MQTT_TOPIC_PREFIX "grp/%d/switch/get"

#define MQTT_TOPIC_WILDCARD             MQTT_TOPIC_PREFIX "#"

void mqtt_dev_parse_message(struct mqtt_context *context, const char *topic, const char *msg);

void mqtt_dev_all_set(struct mqtt_context *context, uint8_t val);
void mqtt_dev_ch_set_br(struct mqtt_context *context, int ch, uint8_t value);
void mqtt_dev_ch_set_sw(struct mqtt_context *context, int ch, uint8_t val);
void mqtt_dev_grp_set_rgb(struct mqtt_context *context, int grp, uint8_t r, uint8_t g, uint8_t b);
void mqtt_dev_grp_set_sw(struct mqtt_context *context, int grp, uint8_t val);
void mqtt_dev_grp_set_br(struct mqtt_context *context, int grp, uint8_t val);

#endif
