/*
 * Snapmaker 3D Printer Firmware
 * Copyright (C) 2023 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of SnapmakerController-IDEX
 * (see https://github.com/Snapmaker/SnapmakerController-IDEX)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "subscribe.h"
#include "event_base.h"
#include "../lib/GD32F1/cores/maple/wirish_time.h"
#include "../J1/common_type.h"
#include "../protocol/protocol_sacp.h"
#include "../debug/debug.h"
#include "../module/system.h"
#include "../module/exception.h"
#include "../module/print_control.h"

Subscribe subscribe;

extern event_cb_info_t * get_event_info(uint8_t cmd_set, uint8_t cmd_id);

static event_param_t event_public_param;

ErrCode Subscribe::enable(event_param_t &event) {
  if (sub_count >= MAX_SUBSCRIBE_COUNT) {
    SERIAL_ECHOLNPAIR("SNMK_ERROR: subscribe count to max:", sub_count);
    return E_NO_MEM;
  }
  if (event.length < 4) {
    SERIAL_ECHOLNPAIR("SNMK_ERROR: subscribe param len faile:", event.length);
    return E_PARAM;
  }
  uint8_t *data = event.data;
  uint8_t cmd_set = data[0];
  uint8_t cmd_id = data[1];

  event_cb_info_t *tmp_cb = get_event_info(cmd_set, cmd_id);
  if (!tmp_cb) {
    SERIAL_ECHOLNPAIR("SNMK_ERROR:heve no cmd_set:", cmd_set, ", cmd_id:", cmd_id);
    return E_PARAM;
  }
  uint8_t index = 0;
  for (; index < sub_count; index++) {
    if (sub[index].info.command_set == cmd_set &&
        (sub[index].info.command_id == cmd_id) &&
        (sub[index].info.recever_id == event.info.recever_id) &&
        (sub[index].source == event.source)) {
      break;
    }
  }
  if (index == sub_count) {
    sub_count++;
  }
  sub[index].info = event.info;
  sub[index].info.command_set = cmd_set;
  sub[index].info.command_id = cmd_id;

  sub[index].cb = tmp_cb->cb;
  uint16_t tmp_time = data[3] << 8 | data[2];
  sub[index].time_interval = tmp_time;
  sub[index].last_time = millis();
  sub[index].write_byte = event.write_byte;
  sub[index].source = event.source;
  sub[index].is_available = true;

  return E_SUCCESS;
}

ErrCode Subscribe::disable(event_param_t &event) {
  if (event.length < 2) {
    SERIAL_ECHOLNPAIR("SNMK_ERROR: unsubscribe param len faile:", event.length);
    return E_PARAM;
  }
  uint8_t *data = event.data;
  uint8_t cmd_set = data[0];
  uint8_t cmd_id = data[1];
  uint8_t index = 0;
  SERIAL_ECHOPAIR("unsubscribe set:", cmd_set, ", id:", cmd_id);
  for (; index < sub_count; index++) {
    if (sub[index].info.command_set == cmd_set &&
        (sub[index].info.command_id == cmd_id) &&
        (sub[index].info.recever_id == event.info.recever_id) &&
        (sub[index].source == event.source)) {
      sub[index].is_available = false;
      SERIAL_ECHOLN(" success");
      return E_SUCCESS;
    }
  }
  SERIAL_ECHOLN(" failed");
  return E_PARAM;
}

void Subscribe::loop_task(void * arg) {
  while (true) {
    for (uint8_t i = 0; i < sub_count; i++) {
      if (sub[i].is_available) {
        // if (sub[i].last_time < millis()) {
        // LOG_I("SB_LOOP: index %i, cur %d, to %d\r\n", i, millis(), sub[i].last_time);
        if (ELAPSED(millis(), sub[i].last_time)) {
          sub[i].last_time = millis() + sub[i].time_interval;
          sub[i].info.sequence = protocol_sacp.sequence_pop();
          event_public_param.write_byte = sub[i].write_byte;
          event_public_param.info = sub[i].info;
          event_public_param.source = sub[i].source;
          event_public_param.length = 0;
          (sub[i].cb)(event_public_param);
        }
      }
    }
  }
}

static void subscribe_task(void * arg) {
  subscribe.loop_task(arg);
}

void subscribe_init(void) {

  TaskHandle_t thandle_subscribe = NULL;
  BaseType_t ret = xTaskCreate(subscribe_task, "subscribe_loop", 1024, NULL, 5, &thandle_subscribe);
  if (ret != pdPASS) {
    SERIAL_ECHO("Failed to create subscribe_loop!\n");
  }
  else {
    SERIAL_ECHO("Created subscribe_loop task!\n");
  }
}


