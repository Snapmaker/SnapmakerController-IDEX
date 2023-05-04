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

#include "event_base.h"
#include "event_enclouser.h"
#include "../module/enclosure.h"
#include "../../../Marlin/src/module/settings.h"


#pragma pack(1)

typedef struct {
  uint8 result;
  uint8_t key;
  uint8 module_status;  // Fixed 2, normal
  uint8_t light_power;
  uint8_t test_status; // Fixed 0
  bool  is_door_open;  // J1 not support
  uint8 fan_power;  // J1 not support
} enclouser_info_t;

typedef struct {
  uint8 result;
  uint8_t key;
  uint8_t light_power;
} enclouser_subscribe_info_t;

#pragma pack()

static ErrCode enclouser_report_info(event_param_t& event) {
  enclouser_info_t * info = (enclouser_info_t *)event.data;
  memset(info, 0, sizeof(enclouser_info_t));
  info->result = E_SUCCESS;
  info->key = enclosure.get_key();
  info->module_status = 2;
  info->light_power = enclosure.get_light_power();
  event.length = sizeof(enclouser_info_t);
  return send_event(event);
}

static ErrCode enclouser_set_light(event_param_t& event) {
  uint8_t power = event.data[1];
  SERIAL_ECHOLNPAIR("SC set enclouser light power:", power);
  enclosure.set_light_power(power);
  extern bool ml_setting_need_save;
  ml_setting_need_save = true;
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode enclouser_set_fan(event_param_t& event) {
  uint8_t power = event.data[1];  // 0 - 100
  SERIAL_ECHOLNPAIR("SC set enclouser fan power:", power);
  enclosure.set_fan_power(power);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode enclouser_subscribe_info(event_param_t& event) {
  enclouser_subscribe_info_t * info = (enclouser_subscribe_info_t *)event.data;
  info->result = E_SUCCESS;
  info->key = enclosure.get_key();
  info->light_power = enclosure.get_light_power();
  event.length = sizeof(enclouser_subscribe_info_t);
  return send_event(event);
}

event_cb_info_t enclouser_cb_info[ENCLOUSER_ID_CB_COUNT] = {
  {ENCLOUSER_ID_REPORT_INFO   , EVENT_CB_DIRECT_RUN, enclouser_report_info},
  {ENCLOUSER_ID_SET_LIGHT     , EVENT_CB_DIRECT_RUN, enclouser_set_light},
  {ENCLOUSER_ID_SET_FAN       , EVENT_CB_DIRECT_RUN, enclouser_set_fan},
  {ENCLOUSER_ID_SUBSCRIBE_INFO, EVENT_CB_DIRECT_RUN, enclouser_subscribe_info},
};