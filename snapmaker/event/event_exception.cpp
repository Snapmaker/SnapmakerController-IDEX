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

#include "event_exception.h"
#include "../module/exception.h"

#define J1_TYPE 2049

#pragma pack(1)

typedef struct {
  uint8_t level;
  uint16_t owner;
  uint8_t value;
} exception_info_t;

#pragma pack()

static void get_exception_info(exception_type_e e, exception_info_t *out) {
  out->owner = J1_TYPE;
  out->level = exception_server.get_level(e);
  out->value = e;
}

static uint8_t get_exception_list(exception_info_t *out) {
  uint32_t u32_bit_status = exception_server.get_exception();
  uint8_t index = 0;
  for (uint8_t i = 0; i < EXCEPTION_TYPE_MAX_COUNT; i++) {
    if (u32_bit_status & BIT(i)) {
      get_exception_info((exception_type_e)i, &out[index]);
      index++;
    }
  }
  return index;
}

static uint8_t get_exception_behavior_list(uint8_t *out) {
  uint8_t * behavior = out;
  uint8_t behavior_index = 0;
  uint32_t cur_behavior = exception_server.get_behavior();
  for (uint32_t i = 0; cur_behavior && i < 32; i++) {
    if (cur_behavior & BIT(i)) {
      behavior[behavior_index++] = i;
    }
  }
  return behavior_index;
}

static ErrCode exception_report_trigger_recv_ack(event_param_t& event) {
  SERIAL_ECHOLNPAIR("recv source:", event.source , " exception trigger ack");
  return E_SUCCESS;
}

static ErrCode exception_report_clean_info(event_param_t& event) {
  SERIAL_ECHOLNPAIR("recv source:", event.source , " exception clean ack");
  return E_SUCCESS;
}

static ErrCode exception_get_ack(event_param_t& event) {
  uint8_t index = 0;
  uint8_t * count = NULL;
  SERIAL_ECHOLNPAIR("source[", event.source, "] req exception info");
  event.data[index++] = E_SUCCESS;
  count = &event.data[index++];  // exception count info addr
  (*count) = get_exception_list((exception_info_t *)&event.data[index]);
  index += sizeof(exception_info_t) * (*count);

  count = &event.data[index++];    // behavior count info addr
  (*count) = get_exception_behavior_list(&event.data[index]);
  event.length = index + (*count);
  return send_event(event);
}

static ErrCode clean_exception_info(event_param_t& event) {
  uint8_t count = event.data[0];
  exception_info_t *exeption = (exception_info_t*)&event.data[1];
  for (uint8_t i = 0; i < count; i++) {
    SERIAL_ECHOLNPAIR("source[", event.source, "] req clean exception code:", exeption[i].value);
    exception_server.clean_exception((exception_type_e)exeption[i].value);
  }
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}


event_cb_info_t exception_cb_info[EXCEPTION_ID_CB_COUNT] = {
  {EXCEPTION_ID_TIRGGER_REPORT       , EVENT_CB_DIRECT_RUN, exception_report_trigger_recv_ack},
  {EXCEPTION_ID_CLEAN_REPORT , EVENT_CB_DIRECT_RUN, exception_report_clean_info},
  {EXCEPTION_ID_GET          , EVENT_CB_DIRECT_RUN, exception_get_ack},
  {EXCEPTION_ID_SC_CLEAN     , EVENT_CB_DIRECT_RUN, clean_exception_info},
};


static ErrCode exception_report_info(exception_type_e e) {
  uint8_t buf[40];
  exception_info_t * info = (exception_info_t *)buf;
  get_exception_info(e, info);
  uint8_t * behavior = (uint8_t *)(buf + sizeof(exception_info_t) + 1);
  uint8_t behavior_count = get_exception_behavior_list(behavior);
  buf[sizeof(exception_info_t)] = behavior_count;
  uint32_t lenght = sizeof(exception_info_t) + 1 + behavior_count;

  send_event(EVENT_SOURCE_HMI, SACP_ID_HMI, SACP_ATTR_REQ,
      COMMAND_SET_EXCEPTION, EXCEPTION_ID_TIRGGER_REPORT, buf, lenght);
  // The SACP protocol cannot be send without enabling it
  send_event(EVENT_SOURCE_MARLIN, SACP_ID_PC, SACP_ATTR_REQ,
    COMMAND_SET_EXCEPTION, EXCEPTION_ID_TIRGGER_REPORT, buf, lenght);
  return E_SUCCESS;
}

static ErrCode exception_report_clear_info(exception_type_e e) {
  uint8_t buf[40];
  uint8_t * exception_count = buf;
  *exception_count = 1;  // One clearance exception is reported at a time
  exception_info_t * info = (exception_info_t *)(exception_count+1);
  get_exception_info(e, info);

  uint8_t * behavior_count = (uint8_t *)((uint8_t *)info + sizeof(exception_info_t));
  uint8_t * behavior = behavior_count + 1;
  *behavior_count = get_exception_behavior_list(behavior);
  uint32_t lenght = sizeof(exception_info_t) + 2 + (*behavior_count);

  send_event(EVENT_SOURCE_HMI, SACP_ID_HMI, SACP_ATTR_REQ,
      COMMAND_SET_EXCEPTION, EXCEPTION_ID_CLEAN_REPORT, buf, lenght);
  // The SACP protocol cannot be send without enabling it
  send_event(EVENT_SOURCE_MARLIN, SACP_ID_PC, SACP_ATTR_REQ,
    COMMAND_SET_EXCEPTION, EXCEPTION_ID_CLEAN_REPORT, buf, lenght);
  return E_SUCCESS;
}

void exception_event_loop(void) {
  static uint32_t last_exception = 0;
  uint32_t cur_exception = exception_server.get_exception();

  // exception status changes are reported
  if (last_exception != cur_exception) {
    for (uint32_t i = 0; i < EXCEPTION_TYPE_MAX_COUNT; i++) {
      if ((last_exception & BIT(i)) != (cur_exception & BIT(i))) {
        if (cur_exception & BIT(i)) {
          SERIAL_ECHOLNPAIR("Report trigger exception code[", i, "]");
          exception_report_info((exception_type_e)i);
        } else {
          SERIAL_ECHOLNPAIR("Report clear exception code[", i, "]");
          exception_report_clear_info((exception_type_e)i);
        }
      }
    }
    last_exception = cur_exception;
  }

  // Report an exception that causes an SACP event error
  exception_type_e e = exception_server.get_wait_report_exception();
  if (e != EXCEPTION_TYPE_NONE) {
    exception_report_info(e);
  }
}