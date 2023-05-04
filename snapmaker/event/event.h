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

#ifndef EVEVT_H
#define EVEVT_H
#include "event_base.h"
#include "../J1/common_type.h"
#include "../protocol/protocol_sacp.h"

#define EVENT_CACHE_COUNT 6

typedef enum {
  EVENT_CACHT_STATUS_IDLE,
  EVENT_CACHT_STATUS_WAIT,
  EVENT_CACHT_STATUS_BUSY,
} event_cache_node_status_e;


typedef struct {
  event_cache_node_status_e block_status;  // idle, wait, busy
  event_param_t param;  // Parameters to be passed into the callback function
  evevnt_cb_f cb;  // event callback
} event_cache_node_t;

typedef struct {
  bool enable;
  SACP_param_t sacp_params;
  event_source_e recv_source;  // Event source
} recv_data_info_t;

class EventHandler {
  public:
    EventHandler() {
      for (uint8_t i = 0; i < EVENT_CACHE_COUNT; i++) {
        event_cache[i].block_status = EVENT_CACHT_STATUS_IDLE;
      }
    }

    void loop_task();
    void recv_task();
    void recv_enable(event_source_e source, bool enable);
    void recv_enable(event_source_e source);

  private:
    ErrCode parse(recv_data_info_t *recv_info);
    void parse_event_info(recv_data_info_t *recv_info, event_cache_node_t *event);
    event_cache_node_t * get_event_cache();

  private:
    event_cache_node_t err_result_event;
    event_cache_node_t event_cache[EVENT_CACHE_COUNT];
    recv_data_info_t recv_data_info[EVENT_SOURCE_ALL] = {0};
};

typedef enum {
  LE_NONE,
  LE_XY_CALI,
} local_event_t;

void event_task(void * arg);
void event_init();
void event_port_init();
void local_event_loop();
void gen_local_event(local_event_t event);

extern EventHandler event_handler;
#endif
