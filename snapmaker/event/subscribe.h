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

#ifndef SUBSCRIBE_H
#define SUBSCRIBE_H

#include "../J1/common_type.h"
#include "../protocol/protocol_sacp.h"
#include "event_base.h"

#define MAX_SUBSCRIBE_COUNT 30

typedef struct {
  bool is_available;
  event_source_e source;
  uint16_t time_interval;
  uint32_t last_time;
  SACP_head_base_t info;
  write_byte_f write_byte;
  evevnt_cb_f cb;
} subscribe_node_t;

class Subscribe {
  public:
    ErrCode enable(event_param_t &event);
    ErrCode disable(event_param_t &event);
    void loop_task(void *arg);
  private:
    subscribe_node_t sub[MAX_SUBSCRIBE_COUNT];
    uint8_t sub_count;
};
void subscribe_init(void);
extern Subscribe subscribe;
#endif
