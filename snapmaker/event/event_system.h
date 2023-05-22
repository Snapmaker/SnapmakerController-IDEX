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

#ifndef EVENT_SYSTEM_H
#define EVENT_SYSTEM_H
#include "event_base.h"

enum : uint8_t {
  SYS_ID_SUBSCRIBE                      = 0x00,
  SYS_ID_UNSUBSCRIBE                    = 0x01,
  SYS_ID_RUN_GCODE                      = 0x02,
  SYS_ID_SET_LOG_GRADE                  = 0x10,
  SYS_ID_PC_PORT_TO_GCODE               = 0x11,
  SYS_ID_SET_DEBUG_MODE                 = 0x12,
  SYS_ID_FACTORY_RESET                  = 0x13,
  SYS_ID_HEARTBEAT                      = 0xA0,
  SYS_ID_REPORT_LOG                     = 0xA1,
  SYS_ID_REQ_MODULE_INFO                = 0x20,
  SYS_ID_REQ_MACHINE_INFO               = 0x21,
  SYS_ID_REQ_MACHINE_SIZE               = 0x22,
  SYS_ID_SAVE_SETTING                   = 0x24,
  SYS_ID_REQ_COORDINATE_SYSTEM          = 0x30,
  SYS_ID_SET_COORDINATE_SYSTEM          = 0x31,
  SYS_ID_SET_ORIGIN                     = 0x32,
  SYS_ID_MOVE_RELATIVE                  = 0x33,
  SYS_ID_MOVE                           = 0x34,
  SYS_ID_HOME                           = 0x35,
  SYS_ID_HOME_END                       = 0x36,
  SYS_ID_GET_MOTOR_ENABLE               = 0x37,
  SYS_ID_SET_MOTOR_ENABLE               = 0x38,
  SYS_ID_MOVE_TO_RELATIVE_HOME          = 0x3C,
  SYS_ID_INPUTSHAPER_SET                = 0x3E,
  SYS_ID_INPUTSHAPER_GET                = 0x3F,
  SYS_ID_RESONANCE_COMPENSATION_SET     = 0x40,
  SYS_ID_RESONANCE_COMPENSATION_GET     = 0x41,
  SYS_ID_SET_Z_HOME_SG                  = 0x42,
  SYS_ID_GET_Z_HOME_SG                  = 0x43,
  SYS_ID_SET_BUILD_PLATE_TKNESS         = 0x44,
  SYS_ID_GET_BUILD_PLATE_TKNESS         = 0x45,
  SYS_ID_GET_DISTANCE_RELATIVE_HOME     = 0xA3,
  SYS_ID_SUBSCRIBE_MOTOR_ENABLE_STATUS  = 0xA4,
};

#define SYS_ID_CB_COUNT 33

extern event_cb_info_t system_cb_info[SYS_ID_CB_COUNT];


#endif
