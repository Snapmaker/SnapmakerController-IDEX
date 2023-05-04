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

#ifndef EVENT_FDM_H
#define EVENT_FDM_H
#include "event_base.h"

enum : uint8_t {
  FDM_ID_GET_INFO                = 0x01,
  FDM_ID_SET_TEMPERATURE         = 0x02,
  FDM_ID_SET_WORK_SPEED          = 0x03,
  FDM_ID_ENABLE_FILAMENT_SENSOR  = 0x04,
  FDM_ID_CHANGE_TOOL_HEAD        = 0x05,
  FDM_ID_SET_FAN_SPEED           = 0x06,
  FDM_ID_SET_NOZZLE_SPACING      = 0x07,
  FDM_ID_GET_NOZZLE_SPACING      = 0x08,
  FDM_ID_EXTRUSION_CONTROL       = 0x09,
  FDM_ID_EXTRUSION_UNTIL         = 0x0A,
  FDM_ID_SUBSCRIBE_EXTRUDER_INFO   = 0xA0,
  FDM_ID_SUBSCRIBE_EXTRUSION_STATUS = 0xA1,
  FDM_ID_SUBSCRIBE_FAN_INFO         = 0xA3,
  FDM_ID_SUBSCRIBE_MODULE_INFO      = 0xA4,
};

#define FDM_ID_CB_COUNT 14

extern event_cb_info_t fdm_cb_info[FDM_ID_CB_COUNT];


#endif
