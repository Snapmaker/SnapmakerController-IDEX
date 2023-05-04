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

#ifndef MODULE_BASE_H
#define MODULE_BASE_H

#include "stdint.h"
#include "../../Marlin/src/MarlinCore.h"

#define MODULE_VER_LEN 32


typedef enum {
  MODULE_PRINT             = 0,  // 0
  MODULE_CNC               = 1,  // 1
  MODULE_LASER             = 2,  // 2
  MODULE_LINEAR            = 3,  // 3
  MODULE_LIGHT             = 4,  // 4
  MODULE_ENCLOSURE         = 5,  // 5
  MODULE_ROTATE            = 6,  // 6
  MODULE_PURIFIER          = 7,  // 7
  MODULE_EMERGENCY_STOP    = 8,  // 8
  MODULE_CNC_TOOL_SETTING  = 9,  // 9
  MODULE_PRINT_V_SM1       = 10, // 10
  MODULE_FAN               = 11, // 11
  MODULE_LINEAR_TMC        = 12, // 12
  MODULE_3DP_DUAL_EXTRUDER = 13, // 13
  MODULE_LASER_10W         = 14, // 14
  MODULE_BED               = 18, // 18
} MODULE_TYPE;

#define J_BED_MODULE 513

#pragma pack(1)
typedef struct {
  uint8_t key;  // Unique ID of the module after power on
  uint16_t module_id;  // snapmaker module id
  uint8_t module_index;
  uint8_t module_state;  // 0-normal 1-Being upgraded 2-Not available 3-Upgrade failure
  uint32_t sn;
  uint8_t hw_version;
  uint16_t version_length;
  uint8_t version[MODULE_VER_LEN];
} module_info_t;
#pragma pack()


#endif
