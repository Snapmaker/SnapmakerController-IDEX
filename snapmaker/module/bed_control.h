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

#ifndef BED_CONTROL_H
#define BED_CONTROL_H

#include "../J1/common_type.h"
#include "module_base.h"

#define BED_MAX_TEMP  (200)

#pragma pack(1)
typedef struct {
  uint8_t key;
  uint8_t bed_count;  // Fixed 1 for SACP protocol compatibility
  uint8_t zone_index;  // Fixed 0 for SACP protocol compatibility
  float_to_int_t cur_temp;
  int16_t target_temp;
} bed_control_info_t;

#pragma pack()
class BedControl {
  public:
    bool self_check();
    ErrCode set_temperature(uint16_t temperature, bool is_save=true);
    ErrCode get_info(bed_control_info_t &info);
    ErrCode get_module_info(module_info_t &info);
  public:
    bool is_error = false;
};
extern BedControl bed_control;
#endif
