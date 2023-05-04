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

#pragma once
#include <stdint.h>
#include "system.h"
#include "../J1/common_type.h"
#include "../debug/debug.h"
#include "../../../Marlin/src/core/macros.h"
#include <EEPROM.h>

#define FACTORY_DATA_MAGIC                  "SNAP"
#define DEFAULT_BUILD_PLATE_THICKNESS       (5.0)
#define MAX_BUILD_PLATE_THICKNESS           (5.5)
#define MIN_BUILD_PLATE_THICKNESS           (4.699)

#pragma pack(push, 1)

typedef struct {
  float build_plate_thickness;
} fd_info_t;

typedef struct{
  uint8_t     magic[4];
  uint32_t    len;
  fd_info_t   fd;
  uint8_t     dump[FACTORY_DATA_SIZE - 12 - sizeof(fd_info_t)];
  uint32_t    checksum;
} fd_flash_t;
#pragma pack(pop)

class factory_data_srv {
public:
  bool init();
  bool load();
  bool save();
  void reset();
  void erase();
  void log();

  bool setBuildPlateThickness(float bpt);
  float getBuildPlateThickness();

private:
  bool check(fd_flash_t &fd);

private:
  fd_flash_t _data;
};

extern factory_data_srv fd_srv;

