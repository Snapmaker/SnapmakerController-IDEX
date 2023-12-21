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

#ifndef SYSTEM_H
#define SYSTEM_H

#include "stdint.h"
#include "../J1/common_type.h"
#include "module_base.h"

enum {
  HW_VER_1 = 1,
  HW_VER_2,
  HW_VER_3,
  HW_VER_4,
  HW_VER_5,
  HW_VER_6,
  HW_VER_ERR,
};

enum : uint8_t {
  AXIS_X1 = 0,
  AXIS_Y1 = 1,
  AXIS_Z1 = 2,
  AXIS_X2 = 6,
  AXIS_E0 = 7,
  AXIS_E1 = 8,
};

typedef enum : uint8_t {
  // job contorl
  SYSTEM_STATUE_IDLE,
  SYSTEM_STATUE_STARTING,
  SYSTEM_STATUE_PRINTING,
  SYSTEM_STATUE_PAUSING,
  SYSTEM_STATUE_PAUSED,
  SYSTEM_STATUE_STOPPING,
  SYSTEM_STATUE_STOPPED,
  SYSTEM_STATUE_FINISHING,
  SYSTEM_STATUE_COMPLETED,
  SYSTEM_STATUE_RECOVERING,
  SYSTEM_STATUE_RESUMING,
  SYSTEM_STATUE_POWER_LOSS_RESUMING,

  // 3Dp calibtration
  SYSTEM_STATUE_CAlIBRATION = 31,
  SYSTEM_STATUE_CAlIBRATION_Z_PROBING,
  SYSTEM_STATUE_CAlIBRATION_XY_PROBING,
  SYSTEM_STATUE_PID_AUTOTUNE,
} system_status_e;

typedef enum {
  SYSTEM_STATUE_SCOURCE_NONE,  // Do not change the trigger source
  SYSTEM_STATUE_SCOURCE_SACP,
  SYSTEM_STATUE_SCOURCE_GCODE,
  SYSTEM_STATUE_SCOURCE_FILAMENT,
  SYSTEM_STATUE_SCOURCE_PL,
  SYSTEM_STATUE_SCOURCE_TOOL_CHANGE,
  SYSTEM_STATUE_SCOURCE_STOP_EXTRUDE,
  SYSTEM_STATUE_SCOURCE_EXCEPTION,
  SYSTEM_STATUE_SCOURCE_DONE,
  SYSTEM_STATUE_SCOURCE_Z_LIVE_OFFSET,
  SYSTEM_STATUE_SCOURCE_M600,
} system_status_source_e;

#define AXIS_COUNT 4  // x x1 y z

#pragma pack(1)
typedef struct {
  uint8_t Ji_num;  // J generation machine is 4
  uint8_t hw_version; // J1 is 0
  uint32_t sn;
  uint16_t version_length;
  uint8_t version[];
} machine_info_t;

typedef struct {
  uint8_t axis;
  float_to_int_t position;
} coordinate_info_t;

typedef struct {
  uint8_t homed;  // 0-homed,  1-Not all axes go back to home
  uint8_t coordinate_system_id;  // 0-G53, 1-G54
  bool is_origin_offset;  // J1 does not support and defaults to true
  uint8_t coordinate_axis_count;  // AXIS_COUNT
  coordinate_info_t coordinate_axis_info[AXIS_COUNT];
  uint8_t origin_offset_count;
  coordinate_info_t origin_offset_info[AXIS_COUNT];
} coordinate_system_t;

typedef struct {
  uint8_t size_count;
  coordinate_info_t size[AXIS_COUNT];
  uint8_t home_offset_count;
  coordinate_info_t home_offset[XYZ];
} machine_size_t;

#pragma pack()
class SystemService {
  public:
    void init(void);
    void get_coordinate_system_info(coordinate_system_t * info, bool is_logical=false);
    void get_machine_info(machine_info_t *info);
    void get_machine_size(machine_size_t *size);
    ErrCode set_origin(coordinate_info_t axis);
    system_status_e get_status() {return status_;}
    uint8_t *get_sn_addr(uint16_t *sn_len);
    system_status_source_e get_source() {return source_;}
    ErrCode set_status(system_status_e status, system_status_source_e source=SYSTEM_STATUE_SCOURCE_NONE);
    bool is_calibtration_status() { return (status_ >= SYSTEM_STATUE_CAlIBRATION) && (status_ <= SYSTEM_STATUE_PID_AUTOTUNE);}
    bool is_working();
    bool is_printing();
    bool is_idle() {return status_ == SYSTEM_STATUE_IDLE;}
    void factory_reset(void);
    uint8_t get_hw_version(bool is_refresh = false);
    void save_setting();
    void return_to_idle();

  private:
    SemaphoreHandle_t lock_;
    system_status_e status_ = SYSTEM_STATUE_IDLE;
    system_status_source_e source_ = SYSTEM_STATUE_SCOURCE_NONE;
    uint8_t hw_version = 0xff;
};

extern SystemService system_service;
#endif
