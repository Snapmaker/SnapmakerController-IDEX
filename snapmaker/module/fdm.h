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

#ifndef FDM_H
#define FDM_H
#include "../J1/common_type.h"
#include "module_base.h"
#include "system.h"

#pragma pack(1)

#define HOTEND_MAX_TEMP  (400)

enum {
  FAN_TYPE_COLD_MODULE = 0,
  FAN_TYPE_COLD_EXTRUDER = 2,
};

typedef enum {
  FILAMENT_CHANGE_STOP,
  FILAMENT_CHANGE_EXTRUDER,
  FILAMENT_CHANGE_RETRACK,
} filamenter_change_status_e;

typedef enum {
  BRASS_NTC_E,
  BRASS_PT100_E,
  HARDENING_STEEL_PT100,
  UNKNOWN_NOZZLE_TYPE = 0xFF,
} nozzle_texture_type_e;

typedef struct {
  uint8_t index;
  uint8_t filament_status;
  uint8_t filament_enable_status;
  bool is_available;
  uint8_t type;
  float_to_int_t diameter;
  float_to_int_t cur_temp;
  float_to_int_t target_temp;
}extruder_info_t;

typedef struct {
  uint8_t index;
  uint8_t type;
  uint8_t speed;
}extruder_fan_info_t;

typedef struct {
  uint8_t key;
  uint8_t head_status;
  bool head_active;
  uint8_t extruder_count;
  extruder_info_t extruder_info;
  uint8_t fan_count;
  extruder_fan_info_t extruder_fan[1];
}FDM_info;

typedef struct {
  uint8_t key;
  uint8_t index;
  filamenter_change_status_e status;
}extruder_move_status_t;

#pragma pack()

typedef struct {
  nozzle_texture_type_e  texture;
  float caliber ;
  uint16_t adc_min;
  uint16_t adc_max;
} nozzle_type_t;

class FDM_Head {
  public:

    uint8_t stop_single_extruder_e;
    uint8_t stop_single_extruder_en;

    void    init();
    ErrCode set_temperature(uint8_t e, uint16_t temperature, bool is_save=true);
    float   get_temperature(uint8_t e);
    ErrCode set_work_speed(float speed);
    ErrCode change_filamenter(uint8_t e, float feedrate, filamenter_change_status_e status);
    ErrCode change_tool(uint8_t e);
    ErrCode set_fan_speed(uint8_t e, uint8_t fan_index, uint8_t speed);
    ErrCode get_fan_speed(uint8_t e, uint8_t fan_index, uint8_t &speed);
    ErrCode set_extruder_diff(uint8_t axis, float diff);
    ErrCode get_fdm_info(uint8_t e, FDM_info *fdm);
    ErrCode get_extruder_info(uint8_t e, extruder_info_t *info);
    ErrCode get_module_info(uint8_t e, module_info_t &info);
    uint8_t get_key(uint8_t e);
    uint16_t get_nozzle_type(uint8_t e, nozzle_texture_type_e *texture, float *caliber);

    bool is_duplication_enabled(uint8_t e) {return duplication_enabled_move[e] || !system_service.is_working();}
    bool extraduer_enable(uint8_t e) { return duplication_enabled_move[e]; }
    // Double - head printing will not work after disable
    void set_duplication_enabled(uint8_t e, bool status) {duplication_enabled_move[e] = status;}
    bool is_duplicating();
    filamenter_change_status_e get_change_filamenter_status(uint8_t e) {
      return (change_filamenter_status[e]);
    }
    bool is_change_filamenter(uint8_t e) {
      return (change_filamenter_status[e] != FILAMENT_CHANGE_STOP );
    }
    bool is_change_filamenter() {
      return is_change_filamenter(0)||is_change_filamenter(1);
    }
    bool get_filamenter_dir(uint8_t e) {
      if (change_filamenter_status[e] == FILAMENT_CHANGE_EXTRUDER) {
        return true;
      }
      return false;
    }
  private:
    module_info_t module_info[EXTRUDERS];
    filamenter_change_status_e change_filamenter_status[EXTRUDERS] = {FILAMENT_CHANGE_STOP, FILAMENT_CHANGE_STOP};
    bool filamenter_dir[EXTRUDERS] = {true, true};
    bool duplication_enabled_move[EXTRUDERS] = {true, true};
};

extern FDM_Head fdm_head;

#endif
