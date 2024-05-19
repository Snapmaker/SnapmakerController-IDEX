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

#ifndef EXCEPTION_H
#define EXCEPTION_H

#include "../J1/common_type.h"

#define EXCEPTION_TRIGGER(type) (exception_status |= (1 << type))
#define EXCEPTION_CLEAN(type) (exception_status &= ~(1 << type))

#define EXCEPTION_TMC_FILED_BEHAVIOR (E_B(EXCEPTION_BAN_MOVE))
#define EXCEPTION_BED_NOT_FIND_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_BED))
#define EXCEPTION_BED_SELF_CHECK_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_BED))
#define EXCEPTION_LEFT_NOZZLE_LOSS_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_NOZZLE))
#define EXCEPTION_RIGHT_NOZZLE_LOSS_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_NOZZLE))
#define EXCEPTION_BOTH_NOZZLE_LOSS_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_NOZZLE))
#define EXCEPTION_LEFT_NOZZLE_TEMP_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_NOZZLE))
#define EXCEPTION_RIGHT_NOZZLE_TEMP_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_NOZZLE))
#define EXCEPTION_BOTH_NOZZLE_TEMP_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_NOZZLE))
#define EXCEPTION_BED_TEMP_BEHAVIOR (E_B(EXCEPTION_BAN_HEAT_NOZZLE))
#define EXCEPTION_TYPE_MOVE_BEHAVIOR (E_B(EXCEPTION_BAN_MOVE))

enum {
  EXCEPTION_LEVLE_0,  // non-blocking
  EXCEPTION_LEVLE_1,  // blocking
};

typedef enum {
  EXCEPTION_BAN_BED_POWER = 4,
  EXCEPTION_BAN_MOVE = 8,
  EXCEPTION_BAN_WORK = 9,
  EXCEPTION_BAN_HEAT_NOZZLE = 10,
  EXCEPTION_BAN_HEAT_BED = 11,
  EXCEPTION_BAN_WORK_AND_STOP = 14,
} exception_behavior_e;

typedef enum {
  EXCEPTION_TYPE_NONE,
  EXCEPTION_TYPE_X1_TMC_FILED,
  EXCEPTION_TYPE_BED_NOT_FIND,
  EXCEPTION_TYPE_BED_SELF_CHECK,
  EXCEPTION_TYPE_LEFT_NOZZLE_LOSS,
  EXCEPTION_TYPE_RIGHT_NOZZLE_LOSS,
  EXCEPTION_TYPE_BOTH_NOZZLE_LOSS,
  EXCEPTION_TYPE_LEFT_NOZZLE_TEMP,
  EXCEPTION_TYPE_RIGHT_NOZZLE_TEMP,
  EXCEPTION_TYPE_BOTH_NOZZLE_TEMP,
  EXCEPTION_TYPE_BED_TEMP,
  EXCEPTION_TYPE_MOVE,
  EXCEPTION_TYPE_LEFT_NOZZLE_TEMP_TIMEOUT,
  EXCEPTION_TYPE_RIGHT_NOZZLE_TEMP_TIMEOUT,
  EXCEPTION_TYPE_BED_TEMP_TIMEOUT,
  EXCEPTION_TYPE_BOTH_NOZZLE_TEMP_TIMEOUT,
  EXCEPTION_TYPE_X2_TMC_FILED,
  EXCEPTION_TYPE_Y_TMC_FILED,
  EXCEPTION_TYPE_Z_TMC_FILED,
  EXCEPTION_TYPE_E0_TMC_FILED,
  EXCEPTION_TYPE_E1_TMC_FILED,
  EXCEPTION_TYPE_ALL_TMC_FILED,
  EXCEPTION_TYPE_LEFT_NOZZLE_OVERTEMP,
  EXCEPTION_TYPE_RIGHT_NOZZLE_OVERTEMP,
  EXCEPTION_TYPE_MAX_COUNT
} exception_type_e;

class Exception {
  public:
    bool is_exception();
    uint32_t get_exception();
    uint32_t get_behavior();
    exception_type_e get_wait_report_exception();
    bool trigger_exception(exception_type_e e);
    void trigger_behavior(exception_behavior_e e, bool same_sta = false);
    void recover_behavior(exception_behavior_e e);
    void clean_exception(exception_type_e e);
    uint8_t get_level(exception_type_e e);
    bool is_allow_work(bool is_err_report=true);
    bool is_allow_heat_nozzle(bool is_err_report=true);
    bool is_allow_heat_bed(bool is_err_report=true);
    bool is_allow_move(bool is_err_report=true);
  private:
    exception_type_e is_ban_behavior_and_report(uint32_t behavior_bit_code, bool is_err_report=true);
  private:
    uint32_t exception_status = 0;
    uint32_t exception_behavior = 0;
    exception_type_e wait_report_exception = EXCEPTION_TYPE_NONE;
};

extern Exception exception_server;

#endif
