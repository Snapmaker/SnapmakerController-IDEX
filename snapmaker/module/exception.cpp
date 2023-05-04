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

#include "exception.h"
#include "src/core/serial.h"
#include "system.h"
#include "fdm.h"
#include "bed_control.h"
#include "../../Marlin/src/module/temperature.h"

Exception exception_server;

typedef struct {
  uint32_t behavior;
  uint8_t level;
} exception_behavior_t;

const exception_behavior_t exception_behavior_map[] = {
  {0,                          EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_NONE
  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_X1_TMC_FILED
  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_NOT_FIND
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_SELF_CHECK
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_LEFT_NOZZLE_LOSS
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_RIGHT_NOZZLE_LOSS
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BOTH_NOZZLE_LOSS
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_LEFT_NOZZLE_TEMP
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_RIGHT_NOZZLE_TEMP
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BOTH_NOZZLE_TEMP
  {(BIT(EXCEPTION_BAN_HEAT_BED) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_TEMP
  {(BIT(EXCEPTION_BAN_MOVE)),         EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_MOVE

  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_LEFT_NOZZLE_TEMP_TIMEOUT
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_RIGHT_NOZZLE_TEMP_TIMEOUT
  {(BIT(EXCEPTION_BAN_HEAT_BED) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_TEMP_TIMEOUT
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE) | BIT(EXCEPTION_BAN_WORK_AND_STOP)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BOTH_NOZZLE_TEMP_TIMEOUT

  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_X2_TMC_FILED
  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_Y_TMC_FILED
  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_Z_TMC_FILED
  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_E0_TMC_FILED
  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_E1_TMC_FILED
  {(BIT(EXCEPTION_BAN_MOVE) | BIT(EXCEPTION_BAN_WORK)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_ALL_TMC_FILED
};

bool Exception::is_exception() {
  return exception_status != 0;
}

uint32_t Exception::get_exception() {
  return exception_status;
}

uint32_t Exception::get_behavior() {
  return  exception_behavior;
}

exception_type_e Exception::get_wait_report_exception() {
  exception_type_e ret = wait_report_exception;
  wait_report_exception = EXCEPTION_TYPE_NONE;
  return ret;
}

uint8_t Exception::get_level(exception_type_e e) {
  return exception_behavior_map[e].level;
}

void Exception::trigger_behavior(exception_behavior_e e, bool same_sta) {
  switch (e) {
    case EXCEPTION_BAN_BED_POWER :
      OUT_WRITE(HEATER_BED_PWR_PIN, LOW);
      break;

    case EXCEPTION_BAN_MOVE :
      if (system_service.is_printing()) {
        if (!same_sta)
          LOG_I("pause working cause exception!");
        if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSING, SYSTEM_STATUE_SCOURCE_EXCEPTION)) {
          LOG_E("can NOT set to SYSTEM_STATUE_PAUSING\r\n");
          system_service.return_to_idle();
        }
      }
      break;

    case EXCEPTION_BAN_WORK :
      if (system_service.is_printing()) {
        if (!same_sta)
          LOG_I("pause working cause exception!");
        if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSING, SYSTEM_STATUE_SCOURCE_EXCEPTION)) {
          LOG_E("can NOT set to SYSTEM_STATUE_PAUSING\r\n");
          system_service.return_to_idle();
        }
      }
      break;

    case EXCEPTION_BAN_HEAT_NOZZLE :
      if (!same_sta)
        LOG_I("stop heating hotend cause exception! c0: %d / t0: %d, c1: %d / t1: %d\n",
              (int)thermalManager.degHotend(0), thermalManager.degTargetHotend(0),
              (int)thermalManager.degHotend(1), thermalManager.degTargetHotend(1));
      fdm_head.set_temperature(0, 0);
      fdm_head.set_temperature(1, 0);
      break;

    case EXCEPTION_BAN_HEAT_BED :
      if (!same_sta)
        LOG_I("stop heating bed cause exception! c: %d / t: %d\n",
            (int)thermalManager.degBed(), thermalManager.degTargetBed());
      bed_control.set_temperature(0);
      break;

    case EXCEPTION_BAN_WORK_AND_STOP :
      if (system_service.is_working()) {
        if (!same_sta)
          LOG_I("stop working cause exception!");
        if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_STOPPING, SYSTEM_STATUE_SCOURCE_EXCEPTION)) {
          LOG_E("can NOT set to SYSTEM_STATUE_STOPPING\r\n");
          system_service.return_to_idle();
        }
      }
      break;
  }
}

bool Exception::trigger_exception(exception_type_e e) {
  bool same_sta = !!(exception_status & BIT(e));
  uint32_t behavior = exception_behavior_map[e].behavior;
  // Update the behavior corresponding to the exception
  for (uint8_t i = 0; behavior; i++, behavior >>= 1) {
    if (0x1 & behavior) {
      trigger_behavior((exception_behavior_e)i, same_sta);
    }
  }

  EXCEPTION_TRIGGER(e);
  exception_behavior |= exception_behavior_map[e].behavior;

  if (same_sta)
    return false;

  LOG_I("trigger exception:%d, cur exception code:0x%x, behavior code:0x%x\n", e, exception_status, exception_behavior);
  return true;
}

void Exception::recover_behavior(exception_behavior_e e) {
  switch (e) {
    case EXCEPTION_BAN_BED_POWER :
      OUT_WRITE(HEATER_BED_PWR_PIN, HIGH);
      break;
    case EXCEPTION_BAN_MOVE :
      break;
    case EXCEPTION_BAN_WORK :
      break;
    case EXCEPTION_BAN_HEAT_NOZZLE :
      break;
    case EXCEPTION_BAN_HEAT_BED :
      break;
    case EXCEPTION_BAN_WORK_AND_STOP :
      break;
  }
}

void Exception::clean_exception(exception_type_e e) {
  if (!(exception_status & (BIT(e))))
    return;

  EXCEPTION_CLEAN(e);
  exception_behavior = 0;
  uint32_t exception = exception_status;
  for (uint8_t i = 0; exception; exception >>= 1, i++) {
    if (0x1 & exception) {
      exception_behavior |= exception_behavior_map[i].behavior;
    }
  }

  // Update the behavior corresponding to the exception
  uint32_t behavior = exception_behavior_map[e].behavior;
  for (uint8_t i = 0; behavior; i++, behavior >>= 1) {
    if ((0x1 & behavior) && !(BIT(i) & exception_behavior)) {
      recover_behavior((exception_behavior_e)i);
    }
  }
  LOG_I("clear exception:%d, cur exception code:0x%x, behavior code:0x%x\n", e, exception_status, exception_behavior);
}

bool Exception::is_allow_work(bool is_err_report) {
  return !is_ban_behavior_and_report(BIT(EXCEPTION_BAN_WORK) | BIT(EXCEPTION_BAN_WORK_AND_STOP), is_err_report);
}

bool Exception::is_allow_heat_nozzle(bool is_err_report) {
  return !is_ban_behavior_and_report(BIT(EXCEPTION_BAN_HEAT_NOZZLE), is_err_report);
}

bool Exception::is_allow_heat_bed(bool is_err_report) {
  return !is_ban_behavior_and_report(BIT(EXCEPTION_BAN_BED_POWER) | BIT(EXCEPTION_BAN_HEAT_BED), is_err_report);
}

bool Exception::is_allow_move(bool is_err_report) {
  return !is_ban_behavior_and_report(BIT(EXCEPTION_BAN_MOVE), is_err_report);
}

exception_type_e Exception::is_ban_behavior_and_report(uint32_t behavior_bit_code, bool is_err_report) {
  uint32_t exception = exception_status;
  exception_type_e ret = EXCEPTION_TYPE_NONE;
  for (uint8_t i = 0; exception; i++, exception >>= 1) {
    if ((0x1 & exception) && (exception_behavior_map[i].behavior & behavior_bit_code)) {
      ret = (exception_type_e)i;
    }
  }
  if (is_err_report) {
    wait_report_exception = ret;
  }
  return EXCEPTION_TYPE_NONE;
}
