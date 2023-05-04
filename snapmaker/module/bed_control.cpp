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

#include "bed_control.h"
#include "src/module//temperature.h"
#include "exception.h"
#include "./power_loss.h"

BedControl bed_control;

bool BedControl::self_check() {
  uint32_t delay_time = 0;
  is_error = false;
  SET_INPUT(HEATER_BED_BACK_PIN);
  OUT_WRITE(HEATER_BED_PIN, HIGH);
  delay_time = millis() + 20;
  while (PENDING(millis(), delay_time));
  if (READ(HEATER_BED_BACK_PIN) == HIGH) {
    is_error = true;
    LOG_E("BED open self check failed\n");
  }

  OUT_WRITE(HEATER_BED_PIN, LOW);
  delay_time = millis() + 20;
  while (PENDING(millis(), delay_time));
  if (READ(HEATER_BED_BACK_PIN) == LOW) {
    LOG_E("BED close self check failed\n");
    is_error = true;
    exception_server.trigger_exception(EXCEPTION_TYPE_BED_SELF_CHECK);
  }
  if (is_error) {
    OUT_WRITE(HEATER_BED_PWR_PIN, LOW);
  } else {
    LOG_E("BED self check success\n");
  }
  return is_error;
}

ErrCode BedControl::set_temperature(uint16_t temperature, bool is_save) {
  if (temperature > 0 && !exception_server.is_allow_heat_bed()) {
    return E_SYSTEM_EXCEPTION;
  }
  if (temperature > BED_MAX_TEMP) {
    return E_PARAM;
  }
  if (is_save) {
    power_loss.stash_data.bed_temp = temperature;
  }
  thermalManager.setTargetBed(temperature);
  return E_SUCCESS;
}

ErrCode BedControl::get_info(bed_control_info_t &info) {
  float temperature = thermalManager.degBed();
  info.key = MODULE_KEY(MODULE_BED, 0);
  info.bed_count = 1;
  info.zone_index = 0;
  info.cur_temp = FLOAT_TO_INT(temperature);
  info.target_temp = (int16_t)thermalManager.degTargetBed();
  if (temperature >= 300) {
    exception_server.trigger_exception(EXCEPTION_TYPE_BED_NOT_FIND);
  } else {
    exception_server.clean_exception(EXCEPTION_TYPE_BED_NOT_FIND);
  }
  return E_SUCCESS;
}

ErrCode BedControl::get_module_info(module_info_t &info) {
  info.key = MODULE_KEY(MODULE_BED, 0);
  info.module_id = J_BED_MODULE;
  info.module_index = 0;
  info.module_state = 0;
  info.sn = 0;
  info.version_length = MODULE_VER_LEN;
  info.version[0] = 0;
  return E_SUCCESS;
}
