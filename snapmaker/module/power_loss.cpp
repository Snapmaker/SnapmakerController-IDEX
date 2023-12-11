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

#include "power_loss.h"
#include "print_control.h"
#include "src/module/motion.h"
#include "src/module/tool_change.h"
#include "src/module/planner.h"
#include "src/gcode/gcode.h"
#include "src/module/stepper.h"
#include "motion_control.h"
#include "enclosure.h"
#include "../../Marlin/src/module/temperature.h"
#include "system.h"
#include "filament_sensor.h"
#include "fdm.h"
#include "HAL.h"
#include <EEPROM.h>


PowerLoss power_loss;

extern feedRate_t fast_move_feedrate;

void PowerLoss::stash_print_env() {

  xyze_pos_t cur_position;
  cur_position[E_AXIS] = planner.get_axis_position_mm(E_AXIS);
  cur_position[X_AXIS] = planner.get_axis_position_mm(X_AXIS);
  cur_position[Y_AXIS] = planner.get_axis_position_mm(Y_AXIS);
  cur_position[Z_AXIS] = planner.get_axis_position_mm(Z_AXIS);
  stash_data.position = cur_position;

  uint32_t cur_line = print_control.get_cur_line();
  if (cur_line) {
    if (cur_line > stash_data.file_position) {
      stash_data.file_position = cur_line - 1;
    }
  }
  else {
    stash_data.file_position = 0;
  }

  stash_data.dual_x_carriage_mode = dual_x_carriage_mode;
  stash_data.print_feadrate = feedrate_mm_s;
  stash_data.feedrate_percentage = feedrate_percentage;
  stash_data.active_extruder = active_extruder;
  // stash_data.motion_extruder = cur_extruder;
  stash_data.travel_feadrate = fast_move_feedrate;
  stash_data.axis_relative = gcode.axis_relative;
  stash_data.print_mode = print_control.mode_;
  stash_data.duplicate_extruder_x_offset = duplicate_extruder_x_offset;
  stash_data.home_offset = home_offset;
  stash_data.print_offset = print_control.xyz_offset;
  stash_data.work_time = print_control.get_work_time();
  stash_data.noise_mode = print_control.get_noise_mode();

  stash_data.bed_temp = thermalManager.degTargetBed();
  HOTEND_LOOP() {
    if (fdm_head.extraduer_enable(e)) {
      stash_data.nozzle_temp[e] = thermalManager.degTargetHotend(e);
    }
    else {
      stash_data.nozzle_temp[e] = 0;
    }
    stash_data.extruder_dual_enable[e] = fdm_head.is_duplication_enabled(e);
    stash_data.extruder_temperature_lock[e] = print_control.temperature_lock(e);
    for (uint8_t i = 0; i < 2; i++) {
      fdm_head.get_fan_speed(e, i, stash_data.fan[e][i]);
    }
    stash_data.flow_percentage[e] = planner.flow_percentage[e];
  }

}

bool PowerLoss::wait_temp_resume() {
  uint32_t log_timeout;
  uint8_t temp_target_count = 0;
  LOG_I("wait for the temperature to recover\n");
  thermalManager.print_heater_states(active_extruder);
  SERIAL_EOL();
  log_timeout = millis() + 10 * 1000;
  while (system_service.get_status() == SYSTEM_STATUE_RESUMING || system_service.get_status() == SYSTEM_STATUE_POWER_LOSS_RESUMING)
  // while (system_service.get_status() == SYSTEM_STATUE_RESUMING)
  {
    celsius_float_t ht0, ht1, bt, hc0, hc1, bc;
    ht0 = thermalManager.degTargetHotend(0);
    hc0 = thermalManager.degHotend(0);
    ht1 = thermalManager.degTargetHotend(1);
    hc1 = thermalManager.degHotend(1);
    bt = thermalManager.degTargetBed();
    bc = thermalManager.degBed();
    if ( hc0 > (ht0 - 2.0) && hc1 > (ht1 - 2.0) && bc > (bt - 2.0) ) {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    temp_target_count++;
    if (ELAPSED(millis(), log_timeout)) {
      thermalManager.print_heater_states(active_extruder);
      SERIAL_EOL();
      log_timeout = millis() + 10 * 1000;
    }
  }
  thermalManager.print_heater_states(active_extruder);
  SERIAL_EOL();
  LOG_I("Temperature recovery End\n");
  return true;
}

ErrCode PowerLoss::extrude_before_resume() {

  uint8_t need_active_extruder = 0;
  bool tool_change_no_move = false;

  filament_sensor.reset();

  HOTEND_LOOP() {
    print_control.temperature_lock(e, stash_data.extruder_temperature_lock[e]);
  }

  if (stash_data.dual_x_carriage_mode >= DXC_DUPLICATION_MODE) {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    // tool_change(0, true);
    tool_change_no_move = true;
    // Use mirror mode to facilitate both heads extruding together
    dual_x_carriage_mode = DXC_MIRRORED_MODE;
    duplicate_extruder_x_offset = MIRRORED_MODE_X_OFFSET;
    idex_set_mirrored_mode(true);
    set_duplication_enabled(true);
  }
  else {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    // tool_change(stash_data.active_extruder);
    need_active_extruder = stash_data.active_extruder;
    set_duplication_enabled(false);
  }

  thermalManager.setTargetBed(stash_data.bed_temp);
  HOTEND_LOOP() {
    fdm_head.set_duplication_enabled(e, stash_data.extruder_dual_enable[e]);
    if ((dual_x_carriage_mode == DXC_MIRRORED_MODE) || (e == stash_data.active_extruder)) {
      stash_data.nozzle_temp[e] = stash_data.nozzle_temp[e] > PREHEAT_1_TEMP_HOTEND? \
              stash_data.nozzle_temp[e] : PREHEAT_1_TEMP_HOTEND;
    }
    if (!stash_data.extruder_dual_enable[e]) {
      stash_data.nozzle_temp[e] = 0;
    }
    if (fdm_head.extraduer_enable(e)) {
      thermalManager.setTargetHotend(stash_data.nozzle_temp[e], e);
      for (uint8_t i = 0; i < 2; i++) {
        fdm_head.set_fan_speed(e, i, stash_data.fan[e][i]);
      }
    }
    else {
      thermalManager.setTargetHotend(0, e);
    }
  }
  wait_temp_resume();

  if (homing_needed()) {
    motion_control.home();
  }
  // else {
  //   motion_control.home_x();
  // }

  tool_change(need_active_extruder, tool_change_no_move);

  if(fabs(current_position.x - x_home_pos(active_extruder)) > 0.1) {
    motion_control.home_x();
  }

  int16_t move_distance = EXTRUDE_X_MOVE_DISTANCE;
  if (active_extruder) {
    move_distance = -move_distance;
  }
  dual_x_carriage_unpark();

  motion_control.move_x(move_distance, PRINT_TRAVEL_FEADRATE);
  motion_control.synchronize();

  // motion_control.extrude_e(EXTRUDE_E_DISTANCE, CHANGE_FILAMENT_SPEED);
  destination.set(current_position.x, current_position.y, current_position.z, current_position.e + EXTRUDE_E_DISTANCE);
  prepare_internal_move_to_destination(MMM_TO_MMS(CHANGE_FILAMENT_SPEED));
  motion_control.synchronize();

  ErrCode ret = E_SUCCESS;
  if (filament_sensor.is_trigger()) {
    // dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    // idex_set_mirrored_mode(false);
    ret = E_COMMON_ERROR;
  }
  // else {
  //   // resume dual_x_carriage_mode
  //   dual_x_carriage_mode = (DualXMode)stash_data.dual_x_carriage_mode;
  //   idex_set_mirrored_mode(dual_x_carriage_mode == DXC_MIRRORED_MODE);
  // }

  motion_control.move_x(-move_distance, PRINT_TRAVEL_FEADRATE / 10);
  motion_control.synchronize();

  vTaskDelay(pdMS_TO_TICKS(1000));

  if (E_SUCCESS == ret) {
    motion_control.move_x(move_distance, PRINT_TRAVEL_FEADRATE);
    motion_control.synchronize();
    vTaskDelay(pdMS_TO_TICKS(1000));
    motion_control.move_x(-move_distance, PRINT_TRAVEL_FEADRATE / 10);
    motion_control.synchronize();
  }

  // resume dual_x_carriage_mode
  dual_x_carriage_mode = (DualXMode)stash_data.dual_x_carriage_mode;
  idex_set_mirrored_mode(dual_x_carriage_mode == DXC_MIRRORED_MODE);

  return ret;
}

void PowerLoss::resume_print_env() {

  print_control.set_work_time(stash_data.work_time);
  print_control.mode_ = (print_mode_e)stash_data.print_mode;
  thermalManager.setTargetBed(stash_data.bed_temp);
  HOTEND_LOOP() {
    fdm_head.set_duplication_enabled(e, stash_data.extruder_dual_enable[e]);
    if (fdm_head.is_duplication_enabled(e)) {
      thermalManager.setTargetHotend(stash_data.nozzle_temp[e], e);
      fdm_head.set_fan_speed(e, 0, stash_data.fan[e][0]);
    }
    else {
      thermalManager.setTargetHotend(0, e);
      fdm_head.set_fan_speed(e, 0, 0);
    }
    planner.set_flow(e, stash_data.flow_percentage[e]);
  }

  feedrate_mm_s = stash_data.print_feadrate;
  fast_move_feedrate = stash_data.travel_feadrate;
  gcode.axis_relative = stash_data.axis_relative;
  duplicate_extruder_x_offset = stash_data.duplicate_extruder_x_offset;
  print_control.set_noise_mode(print_noise_mode_e(stash_data.noise_mode & 0xFF));

  LOG_I("Resume x %f\r\n", stash_data.position[X_AXIS]);
  LOG_I("current s: %f\r\n", current_position.x);
  LOG_I("Active extruder %d\r\n", active_extruder);

  // home_offset = stash_data.home_offset;
  // sync_plan_position();

  if (system_service.get_source() != SYSTEM_STATUE_SCOURCE_Z_LIVE_OFFSET) {
    idex_set_parked(true);
    dual_x_carriage_unpark();
    motion_control.move_to_z(stash_data.position[Z_AXIS] + Z_DOWN_SAFE_DISTANCE, PRINT_TRAVEL_FEADRATE);
    motion_control.move_to_xy(stash_data.position[X_AXIS], stash_data.position[Y_AXIS], PRINT_TRAVEL_FEADRATE);
    motion_control.move_to_z(stash_data.position[Z_AXIS], PRINT_TRAVEL_FEADRATE);
  }
  else {
    motion_control.move_to_xyz(stash_data.position[X_AXIS], stash_data.position[Y_AXIS], stash_data.position[Z_AXIS]);
  }

  // motion_control.move_to_z(stash_data.position[Z_AXIS] + Z_DOWN_SAFE_DISTANCE, PRINT_TRAVEL_FEADRATE);
  // motion_control.move_to_xy(stash_data.position[X_AXIS], stash_data.position[Y_AXIS], PRINT_TRAVEL_FEADRATE);
  // motion_control.move_to_z(stash_data.position[Z_AXIS], PRINT_TRAVEL_FEADRATE);

  planner.synchronize();

  current_position.e = stash_data.position.e;
  print_control.xyz_offset = stash_data.print_offset;
  feedrate_percentage = stash_data.feedrate_percentage;
  sync_plan_position();

  if (power_loss.m600_cur_line >= 0) {
    stash_data.file_position = power_loss.m600_cur_line + 1;
    power_loss.m600_cur_line = -1;
    LOG_I(">>>> stash_data.file_position %d\r\n", stash_data.file_position);
  }

  next_req = cur_line = line_number_sum = stash_data.file_position;
}

 /**
 * save the power panic data to flash
 */
void PowerLoss::write_flash(void)
{
  uint32_t addr = FLASH_MARLIN_POWERPANIC;
	uint32_t u32data;
	uint8_t *buff = (uint8_t *)&stash_data;
  stash_data.state = PL_WAIT_RESUME;
  stash_data.check_num = 0;
  for (uint32_t i = 0; i < sizeof(power_loss_t) - 4; i++) {
    stash_data.check_num += buff[i];
  }

	FLASH_Unlock();
	for (uint32_t i = 0; i < sizeof(power_loss_t); i = i + 4) {
		u32data = (buff[i + 3] << 24) | (buff[i + 2] << 16) | (buff[i + 1] << 8) | buff[i];
		FLASH_ProgramWord(addr, u32data);
		addr = addr + 4;
	}
	FLASH_Lock();
}

void PowerLoss::show_power_loss_info() {
  SERIAL_ECHOLN("-----PL data info-----");
  SERIAL_ECHOLNPAIR("name:", (const char *)&stash_data.gcode_file_name);
  SERIAL_ECHOLNPAIR("MD5:", (const char *)&stash_data.gcode_file_md5);
  SERIAL_ECHOLNPAIR("file_position:", stash_data.file_position);
  HOTEND_LOOP() {
    SERIAL_ECHOPAIR("Head:", e, " temp:", stash_data.nozzle_temp[e]);
    for (uint8_t i = 0; i < 2; i++) {
      SERIAL_ECHOPAIR(" fan_speed[", i, "]:", stash_data.fan[e][i]);
    }
    SERIAL_ECHO("\n");
  }
  SERIAL_ECHOLNPAIR("Bed temp:", stash_data.bed_temp);
  SERIAL_ECHOLNPAIR("position x:", stash_data.position.x,
                           " y:", stash_data.position.y,
                           " z:", stash_data.position.z,
                           " e:", stash_data.position.e);
  SERIAL_ECHOLNPAIR("active_extruder:", stash_data.active_extruder);
  SERIAL_ECHOLNPAIR("extruder_dual_enable[0]:", stash_data.extruder_dual_enable[0]);
  SERIAL_ECHOLNPAIR("extruder_dual_enable[1]:", stash_data.extruder_dual_enable[1]);
  SERIAL_ECHOLNPAIR("print feadrate:", stash_data.print_feadrate);
  SERIAL_ECHOLNPAIR("travel feadrate:", stash_data.travel_feadrate);
  SERIAL_ECHOLNPAIR("accumulator:", stash_data.accumulator);
  SERIAL_ECHOLNPAIR("dual_x_carriage_mode:", stash_data.dual_x_carriage_mode);
  SERIAL_ECHOLN("-----PL data end-----");
}

void PowerLoss::init() {
  uint8_t *flash_addr = (uint8_t *)FLASH_MARLIN_POWERPANIC;
  uint8_t *ram_addr = (uint8_t *)&stash_data;
  uint32_t check_num = 0;

  SET_INPUT_PULLUP(HW_1_2(POWER_LOST_220V_HW1_PIN, POWER_LOST_220V_HW2_PIN));

  for (uint32_t i = 0; i < sizeof(power_loss_t); i++) {
    ram_addr[i] = flash_addr[i];
    if (i < sizeof(power_loss_t) - 4) {
      check_num += flash_addr[i];
    }
  }
  if (stash_data.state == PL_WAIT_RESUME) {
    if (check_num == stash_data.check_num) {
      SERIAL_ECHOLNPAIR("PL: Got available data!");
      // show_power_loss_info();
    } else {
      stash_data.state = PL_NO_DATE;
      SERIAL_ECHOLNPAIR("PL: Unavailable data!, checknum:", check_num, "-", stash_data.check_num);
    }
  } else {
    SERIAL_ECHOLNPAIR("PL: No data!");
  }

  if (power_loss.is_power_220v_pin_trigger()) {
    power_loss.power_loss_en = false;
    SERIAL_ECHOLNPAIR("power-loss signal is abnormal, disable the power-loss function");
  }
  is_inited = true;
}

void PowerLoss::clear() {
  SERIAL_ECHOLNPGM("PL: clear power loss data!");
  uint8_t *flash_addr = (uint8_t *)FLASH_MARLIN_POWERPANIC;
  for (uint32_t i = 0; i < sizeof(power_loss_t); i++) {
    if (flash_addr[i] != 0xff) {
      SERIAL_ECHOLNPGM("PL: erase flash data!");
      FLASH_Unlock();
      FLASH_ErasePage(FLASH_MARLIN_POWERPANIC);
      FLASH_Lock();
      break;
    }
  }
  stash_data.state = PL_NO_DATE;
}

ErrCode PowerLoss::is_power_loss_data() {
  if (stash_data.state == PL_WAIT_RESUME) {
    return E_SUCCESS;
  }
  return PRINT_RESULT_NO_PL_DATA_E;
}

bool PowerLoss::change_head() {
  if (stash_data.print_mode != PRINT_BACKUP_MODE) {
    return false;
  }

  stash_data.active_extruder = !stash_data.active_extruder;
  uint16_t val = stash_data.nozzle_temp[0];
  stash_data.nozzle_temp[0] = stash_data.nozzle_temp[1];
  stash_data.nozzle_temp[1] = val;

  val = stash_data.fan[0][0];
  stash_data.fan[0][0] = stash_data.fan[1][0];
  stash_data.fan[1][0] = val;

  val = stash_data.extruder_temperature_lock[0];
  stash_data.extruder_temperature_lock[0] = stash_data.extruder_temperature_lock[1];
  stash_data.extruder_temperature_lock[1] = val;

  return true;
}

ErrCode PowerLoss::power_loss_resume() {
  if (stash_data.state != PL_WAIT_RESUME || system_service.get_status() != SYSTEM_STATUE_IDLE) {
    return PRINT_RESULT_PL_RESUME_ERR_E;
  }

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_POWER_LOSS_RESUMING, SYSTEM_STATUE_SCOURCE_PL)) {
    LOG_E("can NOT set to SYSTEM_STATUE_POWER_LOSS_RESUMING\r\n");
    return PRINT_RESULT_PL_RESUME_ERR_E;
  }

  home_offset = stash_data.home_offset;
  next_req = cur_line = line_number_sum = stash_data.file_position;
  update_workspace_offset(Z_AXIS);
  clear();
  print_control.set_work_time(stash_data.work_time);
  print_control.mode_ = (print_mode_e)stash_data.print_mode;
  print_control.set_noise_mode(print_noise_mode_e(stash_data.noise_mode & 0xFF));

  // The print needs to be extruded before resuming
  if (extrude_before_resume() != E_SUCCESS) {
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSED)) {
      LOG_E("can NOT set to SYSTEM_STATUE_PAUSED\r\n");
      system_service.return_to_idle();
    }
    return E_SYSTEM_EXCEPTION;
  } else {
    resume_print_env();
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PRINTING)) {
      LOG_E("can NOT set to SYSTEM_STATUE_PRINTING\r\n");
      system_service.return_to_idle();
    }
  }

  return E_SUCCESS;
}

ErrCode PowerLoss::set_file_name(uint8_t *name, uint8_t len) {
  SERIAL_ECHOPGM("set file name:");
  if (len >= GCODE_FILE_NAME_SIZE - 1) {
    len = 0;
    SERIAL_ECHOLN("filed");
    return PRINT_RESULT_NO_FILE_INFO_E;
  } else {
    memcpy(stash_data.gcode_file_name, name, len);
    stash_data.gcode_file_name_len = len;
    stash_data.gcode_file_name[len] = 0;
  }
  SERIAL_ECHOLN((char *)stash_data.gcode_file_name);
  return E_SUCCESS;
}

ErrCode PowerLoss::set_file_md5(uint8_t *md5, uint8_t len) {
  SERIAL_ECHOPGM("set file md5:");
  if (len >= GCODE_MD5_LENGTH -1) {
    len = 0;
    SERIAL_ECHOLN("filed");
    return PRINT_RESULT_NO_FILE_INFO_E;
  } else {
    memcpy(stash_data.gcode_file_md5, md5, len);
    stash_data.gcode_file_md5[len] = 0;
    stash_data.gcode_file_md5_len = len;
  }
  SERIAL_ECHOLN((char *)stash_data.gcode_file_md5);
  return E_SUCCESS;
}

uint8_t * PowerLoss::get_file_name(uint8_t &len) {
  len = stash_data.gcode_file_name_len;
  return stash_data.gcode_file_name;
}

uint8_t * PowerLoss::get_file_md5(uint8_t &len) {
  len = stash_data.gcode_file_md5_len;
  return stash_data.gcode_file_md5;
}

bool PowerLoss::is_power_220v_pin_trigger() {
  return READ(HW_1_2(POWER_LOST_220V_HW1_PIN, POWER_LOST_220V_HW2_PIN)) == POWER_LOSS_220V_TRIGGER_STATUS;
}

bool PowerLoss::is_power_loss_trigger() {
  return is_trigger;
}

bool PowerLoss::check() {
  if (is_trigger && is_inited && power_loss_en && system_service.is_working() && !print_control.is_calibretion_mode) {
      switch (power_loss_status) {
        case POWER_LOSS_TRIGGER:
          close_peripheral_power();
          wait_for_heatup = false;
          stepper.quick_stop();
          power_loss_status = POWER_LOSS_STOP_MOVE;
          return true;
        case POWER_LOSS_STOP_MOVE:
          if (system_service.get_status() == SYSTEM_STATUE_PRINTING) {
            stash_print_env();
          }
          write_flash();
          power_loss_status = POWER_LOSS_WAIT_Z_MOVE;
          return true;
        default:
          break;
      }
  }
  return false;
}

void PowerLoss::close_peripheral_power() {
  OUT_WRITE(HEATER_PWR_PIN, LOW);
  OUT_WRITE(SCREEN_PWR_PIN, LOW);
  OUT_WRITE(HEATER_BED_PWR_PIN, LOW);
  motion_control.motor_disable(X_AXIS, 0);
  motion_control.motor_disable(X_AXIS, 1);
  motion_control.motor_disable(Y_AXIS, 0);
  motion_control.motor_disable(E_AXIS, 0);
  motion_control.motor_disable(E_AXIS, 1);
  enclosure.set_light_power(0);
  enclosure.set_fan_power(0);
}

void PowerLoss::process() {
  static uint32_t trigger_wait_time = 0;

  if (!power_loss.power_loss_en) {
    return;
  }

  if (is_power_220v_pin_trigger()) {
    if (power_loss_status == POWER_LOSS_IDLE) {
      power_loss_status = POWER_LOSS_RECONFIRM;
      trigger_wait_time = millis() + 10;
      // SERIAL_ECHOLNPAIR("power loss first trigger");
    } else if (power_loss_status == POWER_LOSS_RECONFIRM) {
      if (ELAPSED(millis(), trigger_wait_time)) {
        power_loss_status = POWER_LOSS_TRIGGER;
        is_trigger = true;
        // SERIAL_ECHOLNPAIR("trigger power loss");
      }
    } else if (power_loss_status == POWER_LOSS_WAIT_Z_MOVE) {
      power_loss_status = POWER_LOSS_DONE;
      // SERIAL_ECHOLNPAIR("trigger power loss done");
      motion_control.synchronize();
      sync_plan_position();
      motion_control.move_z(POWERLOSS_Z_DOWN_DISTANCE, 600);
      SERIAL_ECHOLNPAIR("power loss kill");
      kill();
    }
  }
  else {
    is_trigger = false;
    power_loss_status = POWER_LOSS_IDLE;
  }
}
