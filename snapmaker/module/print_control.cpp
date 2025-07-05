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

#include "print_control.h"
#include "src/module/motion.h"
#include "src/module/tool_change.h"
#include "src/module/planner.h"
#include "src/gcode/gcode.h"
#include "motion_control.h"
#include "../../Marlin/src/module/temperature.h"
#include "../../Marlin/src/module/settings.h"
#include "../../Marlin/src/module/stepper.h"
#include "../../Marlin/src/feature/tmc_util.h"
#include "system.h"
#include "fdm.h"
#include "power_loss.h"
#include "../module/filament_sensor.h"
#include "exception.h"

bool is_hmi_printing = false;  // Default to false (not HMI)

#define PAUSE_RESUME_MOVE_FEEDRATE_MMM (9000)

PrintControl print_control;


#define GCODE_BUFFER_SIZE (1024*2)
uint16_t buffer_head = 0;
uint16_t buffer_tail = 0;
static uint8_t gcode_buffer[GCODE_BUFFER_SIZE];

void PrintControl::init() {
  print_noise_mode = NOISE_NOIMAL_MODE;
  pnm_param.max_acc = 3000;
  pnm_param.max_speed = 350;
  pnm_param.XY_tmc_stealChop = true;
  noise_mode_apply();
}

bool PrintControl::buffer_is_empty() {
 return buffer_head == buffer_tail && !planner.has_blocks_queued();
}

bool PrintControl::is_backup_mode() {
  return mode_ == PRINT_BACKUP_MODE;
}

void PrintControl::clear_gcode_buf() {
  buffer_head = buffer_tail;
}

uint32_t PrintControl::get_buf_used() {
  return (buffer_head + GCODE_BUFFER_SIZE - buffer_tail) % GCODE_BUFFER_SIZE;
}

uint32_t PrintControl::get_buf_free() {
  return GCODE_BUFFER_SIZE - get_buf_used();
}

uint32_t PrintControl::get_cur_line() {
  return power_loss.cur_line;
}

uint32_t PrintControl::next_req_line() {
  return power_loss.next_req;
}


bool PrintControl::filament_check() {
  if (!is_hmi_printing) return false; // Skip for OctoPrint prints
  bool is_trigger = false;
  if (dual_x_carriage_mode < DXC_DUPLICATION_MODE) {
    is_trigger = filament_sensor.is_trigger(active_extruder);
  } else {
    is_trigger = filament_sensor.is_trigger();
  }
  if (!is_trigger) {
    return false;
  }

  system_status_source_e source = SYSTEM_STATUE_SCOURCE_NONE;
  switch (mode_) {
    case PRINT_FULL_MODE:
    case PRINT_DUPLICATION_MODE:
    case PRINT_MIRRORED_MODE:
      source = SYSTEM_STATUE_SCOURCE_FILAMENT;
      break;
    case PRINT_BACKUP_MODE:
      filament_sensor.reset();
      source = SYSTEM_STATUE_SCOURCE_TOOL_CHANGE;
      break;
  }
  SERIAL_ECHOLNPAIR("flilament trigger and source:", source);
  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSING, source)) {
    LOG_E("can NOT set to SYSTEM_STATUE_PAUSING\r\n");
    system_service.return_to_idle();
  }
  return true;
}

void PrintControl::loop() {
  static uint32_t last_ms = 0;

  if (req_clear_work_time) {
    work_time_ms = 0;
    last_ms = millis();
    req_clear_work_time = false;
  }

  if (req_set_work_time) {
    work_time_ms = set_work_time_ms;
    req_set_work_time = false;
  }

  if (system_service.get_status() == SYSTEM_STATUE_PRINTING) {
    if (ELAPSED(millis(), last_ms + 10)) {
      work_time_ms += (millis() - last_ms);
      last_ms = millis();
    }
  }
}

bool PrintControl::get_commands(uint8_t *cmd, uint32_t &line, uint16_t max_len) {

  if (power_loss.power_loss_status != POWER_LOSS_IDLE) {
    return false;
  }

  if (system_service.get_status() != SYSTEM_STATUE_PRINTING) {
    return false;
  }

  if (filament_check()) {
    return false;
  }

  if(commands_lock_) {
    return false;
  }

  while (buffer_head != buffer_tail) {
    if (gcode_buffer[buffer_tail] == ' ' || gcode_buffer[buffer_tail] == '\n') {
      if (gcode_buffer[buffer_tail] == '\n') {
        power_loss.line_number_sum++;
      }
      buffer_tail = (buffer_tail + 1) % GCODE_BUFFER_SIZE;
    } else {
      break;
    }
  }
  uint32_t get_commands = 0;
  while (buffer_head != buffer_tail) {
    if (get_commands >= max_len) {
      SERIAL_ECHOLNPAIR("cmd too long failed!");
      return false;
    }

    cmd[get_commands] = gcode_buffer[buffer_tail];
    buffer_tail = (buffer_tail + 1) % GCODE_BUFFER_SIZE;

    if (cmd[get_commands] == '\n') {
      cmd[get_commands] = 0;
      power_loss.line_number_sum++;
      line = power_loss.line_number_sum;
      return true;
    }
    get_commands++;
  }
  return false;
}

ErrCode PrintControl::push_gcode(uint32_t start_line, uint32_t end_line, uint8_t *data, uint16_t size) {
  uint8_t gcode_count = 0;
  uint32_t free = get_buf_free();

  if (free < size) {
    SERIAL_ECHOLNPAIR("gcode no memory ,free:", free, " cur:", size);
    return E_NO_MEM;
  }

  for (uint16_t i = 0; i < size; i++) {
    if (data[i] == '\n') {
      gcode_count ++;
    }
  }

  if (power_loss.next_req != start_line) {
    LOG_E("HIM gcode start line is NOT equal req, req %d, get %d\r\n", power_loss.next_req, start_line);
    return E_PARAM;
  }

  if ((end_line - start_line + 1) != gcode_count) {
    SERIAL_ECHOLNPAIR("failed line start:", start_line, " end:", end_line, " count:", gcode_count, " next_req:", power_loss.next_req);
    return E_PARAM;
  }

  for (uint32_t i = 0; i < size; i++) {
    gcode_buffer[buffer_head] = data[i];
    buffer_head = (buffer_head + 1) % GCODE_BUFFER_SIZE;
  }
  power_loss.next_req = end_line + 1;

  return E_SUCCESS;
}

void PrintControl::start_work_time() {
  // work_time_ms = 0;
  req_clear_work_time = true;
  // work_start_time = millis();
}

void PrintControl::stop_work_time() {
  // work_time_ms = work_start_time = 0;
  // work_time_ms = 0;
}


uint32_t PrintControl::get_work_time() {
  if (!system_service.is_working()) {
    return 0;
  }
  else {
    return work_time_ms;
  }
}

void PrintControl::set_work_time(uint32_t time) {
  req_set_work_time = true;
  set_work_time_ms = time;
}

bool PrintControl::set_noise_mode(print_noise_mode_e pnm) {
  if (NOISE_NOIMAL_MODE <= pnm && pnm <= NOISE_LOUD_MODE) {
    print_noise_mode = pnm;
    noise_mode_apply();
    return true;
  }
  else {
    return false;
  }
}

print_noise_mode_e PrintControl::get_noise_mode() {
  return print_noise_mode;
}

void PrintControl::noise_mode_apply() {
  LOG_I("apply print noise mode: %d\n", print_noise_mode);
  switch (print_noise_mode) {
    case NOISE_NOIMAL_MODE:
    /*
    Enable XY TMC stealChop; M203 X200 Y200; M201 X8000 Y8000;
    */
    pnm_param.XY_tmc_stealChop = true;
    pnm_param.max_x_speed = 200;
    pnm_param.max_y_speed = 200;
    pnm_param.max_speed = 350;
    pnm_param.max_acc = 8000;
    break;

    case NOISE_SILENT_MODE:
    /*
    Enable XY TMC stealChop; M203 X105 Y105; M201 X3000 Y3000
    */
    pnm_param.XY_tmc_stealChop = true;
    pnm_param.max_x_speed = 105;
    pnm_param.max_y_speed = 75;
    pnm_param.max_speed = 350;
    pnm_param.max_acc = 3000;
    break;

    case NOISE_LOUD_MODE:
    /*
    Disable XY TMC stealChop(Z enable); M203 X350 Y350; M201 X10000 Y10000
    */
    pnm_param.XY_tmc_stealChop = false;
    pnm_param.max_x_speed = 350;
    pnm_param.max_y_speed = 350;
    pnm_param.max_speed = 350;
    pnm_param.max_acc = 10000;
    break;

    default:
    break;
  }

  tmc_set_stealthChop(X_AXIS, pnm_param.XY_tmc_stealChop); tmc_set_stealthChop(Y_AXIS, pnm_param.XY_tmc_stealChop);
  // planner.set_max_feedrate(X_AXIS, pnm_param.max_speed); planner.set_max_feedrate(Y_AXIS, pnm_param.max_speed);
  // planner.set_max_acceleration(X_AXIS, pnm_param.max_acc); planner.set_max_feedrate(Y_AXIS, pnm_param.max_acc);
}

ErrCode PrintControl::start() {

  if (!exception_server.is_allow_work()) {
    return E_SYSTEM_EXCEPTION;
  }
  if (system_service.is_working()) {
    return PRINT_RESULT_START_ERR_E;
  }
  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PRINTING)) {
    LOG_E("can NOT set to SYSTEM_STATUE_PRINTING\r\n");
    system_service.return_to_idle();
    return PRINT_RESULT_START_ERR_E;
  }
  
  is_hmi_printing = true; // Set for HMI-initiated prints

  if (homing_needed()) {
    motion_control.home();
  }
  if (SYSTEM_STATUE_PRINTING != system_service.get_status()) {
    LOG_I("Work start abort\r\n");
    return PRINT_RESULT_START_ERR_E;
  }

  // prepare toolhead
  dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
  if (mode_ >= PRINT_DUPLICATION_MODE) {
    tool_change(0);
    dual_x_carriage_mode = (DualXMode)mode_;
    duplicate_extruder_x_offset = (dual_x_carriage_mode == DXC_DUPLICATION_MODE) ? \
                                  DUPLICATION_MODE_X_OFFSET : MIRRORED_MODE_X_OFFSET;
    idex_set_mirrored_mode(dual_x_carriage_mode == DXC_MIRRORED_MODE);
    first_start_gcode = true;
    extern bool x_first_move;
    if (DXC_DUPLICATION_MODE == dual_x_carriage_mode)
      x_first_move = false;
    else
      x_first_move = true;
    // motion_control.home_x();
  }

  power_loss.stash_data.file_position = 0;
  power_loss.cur_line = power_loss.line_number_sum = 0;
  power_loss.next_req = 0;
  buffer_head = buffer_tail = 0;
  power_loss.clear();

  filament_sensor.reset();
  memset(&print_err_info, 0, sizeof(print_err_info));
  commands_unlock();
  start_work_time();

  /*
  Allways reset to noimal mode
  */
  print_noise_mode = NOISE_NOIMAL_MODE;
  noise_mode_apply();

  return E_SUCCESS;
}

ErrCode PrintControl::pause() {

  motion_control.wait_G28();

  commands_lock();
  buffer_head = buffer_tail = 0;

  // wait for auto park finish
  while(axisManager.T0_T1_simultaneously_move || axisManager.T0_T1_simultaneously_move_req || tool_changeing) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // wait for stepper slow down and stop
  // LOG_I("--- req_pause\r\n");
  stepper.req_pause = true;

  while(1) {
    if (stepper.can_pause) {
      // LOG_I("--- can_pause\r\n");
      stepper.can_pause = false;
      quickstop_stepper();
      // LOG_I("--- pause done\r\n");
      stepper.delta_t = 0;
      break;
    }
    else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  uint8_t e_en_0 = fdm_head.extraduer_enable(0);
  uint8_t e_en_1 = fdm_head.extraduer_enable(1);

  if (system_service.get_source() == SYSTEM_STATUE_SCOURCE_STOP_EXTRUDE) {
    fdm_head.set_duplication_enabled(fdm_head.stop_single_extruder_e, fdm_head.stop_single_extruder_en);
    print_control.temperature_lock(fdm_head.stop_single_extruder_e, !fdm_head.stop_single_extruder_en);
  }

  vTaskDelay(pdMS_TO_TICKS(5));
  power_loss.stash_print_env();

  if (system_service.get_source() == SYSTEM_STATUE_SCOURCE_Z_LIVE_OFFSET) {
    // motion_control.retrack_e(Z_LIVE_OFFSET_RETRACE_D, CHANGE_FILAMENT_SPEED);
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSED)) {
      LOG_E("can NOT set to SYSTEM_STATUE_PAUSED\r\n");
      system_service.return_to_idle();
      return E_FAILURE;
    }
    else {
      return E_SUCCESS;
    }
  }

  motion_control.retrack_e(PRINT_RETRACK_DISTANCE, CHANGE_FILAMENT_SPEED);
  motion_control.synchronize();

  idex_set_parked(false);
  if (current_position.z + Z_DOWN_SAFE_DISTANCE < Z_MAX_POS) {
    motion_control.move_z(Z_DOWN_SAFE_DISTANCE, PRINT_TRAVEL_FEADRATE);
  } else {
    motion_control.move_to_z(Z_MAX_POS, PRINT_TRAVEL_FEADRATE);
  }
  motion_control.synchronize();

  // motion_control.home_x();
  // motion_control.home_y();

  dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
  set_duplication_enabled(false);

  float active_x_pos = current_position.x;
  uint8_t save_active_extruder = active_extruder;
  uint8_t inactive_extruder = !active_extruder;

  // parked the active extruder
  float x_pack_pos = x_home_pos(active_extruder);
  if (active_extruder ? e_en_1 : e_en_0)
    motion_control.move_to_x(x_pack_pos, PAUSE_RESUME_MOVE_FEEDRATE_MMM);
  else
    current_position.x = x_pack_pos;
  LOG_I("Active extruder:%d mvoe to home %f\r\n", active_extruder, x_pack_pos);

  // parked the inactive extruder
  if (DXC_DUPLICATION_MODE == power_loss.stash_data.dual_x_carriage_mode) {
    if (  (abs(active_x_pos - inactive_extruder_x) - power_loss.stash_data.duplicate_extruder_x_offset) < 0.1 ) {
      if (active_extruder == 0)
        inactive_extruder_x = active_x_pos + power_loss.stash_data.duplicate_extruder_x_offset;
      else
        inactive_extruder_x = active_x_pos - power_loss.stash_data.duplicate_extruder_x_offset;
    }
  }
  else if (DXC_MIRRORED_MODE == power_loss.stash_data.dual_x_carriage_mode) {
    inactive_extruder_x = (x_home_pos(!active_extruder) + x_home_pos(active_extruder)) - active_x_pos;
  }
  LOG_I("inactive_extruder_x %f\r\n", inactive_extruder_x);
  tool_change(inactive_extruder, true);
  x_pack_pos = x_home_pos(inactive_extruder);
  if (inactive_extruder ? e_en_1 : e_en_0)
    motion_control.move_to_x(x_pack_pos, PAUSE_RESUME_MOVE_FEEDRATE_MMM);
  else
    current_position.x = x_pack_pos;
  LOG_I("Inactive extruder:%d mvoe to home %f\r\n", inactive_extruder, x_pack_pos);

  tool_change(save_active_extruder);
  motion_control.move_to_y(0, PAUSE_RESUME_MOVE_FEEDRATE_MMM);

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSED)) {
    LOG_E("can NOT set to SYSTEM_STATUE_PAUSED\r\n");
    system_service.return_to_idle();
    return E_FAILURE;
  }
  else {
    // reset to normal
    print_control.set_noise_mode(NOISE_NOIMAL_MODE);
    return E_SUCCESS;
  }
}

ErrCode PrintControl::resume() {

  buffer_head = buffer_tail = 0;

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_RESUMING)) {
    LOG_E("can NOT set to SYSTEM_STATUE_RESUMING\r\n");
    system_service.return_to_idle();
    return E_FAILURE;
  }

  if (system_service.get_source() == SYSTEM_STATUE_SCOURCE_Z_LIVE_OFFSET) {
    dual_x_carriage_mode = (DualXMode)power_loss.stash_data.dual_x_carriage_mode;
    idex_set_mirrored_mode(dual_x_carriage_mode == DXC_MIRRORED_MODE);

    power_loss.resume_print_env();
    commands_unlock();
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PRINTING)) {
      LOG_E("can NOT set to SYSTEM_STATUE_PRINTING\r\n");
      system_service.return_to_idle();
      return E_FAILURE;
    }
    else {
      return E_SUCCESS;
    }
  }

  if (power_loss.extrude_before_resume() == E_SUCCESS) {
    power_loss.resume_print_env();
    commands_unlock();
    if (SYSTEM_STATUE_RESUMING == system_service.get_status()) {
      if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PRINTING)) {
        LOG_E("can NOT set to SYSTEM_STATUE_PRINTING\r\n");
        system_service.return_to_idle();
        return E_FAILURE;
      }
    }
    return E_SUCCESS;
  }
  else {
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSED)) {
      LOG_E("can NOT set to SYSTEM_STATUE_PAUSED\r\n");
      system_service.return_to_idle();
      return E_FAILURE;
    }
    else {
      return E_SYSTEM_EXCEPTION;
    }
  }
}

ErrCode PrintControl::stop() {
  if (system_service.get_status() != SYSTEM_STATUE_IDLE) {
    motion_control.wait_G28();
    power_loss.clear();

    // wait for auto park finish
    while(axisManager.T0_T1_simultaneously_move || axisManager.T0_T1_simultaneously_move_req || tool_changeing) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    // motion_control.quickstop();
    commands_lock();
    buffer_head = buffer_tail = 0;

    // // set to 0, do not waiting in M109 or M190
    HOTEND_LOOP() {
      thermalManager.setTargetHotend(0, e);
      fdm_head.set_fan_speed(e, 0, 0);
      set_work_flow_percentage(e, 100);
    }
    thermalManager.setTargetBed(0);

    stepper.req_pause = true;
    while(1) {
      if (stepper.can_pause) {
        stepper.can_pause = false;
        quickstop_stepper();
        stepper.delta_t = 0;
        break;
      }
      else {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    buffer_head = buffer_tail = 0;
    is_calibretion_mode = false;
    idex_set_parked(false);
    motion_control.retrack_e(PRINT_RETRACK_DISTANCE, PRINT_TRAVEL_FEADRATE);
    motion_control.home();
    system_service.set_status(SYSTEM_STATUE_IDLE);
    mode_ = PRINT_FULL_MODE;
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    set_duplication_enabled(false);

    HOTEND_LOOP() {
      thermalManager.setTargetHotend(0, e);
      fdm_head.set_fan_speed(e, 0, 0);
       set_work_flow_percentage(e, 100);
    }
    thermalManager.setTargetBed(0);

    set_feedrate_percentage(100);
    set_print_offset(0, 0, 0);
    stop_work_time();
  }
  // reset to normal
  print_control.set_noise_mode(NOISE_NOIMAL_MODE);
  
  // Reset HMI printing flag after print stops
  is_hmi_printing = false;

  return E_SUCCESS;
}

ErrCode PrintControl::set_mode(print_mode_e mode) {
  mode_ = mode;
  return E_SUCCESS;
}

ErrCode PrintControl::set_print_offset(float x, float y, float z) {
  if (system_service.is_working()) {
    SERIAL_ECHOLNPAIR("set print offset filed !");
    return E_COMMON_ERROR;
  }
  xyz_offset.x = x;
  xyz_offset.y = y;
  xyz_offset.z = z;
  return E_SUCCESS;
}

void PrintControl::set_feedrate_percentage(int16_t percentage) {
  feedrate_percentage = percentage;
  power_loss.stash_data.feedrate_percentage = percentage;
}

int16_t PrintControl::get_feedrate_percentage() {
  return feedrate_percentage;
}

int16_t PrintControl::get_work_flow_percentage(uint8_t e) {
  return planner.flow_percentage[e];
}

void PrintControl::set_work_flow_percentage(uint8_t e, int16_t percentage) {
  power_loss.stash_data.flow_percentage[e] = percentage;
  planner.set_flow(e, percentage);
}

void PrintControl::error_and_stop() {
  LOG_E("Print timeout will automatically stop working\n");
  print_err_info.is_err = true;
  print_err_info.err_line = next_req_line();
  LOG_E("timeout line:%d\n", print_err_info.err_line);
  buffer_head = buffer_tail = 0;
  motion_control.quickstop();
  power_loss.stash_print_env();
  power_loss.write_flash();
  motion_control.synchronize();
  motion_control.retrack_e(PRINT_RETRACK_DISTANCE, CHANGE_FILAMENT_SPEED);
  motion_control.home();
  system_service.set_status(SYSTEM_STATUE_IDLE);
  // Reset HMI printing flag after print stops
  is_hmi_printing = false;
}
