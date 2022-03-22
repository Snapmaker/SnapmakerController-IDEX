#include "power_loss.h"
#include "print_control.h"
#include "src/module/motion.h"
#include "src/module/tool_change.h"
#include "src/module/planner.h"
#include "src/gcode/gcode.h"
#include "src/module/stepper.h"
#include "motion_control.h"
#include "../../Marlin/src/module/temperature.h"
#include "system.h"
#include "fdm.h"
#include "HAL.h"
#include <EEPROM.h>


PowerLoss power_loss;

extern feedRate_t fast_move_feedrate;

void PowerLoss::stash_print_env() {
  uint32_t cur_line = print_control.get_cur_line();
  stash_data.file_position = cur_line ? cur_line - 1 : 0;  // The requested index starts at 0
  stash_data.position = current_position;
  stash_data.dual_x_carriage_mode = dual_x_carriage_mode;
  stash_data.bed_temp = thermalManager.degTargetBed();
  stash_data.print_feadrate = feedrate_mm_s;
  stash_data.active_extruder = active_extruder;
  stash_data.travel_feadrate = fast_move_feedrate;
  stash_data.axis_relative = gcode.axis_relative;
  stash_data.print_mode = print_control.mode_;
  stash_data.duplicate_extruder_x_offset = duplicate_extruder_x_offset;
  HOTEND_LOOP() {
    stash_data.nozzle_temp[e] = thermalManager.degTargetHotend(e);
    stash_data.extruder_dual_enable[e] = fdm_head.is_duplication_enabled(e);
    stash_data.extruder_temperature_lock[e] = print_control.temperature_lock(e);
    for (uint8_t i = 0; i < 2; i++) {
      fdm_head.get_fan_speed(e, i, stash_data.fan[e][i]);
    }
  }
}

void PowerLoss::extrude_before_resume() {
  HOTEND_LOOP() {
    fdm_head.set_duplication_enabled(e, true);
    print_control.temperature_lock(e, stash_data.extruder_temperature_lock[e]);
  }
  if (stash_data.dual_x_carriage_mode >= DXC_DUPLICATION_MODE) {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    tool_change(0);
    // Use mirror mode to facilitate both heads extruding together
    dual_x_carriage_mode = DXC_MIRRORED_MODE;
    duplicate_extruder_x_offset = MIRRORED_MODE_X_OFFSET;
    idex_set_mirrored_mode(true);
  } else {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    tool_change(stash_data.active_extruder);
  }

  thermalManager.setTargetBed(stash_data.bed_temp);
  HOTEND_LOOP() {
    uint16_t temp = 0;
    if ((dual_x_carriage_mode == DXC_MIRRORED_MODE) || (e == stash_data.active_extruder)) {
      temp = stash_data.nozzle_temp[e] > PREHEAT_1_TEMP_HOTEND? \
              stash_data.nozzle_temp[e] : PREHEAT_1_TEMP_HOTEND;
    }
    thermalManager.setTargetHotend(temp, e);
    for (uint8_t i = 0; i < 2; i++) {
      fdm_head.set_fan_speed(e, i, stash_data.fan[e][i]);
    }
  }
  if (homing_needed()) {
    motion_control.home();
  } else {
    motion_control.home_x();
  }
  HOTEND_LOOP() {
    thermalManager.wait_for_hotend(e);
  }
    
  int16_t move_dis = EXTRUDE_X_MOVE_DISTANCE;
  if (active_extruder) {
    move_dis = -move_dis;
  }
  prepare_line_to_destination();
  motion_control.move_x(move_dis, PRINT_TRAVEL_FEADRATE);
  motion_control.synchronize();
  motion_control.extrude_e(EXTRUDE_E_DISTANCE, PRINT_RETRACK_SPEED);
  motion_control.synchronize();
}

void PowerLoss::resume_print_env() {
  // The print needs to be extruded before resuming
  extrude_before_resume();
  // resume dual_x_carriage_mode
  dual_x_carriage_mode = (DualXMode)stash_data.dual_x_carriage_mode;
  idex_set_mirrored_mode(dual_x_carriage_mode == DXC_MIRRORED_MODE);

  motion_control.home_x();
  motion_control.home_y();

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

  }
  thermalManager.wait_for_bed();

  feedrate_mm_s = stash_data.print_feadrate;
  fast_move_feedrate = stash_data.travel_feadrate;
  gcode.axis_relative = stash_data.axis_relative;
  duplicate_extruder_x_offset = stash_data.duplicate_extruder_x_offset;
  next_req = cur_line = line_number_sum = stash_data.file_position;
  print_control.mode_ = (print_mode_e)stash_data.print_mode;
  prepare_line_to_destination();
  motion_control.move_to_xyz(stash_data.position, PRINT_TRAVEL_FEADRATE);
  motion_control.synchronize();
  current_position.e = stash_data.position.e;
  sync_plan_position();
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

  SET_INPUT_PULLUP(POWER_LOST_PIN);

  for (uint32_t i = 0; i < sizeof(power_loss_t); i++) {
    ram_addr[i] = flash_addr[i];
    if (i < sizeof(power_loss_t) - 4) {
      check_num += flash_addr[i];
    }
  }
  if (stash_data.state == PL_WAIT_RESUME) {
    if (check_num == stash_data.check_num) {
      SERIAL_ECHOLNPAIR("PL: Got available data!");
      show_power_loss_info();
    } else {
      stash_data.state = PL_NO_DATE;
      SERIAL_ECHOLNPAIR("PL: Unavailable data!, checknum:", check_num, "-", stash_data.check_num);
    }  
  } else {
    SERIAL_ECHOLNPAIR("PL: No data!");
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

ErrCode PowerLoss::power_loss_status() {
  if (stash_data.state == PL_WAIT_RESUME) {
    return E_SUCCESS;
  }
  return PRINT_RESULT_NO_PL_DATA_E;
}

bool PowerLoss::change_head() {
  if (stash_data.print_mode != PRINT_AUTO_PARK_MODE) {
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
  system_service.set_status(SYSTEM_STATUE_RESUMING, SYSTEM_STATUE_SCOURCE_PL);
  resume_print_env();
  system_service.set_status(SYSTEM_STATUE_PRINTING);
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

bool PowerLoss::is_power_pin_trigger() {
  return READ(POWER_LOST_PIN) == POWER_LOSS_TRIGGER_STATUS;
}

void PowerLoss::check() {
  if (is_inited && power_loss_en && system_service.is_working()) {
    if (is_power_pin_trigger()) {
      stash_print_env();
      write_flash();
      wait_for_heatup = false;
      stepper.quick_stop();
      is_power_loss = true;
    }
  }
}