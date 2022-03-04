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
  stash_data.print_feadrate = fast_move_feedrate;
  stash_data.axis_relative = gcode.axis_relative;
  HOTEND_LOOP() {
    stash_data.nozzle_temp[e] = thermalManager.degTargetHotend(e);
    for (uint8_t i = 0; i < 2; i++) {
      fdm_head.get_fan_speed(e, i, stash_data.fan[e][i]);
    }
  }
}

void PowerLoss::resume_print_env() {
  thermalManager.setTargetBed(stash_data.bed_temp);
  HOTEND_LOOP() {
    thermalManager.setTargetHotend(stash_data.nozzle_temp[e], e);
    for (uint8_t i = 0; i < 2; i++) {
      fdm_head.set_fan_speed(e, i, stash_data.fan[e][i]);
    }
  }
  thermalManager.wait_for_bed();
  HOTEND_LOOP() {
    thermalManager.wait_for_hotend(e);
  }
  dual_x_carriage_mode = (DualXMode)stash_data.dual_x_carriage_mode;
  next_req = cur_line = line_number_sum = stash_data.file_position;
  motion_control.move_to_xyz(stash_data.position);
  feedrate_mm_s = stash_data.print_feadrate;
  fast_move_feedrate = stash_data.print_feadrate;
  gcode.axis_relative = stash_data.axis_relative;
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
  clear();
  write_flash();
  for (uint32_t i = 0; i < sizeof(power_loss_t); i++) {
    ram_addr[i] = flash_addr[i];
    if (i < sizeof(power_loss_t) - 4) {
      check_num += flash_addr[i];
    }
  }
  if (stash_data.state == PL_WAIT_RESUME) {
    if (check_num == stash_data.check_num) {
      SERIAL_ECHOLNPAIR("PL: Got available data!");
    } else {
      stash_data.state = PL_NO_DATE;
      SERIAL_ECHOLNPAIR("PL: Unavailable data!, checknum:", check_num, "-", stash_data.check_num);
    }  
    show_power_loss_info();
  } else {
    SERIAL_ECHOLNPAIR("PL: No data!", stash_data.bed_temp);
  }
}

void PowerLoss::clear() {
  SERIAL_ECHOLNPGM("PL: clear power loss data!");
  char *flash_addr = (char *)FLASH_MARLIN_POWERPANIC;
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

ErrCode PowerLoss::power_loss_resume() {
  if (stash_data.state != PL_WAIT_RESUME || system_service.get_status() != SYSTEM_STATUE_IDLE) {
    return PRINT_RESULT_PL_RESUME_ERR_E;
  }
  system_service.set_status(SYSTEM_STATUE_RESUMING);
  resume_print_env();
  system_service.set_status(SYSTEM_STATUE_PRINTING);
  return E_SUCCESS;
}

ErrCode PowerLoss::set_file_name(uint8_t *name, uint8_t len) {
  if (len > GCODE_FILE_NAME_SIZE) {
    len = 0;
    return PRINT_RESULT_NO_FILE_INFO_E;
  } else {
    memcpy(stash_data.gcode_file_name, name, len);
    stash_data.gcode_file_name_len = len;
  }
  return E_SUCCESS;
}

ErrCode PowerLoss::set_file_md5(uint8_t *md5, uint8_t len) {
  if (len > GCODE_MD5_LENGTH) {
    len = 0;
    return PRINT_RESULT_NO_FILE_INFO_E;
  } else {
    memcpy(stash_data.gcode_file_md5, md5, len);
    stash_data.gcode_file_md5_len = len;
  }
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

void PowerLoss::check() {
  if (power_loss_en && system_service.get_status() != SYSTEM_STATUE_IDLE) {
    if (READ(POWER_LOST_PIN) == POWER_LOSS_TRIGGER_STATUS) {
      stash_print_env();
      write_flash();
      stepper.quick_stop();
      is_power_loss = true;
    }
  }
}