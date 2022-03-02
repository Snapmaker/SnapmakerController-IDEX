#include "print_control.h"
#include "src/module/motion.h"
#include "src/module/tool_change.h"
#include "src/module/planner.h"
#include "src/gcode/gcode.h"
#include "motion_control.h"
#include "../../Marlin/src/module/temperature.h"
#include "system.h"
#include "fdm.h"


PrintControl print_control;


#define GCODE_BUFFER_SIZE (1024*2)
uint16_t buffer_head = 0;
uint16_t buffer_tail = 0;
static uint8_t gcode_buffer[GCODE_BUFFER_SIZE];
power_loss_t power_loss_data;

void PrintControl::stash_print_env() {
  power_loss_data.file_position = cur_line ? cur_line - 1 : 0;  // The requested index starts at 0
  power_loss_data.position = current_position;
  power_loss_data.dual_x_carriage_mode = dual_x_carriage_mode;
  power_loss_data.bed_temp = thermalManager.degTargetBed();
  HOTEND_LOOP() {
    power_loss_data.nozzle_temp[e] = thermalManager.degTargetHotend(e);
    for (uint8_t i = 0; i < 2; i++) {
      fdm_head.get_fan_speed(e, i, power_loss_data.fan[e][i]);
    }
  }
}

void PrintControl::resume_print_env() {
  thermalManager.setTargetBed(power_loss_data.bed_temp);
  HOTEND_LOOP() {
    thermalManager.setTargetHotend(power_loss_data.nozzle_temp[e], e);
    for (uint8_t i = 0; i < 2; i++) {
      fdm_head.set_fan_speed(e, i, power_loss_data.fan[e][i]);
    }
  }
  thermalManager.wait_for_bed();
  HOTEND_LOOP() {
    thermalManager.wait_for_hotend(e);
  }
  dual_x_carriage_mode = (DualXMode)power_loss_data.dual_x_carriage_mode;
  next_req = cur_line = line_number_sum = power_loss_data.file_position;
  motion_control.move_to_xyz(power_loss_data.position);
}

void PrintControl::init() {

}

bool PrintControl::buffer_is_empty() {
 return buffer_head == buffer_tail && !planner.has_blocks_queued();
}

bool PrintControl::is_auto_pack_mode() {
  return dual_x_carriage_mode == DXC_AUTO_PARK_MODE;
}

uint32_t PrintControl::get_buf_used() {
  return (buffer_head + GCODE_BUFFER_SIZE - buffer_tail) % GCODE_BUFFER_SIZE;
}

uint32_t PrintControl::get_buf_free() {
  return GCODE_BUFFER_SIZE - get_buf_used();
}

uint32_t PrintControl::get_cur_line() {
  return cur_line;
}

uint32_t PrintControl::next_req_line() {
  return next_req;
}



bool PrintControl::get_commands(uint8_t *cmd, uint32_t &line, uint16_t max_len) {
  while (buffer_head != buffer_tail) {
    if (gcode_buffer[buffer_head] == ' ' || gcode_buffer[buffer_head] == '\n') {
      if (gcode_buffer[buffer_head] == '\n') {
        line_number_sum++;
      }
      buffer_head = (buffer_head + 1) % GCODE_BUFFER_SIZE;
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

    cmd[get_commands] = gcode_buffer[buffer_head];
    buffer_head = (buffer_head + 1) % GCODE_BUFFER_SIZE;

    if (cmd[get_commands] == '\n') {
      cmd[get_commands] = 0;
      line_number_sum++;
      line = line_number_sum;
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
  if ((end_line - start_line + 1) != gcode_count) {
    SERIAL_ECHOLNPAIR("failed line start:", start_line, " end:", end_line, " count:", gcode_count, " next_req:", next_req);
    return E_PARAM;
  }
  for (uint32_t i = 0; i < size; i++) {
    gcode_buffer[buffer_tail] = data[i];
    buffer_tail = (buffer_tail + 1) % GCODE_BUFFER_SIZE;
  }
  next_req = end_line + 1;

  return E_SUCCESS;
}

ErrCode PrintControl::start() {
  if (system_service.get_status() != SYSTEM_STATUE_IDLE) {
    return PRINT_RESULT_START_ERR_E;
  } else if (homing_needed()) {
    return PRINT_RESULT_NO_HOME_E;
  }
  cur_line = line_number_sum = 0;
  next_req = 0;
  buffer_head = buffer_tail = 0;
  system_service.set_status(SYSTEM_STATUE_PRINTING);
  return E_SUCCESS;
}

ErrCode PrintControl::pause() {
  if (system_service.get_status() != SYSTEM_STATUE_PRINTING) {
    return PRINT_RESULT_PAUSE_ERR_E;
  }
  system_service.set_status(SYSTEM_STATUE_PAUSING);
  buffer_head = buffer_tail = 0;
  quickstop_stepper();
  stash_print_env();
  if (current_position.z + 5 < Z_MAX_POS) {
    motion_control.move_z(5);
  } else {
    motion_control.move_to_z(Z_MAX_POS);
  }
  system_service.set_status(SYSTEM_STATUE_PAUSED);
  return E_SUCCESS;
}

ErrCode PrintControl::resume() {
  if (system_service.get_status() != SYSTEM_STATUE_PAUSED) {
    return PRINT_RESULT_RESUME_ERR_E;
  }
  system_service.set_status(SYSTEM_STATUE_RESUMING);
  resume_print_env();
  system_service.set_status(SYSTEM_STATUE_PRINTING);
  return E_SUCCESS;
}

ErrCode PrintControl::stop() {
  if (system_service.get_status() != SYSTEM_STATUE_IDLE) {
    power_loss_data.gcode_file_md5_len = 0;
    power_loss_data.gcode_file_name_len = 0;
    quickstop_stepper();
    buffer_head = buffer_tail = 0;
    motion_control.home_z();
    system_service.set_status(SYSTEM_STATUE_IDLE);
  }
  return E_SUCCESS;
}

ErrCode PrintControl::power_loss_status() {
  return PRINT_RESULT_NO_PL_DATA_E;
}

ErrCode PrintControl::power_loss_resume() {
  return PRINT_RESULT_PL_RESUME_ERR_E;
}

ErrCode PrintControl::set_file_name(uint8_t *name, uint8_t len) {
  if (len > GCODE_FILE_NAME_SIZE) {
    len = 0;
    return PRINT_RESULT_NO_FILE_INFO_E;
  } else {
    memcpy(power_loss_data.gcode_file_name, name, len);
    power_loss_data.gcode_file_name_len = len;
  }
  return E_SUCCESS;
}

ErrCode PrintControl::set_file_md5(uint8_t *md5, uint8_t len) {
  if (len > GCODE_MD5_LENGTH) {
    len = 0;
    return PRINT_RESULT_NO_FILE_INFO_E;
  } else {
    memcpy(power_loss_data.gcode_file_md5, md5, len);
    power_loss_data.gcode_file_md5_len = len;
  }
  return E_SUCCESS;
}

ErrCode PrintControl::set_mode(print_mode_e mode) {
  if (system_service.get_status() != SYSTEM_STATUE_IDLE) {
    if (dual_x_carriage_mode > DXC_AUTO_PARK_MODE || mode > PRINT_AUTO_PARK_MODE) {
      SERIAL_ECHOLNPAIR("system status-", system_service.get_status(), " no support ", dual_x_carriage_mode, " to ", mode);
      return PRINT_RESULT_SER_MODE_ERR_E;
    }
  }

  planner.synchronize();
  bool is_duplication_enable = false;
  dual_x_carriage_mode = (DualXMode)mode;
  idex_set_mirrored_mode(false);

  switch (dual_x_carriage_mode) {

    case DXC_FULL_CONTROL_MODE:
    case DXC_AUTO_PARK_MODE:
      break;

    case DXC_DUPLICATION_MODE:
    case DXC_MIRRORED_MODE:
      // Set the X offset, but no less than the safety gap
      duplicate_extruder_x_offset = dual_x_carriage_mode == DXC_DUPLICATION_MODE ? \
                  (X1_MAX_POS - X1_MIN_POS) / 2 : (X2_MAX_POS - X1_MIN_POS - HOMING_X_POX_TO_ENDSTOP);
      // Always switch back to tool 0
      if (active_extruder != 0) tool_change(0);
      is_duplication_enable = true;
      break;
    default:
      dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
      break;
  }

  idex_set_parked(is_duplication_enable);
  set_duplication_enabled(false);
  if (is_duplication_enable) {
    motion_control.home_x();
    dual_x_carriage_unpark();
  }
  return E_SUCCESS;
}

uint8_t * PrintControl::get_file_name(uint8_t &len) {
  len = power_loss_data.gcode_file_name_len;
  return power_loss_data.gcode_file_name;
}

uint8_t * PrintControl::get_file_md5(uint8_t &len) {
  len = power_loss_data.gcode_file_md5_len;
  return power_loss_data.gcode_file_md5;
}
