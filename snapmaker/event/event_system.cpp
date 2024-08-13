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

#include "event_system.h"
#include "event_base.h"
#include "subscribe.h"
#include "../module/fdm.h"
#include "../module/bed_control.h"
#include "../module/system.h"
#include "../module/motion_control.h"
#include "../module/enclosure.h"
#include "event.h"
#include "../debug/debug.h"
#include "src/module/settings.h"
#include "../../../src/module/AxisManager.h"
#include "../module/print_control.h"
#include "../module/factory_data.h"
#include "../module/calibtration.h"


#pragma pack(1)

typedef struct {
  uint8_t axis;
  bool state;
} motor_state_t;

#pragma pack()

static ErrCode subscribe_event(event_param_t& event) {
  event.data[0] = subscribe.enable(event);
  event.length = 1;
  return send_event(event);
}

static ErrCode unsubscribe_event(event_param_t& event) {
  event.data[0] = subscribe.disable(event);
  event.length = 1;
  return send_event(event);
}

static ErrCode run_gcode(event_param_t& event) {
  if (event.length > 96 + 2) {
    LOG_E("Gcode is too large\n");
    event.data[0] = E_PARAM;
  }
  else {
    uint16_t gcode_len = event.data[0] + (event.data[1]<<8);
    event.data[gcode_len + 2] = 0;
    memset(event.data + gcode_len + 2, 0, PACK_PARSE_MAX_SIZE - gcode_len - 2);
    event.data[0] = req_run_gcode((char *)event.data + 2) ? E_SUCCESS : E_COMMON_ERROR;
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode set_log_grade(event_param_t& event) {
  SNAP_DEBUG_SET_LEVEL(event.data[0]);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode req_pc_port_to_gcode(event_param_t& event) {
  LOG_V("SC set pc port to gcode\n");
  event_handler.recv_enable(EVENT_SOURCE_MARLIN, false);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode set_debug_mode(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode factory_reset(event_param_t& event) {
  LOG_I("SC req factory reset\n");
  system_service.factory_reset();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode heart_event(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.data[1] = system_service.get_status();
  event.length = 2;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode retport_log(event_param_t& event) {
  return E_SUCCESS;
}


static ErrCode req_module_info(event_param_t& event) {
  uint8_t *array_count = &event.data[1];
  module_info_t *module_info = (module_info_t *)(event.data + 2);
  event.data[0] = E_SUCCESS;
  SERIAL_ECHOLN("SC req module info");
  uint8_t index = 0;
  fdm_head.get_module_info(0, module_info[index++]);
  fdm_head.get_module_info(1, module_info[index++]);
  bed_control.get_module_info(module_info[index++]);
  enclosure.get_module_info(module_info[index++]);
  *array_count = index;
  event.length = index * sizeof(module_info_t) + 2;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_machine_info(event_param_t& event) {
  SERIAL_ECHOLN("SC req machine info");
  machine_info_t *machine_info = (machine_info_t *)(event.data + 1);
  event.data[0] = E_SUCCESS;
  system_service.get_machine_info(machine_info);
  event.length = sizeof(machine_info_t) + machine_info->version_length + 1;
  uint16_t *sn_lenght = (uint16_t*)&event.data[event.length];
  uint8_t * sn_addr = &event.data[event.length+2];
  uint8_t *temp_sn = system_service.get_sn_addr(sn_lenght);
  LOG_I("SN len: %d - %s\n", *sn_lenght, temp_sn);
  for (uint16_t i = 0; i < *sn_lenght; i++) {
    sn_addr[i+2] = temp_sn[i];
  }
  event.length += *sn_lenght + 2;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_machine_size(event_param_t& event) {
  SERIAL_ECHOLN("SC req machine size");
  event.data[0] = E_SUCCESS;
  machine_size_t *machine_size = (machine_size_t *)(event.data + 1);
  system_service.get_machine_size(machine_size);
  event.length = sizeof(machine_size_t) + 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_save_setting(event_param_t& event) {
  SERIAL_ECHOLN("SC req save setting");
  event.data[0] = E_SUCCESS;
  system_service.save_setting();
  event.length = 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_coordinate_system(event_param_t& event) {
  uint8_t mode = event.data[0];
  coordinate_system_t * info = (coordinate_system_t *)(event.data + 1);
  event.data[0] = E_SUCCESS;
  system_service.get_coordinate_system_info(info, !mode);
  event.length = sizeof(coordinate_system_t) + 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_distance_relative_home(event_param_t& event) {
  float home_pos[AXIS_COUNT];
  float cur_pos[AXIS_COUNT];
  motion_control.get_home_pos(home_pos);
  motion_control.get_xyz_pos(cur_pos);
  event.data[0] = E_SUCCESS;
  event.data[1] = AXIS_COUNT;
  LOG_V("SC req distance relative home:");
  coordinate_info_t * info = (coordinate_info_t *)(event.data + 2);
  uint8_t axis_code[AXIS_COUNT] = {AXIS_X1, AXIS_X2, AXIS_Y1, AXIS_Z1};
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    info[i].axis = axis_code[i];
    info[i].position = FLOAT_TO_INT(cur_pos[i] - home_pos[i]);
    LOG_V(" %d:%.2f", info[i].axis, INT_TO_FLOAT(info[i].position));
  }
  LOG_V("\n");
  event.length = sizeof(coordinate_info_t) * AXIS_COUNT + 2;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode set_coordinate_system(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode set_origin(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode move_relative_home(event_param_t& event) {
  axis_move_t *move = (axis_move_t *)(event.data);
  float distance = INT_TO_FLOAT(move->distance);
  uint16_t speed = *((uint16_t *)(event.data + sizeof(axis_move_t)));
  SERIAL_ECHOLNPAIR("SC move x relative home to ", distance, " F:", speed);
  motion_control.move_x_to_relative_home(distance, speed);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}


#define DEFAULT_IS_TYPE  (1) //EI
#define DEFAULT_IS_DAMP  (0.1)
#pragma pack(1)
typedef struct {
  uint8_t axis;
  float_to_int_t freq;
} axis_inputshaper_t;
#pragma pack(0)

static ErrCode inputshaper_set(event_param_t& event) {
  axis_inputshaper_t *is = (axis_inputshaper_t *)(event.data);
  uint8_t axis = is->axis - 1;
  float freq = INT_TO_FLOAT(is->freq);
  LOG_I("SC set inputshaper, axis: %d, F: %f", axis, freq);
  event.data[0] = axisManager.input_shaper_set(axis, DEFAULT_IS_TYPE, freq, DEFAULT_IS_DAMP);
  event.length = 1;
  extern bool ml_setting_need_save;
  ml_setting_need_save = true;
  // settings.save();
  return send_event(event);
}

static ErrCode inputshaper_get(event_param_t& event) {

  int axis, type; float freq, damp;

  axis = event.data[0] - 1;
  event.data[0] = axisManager.input_shaper_get(axis, type, freq, damp);

  int i_freq = FLOAT_TO_INT(freq);
  event.data[1] = i_freq & 0xff;
  event.data[2] = (i_freq>>8) & 0xff;
  event.data[3] = (i_freq>>16) & 0xff;
  event.data[4] = (i_freq>>24) & 0xff;
  event.length = 5;

  return send_event(event);
}

static ErrCode resonance_compensation_set(event_param_t& event) {

  int axis, type; float freq, damp;

  LOG_I("SC resonance compesation set, turn %s \r\n", event.data[0] ? "on" : "off");

  axis = X_AXIS;
  if (E_SUCCESS != axisManager.input_shaper_get(axis, type, freq, damp)) {
    event.data[0] = E_FAILURE;
    event.length = 1;
    return send_event(event);
  }
  type = event.data[0] ? type : 0;
  if (E_SUCCESS != axisManager.input_shaper_set(axis, type, freq, damp)) {
    event.data[0] = E_FAILURE;
    event.length = 1;
    return send_event(event);
  }

  axis = Y_AXIS;
  if (E_SUCCESS != axisManager.input_shaper_get(axis, type, freq, damp)) {
    event.data[0] = E_FAILURE;
    event.length = 1;
    return send_event(event);
  }
  type = event.data[0] ? type : 0;
  if (E_SUCCESS != axisManager.input_shaper_set(axis, type, freq, damp)) {
    event.data[0] = E_FAILURE;
    event.length = 1;
    return send_event(event);
  }

  extern bool ml_setting_need_save;
  ml_setting_need_save = true;
  // settings.save();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode resonance_compensation_get(event_param_t& event) {

  bool x_on_off, y_on_off;
  int axis, type; float freq, damp;

  axis = X_AXIS;
  if (E_SUCCESS != axisManager.input_shaper_get(axis, type, freq, damp)) {
      LOG_I("SC resonance compesation get E_FAILURE \r\n");
    event.data[0] = E_FAILURE;
    event.length = 2;
    return send_event(event);
  }
  x_on_off = type;

  axis = Y_AXIS;
  if (E_SUCCESS != axisManager.input_shaper_get(axis, type, freq, damp)) {
    LOG_I("SC resonance compesation get E_FAILURE \r\n");
    event.data[0] = E_FAILURE;
    event.length = 2;
    return send_event(event);
  }
  y_on_off = type;

  LOG_I("SC resonance compesation get %d \r\n", x_on_off && y_on_off);

  event.data[0] = E_SUCCESS;
  event.data[1] = x_on_off && y_on_off;
  event.length = 2;
  return send_event(event);

}

static ErrCode set_z_home_sg(event_param_t& event) {
  LOG_I("SC set z home sg: %s \r\n", event.data[0] ? "on" : "off");
  print_control.set_z_home_sg(event.data[0]);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  extern bool ml_setting_need_save;
  ml_setting_need_save = true;
  // settings.save();
  return send_event(event);
}

static ErrCode get_z_home_sg(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.data[1] = print_control.get_z_home_sg();
  event.length = 2;
  return send_event(event);
}

static ErrCode set_build_plate_thickness(event_param_t& event) {
  float_to_int_t rx_bpt;
  memcpy(&rx_bpt, event.data, sizeof(float_to_int_t));
  float bpt = INT_TO_FLOAT(rx_bpt);
  LOG_I("SC set build plate thickness: %f\r\n", bpt);
  if (fd_srv.setBuildPlateThickness(bpt) && fd_srv.save()) {
    calibtration.updateBuildPlateThicknessWithHomeOffset(bpt);
    event.data[0] = E_SUCCESS;
  }
  else {
    event.data[0] = E_PARAM;
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode get_build_plate_thickness(event_param_t& event) {
  int bpt = FLOAT_TO_INT(fd_srv.getBuildPlateThickness());
  event.data[0] = E_SUCCESS;
  event.data[1] = bpt & 0xff;
  event.data[2] = (bpt>>8) & 0xff;
  event.data[3] = (bpt>>16) & 0xff;
  event.data[4] = (bpt>>24) & 0xff;
  event.length = 5;
  return send_event(event);
}

static ErrCode move_relative(event_param_t& event) {
  mobile_instruction_t *move = (mobile_instruction_t *)(event.data);
  if (fdm_head.is_change_filamenter()) {
    event.data[0] = E_COMMON_ERROR;
  } else {
    motion_control.move_axis(move);
    event.data[0] = E_SUCCESS;
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode move(event_param_t& event) {
  mobile_instruction_t *move = (mobile_instruction_t *)(event.data);
  if (fdm_head.is_change_filamenter()) {
    event.data[0] = E_COMMON_ERROR;
  } else {
    motion_control.move_axis_to(move);
    event.data[0] = E_SUCCESS;
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode home(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.length = 1;
  send_event(event);
  if (fdm_head.is_change_filamenter()) {
    event.data[0] = E_COMMON_ERROR;
  } else {
    motion_control.synchronize();
    event.data[0] = E_SUCCESS;
    motion_control.home();
  }
  event.info.attribute = SACP_ATTR_REQ;
  event.info.command_id = SYS_ID_HOME_END;
  event.length = 1;
  return send_event(event);
}

static bool motor_remap_to_marlin(uint8_t sacp_axis, uint8_t &marlin, uint8_t &index) {
  switch (sacp_axis) {
    case AXIS_X1: marlin = X_AXIS; index = 0; return true;
    case AXIS_Y1: marlin = Y_AXIS; index = 0; return true;
    case AXIS_Z1: marlin = Z_AXIS; index = 0; return true;
    case AXIS_X2: marlin = X_AXIS; index = 1; return true;
    case AXIS_E0: marlin = E_AXIS; index = 0; return true;
    case AXIS_E1: marlin = E_AXIS; index = 1; return true;
  }
  return false;
}

static ErrCode get_motor_enable(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC get motor enable state");
  uint8_t axis_count = 0;
  uint8_t axis, index;
  motor_state_t *motor_state = (motor_state_t *)(event.data + 2);

  #define GET_AXIS_STATE(Axis) \
    motor_remap_to_marlin(AXIS_ ## Axis, axis, index); \
    motor_state[axis_count].axis = AXIS_##Axis; \
    motor_state[axis_count].state = motion_control.is_motor_enable(axis, index); \
    axis_count++;

  GET_AXIS_STATE(X1);
  GET_AXIS_STATE(Y1);
  GET_AXIS_STATE(Z1);
  GET_AXIS_STATE(X2);
  GET_AXIS_STATE(E0);
  GET_AXIS_STATE(E1);

  event.data[0] = E_SUCCESS;
  event.data[1] = axis_count;
  event.length = sizeof(motor_state_t) * axis_count + 2;
  return send_event(event);
}

static ErrCode set_motor_enable(event_param_t& event) {
  uint8_t axis_count = event.data[0];
  SERIAL_ECHOLNPAIR("SC set motor enable and count:", axis_count);
  motor_state_t *motor_state = (motor_state_t *)(event.data + 1);
  for (uint8_t i = 0; i < axis_count; i++) {
    uint8_t axis = motor_state[i].axis;
    uint8_t index = 0;
    uint8_t state = motor_state[i].state;
    if (motor_remap_to_marlin(axis, axis, index)) {
      SERIAL_ECHOLNPAIR("set axis:", axis, " index:", index, " state:", state);
      if (state)
        motion_control.motor_enable(axis, index);
      else
        motion_control.motor_disable(axis, index);
    }
  }
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}


event_cb_info_t system_cb_info[SYS_ID_CB_COUNT] = {
  {SYS_ID_SUBSCRIBE             ,         EVENT_CB_DIRECT_RUN,    subscribe_event},
  {SYS_ID_UNSUBSCRIBE           ,         EVENT_CB_DIRECT_RUN,    unsubscribe_event},
  {SYS_ID_RUN_GCODE             ,         EVENT_CB_DIRECT_RUN,    run_gcode},
  {SYS_ID_SET_LOG_GRADE         ,         EVENT_CB_DIRECT_RUN,    set_log_grade},
  {SYS_ID_PC_PORT_TO_GCODE      ,         EVENT_CB_DIRECT_RUN,    req_pc_port_to_gcode},
  {SYS_ID_SET_DEBUG_MODE        ,         EVENT_CB_DIRECT_RUN,    set_debug_mode},
  {SYS_ID_FACTORY_RESET         ,         EVENT_CB_TASK_RUN  ,    factory_reset},
  {SYS_ID_HEARTBEAT             ,         EVENT_CB_DIRECT_RUN,    heart_event},
  {SYS_ID_REPORT_LOG            ,         EVENT_CB_DIRECT_RUN,    retport_log},
  {SYS_ID_REQ_MODULE_INFO       ,         EVENT_CB_DIRECT_RUN,    req_module_info},
  {SYS_ID_REQ_MACHINE_INFO      ,         EVENT_CB_DIRECT_RUN,    req_machine_info},
  {SYS_ID_REQ_MACHINE_SIZE      ,         EVENT_CB_DIRECT_RUN,    req_machine_size},
  {SYS_ID_SAVE_SETTING          ,         EVENT_CB_DIRECT_RUN,    req_save_setting},
  {SYS_ID_REQ_COORDINATE_SYSTEM ,         EVENT_CB_DIRECT_RUN,    req_coordinate_system},
  {SYS_ID_SET_COORDINATE_SYSTEM ,         EVENT_CB_DIRECT_RUN,    set_coordinate_system},
  {SYS_ID_SET_ORIGIN            ,         EVENT_CB_DIRECT_RUN,    set_origin},
  {SYS_ID_MOVE_RELATIVE         ,         EVENT_CB_TASK_RUN  ,    move_relative},
  {SYS_ID_MOVE                  ,         EVENT_CB_TASK_RUN  ,    move},
  {SYS_ID_HOME                  ,         EVENT_CB_TASK_RUN  ,    home},
  {SYS_ID_GET_MOTOR_ENABLE      ,         EVENT_CB_DIRECT_RUN,    get_motor_enable},
  {SYS_ID_SET_MOTOR_ENABLE      ,         EVENT_CB_DIRECT_RUN,    set_motor_enable},
  {SYS_ID_MOVE_TO_RELATIVE_HOME ,         EVENT_CB_TASK_RUN  ,    move_relative_home},

  {SYS_ID_INPUTSHAPER_SET ,               EVENT_CB_TASK_RUN,      inputshaper_set},
  {SYS_ID_INPUTSHAPER_GET ,               EVENT_CB_TASK_RUN,      inputshaper_get},
  {SYS_ID_RESONANCE_COMPENSATION_SET ,    EVENT_CB_TASK_RUN,      resonance_compensation_set},
  {SYS_ID_RESONANCE_COMPENSATION_GET ,    EVENT_CB_TASK_RUN,      resonance_compensation_get},
  {SYS_ID_SET_Z_HOME_SG ,                 EVENT_CB_TASK_RUN,      set_z_home_sg},
  {SYS_ID_GET_Z_HOME_SG ,                 EVENT_CB_TASK_RUN,      get_z_home_sg},
  {SYS_ID_SET_BUILD_PLATE_TKNESS ,        EVENT_CB_TASK_RUN,      set_build_plate_thickness},
  {SYS_ID_GET_BUILD_PLATE_TKNESS ,        EVENT_CB_TASK_RUN,      get_build_plate_thickness},
  {SYS_ID_GET_DISTANCE_RELATIVE_HOME ,    EVENT_CB_TASK_RUN,      req_distance_relative_home},
  {SYS_ID_SUBSCRIBE_MOTOR_ENABLE_STATUS , EVENT_CB_DIRECT_RUN,    get_motor_enable},
};