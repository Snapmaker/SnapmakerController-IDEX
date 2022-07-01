/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
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

#include "debug.h"
#include "../event/event_base.h"
#include "../event/event_system.h"
#include "../module/fdm.h"
#include "../module/bed_control.h"
#include "../module/filament_sensor.h"
#include "../module/power_loss.h"
#include "../module/exception.h"
#include "../module/motion_control.h"
#include "../j1/switch_detect.h"
#include "../module/enclosure.h"
#include "../../Marlin/src/inc/Version.h"
#include "../../Marlin/src/module/motion.h"
#include "../../Marlin/src/module/settings.h"
#include "../../Marlin/src/module/temperature.h"
#include "../../Marlin/src/module/planner.h"

#if (SNAP_DEBUG == 1)

SnapDebug debug;

#if defined (__GNUC__)                /* GNU GCC Compiler */
  /* the version of GNU GCC must be greater than 4.x */
  typedef __builtin_va_list       __gnuc_va_list;
  typedef __gnuc_va_list          va_list;
  #define va_start(v,l)           __builtin_va_start(v,l)
  #define va_end(v)               __builtin_va_end(v)
  #define va_arg(v,l)             __builtin_va_arg(v,l)
#else
  #error "Snap debug only support GNU compiler for now"
#endif

static debug_level_e  debug_msg_level = SNAP_DEBUG_LEVEL_VERBOSE;
static char log_buf[SNAP_LOG_BUFFER_SIZE + 2];

const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_TRACE_STR,
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};

// output debug message, will not output message whose level
// is less than msg_level
// param:
//    level - message level
//    fmt - format of messages
//    ... - args
void SnapDebug::Log(debug_level_e level, const char *fmt, ...) {
  va_list args;

  if (level < debug_msg_level)
    return;

  va_start(args, fmt);
  char * data = log_buf + 4;
  vsnprintf(data, SNAP_LOG_BUFFER_SIZE - 4, fmt, args);
  log_buf[0] = E_SUCCESS;
  log_buf[1] = level;
  uint16_t *data_len = ((uint16_t *)(&log_buf[2]));
  *data_len = strlen(data);
  va_end(args);

  SACP_head_base_t sacp = {SACP_ID_HMI, SACP_ATTR_ACK, 0, COMMAND_SET_SYS, SYS_ID_REPORT_LOG};
  send_event(EVENT_SOURCE_HMI, sacp, (uint8_t*)log_buf, *data_len + 4);
  if (!evevnt_serial[EVENT_SOURCE_MARLIN]->enable_sacp()) {
    send_data(EVENT_SOURCE_MARLIN, (uint8_t *)data, *data_len);
  } else {
    sacp.recever_id = SACP_ID_PC;
    send_event(EVENT_SOURCE_MARLIN, sacp, (uint8_t*)log_buf, *data_len + 4);
  }
}


// set current debug level message level less than this level
// will not be outputed, set by M2000
void SnapDebug::set_level(debug_level_e l) {
  Log(SNAP_DEBUG_LEVEL_INFO, "old debug level: %d\n", debug_msg_level);

  if (l > SNAP_DEBUG_LEVEL_MAX)
    return;

  debug_msg_level = l;
  Log(SNAP_DEBUG_LEVEL_INFO, "new debug level: %d\n", debug_msg_level);
}

debug_level_e SnapDebug::get_level() {
  return debug_msg_level;
}

void SnapDebug::show_all_status() {

  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "J1 version:%s\n", J1_BUILD_VERSION);
  SERIAL_ECHO(log_buf);
  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "HW version:%d\n", system_service.get_hw_version());
  SERIAL_ECHO(log_buf);
  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "active_extruder:%d\n", active_extruder);
  SERIAL_ECHO(log_buf);

  extruder_info_t extruder0_info, extruder1_info;
  fdm_head.get_extruder_info(0, &extruder0_info);
  fdm_head.get_extruder_info(1, &extruder1_info);

  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "nozzle id T0:%d T1:%d\n", extruder0_info.type, extruder1_info.type);
  SERIAL_ECHO(log_buf);

  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "nozzle 0 temperature cur:%.2f target:%.2f\n", INT_TO_FLOAT(extruder0_info.cur_temp), INT_TO_FLOAT(extruder0_info.target_temp));
  SERIAL_ECHO(log_buf);

  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "nozzle 1 temperature cur:%.2f target:%.2f\n", INT_TO_FLOAT(extruder1_info.cur_temp), INT_TO_FLOAT(extruder1_info.target_temp));
  SERIAL_ECHO(log_buf);

  bed_control_info_t bed_info;
  bed_control.get_info(bed_info);
  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "bed temperature cur:%.2f target:%d\n", INT_TO_FLOAT(bed_info.cur_temp), bed_info.target_temp);
  SERIAL_ECHO(log_buf);

  SERIAL_ECHOLNPAIR("filament check enable T0:", filament_sensor.filament_param.enabled[0], ", T1:", filament_sensor.filament_param.enabled[1]);
  SERIAL_ECHOLNPAIR("filament trigger status T0:", filament_sensor.is_trigger(0), ", T1:", filament_sensor.is_trigger(1));
  SERIAL_ECHOLNPAIR("filament check param - diatance:", filament_sensor.filament_param.distance,
                    ", threshold:", filament_sensor.filament_param.threshold,
                    ", times:", filament_sensor.filament_param.check_times);
  SERIAL_ECHOLNPAIR("filament sensor value T0:", filament_sensor.get_adc_val(0),
                    ", T1:", filament_sensor.get_adc_val(1));
  
  SERIAL_ECHOLNPAIR("Homed X:", axis_was_homed(X_AXIS), " Y:", axis_was_homed(Y_AXIS), " Z:", axis_was_homed(Z_AXIS));
  SERIAL_ECHOLNPAIR("X home pos T0:", x_home_pos(0), " T1: ", x_home_pos(1));

  xyze_pos_t logical_position = current_position.asLogical();
  SERIAL_ECHOLNPAIR("logical position X:", logical_position.x, " Y:", logical_position.y, " Z:", logical_position.z, " E:", logical_position.e);
  SERIAL_ECHOLNPAIR("native position X:", current_position.x, " Y:", current_position.y, " Z:", current_position.z, " E:", current_position.e);
  SERIAL_ECHOLNPAIR("inactive_extruder_x:", inactive_extruder_x);
  SERIAL_ECHOLNPAIR("T1 hotend_offset X:", hotend_offset[1][X_AXIS], " Y:", hotend_offset[1][Y_AXIS], " Z:", hotend_offset[1][Z_AXIS]);
  SERIAL_ECHOLNPAIR("dual_x_carriage_mode:", dual_x_carriage_mode);

  SERIAL_ECHOLNPAIR("light status:", enclosure.get_light_power());
  uint8_t t0_module_fan_speed, t1_module_fan_speed;
  fdm_head.get_fan_speed(0, 0, t0_module_fan_speed);
  fdm_head.get_fan_speed(1, 0, t1_module_fan_speed);
  SERIAL_ECHOLNPAIR("module fan T0:", t0_module_fan_speed, " T1:", t1_module_fan_speed);
  SERIAL_ECHOLNPAIR("nozzle fan T0:",READ(E0_AUTO_FAN_PIN), " T1:", READ(E1_AUTO_FAN_PIN));
  SERIAL_ECHOLNPAIR("underframe temperature:", (float)thermalManager.degChamber()," fan:",READ(CHAMBER_AUTO_FAN_PIN));
  SERIAL_ECHOLNPAIR("power loss status 220V:", power_loss.is_power_220v_pin_trigger()," 24V:",READ(CHAMBER_AUTO_FAN_PIN));
  SERIAL_ECHOLNPAIR("synchronize status:", planner.has_blocks_queued());
  SERIAL_ECHOLNPAIR("feedrate mm_s:", feedrate_mm_s, " mm_min:", feedrate_mm_s * 60, " reality mm_s:", motion_control.get_feedrate());
  SERIAL_ECHOLNPAIR("probe status T0:", switch_detect.read_e0_probe_status(), " T1:", switch_detect.read_e1_probe_status());
  #define ENDSTOP_STATUS(S) (READ(S##_PIN) != S##_ENDSTOP_INVERTING)
  SERIAL_ECHOLNPAIR("endstop x_min:", ENDSTOP_STATUS(X_MIN), " x_max:", ENDSTOP_STATUS(X_MAX), " y_min:", ENDSTOP_STATUS(Y_MIN), " z_max:", ENDSTOP_STATUS(Z_MAX));
  snprintf(log_buf, SNAP_LOG_BUFFER_SIZE, "exception_status:0x%x behavior:0x%x\n", (unsigned int)exception_server.get_exception(), (unsigned int)exception_server.get_behavior());
  SERIAL_ECHO(log_buf);
}

#endif // #if (SNAP_DEBUG == 1)
