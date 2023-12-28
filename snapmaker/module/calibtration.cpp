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

#include "calibtration.h"
#include "src/module/settings.h"
#include "../J1/switch_detect.h"
#include "src/module/motion.h"
#include "motion_control.h"
#include "src/module/stepper.h"
#include "src/module/endstops.h"
#include "src/module/tool_change.h"
#include "../../Marlin/src/module/temperature.h"
#include "print_control.h"
#include "power_loss.h"
#include "system.h"

Calibtration calibtration;
planner_settings_t planner_backup_setting;

static uint8_t probe_sg_reg[3] = {20, 20, 80};  // X Y Z1
static float build_plate_thickness = 5.0;

#define Y_POS_DIFF 4
//  ______________________
// |                      |
// | [4]     [1]     [5]  |
// |                      |
// |         [0]          |
// |                      |
// | [2]             [3]  |
// |______________________|

#define HALF_BED_X (X_BED_SIZE / 2)
#define HALF_BED_Y (Y_BED_SIZE / 2)

#define POS_X_L_DIFF 115
#define POS_X_R_DIFF 115
#define POS_Y_U_DIFF 86
#define POS_Y_D_DIFF 94

static float calibration_position_xy[6][2] = {
  /*CAlIBRATION_POS_0*/ {HALF_BED_X, HALF_BED_Y},
  /*CAlIBRATION_POS_1*/ {HALF_BED_X, HALF_BED_Y + POS_Y_U_DIFF},
  /*CAlIBRATION_POS_2*/ {HALF_BED_X - POS_X_L_DIFF, HALF_BED_Y - POS_Y_D_DIFF},
  /*CAlIBRATION_POS_3*/ {HALF_BED_X + POS_X_R_DIFF, HALF_BED_Y - POS_Y_D_DIFF},
  /*CAlIBRATION_POS_4*/ {HALF_BED_X - POS_X_L_DIFF, HALF_BED_Y + POS_Y_U_DIFF},
  /*CAlIBRATION_POS_5*/ {HALF_BED_X + POS_X_R_DIFF, HALF_BED_Y + POS_Y_U_DIFF},
};

// x and y compensation
static float heat_bed_center_offset[2] = {CALIBRATION_INVALID_VALUE, CALIBRATION_INVALID_VALUE};


/**
 * @brief Backup offset data for recovery
 *
 */
void Calibtration::backup_offset() {
  LOG_I("Backup offset\r\n");
  HOTEND_LOOP() {
    hotend_offset_backup[e] = hotend_offset[e];
  }
  home_offset_backup = home_offset;
}

void Calibtration::restore_offset() {
  LOG_I("Restore offset\r\n");
  HOTEND_LOOP() {
    hotend_offset[e] = hotend_offset_backup[e];
  }
  home_offset = home_offset_backup;
}

static void set_hotend_offsets_to_default() {
  xyz_pos_t hotend0_offset = {0, 0, 0};
  xyz_pos_t hotend1_offset = {X2_MAX_POS - X2_MIN_POS, 0, 0};
  set_hotend_offsets(0, hotend0_offset);
  set_hotend_offsets(1, hotend1_offset);
}

void Calibtration::retrack_e() {
  extrude_e(-CAlIBRATIONIN_RETRACK_E_MM, MOTION_RETRACK_E_FEEDRATE);
  need_extrude = true;
}

void Calibtration::extrude_e(float distance, uint16_t feedrate) {
  tool_change(0, true);
  dual_x_carriage_mode = DXC_DUPLICATION_MODE;
  set_duplication_enabled(true);
  motion_control.extrude_e(distance, feedrate);
  planner.synchronize();
  dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
  set_duplication_enabled(false);
}

void Calibtration::updateBuildPlateThickness(float bpt) {
  build_plate_thickness = bpt;
}

void Calibtration::set_heat_bed_center_offset(const float offset[2]) {
  heat_bed_center_offset[0] = offset[0];
  heat_bed_center_offset[1] = offset[1];
  if(!head_bed_center_offset_check()) {
    // LOG_I("Head bed center offset check failed, reset\r\n");
    head_bed_center_offset_reset();
  }
}

void Calibtration::get_heat_bed_center_offset(float *offset) {
  offset[0] = heat_bed_center_offset[0];
  offset[1] = heat_bed_center_offset[1];
}

bool Calibtration::head_bed_center_offset_check(void) {
  if (fabs(heat_bed_center_offset[0]) > 5.0)
    return false;
  if (fabs(heat_bed_center_offset[1]) > 5.0)
    return false;
  return true;
}

void Calibtration::head_bed_center_offset_reset(void) {
  heat_bed_center_offset[0] = CALIBRATION_INVALID_VALUE;
  heat_bed_center_offset[1] = CALIBRATION_INVALID_VALUE;
}

void Calibtration::X1_standby(void) {
  float x_home_offset_backup = home_offset[X_AXIS];
  home_offset.x = 0;
  if (active_extruder == 0)
    motion_control.move_to_x(X1_STANDBY_POS);
  else {
    tool_change(0, true);
    motion_control.move_to_x(X1_STANDBY_POS);
    tool_change(1, true);
  }
  home_offset.x = x_home_offset_backup;
}

void Calibtration::X2_standby(void) {
  float x_home_offset_backup = home_offset[X_AXIS];
  home_offset.x = 0;
  if (active_extruder == 1)
    motion_control.move_to_x(X2_STANDBY_POS);
  else {
    tool_change(1, true);
    motion_control.move_to_x(X2_STANDBY_POS);
    tool_change(0, true);
  }
  home_offset.x = x_home_offset_backup;
}

void Calibtration::X_standby(void) {
  X2_standby();
  X1_standby();
}

void Calibtration::Y_standby(void) {
  float y_home_offset_backup = home_offset[Y_AXIS];
  home_offset.y = 0;
  motion_control.move_to_y(Y_STANDBY_POS);
  home_offset.x = y_home_offset_backup;
}

void Calibtration::Z_standby(void) {
  // float z_offset_backup;
  // z_offset_backup = home_offset.z;
  // home_offset[Z_AXIS] = 0;
  // motion_control.move_to_z(Z_STANDBY_POS);
  motion_control.logical_move_to_z(Z_STANDBY_POS);
  // set_home_offset(Z_AXIS, z_offset_backup);
}

void Calibtration::Z_prepare(void) {
  float z_offset_backup;
  z_offset_backup = home_offset.z;
  home_offset[Z_AXIS] = 0;
  motion_control.move_to_z(Z_PREPARE_POS);
  set_home_offset(Z_AXIS, z_offset_backup);
}

void Calibtration::bed_preapare(uint8_t extruder_index) {

  remember_feedrate_scaling_off();

  if (hotend_offset[1][X_AXIS] < X2_MIN_HOTEND_OFFSET) {
    LOG_I("the hotend offset is too small, it will be reset\n");
    set_hotend_offsets_to_default();
  }

  if(homing_needed()) {
    motion_control.home();
    planner.synchronize();
  }

  if (xy_need_re_home) {
    motion_control.home_x();
    motion_control.home_y();
    xy_need_re_home = false;
  }

  if (z_need_re_home) {
    motion_control.home_z();
    z_need_re_home = false;
  }

  // Switch to single-header mode
  if (dual_x_carriage_mode > DXC_FULL_CONTROL_MODE) {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    set_duplication_enabled(false);
    motion_control.home_x();
  }
  set_duplication_enabled(false);
  Z_prepare();

  float another_x_home_pos = !extruder_index ? x_home_pos(1) : x_home_pos(0);
  float another_x_pos = !extruder_index ? x2_position() : x_position();

  // Make sure the other head is in the home position
  if (another_x_home_pos != another_x_pos) {
    tool_change(!extruder_index, true);
    motion_control.move_to_x(another_x_home_pos, MOTION_TRAVEL_FEADRATE);
  }
  if (active_extruder != extruder_index) {
    tool_change(extruder_index, true);
  }

}

void Calibtration::goto_calibtration_position(uint8_t pos, uint16_t feedrate/* = MOTION_TRAVEL_FEADRATE*/) {

  float x, y;
  if (head_bed_center_offset_check()) {
    x = calibration_position_xy[pos][0] + heat_bed_center_offset[0];
    y = calibration_position_xy[pos][1] + heat_bed_center_offset[1];
  }
  else {
    x = calibration_position_xy[pos][0];
    y = calibration_position_xy[pos][1];
  }
  motion_control.move_to_xy(x, y, feedrate);

}

void probe_axis_move(uint8_t axis, float distance, uint16_t feedrate) {

  if(axis == X_AXIS) {
    motion_control.move_x(distance, feedrate);
  }
  else if(axis == Y_AXIS) {
    motion_control.move_y(distance, feedrate);
  }
  else if(axis == Z_AXIS) {
    motion_control.move_z(distance, feedrate);
  }
  motion_control.synchronize();

}

// bool Calibtration::move_to_sersor_no_trigger(uint8_t axis, float try_distance) {

//   bool probe_status = false;
//   float move_distance = 0;
//   uint32_t count = 0;

//   while (abs(move_distance) < BACK_OFF_DISTANCE) {

//     probe_status = active_extruder ? switch_detect.read_e1_probe_status() : switch_detect.read_e0_probe_status();

//     if (!probe_status) {
//       count++;
//       vTaskDelay(pdMS_TO_TICKS(10));
//       if (count >= 5) {
//         return true;
//       }
//       continue;
//     }
//     else {
//       vTaskDelay(pdMS_TO_TICKS(1));
//       probe_status = active_extruder ? switch_detect.read_e1_probe_status() : switch_detect.read_e0_probe_status();
//       if (probe_status) {
//         count = 0;
//         LOG_I("The Probe sensor is tigger and try back off %.3f mm\n", try_distance);
//         probe_axis_move(axis, try_distance, MOTION_TRAVEL_FEADRATE);
//         move_distance += try_distance;
//       }
//     }

//   }

//   return false;
// }

void set_calibration_move_param() {

  planner_backup_setting = planner.settings;
  planner.settings.min_segment_time_us = DEFAULT_MINSEGMENTTIME;
  planner.settings.acceleration = CALIBRATION_ACC;
  planner.settings.retract_acceleration = CALIBRATION_ACC;
  planner.settings.travel_acceleration = CALIBRATION_ACC;
  planner.settings.min_feedrate_mm_s = feedRate_t(CALIBRATION_FEEDRATE);
  planner.settings.min_travel_feedrate_mm_s = feedRate_t(CALIBRATION_FEEDRATE);

}

void restore_move_param() {
  planner.settings = planner_backup_setting;
}

probe_result_e Calibtration::probe(uint8_t axis, float distance, uint16_t feedrate, bool do_sg/* = true */) {

  bool probe_status;
  // int32_t count;
  // uint32_t trigger_cnt;
  int8_t retry_check_cnt = 5;
  probe_result_e ret = PROBR_RESULT_SUCCESS;
  float pos_before_probe = current_position[axis];

  for (; retry_check_cnt-- > 0; ) {
    probe_status = active_extruder ? switch_detect.read_e1_probe_status() : switch_detect.read_e0_probe_status();
    if (!probe_status)
      break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (retry_check_cnt < 0) {
    LOG_E("probe touch before probe move\r\n");
    return PROBR_RESULT_SENSOR_ERROR;
  }

  // if (probe_status) {
  //   vTaskDelay(pdMS_TO_TICKS(10));
  //   probe_status = active_extruder ? switch_detect.read_e1_probe_status() : switch_detect.read_e0_probe_status();
  //   if (probe_status) {
  //     LOG_E("probe touch before probe move\r\n");
  //     return PROBR_RESULT_SENSOR_ERROR;
  //   }
  // }

  set_calibration_move_param();

  LOG_I("probe: distance %f, feed_rate %d, do_sg %d\r\n", distance, feedrate, do_sg);
  if (do_sg) {
    motion_control.clear_trigger();
    uint16_t sg_value = probe_sg_reg[axis];
    if (Z_AXIS == axis) {
      extern uint16_t z_sg_value;
      if (z_sg_value)
        sg_value = z_sg_value;
    }
    else if (Y_AXIS == axis) {
      extern uint16_t y_sg_value;
      if (y_sg_value)
        sg_value = y_sg_value;
    }
    else if (X_AXIS == axis) {
      if (active_extruder == 0) {
        extern uint16_t x0_sg_value;
        if (x0_sg_value)
          sg_value = x0_sg_value;
      }
      else {
        extern uint16_t x1_sg_value;
        if (x1_sg_value)
          sg_value = x1_sg_value;
      }
    }
    motion_control.enable_stall_guard_only_axis(axis, sg_value, active_extruder);
    LOG_I("axis %d(active_extruder = %d) sg_value set to %d\r\n", axis, active_extruder, sg_value);
    motion_control.clear_trigger();
  }

  switch_detect.enable_probe(0);
  if (Z_AXIS ==  axis) {
    motion_control.move_to_z_no_limit(current_position.z + distance, feedrate);
    motion_control.synchronize();
  }
  else {
    probe_axis_move(axis, distance, feedrate);
  }
  if (do_sg && motion_control.is_sg_trigger()) {
    LOG_E("probe failed, axis stall guard!!!\n");
    motion_control.synchronize();
    motion_control.disable_stall_guard_all();
    switch_detect.disable_probe();
    return PROBR_RESULT_STALL_GUARD;
  }

  // count = 20;
  // trigger_cnt = 0;
  // while(--count) {
  //   if(switch_detect.read_active_extruder_status()) {
  //     trigger_cnt++;
  //     LOG_I("trigger_cnt %d\r\n", trigger_cnt);
  //     if (trigger_cnt >= 10)
  //       break;
  //   }
  //   vTaskDelay(pdMS_TO_TICKS(1));
  // }
  // if (trigger_cnt < 10) {
  //   LOG_E("no probe trigger in touching!!!\r\n");
  //   motion_control.disable_stall_guard_all();
  //   switch_detect.disable_probe();
  //   return PROBR_RESULT_SENSOR_ERROR;
  // }

  current_position[axis] = stepper.position((AxisEnum)axis) / planner.settings.axis_steps_per_mm[axis];
  sync_plan_position();

  switch_detect.enable_probe(1);
  if (Z_AXIS ==  axis) {
    motion_control.move_to_z_no_limit(current_position.z - distance, feedrate);
    motion_control.synchronize();
  }
  else {
    probe_axis_move(axis, -distance, feedrate);
  }

  // count = 20;
  // trigger_cnt = 0;
  // while(--count) {
  //   if(switch_detect.read_active_extruder_status()) {
  //     trigger_cnt++;
  //     LOG_I("trigger_cnt %d\r\n", trigger_cnt);
  //     if (trigger_cnt >= 10)
  //       break;
  //   }
  //   vTaskDelay(pdMS_TO_TICKS(1));
  // }
  // if (trigger_cnt < 10) {
  //   LOG_E("no probe trigger in leaving!!!\r\n");
  //   motion_control.disable_stall_guard_all();
  //   switch_detect.disable_probe();
  //   return PROBR_RESULT_SENSOR_ERROR;
  // }

  current_position[axis] = stepper.position((AxisEnum)axis) / planner.settings.axis_steps_per_mm[axis];
  sync_plan_position();

  float move_d = fabs(pos_before_probe - current_position[axis]);
  LOG_I("Actrual probe distance: %f\r\n", move_d);
  if (move_d >= fabs(distance)) {
    LOG_E("probe failed, move distance %f >= max allrow %f\r\n", move_d, fabs(distance));
    ret = PROBR_RESULT_NO_TRIGGER;
  }

  motion_control.disable_stall_guard_all();
  switch_detect.disable_probe();
  return ret;

}


/**
 * @brief Modified probe result to Z-offset
 *
 * @param pos : The specified calibration point 1 - 5
 * @return ErrCode
 */
ErrCode Calibtration::probe_z_offset(calibtration_position_e pos) {

  float position = 0;
  float last_valid_zoffset = home_offset[Z_AXIS];

  if (pos == CAlIBRATION_POS_0) {
    return E_PARAM;
  }

  backup_offset();
  set_home_offset(Z_AXIS, 0);
  goto_calibtration_position(pos);

  position = multiple_probe(Z_AXIS, -Z_PROBE_DISTANCE, PROBE_FAST_Z_FEEDRATE);
  if (position == CAlIBRATIONING_ERR_CODE) {
    set_home_offset(Z_AXIS, last_valid_zoffset);
    LOG_E("probe z offset failed\n");
    return E_CAlIBRATION_PRIOBE;
  }

  set_home_offset(Z_AXIS, -(position + build_plate_thickness));
  LOG_I("Set z_offset to :%f\n", home_offset[Z_AXIS]);
  return E_SUCCESS;
}

ErrCode Calibtration::probe_hight_offset(calibtration_position_e pos, uint8_t extruder) {

  ErrCode ret = E_SUCCESS;
  uint8_t last_active_extruder = active_extruder;
  static float last_z_probe_distance;

  if (pos == CAlIBRATION_POS_0 || pos >= CAlIBRATION_POS_INVALID) {
    return E_PARAM;
  }

  bool do_sg;
  uint16_t probe_fr;
  float probe_distance;
  float max_delta;

  do_sg = (z_probe_cnt == 0);
  if (0 == z_probe_cnt) {
    max_delta = 0;
    probe_distance = -2 * PROBE_DISTANCE;
  }
  else if (1 == z_probe_cnt) {
    max_delta = 2 * MAX_DELTA_DISTANCE;
    probe_distance = -PROBE_BACKOFF_DISTANCE - 2 * max_delta;
  }
  else {
    max_delta = 2 * (PROBE_BACKOFF_DISTANCE - last_z_probe_distance);
    if (max_delta < MAX_DELTA_DISTANCE) max_delta = MAX_DELTA_DISTANCE;
    probe_distance = -PROBE_BACKOFF_DISTANCE - 2 * max_delta;
  }
  probe_fr = (z_probe_cnt == 0) ? PROBE_FAST_Z_FEEDRATE : PROBE_FAST_Z_FEEDRATE / Z_PROBE_SPEED_SLOW_SCALER;
  z_probe_cnt++;

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION_Z_PROBING)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION_Z_PROBING\r\n");
    return E_PARAM;
  }

  switch_detect.trun_on_probe_pwr();
  float before_probe_z = current_position[Z_AXIS];
  LOG_I("before apply motion limit, probe distance %f", probe_distance);
  xyz_pos_t target = current_position;
  target[Z_AXIS] += probe_distance;
  apply_motion_limits(target);
  probe_distance = target[Z_AXIS] - current_position[Z_AXIS];
  LOG_I(", after limit, probe distance %f\r\n", probe_distance);

  probe_result_e probe_result = probe(Z_AXIS, probe_distance, probe_fr, do_sg);
  planner.synchronize();

  if (probe_result != PROBR_RESULT_SUCCESS) {
    probe_offset = CAlIBRATIONING_ERR_CODE;
    ret = E_CAlIBRATION_PRIOBE;
    z_need_re_home = true;
    z_probe_cnt = 0;
    LOG_E("CAlIBRATIONING_ERR_CODE\r\n");
  }
  else {
    probe_offset = current_position[Z_AXIS] + home_offset[Z_AXIS] + build_plate_thickness;
    LOG_I("JF-Z offset height:%f\n", probe_offset);
  }
  last_z_probe_distance = fabs(current_position[Z_AXIS] - before_probe_z);

  last_probe_pos = current_position.z;
  motion_control.move_z(PROBE_BACKOFF_DISTANCE, PROBE_FAST_Z_FEEDRATE);

  if (last_active_extruder != active_extruder) {
    tool_change(last_active_extruder, true);
  }
  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION\r\n");
    ret = E_PARAM;
  }
  switch_detect.trun_on_probe_pwr();

  return ret;
}

void stop_probe_and_sync() {

  motion_control.synchronize();
  // Wait for the last probe to end
  while (system_service.get_status() > SYSTEM_STATUE_CAlIBRATION) {
    vTaskDelay(pdMS_TO_TICKS(5));
  }

}

ErrCode Calibtration::wait_and_probe_z_offset(calibtration_position_e pos, uint8_t extruder) {

  ErrCode ret = E_SUCCESS;
  uint8_t last_active_extruder = active_extruder;

  if (pos == CAlIBRATION_POS_0 || pos >= CAlIBRATION_POS_INVALID) {
    LOG_E("Points not supported by hot bed calibration: %d\n", pos);
    return E_PARAM;
  }

  cur_pos = pos;
  status = CAlIBRATION_STATE_IDLE;
  stop_probe_and_sync();
  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION_Z_PROBING)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION_Z_PROBING\r\n");
    return E_PARAM;
  }

  switch_detect.trun_on_probe_pwr();
  X_standby();
  bed_preapare(extruder);

  ret = probe_z_offset(pos);
  planner.synchronize();

  last_probe_pos = current_position.z;
  Z_prepare();

  if (last_active_extruder != active_extruder) {
    tool_change(last_active_extruder, true);
  }
  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION\r\n");
    ret = E_PARAM;
  }
  switch_detect.trun_on_probe_pwr();

  return ret;

}

ErrCode Calibtration::probe_bed_base_hight(calibtration_position_e pos, uint8_t extruder) {

  // if (!head_bed_center_offset_check()) {
  //   if(E_SUCCESS != calibtration_xy_center_offset()) {
  //     return E_FAILURE;
  //   }
  // }
  set_calibtration_mode(CAlIBRATION_MODE_BED);
  return wait_and_probe_z_offset(pos, extruder);

}

void Calibtration::move_to_porbe_pos(calibtration_position_e pos, uint8_t extruder) {

  cur_pos = pos;
  status = CAlIBRATION_STATE_IDLE;
  stop_probe_and_sync();
  bed_preapare(extruder);
  goto_calibtration_position(pos);
  z_probe_cnt = 0;

}

ErrCode Calibtration::bed_start_beat_mode() {

  if (mode == CAlIBRATION_MODE_BED) {
    LOG_I("Calibration status set to CAlIBRATION_STATE_BED_BEAT\r\n");
    status = CAlIBRATION_STATE_BED_BEAT;
    calibtration.probe_offset = -CAlIBRATIONING_ERR_CODE;
    return E_SUCCESS;
  } else if (mode == CAlIBRATION_MODE_NOZZLE) {
    LOG_I("Calibration status set to CAlIBRATION_STATE_BED_BEAT\r\n");
    status = CAlIBRATION_STATE_BED_BEAT;
    return E_SUCCESS;
  }

  return E_PARAM;

}

ErrCode Calibtration::bed_end_beat_mode() {

  status = CAlIBRATION_STATE_BED_BEAT_WAIT_END;
  uint32_t timeout = millis() + 5000;
  while (status == CAlIBRATION_STATE_BED_BEAT_WAIT_END) {
    if (ELAPSED(millis(), timeout))
      return E_COMMON_ERROR;
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  return E_SUCCESS;

}

ErrCode Calibtration::nozzle_calibtration_preapare(calibtration_position_e pos) {

  set_calibtration_mode(CAlIBRATION_MODE_NOZZLE);
  ErrCode ret = wait_and_probe_z_offset(pos);
  if (ret == E_SUCCESS) {
    move_to_porbe_pos(pos, 1);
  }

  return ret;
}

/**
 * @brief reset home_offset、hotend_offsets and home
 *
 */
void Calibtration::reset_xy_calibtration_env() {

  set_hotend_offsets_to_default();
  set_home_offset(X_AXIS, 0);
  set_home_offset(Y_AXIS, 0);
  dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
  set_duplication_enabled(false);
  if (homing_needed()) {
    motion_control.home();
    planner.synchronize();
  }

}

float Calibtration::multiple_probe(uint8_t axis, float distance, uint16_t freerate) {

  float pos = 0;
  uint16_t probe_fr;
  float probe_distance;
  bool do_sg;
  float max_ = 0.0;
  float min_ = 10000.0;
  float max_delta;

  for (uint8_t i = 0; i < PROBE_TIMES; i++) {

    /*
    Do stall guard test for first time
    */
    do_sg = (i == 0);
    probe_fr = (i == 0) ? freerate : (freerate / XY_PROBE_SPEED_SLOW_SCALER);
    if (0 == i) {
      max_delta = 0;
      probe_distance = distance;
    }
    else if (1 == i) {
      max_delta = 2 * MAX_DELTA_DISTANCE;
      probe_distance = (distance > EPSILON) ? PROBE_BACKOFF_DISTANCE + 2 * max_delta : -PROBE_BACKOFF_DISTANCE - 2 * max_delta;
    }
    else {
      max_delta = MAX_DELTA_DISTANCE;
      probe_distance = (distance > EPSILON) ? PROBE_BACKOFF_DISTANCE + 2 * max_delta : -PROBE_BACKOFF_DISTANCE - 2 * max_delta;
    }

    float before_probe_pos = current_position[axis];
    probe_result_e probe_result = probe(axis, probe_distance, probe_fr, do_sg);
    planner.synchronize();

    if (probe_result != PROBR_RESULT_SUCCESS) {
      if (axis == X_AXIS || axis == Y_AXIS) {
        xy_need_re_home = true;
      }
      else {
        z_need_re_home = true;
      }
      return CAlIBRATIONING_ERR_CODE;
    }
    float after_probe_pos = current_position[axis];
    float actrual_probe_distance = fabs(after_probe_pos - before_probe_pos);
    LOG_I("%dth actrual probe distance %f\r\n", i, actrual_probe_distance);

    if (0 != i) {
      pos += current_position[axis];
      if (current_position[axis] > max_) max_ = current_position[axis];
      if (current_position[axis] < min_) min_ = current_position[axis];
    }

    motion_control.move(axis, (distance > EPSILON) ? -PROBE_BACKOFF_DISTANCE : PROBE_BACKOFF_DISTANCE, freerate);
  }

  return (pos - max_ - min_) / (PROBE_TIMES - 3);
}

ErrCode Calibtration::calibtration_xy() {

  ErrCode ret = E_SUCCESS;
  float xy_center[HOTENDS][XY] = {{0,0}, {0, 0}};
  uint8_t old_active_extruder = active_extruder;

  if (home_offset[Z_AXIS] == 0) {
    LOG_E("Calibrate XY after calibrating Z offset\n");
    return E_CAlIBRATION_XY;
  }

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION_XY_PROBING)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION_XY_PROBING\r\n");
    return E_PARAM;
  }

  switch_detect.trun_on_probe_pwr();
  X_standby();
  backup_offset();
  reset_xy_calibtration_env();

  HOTEND_LOOP() {

    bed_preapare(e);
    goto_calibtration_position(CAlIBRATION_POS_0);

    // uint8_t sg_value;
    // extern uint16_t z_sg_value;
    // if (z_sg_value)
    //   sg_value = z_sg_value;
    // else
    //   sg_value = probe_sg_reg[Z_AXIS];

    set_calibration_move_param();

    // motion_control.clear_trigger();
    // motion_control.enable_stall_guard_only_axis(Z_AXIS, sg_value, active_extruder);
    // LOG_I("axis %d(active_extruder = %d) sg_value set to %d\r\n", Z_AXIS, active_extruder, sg_value);
    // motion_control.clear_trigger();

    motion_control.logical_move_to_z(XY_CALI_Z_POS, PROBE_FAST_Z_FEEDRATE);

    switch_detect.enable_probe(0);
    if (motion_control.is_sg_trigger()) {
      LOG_E("probe failed, axis stall guard!!!\n");
      motion_control.synchronize();
      motion_control.disable_stall_guard_all();
      switch_detect.disable_probe();
      ret = E_CAlIBRATION_PRIOBE;
      break;
    }

    // motion_control.disable_stall_guard_all();

    for (uint8_t axis = 0; axis <= Y_AXIS; axis++) {

      float pos = multiple_probe(axis, -PROBE_DISTANCE, PROBE_FAST_XY_FEEDRATE);
      if (pos == CAlIBRATIONING_ERR_CODE) {
        ret = E_CAlIBRATION_PRIOBE;
        LOG_E("e:%d axis:%d probe 0 filed\n", e, axis);
        z_need_re_home = true;
        break;
      }

      goto_calibtration_position(CAlIBRATION_POS_0, PROBE_FAST_XY_FEEDRATE);

      float pos_1 = multiple_probe(axis, PROBE_DISTANCE, PROBE_FAST_XY_FEEDRATE);
      if (pos_1 == CAlIBRATIONING_ERR_CODE) {
        ret = E_CAlIBRATION_PRIOBE;
        LOG_E("e:%d axis:%d probe 1 filed\n", e, axis);
        z_need_re_home = true;
        break;
      }

      xy_center[e][axis] += (pos_1 + pos) / 2;
      goto_calibtration_position(CAlIBRATION_POS_0, PROBE_FAST_XY_FEEDRATE);

    }

    if (ret != E_SUCCESS) {
      LOG_E("calibtration e[%d] xy filed\n", e);
      z_need_re_home = true;
      break;
    }

  }

  Z_standby();
  X_standby();
  Y_standby();
  tool_change(old_active_extruder, true);

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION\r\n");
    ret = E_PARAM;
  }

  switch_detect.trun_on_probe_pwr();
  if(ret == E_SUCCESS) {
    LOG_V("JF-XY Extruder1:%f %f\n", xy_center[0][0], xy_center[0][1]);
    LOG_V("JF-XY Extruder2:%f %f\n", xy_center[1][0], xy_center[1][1]);

    // XY center offset from XY home position (0, 0)
    heat_bed_center_offset[0] = xy_center[0][0] - calibration_position_xy[0][0];
    heat_bed_center_offset[1] = xy_center[0][1] - calibration_position_xy[0][1];
    LOG_I("XY center offset:(%.3f, %.3f)\n", heat_bed_center_offset[0], heat_bed_center_offset[1]);

    // extruder_1 offset from extruder_0
    float extruder_1_offset_to_extruder_0[2];
    extruder_1_offset_to_extruder_0[0] = xy_center[1][0] - xy_center[0][0];
    extruder_1_offset_to_extruder_0[1] = xy_center[1][1] - xy_center[0][1];
    set_hotend_offsets(1, X_AXIS, X2_MAX_POS - extruder_1_offset_to_extruder_0[0]);
    set_hotend_offsets(1, Y_AXIS, -extruder_1_offset_to_extruder_0[1]);
    LOG_I("extruder_1 offset:(%.3f, %.3f)\n", extruder_1_offset_to_extruder_0[0], extruder_1_offset_to_extruder_0[1]);

    settings.save();
  }
  else {
    LOG_E("JF-XY calibration: Fail!\n");
    z_need_re_home = true;
  }

  return ret;
}

ErrCode Calibtration::calibtration_xy_center_offset() {

  ErrCode ret = E_SUCCESS;
  float xy_center[XY] = {0, 0};
  uint8_t old_active_extruder = active_extruder;

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION_XY_PROBING)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION_XY_PROBING\r\n");
    return E_PARAM;
  }

  switch_detect.trun_on_probe_pwr();
  X_standby();
  backup_offset();
  reset_xy_calibtration_env();

  bed_preapare(0);
  goto_calibtration_position(CAlIBRATION_POS_0);
  motion_control.move_to_z_no_limit(XY_CENTER_OFFSET_Z_POS);

  for (uint8_t axis = 0; axis <= Y_AXIS; axis++) {

    float pos = multiple_probe(axis, -PROBE_DISTANCE, PROBE_FAST_XY_FEEDRATE);
    if (pos == CAlIBRATIONING_ERR_CODE) {
      ret = E_CAlIBRATION_PRIOBE;
      LOG_E("e:%d axis:%d probe 0 filed\n", 0, axis);
      break;
    }
    float pos_1 = multiple_probe(axis, PROBE_DISTANCE, PROBE_FAST_XY_FEEDRATE);
    if (pos_1 == CAlIBRATIONING_ERR_CODE) {
      ret = E_CAlIBRATION_PRIOBE;
      LOG_E("e:%d axis:%d probe 1 filed\n", 0, axis);
      break;
    }
    xy_center[axis] += (pos_1 + pos) / 2;
    goto_calibtration_position(CAlIBRATION_POS_0, PROBE_FAST_XY_FEEDRATE);

  }

  if (ret != E_SUCCESS) {
    LOG_E("calibtration e[%d] xy filed\n", 0);
  }

  Z_prepare();
  tool_change(old_active_extruder, true);

  if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION)) {
    LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION\r\n");
    ret = E_PARAM;
  }

  switch_detect.trun_on_probe_pwr();
  if(ret == E_SUCCESS) {
    // XY center offset from XY home position (0, 0)
    heat_bed_center_offset[0] = xy_center[0] - calibration_position_xy[0][0];
    heat_bed_center_offset[1] = xy_center[1] - calibration_position_xy[0][1];
    LOG_I("XY center offset:(%.3f, %.3f)\n", heat_bed_center_offset[0], heat_bed_center_offset[1]);
  }
  else {
    LOG_E("XY center calibration: Fail!\n");
  }

  return ret;
}

ErrCode Calibtration::set_hotend_offset(uint8_t axis, float offset) {
  if (axis >= Z_AXIS) {
    return E_PARAM;
  }
  set_hotend_offsets(1, axis, offset);
  return E_SUCCESS;
}

float Calibtration::get_hotend_offset(uint8_t axis) {
  return hotend_offset[1][axis];
}

ErrCode Calibtration::exit(bool is_save) {

  LOG_I("%s\n", is_save ? "Exit with save" : "Exit without save");

  if (system_service.is_calibtration_status()) {

    if (mode != CAlIBRATION_MODE_IDLE) {
      if (!is_save) {
        restore_offset();
      }
      if (mode == CAlIBRATION_MODE_NOZZLE) {
        set_home_offset(Z_AXIS, home_offset_backup[Z_AXIS]);
      }
      if (is_save) {
        settings.save();
      }
    }

    mode = CAlIBRATION_MODE_EXIT;
    status = CAlIBRATION_STATE_IDLE;
    probe_offset = -CAlIBRATIONING_ERR_CODE;
  }

  switch_detect.trun_off_probe_pwr();

  return E_SUCCESS;
}

// Jumping requires blocking tasks so put it in a loop to do
// Probe once per loop
void Calibtration::loop(void) {

  if (mode == CAlIBRATION_MODE_BED && status == CAlIBRATION_STATE_BED_BEAT) {
    if (probe_hight_offset(cur_pos, 0) != E_SUCCESS) {
      LOG_I("probe_hight_offset error, return CAlIBRATION_STATE_IDLE\r\n");
      status = CAlIBRATION_STATE_IDLE;
    }
  }

  else if (mode == CAlIBRATION_MODE_NOZZLE && status == CAlIBRATION_STATE_BED_BEAT) {
    if(probe_hight_offset(cur_pos, 1) != E_SUCCESS) {
      status = CAlIBRATION_STATE_IDLE;
    }
  }

  else if (status == CAlIBRATION_STATE_BED_BEAT_WAIT_END) {
    status = CAlIBRATION_STATE_IDLE;
  }

  else if (mode == CAlIBRATION_MODE_EXIT) {
    motion_control.synchronize();
    Z_standby();
    X_standby();
    Y_standby();

    if (need_extrude) {
      need_extrude = false;
      extrude_e(CAlIBRATIONIN_RETRACK_E_MM);
    }

    HOTEND_LOOP() {
      thermalManager.setTargetHotend(0, e);
    }
    thermalManager.setTargetBed(0);

    restore_move_param();
    mode = CAlIBRATION_MODE_IDLE;
    system_service.set_status(SYSTEM_STATUE_IDLE);
    switch_detect.trun_off_probe_pwr();
    z_probe_cnt = 0;
  }

}


/**
 * @brief Change and apply z_offset
 *
 * @param offset Absolute z-offset value
 * @param is_moved : true - Change the coordinate system by adjusting the hot bed position
 *                   false - Change the coordinate system value directly
 */
void Calibtration::set_z_offset(float offset, bool is_moved) {

  motion_control.wait_G28();

  system_status_source_e cur_pause_sorce = system_service.get_source();
  system_status_e cur_work_status = system_service.get_status();

  if (SYSTEM_STATUE_PRINTING == cur_work_status) {
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_PAUSING, SYSTEM_STATUE_SCOURCE_Z_LIVE_OFFSET)) {
      LOG_E("can NOT set to SYSTEM_STATUE_PAUSING\r\n");
      return;
    }
    while(system_service.get_status() != SYSTEM_STATUE_PAUSED) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  motion_control.synchronize();

  // print_control.commands_lock();
  // motion_control.synchronize();

  LOG_I("Set z offset: after synchronize\r\n");
  xyze_pos_t lpos = current_position.asLogical();
  float cur_z = lpos[Z_AXIS];
  float diff = offset - home_offset[Z_AXIS];
  home_offset[Z_AXIS] = offset;
  update_workspace_offset(Z_AXIS);
  extern bool ml_setting_need_save;
  ml_setting_need_save = true;
  if (!is_moved) {
    current_position[Z_AXIS] += diff;
  } else {
    LOG_I("curZ: %f\n",cur_z);
    motion_control.logical_move_to_z(cur_z, 600);
    sync_plan_position();

    if (system_service.get_status() == SYSTEM_STATUE_PAUSED) {
      power_loss.stash_data.home_offset = home_offset;
      power_loss.stash_data.position[Z_AXIS] -= diff;
    }
  }

  // print_control.commands_unlock();
  if (SYSTEM_STATUE_PRINTING == cur_work_status) {
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_RESUMING, SYSTEM_STATUE_SCOURCE_Z_LIVE_OFFSET)) {
      LOG_E("can NOT set to SYSTEM_STATUE_RESUMING\r\n");
      system_service.return_to_idle();
      return;
    }
    while(system_service.get_status() == SYSTEM_STATUE_RESUMING) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  else {
    if (E_SUCCESS != system_service.set_status(cur_work_status, cur_pause_sorce)) {
      LOG_E("can NOT set to %d\r\n", cur_work_status);
      system_service.return_to_idle();
      return;
    }
  }

  LOG_I("Apply Z offset: %f\n", home_offset[Z_AXIS]);
}

float Calibtration::get_z_offset() {
  return home_offset[Z_AXIS];
}
