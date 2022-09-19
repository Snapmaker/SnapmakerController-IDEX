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

#define PROBE_FAST_Z_FEEDRATE 200
#define PROBE_Z_LEAVE_FEEDRATE 5
#define PROBE_FAST_XY_FEEDRATE 800
#define PROBE_MOVE_XY_FEEDRATE 5000
#define PROBE_MOVE_Z_FEEDRATE 600
#define PROBE_LIFTINT_DISTANCE (1)  // mm
#define PROBE_MOVE_XY_LIFTINT_DISTANCE (5)  // mm
#define Z_REMOVE_PLATE_THICKNESS(z) (z - build_plate_thickness)
#define Z_PROBE_TRY_TO_MOVE_DISTANCE (10)  // mm

#define X2_MIN_HOTEND_OFFSET (X2_MAX_POS - X2_MIN_POS - 20)

static uint8_t probe_sg_reg[3] = {20, 20, 80};  // X Y Z1
static float build_plate_thickness = 5;

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


/**
 * @brief Backup offset data for recovery
 * 
 */
void Calibtration::backup_offset() {
  HOTEND_LOOP() {
    hotend_offset_backup[e] = hotend_offset[e];
  }
  home_offset_backup = home_offset;
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

void Calibtration::bed_preapare(uint8_t extruder_index) {
  // Store feedrate and feedrate scaling
  remember_feedrate_scaling_off();
  if (hotend_offset[1][X_AXIS] < X2_MIN_HOTEND_OFFSET) {
    LOG_I("the hotend offset is too small, it will be reset\n");
    set_hotend_offsets_to_default();
  }
  // Enable endstop
  endstops.enable(true);
  if(homing_needed()) {
    // Step1 home all axis
    motion_control.home();
    planner.synchronize();
  }
  // Switch to single-header mode
  if (dual_x_carriage_mode > DXC_FULL_CONTROL_MODE) {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    set_duplication_enabled(false);
    motion_control.home_x();
  }
  set_duplication_enabled(false);
  float another_x_home_pos = !extruder_index ? x_home_pos(1) : x_home_pos(0);
  float another_x_pos = !extruder_index ? x2_position() : x_position();

  // Make sure the other head is in the home position
  if (another_x_home_pos != another_x_pos) {
    if(current_position[Z_AXIS] < 15) {
      // Avoid damage to hot bed
      motion_control.move_z(15, PROBE_MOVE_Z_FEEDRATE);
    }
    tool_change(!extruder_index, true);
    motion_control.move_to_x(another_x_home_pos, MOTION_TRAVEL_FEADRATE);
  }
  if (active_extruder != extruder_index) {
    tool_change(extruder_index, true);
  }
}

// Run to the specified calibration point
ErrCode Calibtration::goto_calibtration_position(uint8_t pos) {
  xyz_pos_t offset0 = hotend_offset[0];
  xyz_pos_t offset1 = hotend_offset[1];
  // Reset the Hotend Offsets; otherwise, the moving position will be affected
  set_hotend_offsets_to_default();
  motion_control.move_to_xy(calibration_position_xy[pos][0], calibration_position_xy[pos][1], MOTION_TRAVEL_FEADRATE);
  // recover hotend_offsets
  set_hotend_offsets(0, offset0);
  set_hotend_offsets(1, offset1);
  return E_SUCCESS;
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

bool Calibtration::move_to_sersor_no_trigger(uint8_t axis, int16_t try_distance) {
  bool probe_status = false;
  // If the sensor has been triggered, try to detect it at a distance
  probe_status = active_extruder ? switch_detect.read_e1_probe_status() : switch_detect.read_e0_probe_status();
  if (probe_status) {
    float move_distance = 0;
    do {
      move_distance += try_distance;
      LOG_I("The Probe sensor is tigger and try move %d mm\n", try_distance);
      probe_axis_move(axis, try_distance, MOTION_TRAVEL_FEADRATE);
      probe_status = active_extruder ? switch_detect.read_e1_probe_status() : switch_detect.read_e0_probe_status();
      if (probe_status) {
        current_position[axis] = stepper.position((AxisEnum)axis) / planner.settings.axis_steps_per_mm[axis];
        sync_plan_position();
      }
      else {
        break;
      }
    } while(abs(move_distance) < Z_PROBE_TRY_TO_MOVE_DISTANCE);
    if (probe_status) {
        LOG_E("The Probe sensor is not working !\n");
        return false;
    }
  }
  return true;
}

void reset_move_param() {
  planner.settings.min_segment_time_us = DEFAULT_MINSEGMENTTIME;
  planner.settings.acceleration = DEFAULT_ACCELERATION;
  planner.settings.retract_acceleration = DEFAULT_RETRACT_ACCELERATION;
  planner.settings.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.settings.min_feedrate_mm_s = feedRate_t(DEFAULT_MINIMUMFEEDRATE);
  planner.settings.min_travel_feedrate_mm_s = feedRate_t(DEFAULT_MINTRAVELFEEDRATE);
}

probe_result_e Calibtration::move_to_probe_trigger(uint8_t axis, float distance, uint16_t feedrate) {
  probe_result_e ret = PROBR_RESULT_SUCCESS;
  float pos_before_probe = current_position[axis];

  if (!move_to_sersor_no_trigger(axis, distance >= 0 ? -1 : 1)) {
    return PROBR_RESULT_SENSOR_ERROR;
  }

  reset_move_param();

  motion_control.enable_stall_guard_only_axis(axis, probe_sg_reg[axis], active_extruder);
  switch_detect.enable_probe(0);
  probe_axis_move(axis, distance, feedrate);
  current_position[axis] = stepper.position((AxisEnum)axis) / planner.settings.axis_steps_per_mm[axis];
  sync_plan_position();
  if (!motion_control.is_sg_trigger()) {
    motion_control.disable_stall_guard_all();
    switch_detect.enable_probe(1);
    probe_axis_move(axis, -distance, PROBE_Z_LEAVE_FEEDRATE);
    current_position[axis] = stepper.position((AxisEnum)axis) / planner.settings.axis_steps_per_mm[axis];
    sync_plan_position();
  } else {
    LOG_E("probe failed be stall guard!!!\n");
    motion_control.synchronize();
    motion_control.move_z(Z_PROBE_TRY_TO_MOVE_DISTANCE);
    ret = PROBR_RESULT_STALL_GUARD;
  }
  motion_control.disable_stall_guard_all();

  if (abs((pos_before_probe - current_position[axis]) > (abs(distance) - 0.2))) {
    LOG_E("probe failed , sensor no trigger!!!\n");
    ret = PROBR_RESULT_NO_TRIGGER;
  }
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
  float z_probe_distance = 25;
  float last_valid_zoffset = home_offset[Z_AXIS];
  if (pos == CAlIBRATION_POS_0) {
    return E_PARAM;
  }
  backup_offset();
  set_home_offset(Z_AXIS, 0);
  motion_control.move_to_z(15, PROBE_MOVE_Z_FEEDRATE);
  goto_calibtration_position(pos);

  z_probe_distance = 25;
  position = multiple_probe(Z_AXIS, -z_probe_distance, PROBE_FAST_Z_FEEDRATE);
  if (position == CAlIBRATIONING_ERR_CODE) {
    set_home_offset(Z_AXIS, last_valid_zoffset);
    LOG_E("probe z offset failed\n");
    return E_CAlIBRATION_PRIOBE;
  }

  set_home_offset(Z_AXIS, -(position + build_plate_thickness));
  LOG_I("Set z_offset to :%f\n", home_offset[Z_AXIS]);
  motion_control.move_z(PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);
  return E_SUCCESS;
}

ErrCode Calibtration::probe_hight_offset(calibtration_position_e pos, uint8_t extruder) {
  ErrCode ret = E_SUCCESS;
  float z_probe_distance = 15;
  uint8_t last_active_extruder = active_extruder;
  if (pos == CAlIBRATION_POS_0 || pos >= CAlIBRATION_POS_INVALID) {
    return E_PARAM;
  }
  bed_preapare(extruder);
  goto_calibtration_position(pos);
  system_service.set_status(SYSTEM_STATUE_CAlIBRATION_Z_PROBING);

  motion_control.move_to_z(last_probe_pos + PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);
  uint16_t speed = PROBE_FAST_Z_FEEDRATE;
  probe_result_e probe_result = move_to_probe_trigger(Z_AXIS, -z_probe_distance, speed);
  if (probe_result != PROBR_RESULT_SUCCESS) {
    probe_offset = CAlIBRATIONING_ERR_CODE;
    ret = E_CAlIBRATION_PRIOBE;
  } else {
    probe_offset = current_position[Z_AXIS] + home_offset[Z_AXIS] + build_plate_thickness;
    LOG_I("JF-Z offset height:%f\n", probe_offset);
  }
  last_probe_pos = current_position.z;
  motion_control.move_z(PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);
  if (last_active_extruder != active_extruder) {
    tool_change(last_active_extruder, true);
  }
  system_service.set_status(SYSTEM_STATUE_CAlIBRATION);
  return ret;
}

void stop_probe_and_sync() {
  motion_control.synchronize();
  // Wait for the last probe to end
  while (system_service.get_status() > SYSTEM_STATUE_CAlIBRATION) {
    vTaskDelay(10);
  }
}

ErrCode Calibtration::wait_and_probe_z_offset(calibtration_position_e pos, uint8_t extruder) {
  ErrCode ret = E_SUCCESS;
  if (pos == CAlIBRATION_POS_0 || pos >= CAlIBRATION_POS_INVALID) {
    LOG_E("Points not supported by hot bed calibration: %d\n", pos);
    return E_PARAM;
  }
  cur_pos = pos;
  status = CAlIBRATION_STATE_IDLE;
  stop_probe_and_sync();
  // motion_control.move_z(PROBE_MOVE_XY_LIFTINT_DISTANCE);

  uint8_t last_active_extruder = active_extruder;
  system_service.set_status(SYSTEM_STATUE_CAlIBRATION_Z_PROBING);
  bed_preapare(extruder);
  motion_control.home_x();
  ret = probe_z_offset(pos);

  last_probe_pos = current_position.z;
  motion_control.move_z(PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);
  if (last_active_extruder != active_extruder) {
    tool_change(last_active_extruder, true);
  }
  system_service.set_status(SYSTEM_STATUE_CAlIBRATION);
  return ret;
}

ErrCode Calibtration::probe_bed_base_hight(calibtration_position_e pos, uint8_t extruder) {
  set_calibtration_mode(CAlIBRATION_MODE_BED);
  return wait_and_probe_z_offset(pos, extruder);
}

ErrCode Calibtration::move_to_porbe_pos(calibtration_position_e pos, uint8_t extruder) {
  ErrCode ret = E_SUCCESS;
  cur_pos = pos;
  status = CAlIBRATION_STATE_IDLE;
  stop_probe_and_sync();
  motion_control.move_z(PROBE_MOVE_XY_LIFTINT_DISTANCE);
  bed_preapare(extruder);
  goto_calibtration_position(pos);
  return ret;
}

ErrCode Calibtration::bed_start_beat_mode() {
  if (mode == CAlIBRATION_MODE_BED) {
    status = CAlIBRATION_STATE_BED_BEAT;
    return E_SUCCESS;
  } else if (mode == CAlIBRATION_MODE_NOZZLE) {
    status = CAlIBRATION_STATE_BED_BEAT;
    return E_SUCCESS;
  }
  return E_PARAM;
}

ErrCode Calibtration::bed_end_beat_mode() {
  status = CAlIBRATION_STATE_BED_BEAT_WAIT_END;
  uint32_t timeout = millis() + 5000;
  while (status == CAlIBRATION_STATE_BED_BEAT_WAIT_END) {
    if (timeout < millis())
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
  if(homing_needed()) {
    motion_control.home();
    planner.synchronize();
  } else {
    if(current_position[Z_AXIS] < 30) {
      motion_control.move_z(15, PROBE_MOVE_Z_FEEDRATE);
    }
    motion_control.home_x();
    motion_control.home_y();
  }
}

float Calibtration::multiple_probe(uint8_t axis, float distance, uint16_t freerate) {
  #define PROBE_TIMES 3
  float probe_distance =  distance;
  float pos = 0;
  for (uint8_t i = 0; i < PROBE_TIMES; i++) {
    probe_result_e probe_result = move_to_probe_trigger(axis, probe_distance, freerate);
    if (probe_result != PROBR_RESULT_SUCCESS) {
      return CAlIBRATIONING_ERR_CODE;
    }
    pos += current_position[axis];
    probe_distance =  distance >= 0 ? 1 : -1;
    motion_control.move(axis, -probe_distance / 2, freerate);
  }
  return pos / PROBE_TIMES;
}

ErrCode Calibtration::calibtration_xy() {
  ErrCode ret = E_SUCCESS;
  float xy_center[HOTENDS][XY] = {{0,0}, {0, 0}};
  float probe_distance = 15;
  uint8_t old_active_extruder = active_extruder;
  if (home_offset[Z_AXIS] == 0) {
    LOG_E("Calibrate XY after calibrating Z offset\n");
    return E_CAlIBRATION_XY;
  }
  system_service.set_status(SYSTEM_STATUE_CAlIBRATION_XY_PROBING);
  backup_offset();  //  you can choose to restore offset when exit()
  reset_xy_calibtration_env();
  HOTEND_LOOP() {
    bed_preapare(e);
    motion_control.logical_move_to_z(15 - build_plate_thickness);
    goto_calibtration_position(CAlIBRATION_POS_0);
    motion_control.logical_move_to_z(-2 - build_plate_thickness);
    for (uint8_t axis = 0; axis <= Y_AXIS; axis++) {
      goto_calibtration_position(CAlIBRATION_POS_0);

      float pos = multiple_probe(axis, -probe_distance, PROBE_FAST_XY_FEEDRATE);
      if (pos == CAlIBRATIONING_ERR_CODE) {
        ret = E_CAlIBRATION_PRIOBE;
        LOG_E("e:%d axis:%d probe 0 filed\n", e, axis);
        break;
      }
      float pos_1 = multiple_probe(axis, probe_distance, PROBE_FAST_XY_FEEDRATE);
      if (pos_1 == CAlIBRATIONING_ERR_CODE) {
        ret = E_CAlIBRATION_PRIOBE;
        LOG_E("e:%d axis:%d probe 1 filed\n", e, axis);
        break;
      }
      xy_center[e][axis] += (pos_1 + pos) / 2;
      goto_calibtration_position(CAlIBRATION_POS_0);
    }
    if (ret != E_SUCCESS) {
      LOG_E("calibtration e[%d] xy filed\n", e);
      break;
    }
  }
  motion_control.move_z(100);
  motion_control.home_x();
  motion_control.home_y();
  tool_change(old_active_extruder, true);
  system_service.set_status(SYSTEM_STATUE_CAlIBRATION);
  if(ret == E_SUCCESS) {
    LOG_V("JF-XY calibration: Success!\n");
    LOG_V("JF-XY Extruder1:%f %f\n", xy_center[0][0], xy_center[0][1]);
    LOG_V("JF-XY Extruder2:%f %f\n", xy_center[1][0], xy_center[1][1]);

    set_home_offset(X_AXIS, calibration_position_xy[0][0] - xy_center[0][0]);
    set_home_offset(Y_AXIS, calibration_position_xy[0][1] - xy_center[0][1]);
    LOG_I("JF-Extruder1 home offset:%f %f\n", home_offset[X_AXIS], home_offset[Y_AXIS]);

    set_hotend_offsets(1, X_AXIS, X2_MAX_POS - (xy_center[1][0] - xy_center[0][0]));
    set_hotend_offsets(1, Y_AXIS, -(xy_center[1][1] - xy_center[0][1]));
    LOG_I("JF-Extruder2 hotend offset:%f %f\n", hotend_offset[1].x, hotend_offset[1].y);

    // Store to eeprom
    settings.save();
    return ret;
  }
  else {
    LOG_E("JF-XY calibration: Fail!\n");
    return ret;
  }
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
  LOG_V("exit justing\n");
  if (system_service.is_calibtration_status()) {
    if (mode != CAlIBRATION_MODE_IDLE) {
      if (!is_save) {
        HOTEND_LOOP() {
          hotend_offset[e] = hotend_offset_backup[e];
        }
        home_offset = home_offset_backup;
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
    probe_offset = CAlIBRATIONING_ERR_CODE;
  }
  return E_SUCCESS;
}

// Jumping requires blocking tasks so put it in a loop to do
// Probe once per loop
void Calibtration::loop(void) {
  if (mode == CAlIBRATION_MODE_BED && status == CAlIBRATION_STATE_BED_BEAT) {
    if (probe_hight_offset(cur_pos, 0) != E_SUCCESS) {
      status = CAlIBRATION_STATE_IDLE;
    }
    LOG_V("probe offset:%f\n", probe_offset);
  } else if (mode == CAlIBRATION_MODE_NOZZLE && status == CAlIBRATION_STATE_BED_BEAT) {
    if(probe_hight_offset(cur_pos, 1) != E_SUCCESS) {
      status = CAlIBRATION_STATE_IDLE;
    }
  } else if (status == CAlIBRATION_STATE_BED_BEAT_WAIT_END) {
    status = CAlIBRATION_STATE_IDLE;
  } else if (mode == CAlIBRATION_MODE_EXIT) {
    motion_control.synchronize();
    if (current_position[Z_AXIS] < 100) {
      motion_control.move_to_z(100, PROBE_MOVE_Z_FEEDRATE);
    }
    motion_control.home_x();
    motion_control.home_y();
    if (need_extrude) {
      need_extrude = false;
      extrude_e(CAlIBRATIONIN_RETRACK_E_MM);
    }
    HOTEND_LOOP() {
      thermalManager.setTargetHotend(0, e);
    }
    thermalManager.setTargetBed(0);
    mode = CAlIBRATION_MODE_IDLE;
    system_service.set_status(SYSTEM_STATUE_IDLE);
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
  print_control.commands_lock();
  motion_control.synchronize();
  xyze_pos_t lpos = current_position.asLogical();
  float cur_z = lpos[Z_AXIS];
  float diff = offset - home_offset[Z_AXIS];
  home_offset[Z_AXIS] = offset;
  update_workspace_offset(Z_AXIS);
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
  print_control.commands_unlock();
  LOG_I("Apply Z offset: %f\n", home_offset[Z_AXIS]);
}

float Calibtration::get_z_offset() {
  return home_offset[Z_AXIS];
}
