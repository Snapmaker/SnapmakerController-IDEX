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

Calibtration calibtration;

#define PROBE_Z_FEEDRATE 100
#define PROBE_XY_FEEDRATE 200
#define PROBE_MOVE_XY_FEEDRATE 5000
#define PROBE_MOVE_Z_FEEDRATE 600
#define PROBE_LIFTINT_DISTANCE (1)  // mm
#define PROBE_MOVE_XY_LIFTINT_DISTANCE (5)  // mm
#define Z_REMOVE_PLATE_THICKNESS(z) (z - build_plate_thickness)

// static uint8_t probe_sg_reg[3] = {1, 0, 73};  // X Y Z
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
  set_duplication_enabled(false);
  dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
}

void Calibtration::bed_preapare(uint8_t extruder_index) {
  // Store feedrate and feedrate scaling
  remember_feedrate_scaling_off();
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
    motion_control.home_x();
  }
  float another_x_home_pos = !extruder_index ? x_home_pos(1) : x_home_pos(0);
  float another_x_pos = !extruder_index ? x2_position() : x_position();

  // Make sure the other head is in the home position  
  if (another_x_home_pos != another_x_pos) {
    if(current_position[Z_AXIS] < 30) {
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

static void set_hotend_offsets_to_default() {
  xyz_pos_t hotend0_offset = {0, 0, 0};
  xyz_pos_t hotend1_offset = {X2_MAX_POS - X2_MIN_POS, 0, 0};
  set_hotend_offsets(0, hotend0_offset);
  set_hotend_offsets(1, hotend1_offset);
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

float Calibtration::probe(uint8_t axis, float distance, uint16_t feedrate) {
  float ret = distance;
  float pos_before_probe, pos_after_probe;

  switch_detect.enable_probe();
  pos_before_probe = current_position[axis];

  // motion_control.enable_stall_guard_only_axis(axis, probe_sg_reg[axis]);
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

  pos_after_probe = stepper.position((AxisEnum)axis) / planner.settings.axis_steps_per_mm[axis];
  current_position[axis] = pos_after_probe;
  sync_plan_position();
  if (motion_control.is_sg_trigger()) {
    LOG_E("probe failed be stall guard!!!\n");
  } else {
    ret = (pos_before_probe - pos_after_probe);
  }
  // motion_control.disable_stall_guard_all();
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
  float temp_z = 0;
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
  // Try a probe
  temp_z = probe(Z_AXIS, -z_probe_distance, PROBE_Z_FEEDRATE);
  if (temp_z == -z_probe_distance) {
    LOG_E("failed to probe z !!!\n");
    set_home_offset(Z_AXIS, last_valid_zoffset);
    return E_CAlIBRATION_PRIOBE;
  }
  position = accurate_probe(Z_AXIS, -z_probe_distance, PROBE_Z_FEEDRATE);
  set_home_offset(Z_AXIS, -(position + build_plate_thickness));
  LOG_I("Set z_offset to :%f\n", home_offset[Z_AXIS]);
  motion_control.move_z(PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);
  return E_SUCCESS;
}

ErrCode Calibtration::bed_probe(calibtration_position_e pos, uint8_t extruder, bool set_z_offset) {
  ErrCode ret = E_SUCCESS;
  float z_probe_distance;
  uint8_t last_active_extruder = active_extruder;
  if (pos == CAlIBRATION_POS_0 || pos >= CAlIBRATION_POS_INVALID) {
    return E_PARAM;
  }
  bed_preapare(extruder);

  float temp_z = 0;
  if (set_z_offset) {
    motion_control.home_x();
    ret = probe_z_offset(pos);
  } else {
    goto_calibtration_position(pos);
    motion_control.logical_move_to_z(Z_REMOVE_PLATE_THICKNESS(PROBE_LIFTINT_DISTANCE), PROBE_MOVE_Z_FEEDRATE);
    z_probe_distance = 15;
    temp_z = probe(Z_AXIS, -z_probe_distance, PROBE_Z_FEEDRATE);
    if (temp_z == -z_probe_distance) {
      LOG_E("failed to probe z !!!\n");
      probe_offset = CAlIBRATIONING_ERR_CODE;
      ret = E_CAlIBRATION_PRIOBE;
    } else {
      probe_offset = current_position[Z_AXIS] + home_offset[Z_AXIS] + build_plate_thickness;
      LOG_I("JF-Z offset height:%f\n", probe_offset);
    }
  }
  motion_control.logical_move_to_z(Z_REMOVE_PLATE_THICKNESS(PROBE_LIFTINT_DISTANCE), PROBE_MOVE_Z_FEEDRATE);
  if (last_active_extruder != active_extruder) {
    tool_change(last_active_extruder, true);
  }
 
  return ret;
}

ErrCode Calibtration::bed_calibtration_preapare(calibtration_position_e pos, bool is_probe) {
  ErrCode ret = E_SUCCESS;
  if (pos == CAlIBRATION_POS_0 || pos >= CAlIBRATION_POS_INVALID) {
    LOG_E("Points not supported by hot bed calibration: %d\n", pos);
    return E_PARAM;
  }
  cur_pos = pos;
  mode = CAlIBRATION_MODE_BED;
  status = CAlIBRATION_STATE_IDLE;
  motion_control.synchronize();
  motion_control.move_z(PROBE_MOVE_XY_LIFTINT_DISTANCE);
  if (is_probe) {
    ret = bed_probe(pos, 0, true);
  } else {
    bed_preapare(0);
    goto_calibtration_position(pos);
  }
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

ErrCode Calibtration::nozzle_calibtration_preapare(calibtration_position_e pos) {
  if (pos == CAlIBRATION_POS_0 || pos >= CAlIBRATION_POS_INVALID) {
    LOG_E("Points not supported by hot nozzle calibration:%d\n", pos);
    return E_PARAM;
  }
  cur_pos = pos;
  mode = CAlIBRATION_MODE_NOZZLE;
  status = CAlIBRATION_STATE_IDLE;
  bed_probe(pos, 0, true);

  return E_SUCCESS;
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

/**
 * @brief The results of multiple probe, probe must be called once before calling accurate_probe
 * 
 * @param axis : X_AXIS Y_AXIS Z_AXIS
 * @param dir : <0 - Inverted movement  >=0 - Forward movement
 * @param freerate : mm/min
 * @return float 
 */
float Calibtration::accurate_probe(uint8_t axis, int8_t dir, uint16_t freerate) {
  #define PROBE_TIMES 3
  float probe_distance =  dir >= 0 ? 1 : -1;
  float pos = 0;
  for (uint8_t i = 0; i < PROBE_TIMES; i++) {
    motion_control.move(axis, -probe_distance / 2, freerate);
    probe(axis, probe_distance, freerate);
    pos += current_position[axis];
  }
  return pos / PROBE_TIMES;
}

ErrCode Calibtration::calibtration_xy() {
  ErrCode ret = E_SUCCESS;
  float xy_center[HOTENDS][XY];
  float probe_distance = 15;
  float probe_value = 0;
  uint8_t old_active_extruder = active_extruder;
  DualXMode dual_mode = dual_x_carriage_mode;
  if (home_offset[Z_AXIS] == 0) {
    LOG_E("Calibrate XY after calibrating Z offset\n");
    return E_CAlIBRATION_XY;
  }
  backup_offset();  //  you can choose to restore offset when exit()
  reset_xy_calibtration_env();
  HOTEND_LOOP() {
    bed_preapare(e);
    motion_control.logical_move_to_z(15 - build_plate_thickness);
    goto_calibtration_position(CAlIBRATION_POS_0);
    motion_control.logical_move_to_z(-2 - build_plate_thickness);
    for (uint8_t axis = 0; axis <= Y_AXIS; axis++) {
      xy_center[e][axis] = 0;
      goto_calibtration_position(CAlIBRATION_POS_0);
      probe_value = probe(axis, -probe_distance, PROBE_XY_FEEDRATE);
      if (probe_value >= abs(probe_distance) - 5) {
        ret =  E_CAlIBRATION_XY;
        LOG_E("e:%d axis:%d probe 0 filed\n", e, axis);
        break;
      }
      float pos = accurate_probe(axis, -probe_distance, PROBE_XY_FEEDRATE);
      goto_calibtration_position(CAlIBRATION_POS_0);
      probe_value = probe(axis, probe_distance, PROBE_XY_FEEDRATE);
      if (probe_value >= abs(probe_distance) - 5) {
        ret = E_CAlIBRATION_XY;
        LOG_E("e:%d axis:%d probe 1 filed\n", e, axis);
        break;
      }
      float pos_1 = accurate_probe(axis, probe_distance, PROBE_XY_FEEDRATE);
      xy_center[e][axis] += (pos_1 + pos) / 2;
      goto_calibtration_position(CAlIBRATION_POS_0);
    }
    if (ret != E_SUCCESS) {
      LOG_E("calibtration e[%e] xy filed\n", e);
      break;
    }
  }
  motion_control.move_z(100);
  motion_control.home_x();
  motion_control.home_y();
  tool_change(old_active_extruder, true);
  dual_x_carriage_mode = dual_mode;
  if(ret == E_SUCCESS) {
    LOG_V("JF-XY calibration: Success!\n");
    LOG_V("JF-XY Extruder1:%f %f\n", xy_center[0][0], xy_center[0][1]);
    LOG_V("JF-XY Extruder2:%f %f\n", xy_center[1][0], xy_center[1][1]);

    set_hotend_offsets(1, X_AXIS, X2_MAX_POS - (xy_center[1][0] - xy_center[0][0]));
    set_hotend_offsets(1, Y_AXIS, -(xy_center[1][1] - xy_center[0][1]));
    LOG_I("JF-Extruder2 hotend offset:%f %f\n", hotend_offset[1].x, hotend_offset[1].y);

    set_home_offset(X_AXIS, calibration_position_xy[0][0] - xy_center[0][0]);
    set_home_offset(Y_AXIS, calibration_position_xy[0][1] - xy_center[0][1]);
    LOG_I("JF-Extruder1 home offset:%f %f\n", home_offset[X_AXIS], home_offset[Y_AXIS]);

    // Store to eeprom
    settings.save();
    return ret;
  }
  else {
    LOG_E("JF-XY calibration: Fail!\n");
    return ret;
  }
}

ErrCode set_hotend_offset(uint8_t axis, float offset) {
  if (axis > Z_AXIS) {
    return E_PARAM;
  }
  hotend_offset[1][axis] = offset;
  return E_SUCCESS;
}

ErrCode Calibtration::exit(bool is_save) {
  LOG_V("exit justing\n");
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
  return E_SUCCESS;
}

// Jumping requires blocking tasks so put it in a loop to do
// Probe once per loop
void Calibtration::loop(void) {
  if (mode == CAlIBRATION_MODE_BED && status == CAlIBRATION_STATE_BED_BEAT) {
    if (bed_probe(cur_pos) != E_SUCCESS) {
      status = CAlIBRATION_STATE_IDLE;
    }
    LOG_V("probe offset:%f\n", probe_offset);
  } else if (mode == CAlIBRATION_MODE_NOZZLE && status == CAlIBRATION_STATE_BED_BEAT) {
    if(bed_probe(cur_pos, 1) != E_SUCCESS) {
      status = CAlIBRATION_STATE_IDLE;
    }
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
  }
  print_control.commands_unlock();
  LOG_I("Apply Z offset: %f\n", home_offset[Z_AXIS]);
}

float Calibtration::get_z_offset() {
  return home_offset[Z_AXIS];
}
