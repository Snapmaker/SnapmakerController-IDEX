#include "adjusting.h"
#include "src/module/settings.h"
#include "../J1/switch_detect.h"
#include "src/module/motion.h"
#include "motion_control.h"
#include "src/module/stepper.h"
#include "src/module/endstops.h"
#include "src/module/tool_change.h"
#include "../../Marlin/src/module/temperature.h"

Adjusting adjusting;

#define PROBE_Z_FEEDRATE 100
#define PROBE_XY_FEEDRATE 50
#define PROBE_MOVE_XY_FEEDRATE 5000
#define PROBE_MOVE_Z_FEEDRATE 600
#define PROBE_LIFTINT_DISTANCE (3)  // mm
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
  /*ADJUST_POS_0*/ {HALF_BED_X, HALF_BED_Y}, 
  /*ADJUST_POS_1*/ {HALF_BED_X, HALF_BED_Y + POS_Y_U_DIFF},
  /*ADJUST_POS_2*/ {HALF_BED_X - POS_X_L_DIFF, HALF_BED_Y - POS_Y_D_DIFF},
  /*ADJUST_POS_3*/ {HALF_BED_X + POS_X_R_DIFF, HALF_BED_Y - POS_Y_D_DIFF},
  /*ADJUST_POS_4*/ {HALF_BED_X - POS_X_L_DIFF, HALF_BED_Y + POS_Y_U_DIFF},
  /*ADJUST_POS_5*/ {HALF_BED_X + POS_X_R_DIFF, HALF_BED_Y + POS_Y_U_DIFF},
};

void Adjusting::bed_preapare(uint8_t extruder_index) {
  // Store feedrate and feedrate scaling
  remember_feedrate_scaling_off();
  // Enable endstop
  endstops.enable(true);
  if(homing_needed()) {
    // Step1 home all axis
    motion_control.home();
    planner.synchronize();
  }
  if (dual_x_carriage_mode > DXC_FULL_CONTROL_MODE) {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    motion_control.home_x();
  }
  float x_need_move_to = !extruder_index ? x_home_pos(1) : x_home_pos(0);
  float cur_other_x = !extruder_index ? x2_position() : x_position();

  if (x_need_move_to != cur_other_x) {
    if(current_position[Z_AXIS] < 30) {
      // Avoid damage to hot bed
      motion_control.move_z(15, PROBE_MOVE_Z_FEEDRATE);
    }
    tool_change(!extruder_index, true);
    motion_control.move_to_x(x_need_move_to, PROBE_MOVE_XY_FEEDRATE);
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
ErrCode Adjusting::goto_position(uint8_t pos) {
  xyz_pos_t offset0 = hotend_offset[0];
  xyz_pos_t offset1 = hotend_offset[1];
  set_hotend_offsets_to_default();
  motion_control.move_to_xy(calibration_position_xy[pos][0], calibration_position_xy[pos][1], PROBE_MOVE_XY_FEEDRATE);
  set_hotend_offsets(0, offset0);
  set_hotend_offsets(1, offset1);
  return E_SUCCESS;
}

float Adjusting::probe(uint8_t axis, float distance, uint16_t feedrate) {
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
    SERIAL_ECHOLNPAIR("probe failed be stall guard!!!");
  } else {
    ret = (pos_before_probe - pos_after_probe);
  }
  // motion_control.disable_stall_guard_all();
  switch_detect.disable_probe();
  return ret;
}

ErrCode Adjusting::probe_z_offset(adjust_position_e pos) {
  float temp_z = 0;
  float position = 0;
  float z_probe_distance = 25;
  float last_valid_zoffset = home_offset[Z_AXIS];

  set_home_offset(Z_AXIS, 0);
  motion_control.move_to_z(15, PROBE_MOVE_Z_FEEDRATE);
  goto_position(pos);
  z_probe_distance = 25;
  temp_z = probe(Z_AXIS, -z_probe_distance, PROBE_Z_FEEDRATE);
  if (temp_z == -z_probe_distance) {
    SERIAL_ECHOLN("failed to probe z !!!");
    set_home_offset(Z_AXIS, last_valid_zoffset);
    return E_ADJUST_PRIOBE;
  }
  motion_control.move_z(PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);

  temp_z = probe(Z_AXIS, -z_probe_distance, PROBE_Z_FEEDRATE);
  position = current_position[Z_AXIS];
  if (temp_z == -z_probe_distance) {
    SERIAL_ECHOLN("failed to probe z !!!");
    set_home_offset(Z_AXIS, last_valid_zoffset);
    return E_ADJUST_PRIOBE;
  } else {
    set_home_offset(Z_AXIS, -(position + build_plate_thickness));
    SERIAL_ECHOLNPAIR("Set z_offset to :", home_offset[Z_AXIS]);
    // Store to eeprom
    settings.save();
  }
  return E_SUCCESS;
}

ErrCode Adjusting::bed_probe(adjust_position_e pos, uint8_t extruder, bool set_z_offset) {
  ErrCode ret = E_SUCCESS;
  float z_probe_distance;
  uint8_t last_active_extruder = active_extruder;
  if (pos == ADJUST_POS_0 || pos >= ADJUST_POS_INVALID) {
    return E_PARAM;
  }
  DualXMode dual_mode = dual_x_carriage_mode;
  bed_preapare(extruder);

  float temp_z = 0;
  if (set_z_offset) {
    ret = probe_z_offset(pos);
  } else {
    goto_position(pos);
    motion_control.logical_move_to_z(Z_REMOVE_PLATE_THICKNESS(PROBE_LIFTINT_DISTANCE), PROBE_MOVE_Z_FEEDRATE);
    z_probe_distance = 15;
    temp_z = probe(Z_AXIS, -z_probe_distance, PROBE_Z_FEEDRATE);
    if (temp_z == -z_probe_distance) {
      SERIAL_ECHOLN("failed to probe z !!!");
      probe_offset = ADJUSTINT_ERR_CODE;
      ret = E_ADJUST_PRIOBE;
    } else {
      probe_offset = current_position[Z_AXIS] + home_offset[Z_AXIS] + build_plate_thickness;
      SERIAL_ECHOLNPAIR("JF-Z offset height:", probe_offset);
    }
  }
  motion_control.logical_move_to_z(Z_REMOVE_PLATE_THICKNESS(PROBE_LIFTINT_DISTANCE), PROBE_MOVE_Z_FEEDRATE);
  dual_x_carriage_mode = dual_mode;
  if (last_active_extruder != active_extruder) {
    tool_change(last_active_extruder, true);
  }
 
  return ret;
}

ErrCode Adjusting::bed_adjust_preapare(adjust_position_e pos, bool is_probe) {
  ErrCode ret = E_SUCCESS;
  if (pos == ADJUST_POS_0 || pos >= ADJUST_POS_INVALID) {
    SERIAL_ECHOLNPAIR("Points not supported by hot bed calibration:", pos);
    return E_PARAM;
  }
  cur_pos = pos;
  mode = ADJUST_MODE_BED;
  status = ADJUST_STATE_IDLE;
  motion_control.synchronize();
  motion_control.move_z(PROBE_LIFTINT_DISTANCE);
  if (is_probe) {
    ret = bed_probe(pos, 0, true);
  } else {
    bed_preapare(0);
    goto_position(pos);
  }
  return ret;
}

ErrCode Adjusting::bed_start_bead_mode() {
  if (mode == ADJUST_MODE_BED) {
    status = ADJUST_STATE_BED_BEAD;
    return E_SUCCESS;
  } else if (mode == ADJUST_MODE_NOZZLE) {
    status = ADJUST_STATE_BED_BEAD;
    return E_SUCCESS;
  }
  return E_PARAM;
}

ErrCode Adjusting::nozzle_adjust_preapare(adjust_position_e pos) {
  if (pos == ADJUST_POS_0 || pos >= ADJUST_POS_INVALID) {
    SERIAL_ECHOLNPAIR("Points not supported by hot nozzle calibration:", pos);
    return E_PARAM;
  }
  cur_pos = pos;
  mode = ADJUST_MODE_NOZZLE;
  status = ADJUST_STATE_IDLE;
  bed_probe(pos, 0, true);

  return E_SUCCESS;
}

void Adjusting::reset_xy_adjust_env() {
  set_hotend_offsets_to_default();
  set_home_offset(X_AXIS, 0);
  set_home_offset(Y_AXIS, 0);
  dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
  if(homing_needed()) {
    // Step1 home all axis
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

ErrCode Adjusting::adjust_xy() {
  ErrCode ret = E_SUCCESS;
  float xy_center[HOTENDS][XY];
  float probe_distance = 15;
  float probe_value = 0;
  uint8_t probe_times = 3;
  uint8_t old_active_extruder = active_extruder;
  DualXMode dual_mode = dual_x_carriage_mode;
  if (home_offset[Z_AXIS] == 0) {
    SERIAL_ECHOLN("Calibrate XY after calibrating Z offset");
    return E_ADJUST_XY;
  }
  reset_xy_adjust_env();
  HOTEND_LOOP() {
    bed_preapare(e);
    motion_control.logical_move_to_z(15 - build_plate_thickness);
    goto_position(ADJUST_POS_0);
    motion_control.logical_move_to_z(-2 - build_plate_thickness);
    for (uint8_t axis = 0; axis <= Y_AXIS; axis++) {
      xy_center[e][axis] = 0;
      for (uint8_t i = 0; i < probe_times && ret == E_SUCCESS; i++) {
        goto_position(ADJUST_POS_0);
        probe_value = probe(axis, -probe_distance, PROBE_XY_FEEDRATE);
        if (probe_value >= abs(probe_distance) - 5) {
          ret =  E_ADJUST_XY;
          SERIAL_ECHOLNPAIR("e:", e, " axis:", axis, " probe 0 filed");
          break;
        }
        float pos = current_position[axis];
        goto_position(ADJUST_POS_0);
        probe_value = probe(axis, probe_distance, PROBE_XY_FEEDRATE);
        if (probe_value >= abs(probe_distance) - 5) {
          ret = E_ADJUST_XY;
          SERIAL_ECHOLNPAIR("e:", e, " axis:", axis, " probe 1 filed");
          break;
        }
        xy_center[e][axis] += (current_position[axis] + pos) / 2;
        goto_position(ADJUST_POS_0);
      }
      xy_center[e][axis] /= probe_times;
    }
    if (ret != E_SUCCESS) {
      SERIAL_ECHOLNPAIR("adjust e[", e,"] xy filed");
      break;
    }
  }
  goto_position(ADJUST_POS_0);
  motion_control.move_z(100);
  tool_change(old_active_extruder, true);
  if(ret == E_SUCCESS) {
    SERIAL_ECHOLN("JF-XY calibration: Success!");
    SERIAL_ECHOPAIR_F("JF-XY Extruder1:", xy_center[0][0]);
    SERIAL_ECHOLNPAIR_F(" ", xy_center[0][1]);
    SERIAL_ECHOPAIR_F("JF-XY Extruder2:", xy_center[1][0]);
    SERIAL_ECHOLNPAIR_F(" ", xy_center[1][1]);

    set_hotend_offsets(1, X_AXIS, X2_MAX_POS - (xy_center[1][0] - xy_center[0][0]));
    set_hotend_offsets(1, Y_AXIS, -(xy_center[1][1] - xy_center[0][1]));
    SERIAL_ECHOPAIR_F("JF-Extruder2 hotend offset:", hotend_offset[1].x);
    SERIAL_ECHOLNPAIR_F(" ", hotend_offset[1].y);

    set_home_offset(X_AXIS, calibration_position_xy[0][0] - xy_center[0][0]);
    set_home_offset(Y_AXIS, calibration_position_xy[0][1] - xy_center[0][1]);
    SERIAL_ECHOPAIR_F("JF-Extruder1 home offset:", home_offset[X_AXIS]);
    SERIAL_ECHOLNPAIR_F(" ", home_offset[Y_AXIS]);

    // Store to eeprom
    settings.save();
    dual_x_carriage_mode = dual_mode;
    motion_control.home_x();
    motion_control.home_y();
    return ret;
  }
  else {
    SERIAL_ECHOLN("JF-XY calibration: Fail!");
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

ErrCode Adjusting::exit() {
  SERIAL_ECHOLN("exit justing");
  mode = ADJUST_MODE_EXIT;
  status = ADJUST_STATE_IDLE;
  probe_offset = ADJUSTINT_ERR_CODE;
  HOTEND_LOOP() {
    thermalManager.setTargetHotend(0, e);
  }
  return E_SUCCESS;
}

// Jumping requires blocking tasks so put it in a loop to do
// Probe once per loop
void Adjusting::loop(void) {
  if (mode == ADJUST_MODE_BED && status == ADJUST_STATE_BED_BEAD) {
    bed_probe(cur_pos);
    SERIAL_ECHOLNPAIR("probe offset:", probe_offset);
  } else if (mode == ADJUST_MODE_NOZZLE && status == ADJUST_STATE_BED_BEAD) {
    bed_probe(cur_pos, 1);
  } else if (mode == ADJUST_MODE_EXIT) {
    mode = ADJUST_MODE_IDLE;
    motion_control.synchronize();
    motion_control.move_to_z(100, PROBE_MOVE_Z_FEEDRATE);
  }
}

void Adjusting::set_z_offset(float offset, bool is_moved) {
  planner.synchronize();
  float cur_z = current_position[Z_AXIS];
  float diff = offset - home_offset[Z_AXIS];
  if (!is_moved) {
    current_position[Z_AXIS] += diff;
  } else {
    motion_control.move_z(diff, 600);
    current_position[Z_AXIS] = cur_z;
  }
  planner.synchronize();
  home_offset[Z_AXIS] = offset;
  sync_plan_position();

  SERIAL_ECHOLNPAIR("Apply Z offset: ", live_z_offset);
}

float Adjusting::get_z_offset() {
  return home_offset[Z_AXIS];
}
