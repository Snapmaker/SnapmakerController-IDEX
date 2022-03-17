#include "adjusting.h"
#include "src/module/settings.h"
#include "../J1/switch_detect.h"
#include "src/module/motion.h"
#include "motion_control.h"
#include "src/module/stepper.h"
#include "src/module/endstops.h"
#include "src/module/tool_change.h"

Adjusting adjusting;

#define PROBE_Z_FEEDRATE 60
#define PROBE_XY_FEEDRATE 200
#define PROBE_MOVE_XY_FEEDRATE 5000
#define PROBE_MOVE_Z_FEEDRATE 2000
#define PROBE_LIFTINT_DISTANCE (6 - build_plate_thickness)  // mm

static float build_plate_thickness = 5;

#define Y_POS_DIFF 4
static float calibration_position_xy[6][2] = {
  /*ADJUST_POS_0*/ {(X_MAX_POS / 2), (Y_MAX_POS / 2) - Y_POS_DIFF}, 
  /*ADJUST_POS_1*/ {X_MAX_POS / 2, 196 - Y_POS_DIFF},
  /*ADJUST_POS_2*/ {42, 8 - Y_POS_DIFF},
  /*ADJUST_POS_3*/ {272, 8 - Y_POS_DIFF},
  /*ADJUST_POS_4*/ {42, 196 - Y_POS_DIFF},
  /*ADJUST_POS_5*/ {272, 196 - Y_POS_DIFF},
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
  float x_need_move_to = !extruder_index ? X2_MAX_POS : X_MIN_POS;
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

// Run to the specified calibration point
ErrCode Adjusting::goto_position(uint8_t pos) {
  motion_control.move_to_xy(calibration_position_xy[pos][0], calibration_position_xy[pos][1], PROBE_MOVE_XY_FEEDRATE);
  return E_SUCCESS;
}

float Adjusting::probe(uint8_t axis, float distance, uint16_t feedrate) {
    float pos_before_probe, pos_after_probe;

  switch_detect.enable_probe();
  pos_before_probe = current_position[axis];

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

  switch_detect.disable_probe();
  return (pos_before_probe - pos_after_probe);
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
  bool duplication_enabled = extruder_duplication_enabled;
  if (pos == ADJUST_POS_0 || pos >= ADJUST_POS_INVALID) {
    return E_PARAM;
  }
  extruder_duplication_enabled = false;
  bed_preapare(extruder);

  float temp_z = 0;
  if (set_z_offset) {
    ret = probe_z_offset(pos);
  } else {
    goto_position(pos);
    motion_control.logical_move_to_z(PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);
    z_probe_distance = 15;
    temp_z = probe(Z_AXIS, -z_probe_distance, PROBE_Z_FEEDRATE);
    if (temp_z == -z_probe_distance) {
      SERIAL_ECHOLN("failed to probe z !!!");
      probe_offset = ADJUSTINT_ERR_CODE;
      ret = E_ADJUST_PRIOBE;
    } else {
      probe_offset = current_position[Z_AXIS] + home_offset[Z_AXIS];
      SERIAL_ECHOLNPAIR("JF-Z offset height:", probe_offset);
    }
  }
  motion_control.logical_move_to_z(PROBE_LIFTINT_DISTANCE, PROBE_MOVE_Z_FEEDRATE);
  extruder_duplication_enabled = duplication_enabled;
  if (last_active_extruder != active_extruder) {
    tool_change(last_active_extruder, true);
  }
 
  return ret;
}

ErrCode Adjusting::bed_adjust_preapare(adjust_position_e pos, bool is_probe) {
  if (pos == ADJUST_POS_0 || pos >= ADJUST_POS_INVALID) {
    SERIAL_ECHOLNPAIR("Points not supported by hot bed calibration:", pos);
    return E_PARAM;
  }
  cur_pos = pos;
  mode = ADJUST_MODE_BED;
  status = ADJUST_STATE_IDLE;
  motion_control.synchronize();
  if (is_probe) {
    bed_probe(pos, 0, true);
  } else {
    bed_preapare(0);
    goto_position(pos);
  }
  return E_SUCCESS;
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

ErrCode Adjusting::adjust_xy() {
  ErrCode ret = E_SUCCESS;
  float xy_center[HOTENDS][XY];
  float probe_distance = 15;
  float probe_value = 0;
  uint8_t old_active_extruder = active_extruder;
  bool duplication_enabled = extruder_duplication_enabled;

  if (home_offset[Z_AXIS] == 0) {
    SERIAL_ECHOLN("Calibrate XY after calibrating Z offset");
    return E_ADJUST_XY;
  }
  extruder_duplication_enabled = false;
  HOTEND_LOOP() {
    hotend_offset[e].x = 0;
    hotend_offset[e].y = 0;
    bed_preapare(e);
    motion_control.logical_move_to_z(15 - build_plate_thickness);
    goto_position(ADJUST_POS_0);
    motion_control.logical_move_to_z(-2 - build_plate_thickness);
    for (uint8_t axis = 0; axis <= Y_AXIS; axis++) {
      goto_position(ADJUST_POS_0);
      probe_value = probe(axis, -probe_distance, PROBE_XY_FEEDRATE);
      if (probe_value == -probe_distance) {
        ret =  E_ADJUST_XY;
        SERIAL_ECHOLNPAIR("e:", e, " axis:", axis, " probe 0 filed");
        break;
      }
      float pos = current_position[axis];
      goto_position(ADJUST_POS_0);
      probe_value = probe(axis, probe_distance, PROBE_XY_FEEDRATE);
      if (probe_value == probe_distance) {
        ret = E_ADJUST_XY;
        SERIAL_ECHOLNPAIR("e:", e, " axis:", axis, " probe 1 filed");
        break;
      }
      xy_center[e][axis] = (current_position[axis] + pos) / 2;
      goto_position(ADJUST_POS_0);
    }
    if (ret != E_SUCCESS) {
      SERIAL_ECHOLNPAIR("adjust e[", e,"] xy filed");
      break;
    }
  }
  goto_position(ADJUST_POS_0);
  motion_control.move_z(100);
  tool_change(old_active_extruder, true);
  extruder_duplication_enabled = duplication_enabled;
  if(ret == E_SUCCESS) {
    SERIAL_ECHOLN("JF-XY calibration: Success!");
    SERIAL_ECHOPAIR_F("JF-XY Extruder1:", xy_center[0][0]);
    SERIAL_ECHOLNPAIR_F(" ", xy_center[0][1]);
    SERIAL_ECHOPAIR_F("JF-XY Extruder2:", xy_center[1][0]);
    SERIAL_ECHOLNPAIR_F(" ", xy_center[1][1]);

    hotend_offset[1].x -= (xy_center[1][0] - xy_center[0][0]);
    hotend_offset[1].y -= (xy_center[1][1] - xy_center[0][1]);
    SERIAL_ECHOPAIR_F("JF-Extruder2 hotend offset:", hotend_offset[1].x);
    SERIAL_ECHOLNPAIR_F(" ", hotend_offset[1].y);
    // Store to eeprom
    settings.save();
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
  mode = ADJUST_MODE_IDLE;
  status = ADJUST_STATE_IDLE;
  probe_offset = ADJUSTINT_ERR_CODE;
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
  }
}

