#include "calibration.h"
#include "src/module/motion.h"
#include "src/module/endstops.h"
#include "src/module/planner.h"
#include "src/module/stepper.h"
#include "src/module/settings.h"
#include "src/core/macros.h"
#include "src/core/types.h"
#include "move.h"
#include "switch_detect.h"

Calibration calibration;

static float z_offset_cali_pos_xy[2] = {
          X_MAX_POS / 2, 202
        };
static float z_probe_height[4];


/*
* extruder_index: active extruder index of preapare end
*/
void Calibration::preapare(uint8_t extruder_index) {
  extruder_duplication_enabled = false;

  // Store feedrate and feedrate scaling
  remember_feedrate_scaling_off();
  // Enable endstop
  endstops.enable(true);
  if(!(TEST(axis_trusted, X_AXIS) && TEST(axis_trusted, Y_AXIS) && TEST(axis_trusted, Z_AXIS))) {
    // Step1 home all axis
    planner.synchronize();
    homeaxis(Z_AXIS);
  }
  else {
    if(current_position[Z_AXIS] < 30)
      move.move_z(15, 1000);
  }

  active_extruder = !extruder_index;
  homeaxis(X_AXIS);
  active_extruder = extruder_index;
  homeaxis(X_AXIS);

  homeaxis(Y_AXIS);

  if(extruder_index >= EXTRUDERS)
    extruder_index = 0;
  active_extruder_parked = true;
  SERIAL_ECHOPAIR("Cur Pos X:", current_position[X_AXIS]);
  SERIAL_ECHOLNPAIR(" Y:", current_position[Y_AXIS]);
  sync_plan_position();
}

void Calibration::end() {
  endstops.enable(true);
  active_extruder = 1;
  homeaxis(X_AXIS);
  active_extruder = 0;
  homeaxis(X_AXIS);
  homeaxis(Y_AXIS);
  restore_feedrate_and_scaling();
}

/**
  * @brief  Nozzle Probe
  * @param  Axis:Which axis to probe
  * @param  distance: The distance of axis to move down
  * @param  feedrate: Feed rate of the nozzle, in mm/min
  * @retval The probe distance(Posision before - position after)
  */
float Calibration::probe(uint8_t axis, float distance, uint16_t feedrate) {
  float pos_before_probe, pos_after_probe;

  switch_detect.enable_probe();
  pos_before_probe = current_position[axis];

  if(axis == X_AXIS) {
    move.move_x(distance, feedrate);
  }
  else if(axis == Y_AXIS) {
    move.move_y(distance, feedrate);
  }
  else if(axis == Z_AXIS) {
    move.move_z(distance, feedrate);
  }
  planner.synchronize();

  pos_after_probe = stepper.position((AxisEnum)axis) / planner.settings.axis_steps_per_mm[axis];
  current_position[axis] = pos_after_probe;
  sync_plan_position();

  switch_detect.disable_probe();
  return (pos_before_probe - pos_after_probe);
}

/**
  * @brief  Z Axis probe
  * @param  z_distance: The distance of z to move down
  * @param  feedrate: Feed rate of the nozzle, in mm/min
  * @retval The probe distance(Posision before - position after)
  */
float Calibration::z_probe(float z_distance, uint16_t feedrate) {
  return probe(Z_AXIS, z_distance, feedrate);
}

bool Calibration::calibrate_z_offset() {
  uint32_t i;
  bool process_complete;
  uint16_t probe_feedrate;
  float tmp_z_height;
  float z_probe_distance;
  float last_valid_zoffset;

  // Store last valid zoffset for calibration fail
  last_valid_zoffset = home_offset[Z_AXIS];
  set_home_offset(Z_AXIS, 0);
  // Mark calibration success
  process_complete = true;

  // Prepare for calibration
  preapare(0);

  move.move_to_z(15);
  move.move_to_xy(z_offset_cali_pos_xy[0], z_offset_cali_pos_xy[1], 3000);
  probe_feedrate = 150;
  z_probe_distance = 25;
  for(i=0;i<3;i++) {
    tmp_z_height = z_probe(-z_probe_distance, probe_feedrate);
    probe_feedrate = probe_feedrate / 2;
    // Fail
    if(tmp_z_height == -z_probe_distance) {
      process_complete = false;
      break;
    }
    else {
      // Save current Z height
      z_probe_height[0] = current_position[Z_AXIS];
      // Raise Z
      move.move_z(tmp_z_height / 2, 1000);
    }
  }

  // Raise Z
  move.move_z(25, 1000);

  if(process_complete == true) {
    SERIAL_ECHOLN("JF-Z offset calibration: Success!");
    SERIAL_ECHO("JF-Z offset calibration Probe:");
    SERIAL_ECHOLNPAIR_F("JF-Z offset height:", z_probe_height[0], 2);

    // Set Z offset
    set_home_offset(Z_AXIS, -z_probe_height[0]);
    // Store to eeprom
    settings.save();
  }
  else {
    set_home_offset(Z_AXIS, last_valid_zoffset);
    SERIAL_ECHOLN("JF-Z offset calibration: Fail!");
  }
  end();
  return true;
}

