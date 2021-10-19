#include "calibration.h"
#include "src/module/motion.h"
#include "src/module/endstops.h"
#include "src/module/planner.h"
#include "src/module/stepper.h"
#include "src/module/settings.h"
#include "src/module/tool_change.h"
#include "src/core/macros.h"
#include "src/core/types.h"
#include "move.h"
#include "switch_detect.h"
#include "tmc_driver.h"

Calibration calibration;

static float z_offset_cali_pos_xy[2] = {
          X_MAX_POS / 2, 202
        };
static float z_probe_height[4];

static float bed_cali_pos_xy[4][2] = {
  {X_MAX_POS / 2, 202},
  {4, 10 + 3},
  {227 + 96, 10 + 3},
  {X_MAX_POS / 2, 178},
};

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

  x_home();
  tool_change(extruder_index, true);

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
  x_home();
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

void Calibration::move_to_bed_calibration_point(uint8_t index) {
  move.move_to_xy(bed_cali_pos_xy[index][X_AXIS], bed_cali_pos_xy[index][Y_AXIS], Z_CAIL_FEEDRATE);
}

bool Calibration::bed_leveling_probe(uint8_t extruder_index) {
  uint32_t i;
  uint32_t j;
  bool process_complete;
  uint16_t probe_feedrate;
  float tmp_z_height;
  float z_probe_distance;
  float last_valid_zoffset;

  // Store last valid zoffset for calibration fail
  last_valid_zoffset = home_offset[Z_AXIS];
  
  set_home_offset(Z_AXIS, 0);

  process_complete = true;

  preapare(extruder_index);

  // Probe the calibration point
  for(i=0;i<3; i++) {

    move.move_to_z(35, 200);
    // Move to the X Y calibration position
    move_to_bed_calibration_point(i);

    z_probe_distance = 40;

    // probe_feedrate = 400;
    for(j=0;j<3;j++) {
      while(!READ(X0_CAL_PIN));

      if(j == 0) {
        tmc_driver.configure_for_platform_calibration(162);
        probe_feedrate = 300;
      }
      else {
        // Unused, the volacity is too low
        probe_feedrate = 60;
      }
      tmp_z_height = z_probe(-z_probe_distance, probe_feedrate);
      z_probe_height[i] = current_position[Z_AXIS];

      if(j == 0) 
        tmc_driver.configure_for_platform_calibration(40);
      
      // Fail
      if(tmp_z_height == -z_probe_distance) {
        process_complete = false;
        break;
      }
      else {
        // Raise Z
        move.move_z(5, 200);
      }
      // probe_feedrate = probe_feedrate / 2;
    }
    // z_probe_height[i] = tmp_z_height;
    SERIAL_ECHOLNPAIR_F("Probe z height:", z_probe_height[i], 2);
    if(process_complete == false)
      break;
  }

  move.move_z(35, 200);

  tmc_driver.disable_stall_guard();

  homeaxis(X_AXIS);
  homeaxis(Y_AXIS);

  // Restore last valid zoffset
  set_home_offset(Z_AXIS, last_valid_zoffset);

  // Report
  if(process_complete == true) {
    SERIAL_ECHOLN("JF-Bed Level: Success!");
    SERIAL_ECHO("JF-Bed Level Probe:");
    SERIAL_ECHOPAIR_F("JF-0:", z_probe_height[0], 2);
    SERIAL_ECHOPAIR_F(" 1:", z_probe_height[1], 2);
    SERIAL_ECHOPAIR_F(" 2:", z_probe_height[2], 2);
    SERIAL_ECHOLNPAIR_F_P(" 3:", z_probe_height[3], 2);
    return true;
  }
  else {
    SERIAL_ECHOLN("JF-Bed Level: Fail!");
    return false;
  }
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

void Calibration::x_home() {
  tool_change(1, true);
  homeaxis(X_AXIS);
  tool_change(0, true);
  homeaxis(X_AXIS);
}

bool Calibration::calibrate_z_offset() {
  uint32_t i;
  bool process_complete;
  uint16_t probe_feedrate;
  float tmp_z_height;
  float z_probe_distance;
  float last_valid_zoffset;
  uint8_t old_active_extruder = active_extruder;
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
  tool_change(old_active_extruder, true);
  return true;
}

/**
  * @brief  Probe nozzle
  * @param  Extruder: Extruder index
  * @retval True if probe success.
  */
bool Calibration::probe_nozzle(uint8_t extruder) {
  uint32_t i;
  bool process_complete;
  uint16_t probe_feedrate;
  float tmp_z_height;
  float z_probe_distance;

  // Mark calibration success
  process_complete = true;

  // Prepare for calibration
  preapare(extruder);

  move.move_to_z(35);
  move.move_to_xy(z_offset_cali_pos_xy[0], z_offset_cali_pos_xy[1], 1500);

  z_probe_distance = 30;
  for(i=0;i<3;i++) {
    if(i == 0) {
      tmc_driver.configure_for_platform_calibration(162);
      probe_feedrate = 400;
    }
    else {
      tmc_driver.configure_for_platform_calibration(70);
      probe_feedrate = 60;
    }
    tmp_z_height = z_probe(-z_probe_distance, probe_feedrate);
    // Fail
    if(tmp_z_height == -z_probe_distance) {
      process_complete = false;
      break;
    }
    else {
      // Save current Z height
      z_probe_height[0] = current_position[Z_AXIS];
      // Raise Z
      move.move_z(5, 200);
    }
  }
  tmc_driver.disable_stall_guard();

  // Raise Z
  move.move_z(35, 200);
  active_extruder = 0;
  homeaxis(X_AXIS);
  active_extruder = 1;
  homeaxis(X_AXIS);
  active_extruder = 0;
  homeaxis(X_AXIS);
  
  if(process_complete == true) {
    SERIAL_ECHOLN("JF-Z offset calibration: Success!");
    SERIAL_ECHO("JF-Z offset calibration Probe:");
    SERIAL_ECHOLNPAIR_F("JF-Z offset height:", z_probe_height[0], 2);
    return true;
  }
  else {
    SERIAL_ECHOLN("JF-Z offset calibration: Fail!");
    return false;
  }
}

/**
  * @brief  Get the Z probe height of the last
  * @param  height: The buffer to store the height
  * @retval None
  */
void Calibration::get_z_probe_height(float *height) {
  height[0] = z_probe_height[0];
  height[1] = z_probe_height[1];
  height[2] = z_probe_height[2];
  height[3] = z_probe_height[3];
}

/**
  * @brief  Nozzle probe process
  * @retval Reply data length
  */
bool Calibration::calibrate_nozzle_height() {
  float nozzle_height[2][4];
  uint32_t i;
  int32_t turn_round;
  float del;
  bool result = true;

  for(i=0;i<2;i++) {
    if(probe_nozzle(i) == true) {
      get_z_probe_height(nozzle_height[i]);
    }
    else {
      result = false;
      break;
    }
  }

  if(result == false) {
    SERIAL_ECHOLN("nozzle height calibration: Fail!");
    return result;
  }

  // Caculate the deltal z. 
  del = nozzle_height[1][0] - nozzle_height[0][0];
  turn_round = (del * 1000 / ADJUST_NOZZLE_MM_PER_STEP);
  turn_round = turn_round - (turn_round % 1000);

  if((turn_round > -2000) && (turn_round < 2000)) {
    turn_round = 0;
  } else {
    turn_round /= 1000;
  }
  SERIAL_ECHOLNPAIR("right nozzle height turn round:", turn_round);
  return 0;
}


bool Calibration::calibrate_platform() {
  float nozzle_height[4];
  uint32_t i;
  int32_t turn_round[4];
  float del[4];
  float ref;

  // TODO : Need check that the nozzle is working 

  if (bed_leveling_probe(0) == true) {
    get_z_probe_height(nozzle_height);
  } else {
    return false;
  }

  // Set Point3 as reference
  ref = nozzle_height[1];
  nozzle_height[3] = nozzle_height[1];

  // Greater than 0 means need to turn upper
  for(i=0;i<4;i++) {
    del[i] = nozzle_height[i] - ref;
    turn_round[i] = (del[i] * 1000 / ADJUST_BED_MM_PER_STEP);
    turn_round[i] = turn_round[i] - (turn_round[i] % 1000);
    if((turn_round[i] > -2000) && (turn_round[i] < 2000))
      turn_round[i] = 0;
  }

  SERIAL_ECHOLNPAIR("turn left bed round:", turn_round[0] / 1000);
  SERIAL_ECHOLNPAIR("turn right bed round:", turn_round[2] / 1000);

  return true;
}