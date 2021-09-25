#include "src/inc/MarlinConfigPre.h"
#include "src/core/millis_t.h"
#include "HAL.h"
#include "src/module/motion.h"
#include "src/module/temperature.h"
#include "src/module/planner.h"
#include "src/module/endstops.h"
#include "move.h"

Move move;

void Move::move_to_xyz(float x, float y, float z) {
  move_to_xyz(x, y, z, 0);
}

void Move::move_to_xyz(float x, float y, float z, uint16_t feedrate) {
  do_blocking_move_to(x, y, z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_to_x(float x) {
  move_to_x(x, 0);
}

void Move::move_to_y(float y) {
  move_to_y(y, 0);
}

void Move::move_to_z(float z) {
  move_to_z(z, 0);
}

void Move::move_to_x(float x, uint16_t feedrate) {
  do_blocking_move_to(x, current_position.y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_to_y(float y, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_to_z(float z, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, current_position.y, z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_to_xy(float x, float y, uint16_t feedrate) {
  do_blocking_move_to(x, y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_x(float x) {
  move_x(x, 0);
}

void Move::move_y(float y) {
  move_y(y, 0);
}

void Move::move_z(float z) {
  move_z(z, 0);
}

void Move::move_x(float x, uint16_t feedrate) {
  do_blocking_move_to(current_position.x + x, current_position.y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_y(float y, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, current_position.y + y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_z(float z, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, current_position.y, current_position.z + z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_xy(float x, float y, uint16_t feedrate) {
  do_blocking_move_to(current_position.x + x, current_position.y + y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void Move::move_to_work_x(float x, uint16_t feedrate) {
  destination = current_position;
  destination.x = x;
  feedrate_mm_s = MMM_TO_MMS(feedrate);
  prepare_line_to_destination();
  planner.synchronize();
}

void Move::move_to_work_y(float y, uint16_t feedrate) {
  destination = current_position;
  destination.y = y;
  feedrate_mm_s = MMM_TO_MMS(feedrate);
  prepare_line_to_destination();
  planner.synchronize();
}

void Move::move_to_work_z(float z, uint16_t feedrate) {
  destination = current_position;
  destination.z = z;
  feedrate_mm_s = MMM_TO_MMS(feedrate);
  prepare_line_to_destination();
  planner.synchronize();
}

void Move::move_to_work_xy(float x, float y, uint16_t feedrate) {
  destination = current_position;
  destination.x = x;
  destination.y = y;
  feedrate_mm_s = MMM_TO_MMS(feedrate);
  prepare_line_to_destination();
  planner.synchronize();
}

void Move::retrack_e(float distance, uint16_t feedrate) {
    float save_feedrate = feedrate_mm_s;
    feedrate_mm_s = MMM_TO_MMS(feedrate);
    destination = current_position;
    destination.e = destination.e - distance;
    prepare_line_to_destination();
    feedrate_mm_s = save_feedrate;
}

void Move::extrude_e(float distance, uint16_t feedrate) {
    float save_feedrate = feedrate_mm_s;
    feedrate_mm_s = MMM_TO_MMS(feedrate);
    destination = current_position;
    destination.e = destination.e + distance;
    prepare_line_to_destination();
    feedrate_mm_s = save_feedrate;
}

void Move::set_extruder_position(float new_position) {
  current_position.e = new_position;
  sync_plan_position_e();
}

void Move::home_axis() {
  planner.synchronize();
  homeaxis(Z_AXIS);
  active_extruder = 1;
  homeaxis(X_AXIS);
  inactive_extruder_x = current_position.x;
  active_extruder = 0;
  homeaxis(X_AXIS);
  homeaxis(Y_AXIS);
  active_extruder_parked = true;
}

void Move::home_all_axis_smart() {
  uint8_t save_active_extruder;

  save_active_extruder = active_extruder;
  planner.synchronize();
  if(!TEST(axis_trusted, Z_AXIS)) {
    homeaxis(Z_AXIS);
    set_axis_is_at_home(Z_AXIS);
  }

  if(!TEST(axis_trusted, X_AXIS)) {
    active_extruder = 1;
    homeaxis(X_AXIS);
    inactive_extruder_x = current_position.x;
    active_extruder = 0;
    homeaxis(X_AXIS);
  }
  if(!TEST(axis_trusted, Y_AXIS)) {
    homeaxis(Y_AXIS);
  }
  active_extruder = save_active_extruder;
  active_extruder_parked = true;
}

void Move::packing_home() {
  uint8_t save_active_extruder;
  bool save_dup_enable;

  save_dup_enable = extruder_duplication_enabled;
  save_active_extruder = active_extruder;
  extruder_duplication_enabled = false;
  endstops.enable(true);
  active_extruder = 1;
  homeaxis(X_AXIS);
  active_extruder = 0;
  homeaxis(X_AXIS);
  homeaxis(Y_AXIS);
  endstops.not_homing();
  active_extruder = save_active_extruder;
  extruder_duplication_enabled = save_dup_enable;
}
