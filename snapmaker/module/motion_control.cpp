#include "motion_control.h"
#include "src/module/motion.h"
#include "src/module/planner.h"
#include "src/module/endstops.h"
#include "src/inc/MarlinConfigPre.h"
#include "src/gcode/parser.h"
#include "src/gcode/gcode.h"
#include "system.h"

MotionControl motion_control;

void MotionControl::synchronize() {
  planner.synchronize();
}

ErrCode MotionControl::move_axis(mobile_instruction_t move) {
  xyze_pos_t xyze = current_position;
  float save_feedrate = feedrate_mm_s;
  switch (move.axis) {
    case AXIS_X1:
    case AXIS_X2:  // Support activities only one head movement
      xyze.x += INT_TO_FLOAT(move.distance);
      break;
    case AXIS_Y1:
      xyze.y += INT_TO_FLOAT(move.distance);
      break;
    case AXIS_Z1:
      xyze.z += INT_TO_FLOAT(move.distance);
      break;
  }
  feedrate_mm_s = move.feedrate ? MMM_TO_MMS(move.feedrate) : feedrate_mm_s;
  do_blocking_move_to(xyze.x, xyze.y, xyze.z, feedrate_mm_s);
  synchronize();
  feedrate_mm_s = save_feedrate;
  return E_SUCCESS;
}

ErrCode MotionControl::move_axis_to(mobile_instruction_t move) {
  xyze_pos_t xyze = current_position;
  float save_feedrate = feedrate_mm_s;
  switch (move.axis) {
    case AXIS_X1:
    case AXIS_X2:  // Support activities only one head movement
      xyze.x = INT_TO_FLOAT(move.distance);
      break;
    case AXIS_Y1:
      xyze.y = INT_TO_FLOAT(move.distance);
      break;
    case AXIS_Z1:
      xyze.z = INT_TO_FLOAT(move.distance);
      break;
  }
  feedrate_mm_s = move.feedrate ? MMM_TO_MMS(move.feedrate) : feedrate_mm_s;
  do_blocking_move_to(xyze.x, xyze.y, xyze.z, feedrate_mm_s);
  feedrate_mm_s = save_feedrate;
  synchronize();
  return E_SUCCESS;
}

ErrCode MotionControl::home(uint8_t axis) {
  switch (axis) {
    case 0:
      parser.parse((char *)"G28");
      break;
    case 1:
      parser.parse((char *)"G28 X");
      break;
    case 2:
      parser.parse((char *)"G28 Y");
      break;
    case 3:
      parser.parse((char *)"G28 Z");
      break;
    default:
      return E_PARAM;
  }
  gcode.process_parsed_command();
  return E_SUCCESS;
}

ErrCode MotionControl::home() {
  parser.parse((char *)"G28");
  gcode.process_parsed_command();
  return E_SUCCESS;
}

ErrCode MotionControl::move_e(float distance, uint16_t feedrate) {
  float save_feedrate = feedrate_mm_s;
  feedrate_mm_s = MMM_TO_MMS(feedrate);
  destination = current_position;
  destination.e = destination.e + distance;
  prepare_line_to_destination();
  feedrate_mm_s = save_feedrate;
  return E_SUCCESS;
}

void MotionControl::move_x(float x, uint16_t feedrate) {
  do_blocking_move_to(current_position.x + x, current_position.y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_y(float y, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, current_position.y + y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_z(float z, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, current_position.y, current_position.z + z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_xy(float x, float y, uint16_t feedrate) {
  do_blocking_move_to(current_position.x + x, current_position.y + y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_x(float x, uint16_t feedrate) {
  do_blocking_move_to(x, current_position.y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_y(float y, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_z(float z, uint16_t feedrate) {
  do_blocking_move_to(current_position.x, current_position.y, z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_xyz(float x, float y, float z, uint16_t feedrate) {
  do_blocking_move_to(x, y, z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_xy(float x, float y, uint16_t feedrate) {
  do_blocking_move_to(x, y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}


void MotionControl::retrack_e(float distance, uint16_t feedrate) {
  move_e(-distance, feedrate);
}

void MotionControl::extrude_e(float distance, uint16_t feedrate) {
  move_e(distance, feedrate);
}
