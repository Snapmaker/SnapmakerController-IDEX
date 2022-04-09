#include "motion_control.h"
#include "src/module/planner.h"
#include "src/module/endstops.h"
#include "src/inc/MarlinConfigPre.h"
#include "src/gcode/parser.h"
#include "src/gcode/gcode.h"
#include "src/module/tool_change.h"
#include "src/module/stepper/indirection.h"
#include "../../Marlin/src/feature/tmc_util.h"
#include "system.h"
#include "HAL.h"

MotionControl motion_control;

void MotionControl::synchronize() {
  planner.synchronize();
}

void MotionControl::blocking_move_to(float x, float y, float z, float feedrate) {
  float save_feedrate = feedrate_mm_s;
  xyze_pos_t xyz = current_position;
  xyz[X_AXIS] = x;
  xyz[Y_AXIS] = y;
  xyz[Z_AXIS] = z;
  apply_motion_limits(xyz);
  do_blocking_move_to(xyz, feedrate);
  feedrate_mm_s = save_feedrate;
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
  blocking_move_to(xyze.x, xyze.y, xyze.z, feedrate_mm_s);
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
  blocking_move_to(xyze.x, xyze.y, xyze.z, feedrate_mm_s);
  feedrate_mm_s = save_feedrate;
  synchronize();
  return E_SUCCESS;
}

ErrCode MotionControl::home(AxisEnum axis) {
  uint8_t save_active_extruder;
  bool save_dup_enable;
  save_dup_enable = extruder_duplication_enabled;
  DualXMode dual_mode = dual_x_carriage_mode;
  save_active_extruder = active_extruder;
  extruder_duplication_enabled = false;
  endstops.enable(true);
  if (axis == X_AXIS) {
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    tool_change(1);
    homeaxis(X_AXIS);
    tool_change(0);
    homeaxis(X_AXIS);
    tool_change(save_active_extruder);
    dual_x_carriage_mode = dual_mode;
    if (dual_x_carriage_mode >= DXC_DUPLICATION_MODE) {
      idex_set_parked();
    }
  } else {
    homeaxis(axis);
  }
  endstops.not_homing();
  extruder_duplication_enabled = save_dup_enable;
  return E_SUCCESS;
}

ErrCode MotionControl::home_x() {
  home(X_AXIS);
  return E_SUCCESS;
}

ErrCode MotionControl::home_y() {
  home(Y_AXIS);
  return E_SUCCESS;
}

ErrCode MotionControl::home_z() {
  home(Z_AXIS);
  return E_SUCCESS;
}


ErrCode MotionControl::home() {
  quickstop_stepper();
  home(Z_AXIS);
  home(X_AXIS);
  home(Y_AXIS);
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
  blocking_move_to(current_position.x + x, current_position.y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_y(float y, uint16_t feedrate) {
  blocking_move_to(current_position.x, current_position.y + y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_z(float z, uint16_t feedrate) {
  blocking_move_to(current_position.x, current_position.y, current_position.z + z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_xy(float x, float y, uint16_t feedrate) {
  blocking_move_to(current_position.x + x, current_position.y + y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_x(float x, uint16_t feedrate) {
  blocking_move_to(x, current_position.y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_y(float y, uint16_t feedrate) {
  blocking_move_to(current_position.x, y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_z(float z, uint16_t feedrate) {
  blocking_move_to(current_position.x, current_position.y, z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_xyz(float x, float y, float z, uint16_t feedrate) {
  blocking_move_to(x, y, z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_xyz(xyze_pos_t &pos, uint16_t feedrate) {
  blocking_move_to(pos.x, pos.y, pos.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}

void MotionControl::move_to_xy(float x, float y, uint16_t feedrate) {
  blocking_move_to(x, y, current_position.z, MMM_TO_MMS(feedrate));
  planner.synchronize();
}


void MotionControl::retrack_e(float distance, uint16_t feedrate) {
  move_e(-distance, feedrate);
}

void MotionControl::extrude_e(float distance, uint16_t feedrate) {
  move_e(distance, feedrate);
}

void MotionControl::motor_enable(uint8_t axis, uint8_t index) {
  switch (axis) {
    case X_AXIS:
      if (index == 0) ENABLE_STEPPER_X();
      else ENABLE_STEPPER_X2(); break;
    case Y_AXIS: ENABLE_AXIS_Y(); break;
    case Z_AXIS: ENABLE_AXIS_Z(); break;
    case E_AXIS: 
      if (index == 0) ENABLE_STEPPER_E0();
      else ENABLE_STEPPER_E1(); break;
  }
}

void MotionControl::motor_disable(uint8_t axis, uint8_t index) {
  switch (axis) {
    case X_AXIS:
      if (index == 0) DISABLE_STEPPER_X();
      else DISABLE_STEPPER_X2(); break;
    case Y_AXIS: DISABLE_AXIS_Y(); break;
    case Z_AXIS: DISABLE_AXIS_Z(); break;
    case E_AXIS: 
      if (index == 0) DISABLE_STEPPER_E0();
      else DISABLE_STEPPER_E1(); break;
  }
  set_all_unhomed();
}

bool MotionControl::is_motor_enable(uint8_t axis, uint8_t index) {
  bool ret = false;
  switch (axis) {
    case X_AXIS:
      if (index == 0) {ret = X_ENABLE_READ();}
      else {ret = X2_ENABLE_READ();} break;
    case Y_AXIS: ret = Y_ENABLE_READ(); break;
    case Z_AXIS: ret = Z_ENABLE_READ(); break;
    case E_AXIS: 
      if (index == 0) {ret = E0_ENABLE_READ();}
      else {ret = E1_ENABLE_READ();} break;
  }
  return ret;
}

void MotionControl::enable_stall_guard(uint8_t axis, uint8_t sg_value) {
  #define ENABLE_SG(AXIS) stepper##AXIS.SGTHRS(sg_value); \
                          stepper##AXIS.TPWMTHRS(1); \
                          stepper##AXIS.TCOOLTHRS(0xFFFFF)
  switch (axis) {
    case X_AXIS:
      ENABLE_SG(X);
      ENABLE_SG(X2);
      break;
    case Y_AXIS:
      ENABLE_SG(Y);
      break;
    case Z_AXIS:
      ENABLE_SG(Z);
      break;
  }
  ExtiInit(TMC_STALL_GUARD_PIN, EXTI_Falling);
  EnableExtiInterrupt(TMC_STALL_GUARD_PIN);
  sg_enable = true;
  motion_control.set_sg_satats(false);
}

void MotionControl::disable_stall_guard(uint8_t axis) {
  #define DISABLE_SG(AXIS) stepper##AXIS.SGTHRS(0); \
                          stepper##AXIS.TPWMTHRS(1); \
                          stepper##AXIS.TCOOLTHRS(0xFFFFF);

  switch (axis) {
    case X_AXIS:
      DISABLE_SG(X);
      DISABLE_SG(X2);
      break;
    case Y_AXIS:
      DISABLE_SG(Y);
      break;
    case Z_AXIS:
      DISABLE_SG(Z);
      break;
  }
}

void MotionControl::enable_stall_guard_only_axis(uint8_t axis, uint8_t sg_value) {
  disable_stall_guard_all();
  enable_stall_guard(axis, sg_value);
}

void MotionControl::disable_stall_guard_all() {
  LOOP_LOGICAL_AXES(i) {
    disable_stall_guard(i);
  }
  DisableExtiInterrupt(TMC_STALL_GUARD_PIN);
  motion_control.set_sg_satats(false);
  sg_enable = false;
}

void trigger_stall_guard_exit() {
  if (planner.has_blocks_queued() || planner.cleaning_buffer_counter) {
    planner.quick_stop();
    motion_control.set_sg_satats(true);
  }
}

extern "C" {
  void __irq_exti3() {
    trigger_stall_guard_exit();
    ExtiClearITPendingBit(TMC_STALL_GUARD_PIN);
  }
}
