#include "motion_control.h"
#include "src/module/planner.h"
#include "src/module/endstops.h"
#include "src/inc/MarlinConfigPre.h"
#include "src/gcode/parser.h"
#include "src/gcode/gcode.h"
#include "src/module/tool_change.h"
#include "src/module/stepper.h"
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

ErrCode MotionControl::move_axis(mobile_instruction_t *move) {
  xyze_pos_t xyze = current_position;
  float save_feedrate = feedrate_mm_s;
  LOG_I("sc req move to");
  for (uint8_t i = 0; i < move->axis_count; i++) {
    axis_move_t *axis_move = &(&move->axis_move)[i];
    switch (axis_move->axis) {
      case AXIS_X1:
        if (active_extruder == 0) {
          xyze.x = INT_TO_FLOAT(axis_move->distance);
          LOG_I(" x:%f", xyze.x);
        }
        break;
      case AXIS_X2:  // Support activities only one head movement
        if (active_extruder == 1) {
          xyze.x = INT_TO_FLOAT(axis_move->distance);
          LOG_I(" x2:%f", xyze.x);
        }
        break;
      case AXIS_Y1:
        xyze.y += INT_TO_FLOAT(axis_move->distance);
        LOG_I(" y:%f", xyze.y);
        break;
      case AXIS_Z1:
        xyze.z += INT_TO_FLOAT(axis_move->distance);
        LOG_I(" z:%f", xyze.z);
        break;
    }
  }
  uint16_t speed = *((uint16_t*)((uint8_t*)move + 1 + (move->axis_count * sizeof(axis_move_t))));
  LOG_I(" f:%d\n", speed);
  feedrate_mm_s = speed ? MMM_TO_MMS(speed) : feedrate_mm_s;
  blocking_move_to(xyze.x, xyze.y, xyze.z, feedrate_mm_s);
  feedrate_mm_s = save_feedrate;
  synchronize();
  return E_SUCCESS;
}

// pos is float[4], 0-X1  1-X2  2-Y  3-Z
void MotionControl::get_home_pos(float *pos) {
  pos[0] = x_home_pos(0);
  pos[1] = x_home_pos(1);
  pos[2] = Y_MIN_POS;
  pos[3] = Z_MAX_POS;
}

// pos is float[4], 0-X1  1-X2  2-Y  3-Z
void MotionControl::get_xyz_pos(float *pos) {
  pos[0] = x_position();
  pos[1] = x2_position();
  pos[2] = current_position.y;
  pos[3] = current_position.z;
}

ErrCode MotionControl::move_axis_to(mobile_instruction_t *move) {
  xyze_pos_t xyze = current_position;
  bool is_logical = !(*((uint8_t*)move + (3 + (move->axis_count * sizeof(axis_move_t)))));
  float save_feedrate = feedrate_mm_s;
  LOG_I("sc req move");
  if (is_logical) {
    xyze = current_position.asLogical();
  }
  for (uint8_t i = 0; i < move->axis_count; i++) {
    axis_move_t *axis_move = &(&move->axis_move)[i];
    switch (axis_move->axis) {
      case AXIS_X1:
        if (active_extruder == 0) {
          xyze.x = INT_TO_FLOAT(axis_move->distance);
          LOG_I(" x:%f", xyze.x);
        }
        break;
      case AXIS_X2:  // Support activities only one head movement
        if (active_extruder == 1) {
          xyze.x = INT_TO_FLOAT(axis_move->distance);
          LOG_I(" x2:%f", xyze.x);
        }
        break;
      case AXIS_Y1:
        xyze.y = INT_TO_FLOAT(axis_move->distance);
        LOG_I(" y:%f", xyze.y);
        break;
      case AXIS_Z1:
        xyze.z = INT_TO_FLOAT(axis_move->distance);
        LOG_I(" z:%f", xyze.z);
        break;
    }
  }
  uint8_t *data = ((uint8_t*)move + (1 + (move->axis_count * sizeof(axis_move_t))));
  uint16_t speed = *((uint16_t*)data);
  LOG_I(" f:%d", speed);
  LOG_I(" is_logical:%d\n", is_logical);

  feedrate_mm_s = speed ? speed : feedrate_mm_s;
  if (is_logical) {
    logical_move_to_xyz(xyze.x, xyze.y, xyze.z, feedrate_mm_s);
  } else {
    move_to_xyz(xyze.x, xyze.y, xyze.z, feedrate_mm_s);
  }
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
  planner.synchronize();
  uint32_t calibtration_wait = millis() + 20000;
  while (system_service.is_calibtration_status()) {
    if (ELAPSED(millis(), calibtration_wait)) {
      return E_COMMON_ERROR;
    }
    if (xTaskGetCurrentTaskHandle() == thandle_marlin)
      idle();
    else {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    watchdog_refresh();
  }
  home(Z_AXIS);
  home(X_AXIS);
  home(Y_AXIS);
  return E_SUCCESS;
}

void MotionControl::move(uint8_t axis, float distance, uint16_t feedrate) {
  switch (axis) {
    case X_AXIS:
      move_x(distance, feedrate);
      break;
    case Y_AXIS:
      move_y(distance, feedrate);
      break;
    case Z_AXIS:
      move_z(distance, feedrate);
      break;
    case E_AXIS:
      move_e(distance, feedrate);
      break;
  }
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


void MotionControl::move_x_to_relative_home(float x, uint16_t feedrate) {
  if (axis_should_home(X_AXIS)) {
    home_x();
  }
  float x1 = x_position();
  float x2 = x2_position();
  float x1_home = x_home_pos(0);
  float x2_home = x_home_pos(1);
  if ((x1 == x1_home) && (x2 == x2_home)) {
    uint8_t save_active_extruder = active_extruder;
    tool_change(0);
    dual_x_carriage_mode = DXC_MIRRORED_MODE;
    set_duplication_enabled(true);
    idex_set_mirrored_mode(true);
    idex_set_parked(false);
    update_software_endstops(X_AXIS, 0, active_extruder);
    move_to_x(x1_home + x, feedrate);
    planner.synchronize();
    inactive_extruder_x -= x;
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    idex_set_mirrored_mode(false);
    set_duplication_enabled(false);
    tool_change(save_active_extruder);
  } else {
    if (active_extruder == 1) {
      x = -x;
    }
    if (feedrate == 0) {
      feedrate = MOTION_TRAVEL_FEADRATE;
    }
    extruder_duplication_enabled = false;
    dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
    idex_set_mirrored_mode(false);
    set_duplication_enabled(false);
    move_to_x(x_home_pos(active_extruder) + x, feedrate);
    tool_change(!active_extruder);
    move_to_x(x_home_pos(active_extruder) - x, feedrate);
    tool_change(!active_extruder);
  }
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
      if (index == 0) {ret = (X_ENABLE_READ() == X_ENABLE_ON);}
      else {ret = (X2_ENABLE_READ() == X_ENABLE_ON);} break;
    case Y_AXIS: ret = (Y_ENABLE_READ() == Y_ENABLE_ON); break;
    case Z_AXIS: ret = (Z_ENABLE_READ() == Z_ENABLE_ON); break;
    case E_AXIS:
      if (index == 0) {ret = (E0_ENABLE_READ() == E_ENABLE_ON);}
      else {ret = (E1_ENABLE_READ() == E_ENABLE_ON);} break;
  }
  return ret;
}

void MotionControl::init_stall_guard() {
  #define INIT_SG(AXIS) stepper##AXIS.SGTHRS(60); \
                        stepper##AXIS.TPWMTHRS(1); \
                        stepper##AXIS.TCOOLTHRS(0xFFFFF); \
                        ExtiInit(TMC_STALL_GUARD_##AXIS##_PIN, exti_mode); \
                        DisableExtiInterrupt(TMC_STALL_GUARD_##AXIS##_PIN);
  EXTI_MODE_E exti_mode = HW_1_2(EXTI_Falling, EXTI_Rising);
  INIT_SG(X);
  INIT_SG(X2);
  INIT_SG(Y);
  INIT_SG(Z);
  sg_enable_status = 0xf;
}

void MotionControl::enable_stall_guard(uint8_t axis, uint8_t sg_value, uint8_t x_index) {
  #define ENABLE_SG(AXIS) stepper##AXIS.SGTHRS(sg_value); \
                          stepper##AXIS.TPWMTHRS(1); \
                          stepper##AXIS.TCOOLTHRS(0xFFFFF); \
                          if (system_service.get_hw_version() != HW_VER_1) { \
                            set_sg_trigger(SG_##AXIS, false); \
                            set_sg_enable(SG_##AXIS, true); \
                            EnableExtiInterrupt(TMC_STALL_GUARD_##AXIS##_PIN); \
                          }

  switch (axis) {
    case X_AXIS:
      if (x_index == 0 || x_index == 2)
        ENABLE_SG(X);
      if (x_index == 1 || x_index == 2)
        ENABLE_SG(X2);
      break;
    case Y_AXIS:
      ENABLE_SG(Y);
      break;
    case Z_AXIS:
      ENABLE_SG(Z);
      break;
  }
  if (system_service.get_hw_version() == HW_VER_1) {
    set_sg_trigger(0);
    sg_enable_status = 0xf;
    EnableExtiInterrupt(TMC_STALL_GUARD_PIN);
  }
}

void MotionControl::disable_stall_guard(uint8_t axis) {
  #define DISABLE_SG(AXIS) DisableExtiInterrupt(TMC_STALL_GUARD_##AXIS##_PIN);\
                          stepper##AXIS.SGTHRS(0); \
                          stepper##AXIS.TPWMTHRS(1); \
                          stepper##AXIS.TCOOLTHRS(0xFFFFF);
  if (system_service.get_hw_version() == HW_VER_1) {
    sg_enable_status = 0x0;
    DisableExtiInterrupt(TMC_STALL_GUARD_PIN);
  }

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

void MotionControl::enable_stall_guard_only_axis(uint8_t axis, uint8_t sg_value, uint8_t x_index) {
  disable_stall_guard_all();
  enable_stall_guard(axis, sg_value, x_index);
}

void MotionControl::disable_stall_guard_all() {
  LOOP_LOGICAL_AXES(i) {
    disable_stall_guard(i);
  }
}

void MotionControl::wait_G28() {
  while (motion_is_homing) {
    if (xTaskGetCurrentTaskHandle() == thandle_marlin)
      idle();
    else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    watchdog_refresh();
  }
}

void trigger_stall_guard_exit(sg_axis_e axis) {
  stepper.quick_stop();
  motion_control.set_sg_trigger(axis, true);
}

extern "C" {
  void __irq_exti3() {
    if (system_service.get_hw_version() == HW_VER_1) {
      if (stepper.axis_is_moving()) {
        motion_control.set_sg_trigger(0xf);
        stepper.quick_stop();
      }
    } else if (stepper.axis_is_moving() && motion_control.is_sg_enable(SG_X2)) {
      if (active_extruder == 1) {
        trigger_stall_guard_exit(SG_X2);
      }
    }
    ExtiClearITPendingBit(TMC_STALL_GUARD_X2_PIN);
  }

  void __irq_exti9_5() {
    if(ExitGetITStatus(TMC_STALL_GUARD_Z_PIN)) {
      if (stepper.axis_is_moving() && motion_control.is_sg_enable(SG_Z)) {
         trigger_stall_guard_exit(SG_Z);
      }
      ExtiClearITPendingBit(TMC_STALL_GUARD_Z_PIN);
    }

    if(ExitGetITStatus(TMC_STALL_GUARD_Y_PIN)) {
      if (stepper.axis_is_moving() && motion_control.is_sg_enable(SG_Y)) {
         trigger_stall_guard_exit(SG_Y);
      }
      ExtiClearITPendingBit(TMC_STALL_GUARD_Y_PIN);
    }
  }

  void __irq_exti15_10() {
    if(ExitGetITStatus(TMC_STALL_GUARD_X_PIN)) {
      if (stepper.axis_is_moving() && motion_control.is_sg_enable(SG_X)) {
        if (active_extruder == 0) {
          trigger_stall_guard_exit(SG_X);
        }
      }
      ExtiClearITPendingBit(TMC_STALL_GUARD_X_PIN);
    }
  }
}
