#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "../J1/common_type.h"
#include "src/core/types.h"
#include "src/module/motion.h"

#pragma pack(1)
typedef struct {
  uint8_t axis;
  float_to_int_t distance;
  uint16_t feedrate;
}mobile_instruction_t;
#pragma pack(0)
class MotionControl {
  public:
    void blocking_move_to(float x, float y, float z, float feedrate=0);
    ErrCode move_axis(mobile_instruction_t move);
    ErrCode move_axis_to(mobile_instruction_t move);
    ErrCode home(AxisEnum axis);
    ErrCode home();
    ErrCode home_x();
    ErrCode home_y();
    ErrCode home_z();
    ErrCode move_e(float distance, uint16_t feedrate);
    void move_x(float x, uint16_t feedrate=0);
    void move_y(float y, uint16_t feedrate=0);
    void move_z(float z, uint16_t feedrate=0);
    void move_xy(float x, float y, uint16_t feedrate);
    void move_to_x(float x, uint16_t feedrate=0);
    void move_to_y(float y, uint16_t feedrate=0);
    void move_to_z(float z, uint16_t feedrate=0);
    void move_to_xyz(float x, float y, float z, uint16_t feedrate=0);
    void move_to_xyz(xyze_pos_t &pos, uint16_t feedrate=0);
    void move_to_xy(float x, float y, uint16_t feedrate);

    void logical_move_x(float x, uint16_t feedrate=0) {
      move_x(LOGICAL_TO_NATIVE(x, X_AXIS), feedrate);
    }
    void logical_move_y(float y, uint16_t feedrate=0) {
      move_y(LOGICAL_TO_NATIVE(y, Y_AXIS), feedrate);
    }
    void logical_move_z(float z, uint16_t feedrate=0) {
      move_z(LOGICAL_TO_NATIVE(z, Z_AXIS), feedrate);
    }
    void logical_move_xy(float x, float y, uint16_t feedrate) {
      move_xy(LOGICAL_TO_NATIVE(x, X_AXIS), LOGICAL_TO_NATIVE(y, Y_AXIS), feedrate);
    }
    void logical_move_to_x(float x, uint16_t feedrate=0) {
      move_to_x(LOGICAL_TO_NATIVE(x, X_AXIS), feedrate);
    }
    void logical_move_to_y(float y, uint16_t feedrate=0) {
      move_to_y(LOGICAL_TO_NATIVE(y, Y_AXIS), feedrate);
    }
    void logical_move_to_z(float z, uint16_t feedrate=0) {
      move_to_z(LOGICAL_TO_NATIVE(z, Z_AXIS), feedrate);
    }
    void logical_move_to_xyz(float x, float y, float z, uint16_t feedrate=0) {
      move_to_xyz(LOGICAL_TO_NATIVE(x, X_AXIS), LOGICAL_TO_NATIVE(y, Y_AXIS), LOGICAL_TO_NATIVE(z, Z_AXIS), feedrate);
    }
    void logical_move_to_xyz(xyze_pos_t &pos, uint16_t feedrate=0) {
      move_to_xyz(LOGICAL_TO_NATIVE(pos.x, X_AXIS), LOGICAL_TO_NATIVE(pos.y, Y_AXIS), LOGICAL_TO_NATIVE(pos.z, Z_AXIS), feedrate);
    }
    void logical_move_to_xy(float x, float y, uint16_t feedrate) {
      move_to_xy(LOGICAL_TO_NATIVE(x, X_AXIS), LOGICAL_TO_NATIVE(y, Y_AXIS), feedrate);
    }

    void retrack_e(float distance, uint16_t feedrate);
    void extrude_e(float distance, uint16_t feedrate);
    void synchronize();
    void quickstop() {
      quickstop_stepper();
      synchronize();
    }
    void motor_enable(uint8_t axis, uint8_t index=0);
    void motor_disable(uint8_t axis, uint8_t index=0);
    bool is_motor_enable(uint8_t axis, uint8_t index=0);
};

extern MotionControl motion_control;
#endif
