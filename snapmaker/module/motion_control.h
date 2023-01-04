#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "../J1/common_type.h"
#include "src/core/types.h"
#include "src/module/motion.h"

#define MOTION_TRAVEL_FEADRATE        (9000)
#define MOTION_EXTRUDE_E_DISTANCE     (30)
#define MOTION_RETRACK_E_FEEDRATE     (1800)
#define MOTION_EXTRUDE_E_FEEDRATE     (100)
#define MOTION_STALL_GUARD_Z_SPEED    (4.8)
#define MOTION_STALL_GUARD_XY_SPEED   (10.0)

typedef enum {
  SG_X,
  SG_X2,
  SG_Y,
  SG_Z,
} sg_axis_e;

#pragma pack(1)
typedef struct {
  uint8_t axis;
  float_to_int_t distance;
} axis_move_t;
typedef struct {
  uint8_t axis_count;
  axis_move_t axis_move;  // axis_count
}mobile_instruction_t;
#pragma pack(0)
class MotionControl {
  public:
    void init_stall_guard();
    void blocking_move_to(float x, float y, float z, float feedrate=0);
    void blocking_move_to_no_limit(float x, float y, float z, float feedrate=0);
    ErrCode move_axis(mobile_instruction_t *move);
    ErrCode move_axis_to(mobile_instruction_t *move);
    ErrCode home(AxisEnum axis);
    ErrCode home();
    ErrCode home_x();
    ErrCode home_y();
    ErrCode home_z();
    void get_home_pos(float *pos);  // pos is float[4], 0-X1  1-X2  2-Y  3-Z
    void get_xyz_pos(float *pos);  // pos is float[4], 0-X1  1-X2  2-Y  3-Z

    void move(uint8_t axis, float distance, uint16_t feedrate);
    ErrCode move_e(float distance, uint16_t feedrate);
    void move_x(float x, uint16_t feedrate=0);
    void move_y(float y, uint16_t feedrate=0);
    void move_z(float z, uint16_t feedrate=0);
    void move_xy(float x, float y, uint16_t feedrate);
    void move_to_x(float x, uint16_t feedrate=0);
    void move_to_y(float y, uint16_t feedrate=0);
    void move_to_z(float z, uint16_t feedrate=0);
    void move_to_z_no_limit(float z, uint16_t feedrate=0);
    void move_to_xyz(float x, float y, float z, uint16_t feedrate=0);
    void move_to_xyz(xyze_pos_t &pos, uint16_t feedrate=0);
    void move_to_xy(float x, float y, uint16_t feedrate);
    void move_x_to_relative_home(float x, uint16_t feedrate=0);

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

    void enable_stall_guard(uint8_t axis, uint8_t sg_value, uint8_t x_index=2);
    void enable_stall_guard_only_axis(uint8_t axis, uint8_t sg_value, uint8_t x_index=2);
    void disable_stall_guard(uint8_t axis);
    void disable_stall_guard_all();
    void update_feedrate(uint16_t fr) {feedrate = fr;}
    uint16_t get_feedrate() {return feedrate;}
    bool stepper_isr_sg_check(uint8_t axis);
    void set_sg_trigger(sg_axis_e axis, bool status) {SET_BIT(sg_trigger_status, axis, status);}
    void set_sg_trigger(uint8_t status) {sg_trigger_status = status;}
    bool is_sg_trigger(sg_axis_e axis) {return GET_BIT(sg_trigger_status, axis) && GET_BIT(sg_enable_status, axis);}
    bool is_sg_trigger() {return sg_trigger_status;}
    void clear_trigger() {sg_trigger_status  = 0;};
    void set_sg_exti_status(sg_axis_e axis, bool status) {SET_BIT(sg_exti_status, axis, status);}
    void set_sg_enable(sg_axis_e axis, bool status) {SET_BIT(sg_enable_status, axis, status);}
    bool is_sg_enable(sg_axis_e axis) {return GET_BIT(sg_enable_status, axis);}
    bool is_sg_exti(sg_axis_e axis) {return GET_BIT(sg_exti_status, axis);}
    void clear_sg_exit() {sg_exti_status = 0;}
    void wait_G28();
  public:
    uint16_t feedrate = 0;
    uint8_t sg_exti_status = 0;
    uint8_t sg_enable_status = 0;
    uint8_t sg_trigger_status = 0;
};

extern MotionControl motion_control;
#endif
