#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include "../J1/common_type.h"


#pragma pack(1)
typedef struct {
  uint8_t axis;
  float_to_int_t distance;
  uint16_t feedrate;
}mobile_instruction_t;
#pragma pack(0)
class MotionControl {
  public:
    ErrCode move_axis(mobile_instruction_t move);
    ErrCode move_axis_to(mobile_instruction_t move);
    ErrCode home(uint8_t axis);
    ErrCode home();
    ErrCode move_e(float distance, uint16_t feedrate);
    void move_x(float x, uint16_t feedrate=0);
    void move_y(float y, uint16_t feedrate=0);
    void move_z(float z, uint16_t feedrate=0);
    void move_xy(float x, float y, uint16_t feedrate);
    void move_to_x(float x, uint16_t feedrate=0);
    void move_to_y(float y, uint16_t feedrate=0);
    void move_to_z(float z, uint16_t feedrate=0);
    void move_to_xyz(float x, float y, float z, uint16_t feedrate=0);
    void move_to_xy(float x, float y, uint16_t feedrate);

    void retrack_e(float distance, uint16_t feedrate);
    void extrude_e(float distance, uint16_t feedrate);
    void synchronize();
};

extern MotionControl motion_control;
#endif
