#ifndef MOVE_H
#define MOVE_H
#include "stdint.h"
class Move
{
public:
  static void move_to_xyz(float x, float y, float z);
  static void move_to_xyz(float x, float y, float z, uint16_t feedrate);
  static void move_to_x(float x);
  static void move_to_y(float y);
  static void move_to_z(float z);
  static void move_to_x(float x, uint16_t feedrate);
  static void move_to_y(float y, uint16_t feedrate);
  static void move_to_z(float z, uint16_t feedrate);
  static void move_to_xy(float x, float y, uint16_t feedrate);
  static void move_to_work_x(float x, uint16_t feedrate);
  static void move_to_work_y(float y, uint16_t feedrate);
  static void move_to_work_z(float z, uint16_t feedrate);
  static void move_to_work_xy(float x, float y, uint16_t feedrate);
  static void move_x(float x);
  static void move_y(float y);
  static void move_z(float z);
  static void move_x(float x, uint16_t feedrate);
  static void move_y(float y, uint16_t feedrate);
  static void move_z(float z, uint16_t feedrate);
  static void move_xy(float x, float y, uint16_t feedrate);
  static void retrack_e(float distance, uint16_t feedrate);
  static void extrude_e(float distance, uint16_t feedrate);
  static void home_axis();
  static void home_all_axis_smart();
  static void set_extruder_position(float NewPosition);
  static void packing_home();
};

extern Move move;
#endif