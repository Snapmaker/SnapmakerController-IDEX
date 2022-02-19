#ifndef SYSTEM_H
#define SYSTEM_H

#include "stdint.h"
#include "../J1/common_type.h"
#include "module_base.h"

enum : uint8_t {
  AXIS_X1 = 0,
  AXIS_Y1 = 1,
  AXIS_Z1 = 2,
  AXIS_X2 = 6,
};
#define AXIS_COUNT 4

#pragma pack(1)
typedef struct {
  uint8_t type;  // J generation machine is 2
  uint8_t number; // J1 is 0
  uint16_t version_length;
  uint8_t version[MODULE_VER_LEN];
} machine_info_t;

typedef struct {
  uint8_t axis;
  float_to_int_t position;
} coordinate_info_t;

typedef struct {
  uint8_t homed;  // 0-homed,  1-Not all axes go back to home
  uint8_t coordinate_system_id;  // 0-G53, 1-G54
  bool is_origin_offset;  // J1 does not support and defaults to true
  uint8_t coordinate_axis_count;  // AXIS_COUNT
  coordinate_info_t coordinate_axis_info[AXIS_COUNT];
  uint8_t origin_offset_count;
  coordinate_info_t origin_offset_info[AXIS_COUNT];
} coordinate_system_t;

#pragma pack()
class SystemService {
  public:
    void get_coordinate_system_info(coordinate_system_t * info);
    void get_machine_info(machine_info_t *info);
    ErrCode set_origin(coordinate_info_t axis);
};

extern SystemService system_service;
#endif
