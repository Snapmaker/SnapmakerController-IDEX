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

typedef enum : uint8_t {
  SYSTEM_STATUE_IDLE,
  SYSTEM_STATUE_STARTING,
  SYSTEM_STATUE_PRINTING,
  SYSTEM_STATUE_PAUSING,
  SYSTEM_STATUE_PAUSED,
  SYSTEM_STATUE_STOPPING,
  SYSTEM_STATUE_STOPPED,
  SYSTEM_STATUE_FINISHING,
  SYSTEM_STATUE_COMPLETED,
  SYSTEM_STATUE_RECOVERING,
  SYSTEM_STATUE_RESUMING,
} system_status_e;

typedef enum {
  SYSTEM_STATUE_SCOURCE_NONE,  // Do not change the trigger source
  SYSTEM_STATUE_SCOURCE_SACP,
  SYSTEM_STATUE_SCOURCE_GCODE,
  SYSTEM_STATUE_SCOURCE_FILAMENT,
  SYSTEM_STATUE_SCOURCE_PL,
  SYSTEM_STATUE_SCOURCE_TOOL_CHANGE,
} system_status_source_e;

#define AXIS_COUNT 4

#pragma pack(1)
typedef struct {
  uint8_t Ji_num;  // J generation machine is 4
  uint8_t hw_version; // J1 is 0
  uint32_t sn;
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
    system_status_e get_status() {return status_;}
    system_status_source_e get_source() {return source_;}
    void set_status(system_status_e status, system_status_source_e source=SYSTEM_STATUE_SCOURCE_NONE);
  private:
    system_status_e status_ = SYSTEM_STATUE_IDLE;
    system_status_source_e source_ = SYSTEM_STATUE_SCOURCE_NONE;
};

extern SystemService system_service;
#endif
