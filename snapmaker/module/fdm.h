#ifndef FDM_H
#define FDM_H
#include "../J1/common_type.h"
#include "module_base.h"

#pragma pack(1)

typedef struct {
  uint8_t index;
  bool is_available;
  uint8_t type;
  float_to_int_t diameter;
  float_to_int_t cur_temp;
  float_to_int_t target_temp;
}extruder_info_t;

typedef struct {
  uint8_t index;
  uint8_t type;
  uint8_t speed;
}extruder_fan_info_t;

typedef struct {
  uint8_t key;
  uint8_t filament_status;
  uint8_t head_status;
  bool head_active;
  uint8_t extruder_count;
  extruder_info_t extruder_info;
  uint8_t fan_count;
  extruder_fan_info_t extruder_fan[2];
}FDM_info;
#pragma pack()

class FDM_Head {
  public:
    ErrCode set_temperature(uint8_t e, uint16_t temperature);
    float   get_temperature(uint8_t e);
    ErrCode set_work_speed(float speed);
    ErrCode get_filamenter(uint8_t e, bool state);
    ErrCode change_tool(uint8_t e);
    ErrCode set_fan_speed(uint8_t e, uint8_t fan_index, uint8_t speed);
    ErrCode get_fan_speed(uint8_t e, uint8_t fan_index, uint8_t &speed);
    ErrCode set_extruder_diff(uint8_t axis, float diff);
    ErrCode get_fdm_info(uint8_t e, FDM_info *fdm);
    ErrCode get_extruder_info(uint8_t e, extruder_info_t *info);
    ErrCode get_module_info(uint8_t e, module_info_t &info);
    uint8_t get_key(uint8_t e);
  private:
    module_info_t module_info[EXTRUDERS];
};

extern FDM_Head fdm_head;

#endif
