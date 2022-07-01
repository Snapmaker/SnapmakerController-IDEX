#ifndef FDM_H
#define FDM_H
#include "../J1/common_type.h"
#include "module_base.h"
#include "system.h"

#pragma pack(1)

typedef struct {
  uint8_t index;
  uint8_t filament_status;
  uint8_t filament_enable_status;
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
  uint8_t head_status;
  bool head_active;
  uint8_t extruder_count;
  extruder_info_t extruder_info;
  uint8_t fan_count;
  extruder_fan_info_t extruder_fan[1];
}FDM_info;
#pragma pack()

typedef enum {
  FILAMENT_CHANGE_STOP,
  FILAMENT_CHANGE_EXTRUDER,
  FILAMENT_CHANGE_RETRACK,
} filamenter_change_status_e;

typedef enum {
  BRASS_NTC_E,
  BRASS_PT100_E,
  HARDENING_STEEL_PT100,
  UNKNOWN_NOZZLE_TYPE = 0xFF,
} nozzle_texture_type_e;

typedef struct {
  nozzle_texture_type_e  texture;
  float caliber ;
  uint16_t adc_min;
  uint16_t adc_max;
} nozzle_type_t;

class FDM_Head {
  public:
    ErrCode set_temperature(uint8_t e, uint16_t temperature);
    float   get_temperature(uint8_t e);
    ErrCode set_work_speed(float speed);
    ErrCode change_filamenter(uint8_t e, float feedrate, filamenter_change_status_e status);
    ErrCode change_tool(uint8_t e);
    ErrCode set_fan_speed(uint8_t e, uint8_t fan_index, uint8_t speed);
    ErrCode get_fan_speed(uint8_t e, uint8_t fan_index, uint8_t &speed);
    ErrCode set_extruder_diff(uint8_t axis, float diff);
    ErrCode get_fdm_info(uint8_t e, FDM_info *fdm);
    ErrCode get_extruder_info(uint8_t e, extruder_info_t *info);
    ErrCode get_module_info(uint8_t e, module_info_t &info);
    uint8_t get_key(uint8_t e);
    uint16_t get_nozzle_type(uint8_t e, nozzle_texture_type_e *texture, float *caliber);

    bool is_duplication_enabled(uint8_t e) {return duplication_enabled_move[e] || !system_service.is_working();}
    // Double - head printing will not work after disable
    void set_duplication_enabled(uint8_t e, bool status) {duplication_enabled_move[e] = status;}
    bool is_duplicating();
 
    bool is_change_filamenter(uint8_t e) {
      return (change_filamenter_status[e] != FILAMENT_CHANGE_STOP );
    }
    bool is_change_filamenter() {
      return is_change_filamenter(0)||is_change_filamenter(1);
    }
    bool get_filamenter_dir(uint8_t e) {
      if (change_filamenter_status[e] == FILAMENT_CHANGE_EXTRUDER) {
        return true;
      }
      return false;
    }
  private:
    module_info_t module_info[EXTRUDERS];
    filamenter_change_status_e change_filamenter_status[EXTRUDERS] = {FILAMENT_CHANGE_STOP, FILAMENT_CHANGE_STOP};
    bool filamenter_dir[EXTRUDERS] = {true, true};
    bool duplication_enabled_move[EXTRUDERS] = {true, true};
};

extern FDM_Head fdm_head;

#endif
