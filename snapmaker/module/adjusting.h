#ifndef ADJUSTINT_H
#define ADJUSTINT_H

#include "../J1/common_type.h"

#define ADJUSTINT_ERR_CODE (10000)

typedef enum {
  ADJUST_MODE_IDLE,
  ADJUST_MODE_BED,
  ADJUST_MODE_XY,
  ADJUST_MODE_NOZZLE,
  ADJUST_MODE_EXIT,
} adjust_mode_e;

typedef enum {
  ADJUST_STATE_IDLE,
  ADJUST_STATE_PROBE_XY,
  ADJUST_STATE_BED_BEAD,
} adjust_status_e;


//  ______________________
// |                      |
// | [4]     [1]     [5]  |
// |                      |
// |         [0]          |
// |                      |
// | [2]             [3]  |
// |______________________|  
typedef enum {
  ADJUST_POS_0,
  ADJUST_POS_1,
  ADJUST_POS_2,
  ADJUST_POS_3,
  ADJUST_POS_4,
  ADJUST_POS_5,
  ADJUST_POS_INVALID,
} adjust_position_e;

class Adjusting {
  public:
    ErrCode goto_position(uint8_t pos);
    void bed_preapare(uint8_t extruder_index=0);
    float probe(uint8_t axis, float distance, uint16_t feedrate);
    ErrCode exit(void);
    ErrCode bed_probe(adjust_position_e pos, uint8_t extruder=0, bool set_z_offset=false);
    ErrCode bed_adjust_preapare(adjust_position_e pos, bool is_probe=false);
    ErrCode bed_manual_adjust(adjust_position_e pos);
    ErrCode bed_start_bead_mode();
    ErrCode nozzle_adjust_preapare(adjust_position_e pos);
    ErrCode adjust_xy();
    ErrCode set_hotend_offset(uint8_t axis, float offset);
    float get_probe_offset();
    void loop(void);
    void bed_level();
    void set_z_offset(float offset, bool is_moved=false);
    float get_z_offset();

  private:
    ErrCode probe_z_offset(adjust_position_e pos);

  public:
    adjust_position_e cur_pos;
    adjust_mode_e mode = ADJUST_MODE_IDLE;
    adjust_status_e status;
    float probe_offset = ADJUSTINT_ERR_CODE;
    float live_z_offset = 0;
};

extern Adjusting adjusting;

#endif
