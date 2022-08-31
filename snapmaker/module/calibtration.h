#ifndef CAlIBRATIONINT_H
#define CAlIBRATIONINT_H

#include "../J1/common_type.h"
#include "src/inc/MarlinConfigPre.h"
#include "src/core/types.h"
#include "motion_control.h"

#define CAlIBRATIONING_ERR_CODE (10000)
#define CAlIBRATIONIN_RETRACK_E_MM 5
typedef enum {
  CAlIBRATION_MODE_IDLE,
  CAlIBRATION_MODE_BED,
  CAlIBRATION_MODE_XY,
  CAlIBRATION_MODE_NOZZLE,
  CAlIBRATION_MODE_EXIT,
} calibtration_mode_e;

typedef enum {
  CAlIBRATION_STATE_IDLE,
  CAlIBRATION_STATE_PROBE_XY,
  CAlIBRATION_STATE_BED_BEAT,
  CAlIBRATION_STATE_BED_BEAT_WAIT_END,
} calibtration_status_e;


//  ______________________
// |                      |
// | [4]     [1]     [5]  |
// |                      |
// |         [0]          |
// |                      |
// | [2]             [3]  |
// |______________________|  
typedef enum {
  CAlIBRATION_POS_0,
  CAlIBRATION_POS_1,
  CAlIBRATION_POS_2,
  CAlIBRATION_POS_3,
  CAlIBRATION_POS_4,
  CAlIBRATION_POS_5,
  CAlIBRATION_POS_INVALID,
} calibtration_position_e;

class Calibtration {
  public:
    void set_calibtration_mode(calibtration_mode_e m) {mode = m;};
    ErrCode goto_calibtration_position(uint8_t pos);
    void bed_preapare(uint8_t extruder_index=0);
    float probe(uint8_t axis, float distance, uint16_t feedrate);
    ErrCode exit(bool is_save=true);
    ErrCode probe_bed_base_hight(calibtration_position_e pos, uint8_t extruder=0);
    ErrCode move_to_porbe_pos(calibtration_position_e pos, uint8_t extruder=0);
    ErrCode bed_manual_calibtration(calibtration_position_e pos);
    ErrCode bed_start_beat_mode();
    ErrCode bed_end_beat_mode();
    ErrCode nozzle_calibtration_preapare(calibtration_position_e pos);
    ErrCode calibtration_xy();
    ErrCode set_hotend_offset(uint8_t axis, float offset);
    float get_hotend_offset(uint8_t axis);
    float get_probe_offset();
    void loop(void);
    void bed_level();
    void set_z_offset(float offset, bool is_moved=false);
    float get_z_offset();
    void retrack_e();
    void extrude_e(float distance, uint16_t feedrate=MOTION_RETRACK_E_FEEDRATE);
  private:
    ErrCode probe_z_offset(calibtration_position_e pos);
    void reset_xy_calibtration_env();
    float accurate_probe(uint8_t axis, int8_t dir, uint16_t freerate);
    void backup_offset();
    ErrCode wait_and_probe_z_offset(calibtration_position_e pos, uint8_t extruder=0);
    ErrCode probe_hight_offset(calibtration_position_e pos, uint8_t extruder);
    bool move_to_sersor_no_trigger(uint8_t axis, int16_t try_distance);

  public:
    calibtration_position_e cur_pos;
    calibtration_mode_e mode = CAlIBRATION_MODE_IDLE;
    calibtration_status_e status;
    float probe_offset = CAlIBRATIONING_ERR_CODE;
    float live_z_offset = 0;
    xyz_pos_t hotend_offset_backup[HOTENDS];
    xyz_pos_t home_offset_backup;
    bool need_extrude = false;
  private:
    float last_probe_pos = 0;
};

extern Calibtration calibtration;

#endif
