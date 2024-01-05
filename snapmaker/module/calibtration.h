/*
 * Snapmaker 3D Printer Firmware
 * Copyright (C) 2023 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of SnapmakerController-IDEX
 * (see https://github.com/Snapmaker/SnapmakerController-IDEX)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CAlIBRATIONINT_H
#define CAlIBRATIONINT_H

#include "../J1/common_type.h"
#include "src/inc/MarlinConfigPre.h"
#include "src/core/types.h"
#include "motion_control.h"

#define CAlIBRATIONING_ERR_CODE               (10000)
#define CAlIBRATIONIN_RETRACK_E_MM            (5)
#define PROBE_TIMES                           (6)
#define XY_PROBE_SPEED_SLOW_SCALER            (10)
#define Z_PROBE_SPEED_SLOW_SCALER             (4)
#define MAX_DELTA_DISTANCE                    (0.25)

#define CALIBRATION_ACC                       (1000)
#define CALIBRATION_FEEDRATE                  (0.0)

#define PROBE_FAST_Z_FEEDRATE                 (300)
#define PROBE_FAST_XY_FEEDRATE                (800)
#define PROBE_MOVE_Z_FEEDRATE                 (600)
#define PROBE_BACKOFF_DISTANCE                (1)     // mm
#define PROBE_MOVE_XY_LIFTINT_DISTANCE        (5)     // mm
#define BACK_OFF_DISTANCE                     (2)     // mm
#define Z_PROBE_DISTANCE                      (30)    // mm

#define X1_STANDBY_POS                        (X1_MIN_POS)
#define X2_STANDBY_POS                        (X2_MAX_POS)
#define Y_STANDBY_POS                         (Y_MIN_POS)
#define Z_STANDBY_POS                         (100)
#define Z_PREPARE_POS                         (15.0)  // mm
#define XY_CALI_Z_POS                         (-1.2 - build_plate_thickness)
#define XY_CENTER_OFFSET_Z_POS                (0.5)
#define PROBE_DISTANCE                        (15)    // mm

#define X2_MIN_HOTEND_OFFSET (X2_MAX_POS - X2_MIN_POS - 20)
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

typedef enum {
  PROBR_RESULT_SUCCESS,
  PROBR_RESULT_STALL_GUARD,
  PROBR_RESULT_SENSOR_ERROR,
  PROBR_RESULT_NO_TRIGGER,
} probe_result_e;

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
    void goto_calibtration_position(uint8_t pos, uint16_t feedrate = MOTION_TRAVEL_FEADRATE);
    void bed_preapare(uint8_t extruder_index=0);
    probe_result_e probe(uint8_t axis, float distance, uint16_t feedrate, bool do_sg);
    ErrCode exit(bool is_save=true);
    ErrCode probe_bed_base_hight(calibtration_position_e pos, uint8_t extruder=0);
    void move_to_porbe_pos(calibtration_position_e pos, uint8_t extruder=0);
    ErrCode bed_start_beat_mode();
    ErrCode bed_end_beat_mode();
    ErrCode nozzle_calibtration_preapare(calibtration_position_e pos);
    ErrCode calibtration_xy();
    ErrCode calibtration_xy_center_offset();
    ErrCode set_hotend_offset(uint8_t axis, float offset);
    float get_hotend_offset(uint8_t axis);
    float get_probe_offset();
    void loop(void);
    void bed_level();
    void set_z_offset(float offset, bool is_moved=false);
    float get_z_offset();
    void retrack_e();
    void extrude_e(float distance, uint16_t feedrate=MOTION_RETRACK_E_FEEDRATE);
    void updateBuildPlateThickness(float bpt);

    void set_heat_bed_center_offset(const float offset[2]);
    void get_heat_bed_center_offset(float *offset);
    bool head_bed_center_offset_check(void);
    void head_bed_center_offset_reset(void);

    void X1_standby(void);
    void X2_standby(void);
    void X_standby(void);
    void Y_standby(void);
    void Z_standby(void);
    void Z_prepare(void);

  private:
    ErrCode probe_z_offset(calibtration_position_e pos);
    void reset_xy_calibtration_env();
    float multiple_probe(uint8_t axis, float distance, uint16_t freerate);
    void backup_offset();
    void restore_offset();
    ErrCode wait_and_probe_z_offset(calibtration_position_e pos, uint8_t extruder=0);
    ErrCode probe_hight_offset(calibtration_position_e pos, uint8_t extruder);
    // bool move_to_sersor_no_trigger(uint8_t axis, float try_distance);

  public:
    calibtration_position_e cur_pos;
    calibtration_mode_e mode = CAlIBRATION_MODE_IDLE;
    calibtration_status_e status;
    float probe_offset = -CAlIBRATIONING_ERR_CODE;
    xyz_pos_t hotend_offset_backup[HOTENDS];
    xyz_pos_t home_offset_backup;
    bool need_extrude = false;
    bool xy_need_re_home = false;
    bool z_need_re_home = false;
    uint32_t z_probe_cnt = 0;
  private:
    float last_probe_pos = 0;
};

extern Calibtration calibtration;

#endif
