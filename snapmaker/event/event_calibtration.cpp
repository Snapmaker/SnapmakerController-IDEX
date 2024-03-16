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

#include "event_calibtration.h"
#include "../module/calibtration.h"
#include "../module/system.h"
#include "../module/print_control.h"
#include "../module/exception.h"
#include "../../../Marlin/src/module/temperature.h"
#include "../../../Marlin/src/module/settings.h"

#define PID_AUTOTUNE_TEMP                     (235)
#define PID_AUTOTUNE_NCYCLES                  (8)
#define E_PID_AUTOTUNE_HARDWARE               (PRIVATE_ERROR_BASE + 0)
#define E_PID_AUTOTUNE_INVALID_STATE          (PRIVATE_ERROR_BASE + 1)
#define E_PID_AUTOTUNE_BUSY                   (PRIVATE_ERROR_BASE + 2)
#define E_PID_AUTOTUNE_PARAM                  (PRIVATE_ERROR_BASE + 3)
#define E_PID_AUTOTUNE_FAILURE                (PRIVATE_ERROR_BASE + 4)

enum {
  DED_AUTO_CAlIBRATION_MODE = 0,
  BED_MANUAL_CAlIBRATION_MODE = 1,
  NOZZLE_AUTO_CAlIBRATION_MODE = 50,
  NOZZLE_MANUAL_CAlIBRATION_MODE = 51,
  XY_AUTO_CAlIBRATION_MODE = 100,
  XY_CAlIBRATION_MODE_TEST = 102,
  PID_AUTOTUNE_MODE = 150,
};

#pragma pack(1)
typedef struct {
  uint8_t result;
  uint8_t cur_pos;
  float_to_int_t offset;
} report_probe_info_t;

typedef struct {
  uint8_t extruder_index;
  float_to_int_t offset;
} z_offet_info_t;

typedef struct {
  uint8_t key;
  z_offet_info_t info;
} sc_set_z_offet_t;

typedef struct {
  uint8_t axis;
  float_to_int_t offset;
} xy_level_t;

typedef struct {
  uint8_t result;
  uint8_t key;
  uint8_t extruder_count;  // 1
  z_offet_info_t info;
} sc_get_z_offet_t;
#pragma pack()


static ErrCode calibtration_set_mode(event_param_t& event) {
  uint8_t mode = event.data[0];
  LOG_V("set calibtration mode:%d\n", mode);
  if (system_service.is_calibtration_status()) {
    event.data[0] = E_COMMON_ERROR;
  } else {
    switch (mode) {
      case DED_AUTO_CAlIBRATION_MODE:
      case BED_MANUAL_CAlIBRATION_MODE:
      case NOZZLE_AUTO_CAlIBRATION_MODE:
      case NOZZLE_MANUAL_CAlIBRATION_MODE:
      case XY_AUTO_CAlIBRATION_MODE:
      case PID_AUTOTUNE_MODE:
        event.data[0] = E_SUCCESS;
        break;
      case XY_CAlIBRATION_MODE_TEST:
        print_control.is_calibretion_mode = true;
        event.data[0] = E_SUCCESS;
        break;
      default:
        event.data[0] = E_COMMON_ERROR;
    }
    if (event.data[0] == E_SUCCESS) {
      if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_CAlIBRATION)) {
        LOG_E("can NOT set to SYSTEM_STATUE_CAlIBRATION\r\n");
        event.data[0] = E_COMMON_ERROR;
      }
    }
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_move_to_pos(event_param_t& event) {
  calibtration_position_e pos = (calibtration_position_e)event.data[0];
  bool is_probe = event.data[1];
  LOG_I("move to calibtration pos:%d with probe %d\n", pos, is_probe);
  if (is_probe) {
    event.data[0] = calibtration.probe_bed_base_hight(pos);
  } else {
    calibtration.move_to_porbe_pos(pos);
    event.data[0] = E_SUCCESS;
  }
  event.length = 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode calibtration_start_bed_probe(event_param_t& event) {
  if (event.data[0]) {
    LOG_V("SC req start bed beat mode\n");
    event.data[0] = calibtration.bed_start_beat_mode();
  } else {
    LOG_V("SC req end bed beat mode\n");
    event.data[0] = calibtration.bed_end_beat_mode();
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_exit(event_param_t& event) {
  LOG_V("exit calibtration and is save:%d\n", event.data[0]);
  calibtration.exit(event.data[0]);

  uint32_t wait_timeout = millis() + 50000;
  ErrCode ret = E_SUCCESS;
  while (system_service.get_status() != SYSTEM_STATUE_IDLE) {
    if (wait_timeout < millis()) {
      ret = E_COMMON_ERROR;
      LOG_E("exit calibtration failed\n");
      break;
    }
    vTaskDelay(100);
  }
  LOG_V("exit calibtration over\n");
  calibtration.probe_offset = -CAlIBRATIONING_ERR_CODE;
  event.data[0] = ret;
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_retrack_e(event_param_t& event) {
  LOG_V("SC req calibtration status retrack\n");
  calibtration.retrack_e();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_report_bed_offset(event_param_t& event) {
  report_probe_info_t * info = (report_probe_info_t *)event.data;
  info->result = calibtration.probe_offset == CAlIBRATIONING_ERR_CODE ? E_CAlIBRATION_PRIOBE: E_SUCCESS;
  info->cur_pos = calibtration.cur_pos;
  info->offset = FLOAT_TO_INT(calibtration.probe_offset);
  event.length = sizeof(report_probe_info_t);
  LOG_V("SC req calibtration bed: result: %d pos:%d offset:%d\n", info->result, info->cur_pos, info->offset);
  return send_event(event);
}

static ErrCode calibtration_move_nozzle(event_param_t& event) {
  calibtration_position_e pos = (calibtration_position_e)event.data[0];
  ErrCode ret = E_SUCCESS;
  ret = calibtration.nozzle_calibtration_preapare(pos);
  event.data[0] = ret == E_SUCCESS ? ret : E_COMMON_ERROR;
  event.length = 1;
  LOG_V("SC req move to calibtration pos %d, result:%d\n", pos, ret);
  return send_event(event);
}

static ErrCode calibtration_start_xy(event_param_t& event) {
  LOG_V("start calibtration xy\n");
  event.data[0] = calibtration.calibtration_xy();
  event.length = 1;
  LOG_I("calibtration xy result:%d\n", event.data[0]);
  return send_event(event);
}

static ErrCode calibtration_set_xy_offset(event_param_t& event) {
  uint8_t axis_count = event.data[0];
  LOG_V("sc set xy offset, axis_count:%d\n", axis_count);
  xy_level_t * xy_offset = (xy_level_t *)(event.data + 1);
  for (uint8_t i = 0; i < axis_count; i++) {
    float offset = INT_TO_FLOAT(xy_offset[i].offset);
    if (xy_offset[i].axis == 0) {
      LOG_V("sc set x offset:%d\n", offset);
      calibtration.set_hotend_offset(X_AXIS, offset);
    } else if (xy_offset[i].axis == 1) {
      LOG_V("sc set y offset:%d\n", offset);
      calibtration.set_hotend_offset(Y_AXIS, offset);
    }
  }
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_report_xy_offset(event_param_t& event) {
  xy_level_t * xy_offset = (xy_level_t *)(event.data + 2);
  event.data[0] = E_SUCCESS;
  event.data[1] = 2;
  xy_offset[0].axis = 0;  // X
  xy_offset[0].offset = FLOAT_TO_INT(calibtration.get_hotend_offset(X_AXIS));
  xy_offset[1].axis = 1;  // Y
  xy_offset[1].offset = FLOAT_TO_INT(calibtration.get_hotend_offset(Y_AXIS));
  event.length = 2 + sizeof(xy_level_t)*2;
  return send_event(event);
}

static ErrCode calibtration_set_z_offset(event_param_t& event) {
  sc_set_z_offet_t * info = (sc_set_z_offet_t *)event.data;
  float offset = -INT_TO_FLOAT(info->info.offset);
  SERIAL_ECHOLNPAIR_F("SC set z offet:", offset);
  calibtration.set_z_offset(offset, !system_service.is_calibtration_status());
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_get_z_offset(event_param_t& event) {
  uint8_t key = MODULE_KEY(MODULE_PRINT, 0);
  sc_get_z_offet_t * info = (sc_get_z_offet_t *)event.data;
  float z_offset = -calibtration.get_z_offset();

  LOG_V("SC req z offset:%f\n", z_offset);

  info->result = E_SUCCESS;
  info->key = key;
  info->extruder_count = 1;
  info->info.extruder_index = 0;
  info->info.offset = FLOAT_TO_INT(z_offset);
  event.length = sizeof(sc_get_z_offet_t);
  return send_event(event);
}

static ErrCode calibtration_start_pid_autotune(event_param_t& event) {
  ErrCode ret = E_SUCCESS;
  heater_id_t hid;

  LOG_I("SC req start pid autotune\n");

  if (!exception_server.is_allow_heat_nozzle(false)) {
    LOG_E("nozzle heating is limited\n");
    ret = E_PID_AUTOTUNE_HARDWARE;
    goto EXIT;
  }

  if (system_service.set_status(SYSTEM_STATUE_PID_AUTOTUNE)) {
    LOG_E("the current state does not allow pid autotune\n");
    ret = E_PID_AUTOTUNE_INVALID_STATE;
    goto EXIT;
  }

  if (thermalManager.tune_pid_info.pid_autotune_step == PID_AUTOTUNE_RUNNING) {
    LOG_E("pid_autotune is running, current operation not allowed\n");
    ret = E_PID_AUTOTUNE_BUSY;
    goto EXIT;
  }

  // if the key of the left and right nozzles change you can't judge it that way.
  hid = (heater_id_t)MODULE_INDEX(event.data[0]);
  LOG_I("heater_id: %d\n", hid);

  // currently only the nozzle pid_autoturn is supported.
  if (hid < H_E0 || hid > H_E0 + EXTRUDERS) {
    LOG_E("heater_id error, range [%d - %d], recv hid: %d\n", H_E0, H_E0 + EXTRUDERS, hid);
    ret = E_PID_AUTOTUNE_PARAM;
    goto EXIT;
  }

  thermalManager.PID_autotune(PID_AUTOTUNE_TEMP, hid, PID_AUTOTUNE_NCYCLES, true);

  if (thermalManager.tune_pid_info.pid_autotune_err) {
    LOG_E("hid: %d pid_autotune failed, error sta: 0x%x\n", hid, thermalManager.tune_pid_info.pid_autotune_err);
    ret = E_PID_AUTOTUNE_FAILURE;
    goto EXIT;
  }
  else {
    LOG_I("pid_autotune success\n    Kp: %f, Ki: %f, Kd: %f\n", thermalManager.tune_pid_info.tune_pid_value.Kp,
          thermalManager.tune_pid_info.tune_pid_value.Ki, thermalManager.tune_pid_info.tune_pid_value.Kd);
    settings.save();
    ret = E_SUCCESS;
  }

  // if (thermalManager.tune_pid_info.autotune_hid  != hid) {
  //   LOG_E("pid_autotune failed, autotune id does not match, cur: %d target: %d\n",
  //         hid, thermalManager.tune_pid_info.pid_autotune_err, hid);
  //   ret = E_FAILURE;
  //   goto EXIT;
  // }

EXIT:
  LOG_I("pid autotune result: %d\n", ret);
  event.data[0] = ret;
  event.length = 1;
  send_event(event);

  if (thermalManager.tune_pid_info.pid_autotune_step == PID_AUTOTUNE_IDLE &&
      (system_service.get_status() == SYSTEM_STATUE_CAlIBRATION || system_service.get_status() == SYSTEM_STATUE_PID_AUTOTUNE))
    system_service.set_status(SYSTEM_STATUE_IDLE);

  return E_SUCCESS;
}

event_cb_info_t calibtration_cb_info[CAlIBRATION_ID_CB_COUNT] = {
  {CAlIBRATION_ID_SET_MODE         , EVENT_CB_DIRECT_RUN,   calibtration_set_mode},
  {CAlIBRATION_ID_MOVE_TO_POSITION , EVENT_CB_TASK_RUN,     calibtration_move_to_pos},
  {CAlIBRATION_ID_START_BED_PROBE  , EVENT_CB_TASK_RUN,     calibtration_start_bed_probe},
  {CAlIBRATION_ID_EXIT             , EVENT_CB_TASK_RUN,     calibtration_exit},
  {CAlIBRATION_ID_RETRACK_E        , EVENT_CB_TASK_RUN,     calibtration_retrack_e},
  {CAlIBRATION_ID_REPORT_BED_OFFSET, EVENT_CB_DIRECT_RUN,   calibtration_report_bed_offset},
  {CAlIBRATION_ID_MOVE_NOZZLE      , EVENT_CB_TASK_RUN,     calibtration_move_nozzle},
  {CAlIBRATION_ID_SET_Z_OFFSET     , EVENT_CB_TASK_RUN,     calibtration_set_z_offset},
  {CAlIBRATION_ID_GET_Z_OFFSET     , EVENT_CB_DIRECT_RUN,   calibtration_get_z_offset},
  {CAlIBRATION_ID_START_XY         , EVENT_CB_TASK_RUN,     calibtration_start_xy},
  {CAlIBRATION_ID_SET_XY_OFFSET    , EVENT_CB_TASK_RUN,     calibtration_set_xy_offset},
  {CAlIBRATION_ID_REPORT_XY_OFFSET , EVENT_CB_DIRECT_RUN,   calibtration_report_xy_offset},
  {CAlIBRATION_ID_SUBSCRIBE_Z_OFFSET , EVENT_CB_DIRECT_RUN, calibtration_get_z_offset},
  {CAlIBRATION_ID_START_PID_AUTOTUNE , EVENT_CB_TASK_RUN,   calibtration_start_pid_autotune},
};
