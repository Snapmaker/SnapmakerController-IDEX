#include "event_calibtration.h"
#include "../module/calibtration.h"
#include "../module/system.h"
#include "../module/print_control.h"

enum {
  DED_AUTO_CAlIBRATION_MODE = 0,
  BED_MANUAL_CAlIBRATION_MODE = 1,
  NOZZLE_AUTO_CAlIBRATION_MODE = 50,
  NOZZLE_MANUAL_CAlIBRATION_MODE = 51,
  XY_AUTO_CAlIBRATION_MODE = 100,
  XY_CAlIBRATION_MODE_TEST = 102,
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
      system_service.set_status(SYSTEM_STATUE_CAlIBRATION);
    }
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_move_to_pos(event_param_t& event) {
  calibtration_position_e pos = (calibtration_position_e)event.data[0];
  bool is_probe = event.data[1];
  LOG_V("move to calibtration pos:%d, probe status:%d\n", pos, is_probe);
  if (is_probe) {
    event.data[0] = calibtration.probe_bed_base_hight(pos);
  } else {
    event.data[0] = calibtration.move_to_porbe_pos(pos);
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

event_cb_info_t calibtration_cb_info[CAlIBRATION_ID_CB_COUNT] = {
  {CAlIBRATION_ID_SET_MODE         , EVENT_CB_DIRECT_RUN, calibtration_set_mode},
  {CAlIBRATION_ID_MOVE_TO_POSITION , EVENT_CB_TASK_RUN, calibtration_move_to_pos},
  {CAlIBRATION_ID_START_BED_PROBE  , EVENT_CB_DIRECT_RUN, calibtration_start_bed_probe},
  {CAlIBRATION_ID_EXIT             , EVENT_CB_DIRECT_RUN, calibtration_exit},
  {CAlIBRATION_ID_RETRACK_E        , EVENT_CB_TASK_RUN, calibtration_retrack_e},
  {CAlIBRATION_ID_REPORT_BED_OFFSET, EVENT_CB_DIRECT_RUN, calibtration_report_bed_offset},
  {CAlIBRATION_ID_MOVE_NOZZLE      , EVENT_CB_TASK_RUN, calibtration_move_nozzle},
  {CAlIBRATION_ID_SET_Z_OFFSET     , EVENT_CB_TASK_RUN, calibtration_set_z_offset},
  {CAlIBRATION_ID_GET_Z_OFFSET     , EVENT_CB_DIRECT_RUN, calibtration_get_z_offset},
  {CAlIBRATION_ID_START_XY         , EVENT_CB_TASK_RUN, calibtration_start_xy},
  {CAlIBRATION_ID_SET_XY_OFFSET    , EVENT_CB_TASK_RUN, calibtration_set_xy_offset},
  {CAlIBRATION_ID_REPORT_XY_OFFSET , EVENT_CB_DIRECT_RUN, calibtration_report_xy_offset},
  {CAlIBRATION_ID_SUBSCRIBE_Z_OFFSET , EVENT_CB_DIRECT_RUN, calibtration_get_z_offset},
};
