#include "event_calibtration.h"
#include "../module/calibtration.h"
#include "../module/system.h"

enum {
  DED_AUTO_CAlIBRATION_MODE = 0,
  BED_MANUAL_CAlIBRATION_MODE = 1,
  NOZZLE_AUTO_CAlIBRATION_MODE = 50,
  NOZZLE_MANUAL_CAlIBRATION_MODE = 51,
  XY_AUTO_CAlIBRATION_MODE = 100,
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
  uint8_t result;
  uint8_t key;
  uint8_t extruder_count;  // 1
  z_offet_info_t info;
} sc_get_z_offet_t;
#pragma pack()


static ErrCode calibtration_set_mode(event_param_t& event) {
  uint8_t mode = event.data[0];
  SERIAL_ECHOLNPAIR("set calibtration mode:", mode);
  event.data[0] = E_SUCCESS;
  switch (mode) {
    case DED_AUTO_CAlIBRATION_MODE:
    case BED_MANUAL_CAlIBRATION_MODE:
    case NOZZLE_AUTO_CAlIBRATION_MODE:
    case NOZZLE_MANUAL_CAlIBRATION_MODE:
    case XY_AUTO_CAlIBRATION_MODE:
      break;
    default:
      event.data[0] = E_PARAM;
  }
  if (event.data[0] == E_SUCCESS) {
    system_service.set_status(SYSTEM_STATUE_CAlIBRATION);
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_move_to_pos(event_param_t& event) {
  calibtration_position_e pos = (calibtration_position_e)event.data[0];
  bool is_probe = event.data[1];
  SERIAL_ECHOLNPAIR("move to calibtration pos:", pos, ", probe status:", is_probe);
  event.data[0] = calibtration.bed_calibtration_preapare(pos, is_probe);
  event.length = 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode calibtration_start_bed_probe(event_param_t& event) {
  event.data[0] = calibtration.bed_start_beat_mode();
  event.length = 1;
  SERIAL_ECHOLNPAIR("start bed beat mode");
  return send_event(event);
}

static ErrCode calibtration_exit(event_param_t& event) {
  SERIAL_ECHOLNPAIR("exit calibtration and is save:", event.data[0]);
  event.data[0] = calibtration.exit(event.data[0]);
  event.length = 1;
  system_service.set_status(SYSTEM_STATUE_IDLE);
  SERIAL_ECHOLNPAIR("exit calibtration");
  return send_event(event);
}

static ErrCode calibtration_retrack_e(event_param_t& event) {
  calibtration.retrack_e();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_report_bed_offset(event_param_t& event) {
  report_probe_info_t * info = (report_probe_info_t *)event.data;
  if (calibtration.probe_offset == CAlIBRATIONING_ERR_CODE) {
    info->result = E_IN_PROGRESS;
  } else {
    info->result = E_SUCCESS;
  }
  info->cur_pos = calibtration.cur_pos;
  info->offset = FLOAT_TO_INT(calibtration.probe_offset);
  event.length = sizeof(report_probe_info_t);
  return send_event(event);
}

static ErrCode calibtration_move_nozzle(event_param_t& event) {
  calibtration_position_e pos = (calibtration_position_e)event.data[0];
  calibtration.nozzle_calibtration_preapare(pos);
  event.data[0] = calibtration.bed_probe(pos, 1);
  event.length = 1;
  return send_event(event);
}

static ErrCode calibtration_start_xy(event_param_t& event) {
  SERIAL_ECHOLNPAIR("start calibtration xy");
  event.data[0] = calibtration.calibtration_xy();
  event.length = 1;
  SERIAL_ECHOLNPAIR("calibtration xy over");
  return send_event(event);
}

static ErrCode calibtration_set_xy_offset(event_param_t& event) {

  return E_SUCCESS;
}

static ErrCode calibtration_report_xy_offset(event_param_t& event) {

  return E_SUCCESS;
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
  uint8_t key = event.data[0];
  sc_get_z_offet_t * info = (sc_get_z_offet_t *)event.data;
  float z_offset = -calibtration.get_z_offset();

  SERIAL_ECHOLNPAIR_F("SC req z offset:", z_offset);

  info->result = E_SUCCESS;
  info->key = key;
  info->extruder_count = 1;
  info->info.extruder_index = 0;
  info->info.offset = FLOAT_TO_INT(z_offset);
  event.length = sizeof(sc_get_z_offet_t);
  return send_event(event);
}

event_cb_info_t calibtration_cb_info[CAlIBRATION_ID_CB_COUNT] = {
  {CAlIBRATION_ID_SET_MODE         , EVENT_CB_TASK_RUN, calibtration_set_mode},
  {CAlIBRATION_ID_MOVE_TO_POSITION , EVENT_CB_TASK_RUN, calibtration_move_to_pos},
  {CAlIBRATION_ID_START_BED_PROBE  , EVENT_CB_TASK_RUN, calibtration_start_bed_probe},
  {CAlIBRATION_ID_EXIT             , EVENT_CB_TASK_RUN, calibtration_exit},
  {CAlIBRATION_ID_RETRACK_E        , EVENT_CB_TASK_RUN, calibtration_retrack_e},
  {CAlIBRATION_ID_REPORT_BED_OFFSET, EVENT_CB_TASK_RUN, calibtration_report_bed_offset},
  {CAlIBRATION_ID_MOVE_NOZZLE      , EVENT_CB_TASK_RUN, calibtration_move_nozzle},
  {CAlIBRATION_ID_SET_Z_OFFSET     , EVENT_CB_TASK_RUN, calibtration_set_z_offset},
  {CAlIBRATION_ID_GET_Z_OFFSET     , EVENT_CB_TASK_RUN, calibtration_get_z_offset},
  {CAlIBRATION_ID_START_XY         , EVENT_CB_TASK_RUN, calibtration_start_xy},
  {CAlIBRATION_ID_SET_XY_OFFSET    , EVENT_CB_TASK_RUN, calibtration_set_xy_offset},
  {CAlIBRATION_ID_REPORT_XY_OFFSET , EVENT_CB_TASK_RUN, calibtration_report_xy_offset},
};
