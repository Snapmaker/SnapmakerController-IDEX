#include "event_adjust.h"
#include "../module/adjusting.h"
#include "../module/system.h"

enum {
  DED_AUTO_ADJUST_MODE = 0,
  BED_MANUAL_ADJUST_MODE = 1,
  NOZZLE_AUTO_ADJUST_MODE = 50,
  NOZZLE_MANUAL_ADJUST_MODE = 51,
  XY_AUTO_ADJUST_MODE = 100,
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


static ErrCode adjust_set_mode(event_param_t& event) {
  uint8_t mode = event.data[0];
  SERIAL_ECHOLNPAIR("set adjust mode:", mode);
  event.data[0] = E_SUCCESS;
  switch (mode) {
    case DED_AUTO_ADJUST_MODE:
    case BED_MANUAL_ADJUST_MODE:
    case NOZZLE_AUTO_ADJUST_MODE:
    case NOZZLE_MANUAL_ADJUST_MODE:
    case XY_AUTO_ADJUST_MODE:
      break;
    default:
      event.data[0] = E_PARAM;
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode adjust_move_to_pos(event_param_t& event) {
  adjust_position_e pos = (adjust_position_e)event.data[0];
  bool is_probe = event.data[1];
  SERIAL_ECHOLNPAIR("move to adjust pos:", pos, ", probe status:", is_probe);
  event.data[0] = adjusting.bed_adjust_preapare(pos, is_probe);
  event.length = 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode adjust_start_bed_probe(event_param_t& event) {
  event.data[0] = adjusting.bed_start_bead_mode();
  event.length = 1;
  SERIAL_ECHOLNPAIR("start bed bead mode");
  return send_event(event);
}

static ErrCode adjust_exit(event_param_t& event) {
  SERIAL_ECHOLNPAIR("exit adjust and is save:", event.data[0]);
  event.data[0] = adjusting.exit(event.data[0]);
  event.length = 1;
  SERIAL_ECHOLNPAIR("exit adjust");
  return send_event(event);
}

static ErrCode adjust_retrack_e(event_param_t& event) {
  adjusting.retrack_e();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode adjust_report_bed_offset(event_param_t& event) {
  report_probe_info_t * info = (report_probe_info_t *)event.data;
  if (adjusting.probe_offset == ADJUSTING_ERR_CODE) {
    info->result = E_IN_PROGRESS;
  } else {
    info->result = E_SUCCESS;
  }
  info->cur_pos = adjusting.cur_pos;
  info->offset = FLOAT_TO_INT(adjusting.probe_offset);
  event.length = sizeof(report_probe_info_t);
  return send_event(event);
}

static ErrCode adjust_move_nozzle(event_param_t& event) {
  adjust_position_e pos = (adjust_position_e)event.data[0];
  adjusting.nozzle_adjust_preapare(pos);
  event.data[0] = adjusting.bed_probe(pos, 1);
  event.length = 1;
  return send_event(event);
}

static ErrCode adjust_start_xy(event_param_t& event) {
  SERIAL_ECHOLNPAIR("start adjust xy");
  event.data[0] = adjusting.adjust_xy();
  event.length = 1;
  SERIAL_ECHOLNPAIR("adjust xy over");
  return send_event(event);
}

static ErrCode adjust_set_xy_offset(event_param_t& event) {

  return E_SUCCESS;
}

static ErrCode adjust_report_xy_offset(event_param_t& event) {

  return E_SUCCESS;
}

static ErrCode adjust_set_z_offset(event_param_t& event) {
  sc_set_z_offet_t * info = (sc_set_z_offet_t *)event.data;
  float offset = -INT_TO_FLOAT(info->info.offset);
  SERIAL_ECHOLNPAIR_F("SC set z offet:", offset);
  adjusting.set_z_offset(offset, !system_service.is_adjusting());
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode adjust_get_z_offset(event_param_t& event) {
  uint8_t key = event.data[0];
  sc_get_z_offet_t * info = (sc_get_z_offet_t *)event.data;
  float z_offset = -adjusting.get_z_offset();

  SERIAL_ECHOLNPAIR_F("SC req z offset:", z_offset);

  info->result = E_SUCCESS;
  info->key = key;
  info->extruder_count = 1;
  info->info.extruder_index = 0;
  info->info.offset = FLOAT_TO_INT(z_offset);
  event.length = sizeof(sc_get_z_offet_t);
  return send_event(event);
}

event_cb_info_t adjust_cb_info[ADJUST_ID_CB_COUNT] = {
  {ADJUST_ID_SET_MODE         , EVENT_CB_TASK_RUN, adjust_set_mode},
  {ADJUST_ID_MOVE_TO_POSITION , EVENT_CB_TASK_RUN, adjust_move_to_pos},
  {ADJUST_ID_START_BED_PROBE  , EVENT_CB_TASK_RUN, adjust_start_bed_probe},
  {ADJUST_ID_EXIT             , EVENT_CB_TASK_RUN, adjust_exit},
  {ADJUST_ID_RETRACK_E        , EVENT_CB_TASK_RUN, adjust_retrack_e},
  {ADJUST_ID_REPORT_BED_OFFSET, EVENT_CB_TASK_RUN, adjust_report_bed_offset},
  {ADJUST_ID_MOVE_NOZZLE      , EVENT_CB_TASK_RUN, adjust_move_nozzle},
  {ADJUST_ID_SET_Z_OFFSET     , EVENT_CB_TASK_RUN, adjust_set_z_offset},
  {ADJUST_ID_GET_Z_OFFSET     , EVENT_CB_TASK_RUN, adjust_get_z_offset},
  {ADJUST_ID_START_XY         , EVENT_CB_TASK_RUN, adjust_start_xy},
  {ADJUST_ID_SET_XY_OFFSET    , EVENT_CB_TASK_RUN, adjust_set_xy_offset},
  {ADJUST_ID_REPORT_XY_OFFSET , EVENT_CB_TASK_RUN, adjust_report_xy_offset},
};
