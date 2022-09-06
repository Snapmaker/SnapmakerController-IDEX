#include "event_base.h"
#include "event_fdm.h"
#include "subscribe.h"
#include "../module/fdm.h"
#include "../module/motion_control.h"
#include "../module/filament_sensor.h"
#include "../Marlin/src/module/settings.h"

#pragma pack(1)
typedef struct {
  uint8_t key;
  uint8_t index;
  uint16_t temperature;
} set_temperature_t;

typedef struct {
  uint8_t key;
  uint8_t index;
  uint8_t speed;
} set_fan_t;

typedef enum : uint8_t {
  EXTRUDER_TO_RETRACK,
  RETRACK_TO_EXTRUDER,
} extrusion_type_e;

typedef struct {
  uint8_t key;
  extrusion_type_e move_type;
  float_to_int_t extruder_length;
  float_to_int_t extruder_speed;
  float_to_int_t retrack_length;
  float_to_int_t retrack_speed;
} extrusion_control_t;

typedef struct {
  uint8_t key;
  uint8_t type;  // 0-extruder 1-retrack
  float_to_int_t lenght;  // <0 sustain, =0 stop, >0 distance
  float_to_int_t speed;
} extrusion_until_t;

#pragma pack()

static ErrCode fdm_get_info(event_param_t& event) {
  uint8_t e = MODULE_INDEX(event.data[0]);
  event.data[0] = E_SUCCESS;
  FDM_info * info = (FDM_info *)(event.data + 1);
  fdm_head.get_fdm_info(e, info);
  event.length = sizeof(FDM_info) + 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode fdm_report_fan_info(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.data[2] = 1;  // fan count
  extruder_fan_info_t *info = (extruder_fan_info_t *)(event.data + 3);
  HOTEND_LOOP() {
    event.data[1] = fdm_head.get_key(e);
    uint8_t speed;
    fdm_head.get_fan_speed(e, 0, speed);
    info->index = 0;
    info->type = FAN_TYPE_COLD_MODULE;
    info->speed = speed;
    event.length = sizeof(extruder_fan_info_t) + 3;
    send_event(event);
  }
  return E_SUCCESS;
}

static ErrCode fdm_set_temperature(event_param_t& event) {
  set_temperature_t *t = (set_temperature_t *)event.data;
  uint8_t e = MODULE_INDEX(t->key);
  SERIAL_ECHOLNPAIR("set temperature e:", e, " t:", t->temperature);
  event.data[0] = fdm_head.set_temperature(e, t->temperature);
  event.length = 1;
  return send_event(event);
}

static ErrCode fdm_set_work_speed(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode fdm_enable_filament_sensor(event_param_t& event) {
  uint8_t e = MODULE_INDEX(event.data[0]);
  uint8_t enable = event.data[2];
  SERIAL_ECHOLNPAIR("SC set filament_sensor[", e, "]:", enable);
  filament_sensor.filament_param.enabled[e] = enable;
  (void)settings.save();
  event.length = 1;
  event.data[0] = E_SUCCESS;
  return send_event(event);
}

static ErrCode fdm_change_tool_head(event_param_t& event) {
  event.data[0] = fdm_head.change_tool(MODULE_INDEX(event.data[0]));
  event.length = 1;
  return send_event(event);
}

static ErrCode fdm_set_fan_speed(event_param_t& event) {
  set_fan_t *fan = (set_fan_t *)event.data;
  uint8_t e = MODULE_INDEX(fan->key);
  SERIAL_ECHOLNPAIR("set fan e:", e , " index:", fan->index, " speed:", fan->speed);
  fdm_head.set_fan_speed(e, fan->index, fan->speed);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode fdm_set_nozzle_spacing(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode fdm_get_nozzle_spacing(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode fdm_extrusion_until(event_param_t& event) {
  extrusion_until_t *extrusion = (extrusion_until_t *)event.data;
  float  length = INT_TO_FLOAT(extrusion->lenght);
  float  speed = INT_TO_FLOAT(extrusion->speed);
  filamenter_change_status_e status = extrusion->type ? FILAMENT_CHANGE_RETRACK : FILAMENT_CHANGE_EXTRUDER;
  SERIAL_ECHOLNPAIR("SC req extrusion ", MODULE_INDEX(extrusion->key), " lenght:", length, ",speed:", speed, ",type:", status);
  if (speed == 0) {
    speed = 200;
    SERIAL_ECHOLNPAIR("used default speed ", speed);
  }
  if (length < 0) {
    fdm_head.change_filamenter(MODULE_INDEX(extrusion->key), speed, status);
  } else if (length == 0) {
    fdm_head.change_filamenter(MODULE_INDEX(extrusion->key), speed, FILAMENT_CHANGE_STOP);
  } else {
    if (extrusion->type) {
      motion_control.extrude_e(length, speed);
    } else {
      motion_control.retrack_e(length, speed);
    }
  }
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode fdm_extrusion_control(event_param_t& event) {
  extrusion_control_t * extrusion = (extrusion_control_t *)event.data;
  float_to_int_t  retrack_length = INT_TO_FLOAT(extrusion->retrack_length);
  float_to_int_t  retrack_speed = INT_TO_FLOAT(extrusion->retrack_speed);
  float_to_int_t  extruder_length = INT_TO_FLOAT(extrusion->extruder_length);
  float_to_int_t  extruder_speed = INT_TO_FLOAT(extrusion->extruder_speed);
  if (extrusion->move_type == EXTRUDER_TO_RETRACK) {
    motion_control.extrude_e(extruder_length, extruder_speed);
    motion_control.synchronize();
    motion_control.retrack_e(retrack_length, retrack_speed);
    motion_control.synchronize();
  } else {
    motion_control.retrack_e(retrack_length, retrack_speed);
    motion_control.synchronize();
    motion_control.extrude_e(extruder_length, extruder_speed);
    motion_control.synchronize();
  }
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode fdm_subscribe_extruder_info(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.data[2] = 1;  // extruder count
  extruder_info_t *info = (extruder_info_t *)(event.data + 3);
  HOTEND_LOOP() {
    event.data[1] = fdm_head.get_key(e);
    fdm_head.get_extruder_info(e, info);
    event.length = sizeof(extruder_info_t) + 3;
    send_event(event);
  }
  return E_SUCCESS;
}

static ErrCode fdm_subscribe_extrusion_status(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.data[1] = 2;  // extruder count
  extruder_move_status_t *info = (extruder_move_status_t *)(event.data + 2);
  HOTEND_LOOP() {
    info[e].key = fdm_head.get_key(e);
    info[e].index = 0;
    info[e].status = fdm_head.get_change_filamenter_status(e);
  }
  event.length = sizeof(extruder_move_status_t) * event.data[1] + 2;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode subscribe_fdm_info(event_param_t& event) {
  HOTEND_LOOP() {
    event.data[0] = E_SUCCESS;
    FDM_info * info = (FDM_info *)(event.data + 1);
    fdm_head.get_fdm_info(e, info);
    event.length = sizeof(FDM_info) + 1;
    send_event(event);
  }
  return E_SUCCESS;
}

event_cb_info_t fdm_cb_info[FDM_ID_CB_COUNT] = {
  {FDM_ID_GET_INFO              , EVENT_CB_DIRECT_RUN, fdm_get_info},
  {FDM_ID_SET_TEMPERATURE       , EVENT_CB_DIRECT_RUN, fdm_set_temperature},
  {FDM_ID_SET_WORK_SPEED        , EVENT_CB_DIRECT_RUN, fdm_set_work_speed},
  {FDM_ID_ENABLE_FILAMENT_SENSOR, EVENT_CB_DIRECT_RUN, fdm_enable_filament_sensor},
  {FDM_ID_CHANGE_TOOL_HEAD      , EVENT_CB_DIRECT_RUN, fdm_change_tool_head},
  {FDM_ID_SET_FAN_SPEED         , EVENT_CB_DIRECT_RUN, fdm_set_fan_speed},
  {FDM_ID_SET_NOZZLE_SPACING    , EVENT_CB_DIRECT_RUN, fdm_set_nozzle_spacing},
  {FDM_ID_GET_NOZZLE_SPACING    , EVENT_CB_DIRECT_RUN, fdm_get_nozzle_spacing},
  {FDM_ID_EXTRUSION_CONTROL     , EVENT_CB_TASK_RUN, fdm_extrusion_control},
  {FDM_ID_EXTRUSION_UNTIL       , EVENT_CB_TASK_RUN, fdm_extrusion_until},
  {FDM_ID_SUBSCRIBE_EXTRUDER_INFO , EVENT_CB_DIRECT_RUN, fdm_subscribe_extruder_info},
  {FDM_ID_SUBSCRIBE_EXTRUSION_STATUS , EVENT_CB_DIRECT_RUN, fdm_subscribe_extrusion_status},
  {FDM_ID_SUBSCRIBE_FAN_INFO    , EVENT_CB_DIRECT_RUN, fdm_report_fan_info},
  {FDM_ID_SUBSCRIBE_MODULE_INFO    , EVENT_CB_DIRECT_RUN, subscribe_fdm_info},
};