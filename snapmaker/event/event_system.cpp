#include "event_system.h"
#include "event_base.h"
#include "subscribe.h"
#include "../module/fdm.h"
#include "../module/bed_control.h"
#include "../module/system.h"
#include "../module/motion_control.h"
#include "../module/enclosure.h"

#pragma pack(1)

typedef struct {
  uint8_t axis;
  bool state;
} motor_state_t;

#pragma pack()

static ErrCode subscribe_event(event_param_t& event) {
  event.data[0] = subscribe.enable(event);
  event.length = 1;
  return send_event(event);
}

static ErrCode unsubscribe_event(event_param_t& event) {
  event.data[0] = subscribe.disable(event);
  event.length = 1;
  return send_event(event);
}

static ErrCode set_log_grade(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode req_protocol_ver(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode set_debug_mode(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode factory_reset(event_param_t& event) {
  SERIAL_ECHOLN("SC req factory reset");
  system_service.factory_reset();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode heart_event(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.data[1] = system_service.get_status();
  event.length = 2;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode retport_log(event_param_t& event) {
  return E_SUCCESS;
}


static ErrCode req_module_info(event_param_t& event) {
  uint8_t *array_count = &event.data[1];
  module_info_t *module_info = (module_info_t *)(event.data + 2);
  event.data[0] = E_SUCCESS;
  SERIAL_ECHOLN("SC req module info");
  uint8_t index = 0;
  fdm_head.get_module_info(0, module_info[index++]);
  fdm_head.get_module_info(1, module_info[index++]);
  bed_control.get_module_info(module_info[index++]);
  // enclosure.get_module_info(module_info[index++]);
  *array_count = index;
  event.length = index * sizeof(module_info_t) + 2;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_machine_info(event_param_t& event) {
  SERIAL_ECHOLN("SC req machine info");
  machine_info_t *machine_info = (machine_info_t *)(event.data + 1);
  event.data[0] = E_SUCCESS;
  system_service.get_machine_info(machine_info);
  event.length = sizeof(machine_info_t) + 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_machine_size(event_param_t& event) {
  SERIAL_ECHOLN("SC req machine size");
  event.data[0] = E_SUCCESS;
  machine_size_t *machine_size = (machine_size_t *)(event.data + 1);
  system_service.get_machine_size(machine_size);
  event.length = sizeof(machine_size_t) + 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode req_coordinate_system(event_param_t& event) {
  coordinate_system_t * info = (coordinate_system_t *)(event.data + 1);
  event.data[0] = E_SUCCESS;
  system_service.get_coordinate_system_info(info);
  event.length = sizeof(coordinate_system_t) + 1;
  send_event(event);
  return E_SUCCESS;
}

static ErrCode set_coordinate_system(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode set_origin(event_param_t& event) {
  return E_SUCCESS;
}

static ErrCode move_relative_home(event_param_t& event) {
  axis_move_t *move = (axis_move_t *)(event.data);
  float distance = INT_TO_FLOAT(move->distance);
  uint16_t speed = *((uint16_t *)(event.data + sizeof(axis_move_t)));
  SERIAL_ECHOLNPAIR("SC move x relative home to ", distance, " F:", speed);
  motion_control.move_x_to_relative_home(distance, speed);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode move_relative(event_param_t& event) {
  mobile_instruction_t *move = (mobile_instruction_t *)(event.data);
  motion_control.move_axis(move);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode move(event_param_t& event) {
  mobile_instruction_t *move = (mobile_instruction_t *)(event.data);
  motion_control.move_axis_to(move);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode home(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.length = 1;
  send_event(event);
  motion_control.home();
  event.info.attribute = SACP_ATTR_REQ;
  event.info.command_id = SYS_ID_HOME_END;
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static bool motor_remap_to_marlin(uint8_t sacp, uint8_t &marlin, uint8_t &index) {
  switch (sacp) {
    case AXIS_X1: marlin = X_AXIS; index = 0; return true;
    case AXIS_Y1: marlin = Y_AXIS; index = 0; return true;
    case AXIS_Z1: marlin = Z_AXIS; index = 0; return true;
    case AXIS_X2: marlin = X_AXIS; index = 1; return true;
    case AXIS_E0: marlin = E_AXIS; index = 0; return true;
    case AXIS_E1: marlin = E_AXIS; index = 1; return true;
  }
  return false;
}

static ErrCode get_motor_enable(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC get motor enable state");
  uint8_t axis_count = 0;
  uint8_t axis, index;
  motor_state_t *motor_state = (motor_state_t *)(event.data + 2);

  #define GET_AXIS_STATE(AXIS) \
    motor_remap_to_marlin(AXIS_ ## AXIS, axis, index); \
    motor_state[axis_count].axis = AXIS_ ## AXIS; \
    motor_state[axis_count].state = motion_control.is_motor_enable(axis, index); \
    axis_count++;

  GET_AXIS_STATE(X1);
  GET_AXIS_STATE(Y1);
  GET_AXIS_STATE(Z1);
  GET_AXIS_STATE(X2);
  GET_AXIS_STATE(E0);
  GET_AXIS_STATE(E1);

  event.data[0] = E_SUCCESS;
  event.data[1] = axis_count;
  event.length = sizeof(motor_state_t) * axis_count + 2;
  return send_event(event);
}

static ErrCode set_motor_enable(event_param_t& event) {
  uint8_t axis_count = event.data[0];
  SERIAL_ECHOLNPAIR("SC set motor enable and count:", axis_count);
  motor_state_t *motor_state = (motor_state_t *)(event.data + 1);
  for (uint8_t i = 0; i < axis_count; i++) {
    uint8_t axis = motor_state[i].axis;
    uint8_t index = 0;
    uint8_t state = motor_state[i].state;
    if (motor_remap_to_marlin(axis, axis, index)) {
      SERIAL_ECHOLNPAIR("set axis:", axis, " index:", index, " state:", state);
      if (state)
        motion_control.motor_enable(axis, index);
      else
        motion_control.motor_disable(axis, index);
    }
  }
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}


event_cb_info_t system_cb_info[SYS_ID_CB_COUNT] = {
  {SYS_ID_SUBSCRIBE             , EVENT_CB_DIRECT_RUN, subscribe_event},
  {SYS_ID_UNSUBSCRIBE           , EVENT_CB_DIRECT_RUN, unsubscribe_event},
  {SYS_ID_SET_LOG_GRADE         , EVENT_CB_DIRECT_RUN, set_log_grade},
  {SYS_ID_REQ_PROTOCOL_VER      , EVENT_CB_DIRECT_RUN, req_protocol_ver},
  {SYS_ID_SET_DEBUG_MODE        , EVENT_CB_DIRECT_RUN, set_debug_mode},
  {SYS_ID_FACTORY_RESET         , EVENT_CB_TASK_RUN  , factory_reset},
  {SYS_ID_HEARTBEAT             , EVENT_CB_DIRECT_RUN, heart_event},
  {SYS_ID_REPORT_LOG            , EVENT_CB_DIRECT_RUN, retport_log},
  {SYS_ID_REQ_MODULE_INFO       , EVENT_CB_DIRECT_RUN, req_module_info},
  {SYS_ID_REQ_MACHINE_INFO      , EVENT_CB_DIRECT_RUN, req_machine_info},
  {SYS_ID_REQ_MACHINE_SIZE      , EVENT_CB_DIRECT_RUN, req_machine_size},
  {SYS_ID_REQ_COORDINATE_SYSTEM , EVENT_CB_DIRECT_RUN, req_coordinate_system},
  {SYS_ID_SET_COORDINATE_SYSTEM , EVENT_CB_DIRECT_RUN, set_coordinate_system},
  {SYS_ID_SET_ORIGIN            , EVENT_CB_DIRECT_RUN, set_origin},
  {SYS_ID_MOVE_RELATIVE         , EVENT_CB_TASK_RUN  , move_relative},
  {SYS_ID_MOVE                  , EVENT_CB_TASK_RUN  , move},
  {SYS_ID_HOME                  , EVENT_CB_TASK_RUN  , home},
  {SYS_ID_GET_MOTOR_ENABLE      , EVENT_CB_DIRECT_RUN, get_motor_enable},
  {SYS_ID_SET_MOTOR_ENABLE      , EVENT_CB_DIRECT_RUN, set_motor_enable},
  {SYS_ID_MOVE_TO_RELATIVE_HOME , EVENT_CB_TASK_RUN  , move_relative_home},
};