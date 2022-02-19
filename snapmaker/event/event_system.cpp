#include "event_system.h"
#include "event_base.h"
#include "subscribe.h"
#include "../module/motion_control.h"

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

static ErrCode move_relative(event_param_t& event) {
  mobile_instruction_t *move = (mobile_instruction_t *)(event.data + 1);
  motion_control.move_axis(*move);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode move(event_param_t& event) {
  mobile_instruction_t *move = (mobile_instruction_t *)(event.data + 1);
  motion_control.move_axis_to(*move);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

event_cb_info_t system_cb_info[SYS_ID_CB_COUNT] = {
  {SYS_ID_SUBSCRIBE             , EVENT_CB_DIRECT_RUN, subscribe_event},
  {SYS_ID_UNSUBSCRIBE           , EVENT_CB_DIRECT_RUN, unsubscribe_event},
  {SYS_ID_MOVE_RELATIVE         , EVENT_CB_TASK_RUN  , move_relative},
  {SYS_ID_MOVE                  , EVENT_CB_TASK_RUN  , move},
};