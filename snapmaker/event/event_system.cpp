#include "event_system.h"
#include "event_base.h"
#include "subscribe.h"

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


event_cb_info_t system_cb_info[SYS_ID_CB_COUNT] = {
  {SYS_ID_SUBSCRIBE             , EVENT_CB_DIRECT_RUN, subscribe_event},
  {SYS_ID_UNSUBSCRIBE           , EVENT_CB_DIRECT_RUN, unsubscribe_event},
};