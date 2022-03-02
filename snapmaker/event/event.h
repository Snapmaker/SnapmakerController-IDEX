#ifndef EVEVT_H
#define EVEVT_H
#include <functional>
#include "event_base.h"
#include "../J1/common_type.h"
#include "../protocol/protocol_sacp.h"

#define EVENT_CACHE_COUNT 3

typedef enum {
  EVENT_CACHT_STATUS_IDLE,
  EVENT_CACHT_STATUS_WAIT,
  EVENT_CACHT_STATUS_BUSY,
} event_cache_node_status_e;


typedef struct {
  event_cache_node_status_e block_status;  // idle,wait, busy
  event_param_t param;  // Parameters to be passed into the callback function
  evevnt_cb_f cb;  // event callback
} event_cache_node_t;


// Parameters passed by the event source
typedef struct {
  SACP_struct_t *info;  // Full sacp protocol information
  event_source_e onwer;  // Event source
}evevt_struct_t;

class EventHandler {
  public:
    EventHandler() {
      for (uint8_t i = 0; i < EVENT_CACHE_COUNT; i++) {
        event_cache[i].block_status = EVENT_CACHT_STATUS_IDLE;
      }
    }

    ErrCode parse(evevt_struct_t &data);
    void loop_task();

  private:
    void parse_event_info(evevt_struct_t &data, event_cache_node_t *event);
    event_cache_node_t * get_event_cache();

  private:
    event_cache_node_t event_cache[EVENT_CACHE_COUNT];
};
void event_init();

extern EventHandler event_handler;
#endif
