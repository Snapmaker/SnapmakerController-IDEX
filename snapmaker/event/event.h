#ifndef EVEVT_H
#define EVEVT_H
#include "event_base.h"
#include "../J1/common_type.h"
#include "../protocol/protocol_sacp.h"

#define EVENT_CACHE_COUNT 6

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

typedef struct {
  bool enable;
  SACP_param_t sacp_params;
  event_source_e recv_source;  // Event source
} recv_data_info_t;

class EventHandler {
  public:
    EventHandler() {
      for (uint8_t i = 0; i < EVENT_CACHE_COUNT; i++) {
        event_cache[i].block_status = EVENT_CACHT_STATUS_IDLE;
      }
    }

    void loop_task();
    void recv_task();
    void recv_enable(event_source_e source, bool enable);
    void recv_enable(event_source_e source);

  private:
    ErrCode parse(recv_data_info_t *recv_info);
    void parse_event_info(recv_data_info_t *recv_info, event_cache_node_t *event);
    event_cache_node_t * get_event_cache();

  private:
    event_cache_node_t event_cache[EVENT_CACHE_COUNT];
    recv_data_info_t recv_data_info[EVENT_SOURCE_ALL] = {0};
};
void event_init();
void event_port_init();
extern EventHandler event_handler;
#endif
