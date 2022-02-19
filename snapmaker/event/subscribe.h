#ifndef SUBSCRIBE_H
#define SUBSCRIBE_H

#include "../J1/common_type.h"
#include "../protocol/protocol_sacp.h"
#include "event_base.h"

#define MAX_SUBSCRIBE_COUNT 30

typedef struct {
  bool is_available;
  event_source_e source;
  uint16_t time_interval;
  uint32_t last_time;
  SACP_head_base_t info;
  write_byte_f write_byte;
  evevnt_cb_f cb;
} subscribe_node_t;

class Subscribe {
  public:
    ErrCode enable(event_param_t &event);
    ErrCode disable(event_param_t &event);
    void loop_task(void *arg);
  private:
    subscribe_node_t sub[MAX_SUBSCRIBE_COUNT];
    uint8_t sub_count;
};
void subscribe_init(void);
extern Subscribe subscribe;
#endif
