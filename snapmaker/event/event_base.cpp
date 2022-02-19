#include "event_base.h"
#include "../protocol/protocol_sacp.h"
event_cb_info_t * get_evevt_info_by_id(uint8_t id, event_cb_info_t *array, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (array[i].command_id == id) {
      return &array[i];
    }
  }
  return NULL;
}

static bool send_lock = false;
static uint8_t send_buf[PACK_PARSE_MAX_SIZE];
ErrCode send_event(event_param_t &event) {
  while (send_lock) {
    vTaskDelay(2);
  }
  send_lock = true;
  // Package the data and call write_byte to emit the information
  uint16_t pack_len = protocol_sacp.package(event.info, event.data, event.length, send_buf);
  for (int i = 0; i < pack_len; i++) {
    event.write_byte(send_buf[i]);
  }
  send_lock = false;
  return E_SUCCESS;
}
