#include "event_base.h"
#include "../protocol/protocol_sacp.h"

static write_byte_f event_write_cb[EVENT_SOURCE_ALL] = {NULL};

ErrCode write_fun_register(event_source_e source, write_byte_f cb) {
  if (source < EVENT_SOURCE_ALL) {
    event_write_cb[source] = cb;
    return E_SUCCESS;
  }
  return E_PARAM;
}

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

static bool send_to(write_byte_f write, uint8_t *data, uint16_t len) {
  if (write) {
    for (int i = 0; i < len; i++) {
      write(data[i]);
    }
    return true;
  }
  return false;
}

static bool send_data(event_source_e source, uint8_t *data, uint16_t len) {
  if (source == EVENT_SOURCE_ALL) {
    for (uint8_t s = 0; s < EVENT_SOURCE_ALL; s++) {
      send_to(event_write_cb[s], data, len);
    }
  } else if (source < EVENT_SOURCE_ALL){
    send_to(event_write_cb[source], data, len);
  }
  return false;
}

ErrCode send_event(event_param_t &event) {
  send_event(event.source, event.info, event.data, event.length);
  return E_SUCCESS;
}

ErrCode send_event(event_param_t &event, uint8_t *data, uint16_t length) {
  send_event(event.source, event.info, data, length);
  return E_SUCCESS;
}

ErrCode send_event(event_source_e source, SACP_head_base_t &sacp, uint8_t *data, uint16_t length) {
  while (send_lock) {
    vTaskDelay(2);
  }
  send_lock = true;
  // Package the data and call write_byte to emit the information
  uint16_t pack_len = protocol_sacp.package(sacp, data, length, send_buf);
  send_data(source, send_buf, pack_len);
  send_lock = false;
  return E_SUCCESS;
}

ErrCode send_event(event_source_e source, uint8_t recever_id, uint8_t attribute,
                   uint8_t command_set, uint8_t command_id, uint8_t *data, uint16_t length) {
  SACP_head_base_t sacp = {recever_id, attribute, 0, command_set, command_id};
  sacp.sequence = protocol_sacp.sequence_pop();
  send_event(source, sacp, data, length);
  return E_SUCCESS;
}

ErrCode send_result(event_param_t &event, ErrCode result) {
  send_event(event, (uint8_t *)&result, 1);
}
