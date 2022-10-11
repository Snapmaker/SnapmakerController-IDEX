#include "event_base.h"
#include "../protocol/protocol_sacp.h"

HardwareSerial *event_serial[EVENT_SOURCE_ALL] = {&MSerial1, &MSerial2};

write_byte_f event_write_byte[EVENT_SOURCE_ALL] = {
  [](unsigned char ch)->size_t{return event_serial[EVENT_SOURCE_MARLIN]->write_byte(ch);},
  [](unsigned char ch)->size_t{return event_serial[EVENT_SOURCE_HMI]->write_byte(ch);},
};

static SemaphoreHandle_t event_write_lock[EVENT_SOURCE_ALL] {NULL};

read_byte_f event_read_byte[EVENT_SOURCE_ALL] = {
  []()->size_t{return event_serial[EVENT_SOURCE_MARLIN]->read();},
  []()->size_t{return event_serial[EVENT_SOURCE_HMI]->read();},
};

void event_base_init() {
  for (auto &lock : event_write_lock) {
    lock = xSemaphoreCreateMutex();
    configASSERT(lock);
  }
}


event_cb_info_t * get_evevt_info_by_id(uint8_t id, event_cb_info_t *array, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    if (array[i].command_id == id) {
      return &array[i];
    }
  }
  return NULL;
}


static bool send_to(event_source_e source, uint8_t *data, uint16_t len) {
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    if (xSemaphoreTake(event_write_lock[source], portMAX_DELAY) == pdPASS) {
      for (int i = 0; i < len; i++) {
        event_write_byte[source](data[i]);
      }
      xSemaphoreGive(event_write_lock[source]);

      return true;
    }

    return false;
  }
  else {
    for (int i = 0; i < len; i++) {
      event_write_byte[source](data[i]);
    }

    return true;
  }
}

bool send_data(event_source_e source, uint8_t *data, uint16_t len) {
  if (source == EVENT_SOURCE_ALL) {
    for (uint8_t s = 0; s < EVENT_SOURCE_ALL; s++) {
      send_to((event_source_e)s, data, len);
    }
  } else if (source < EVENT_SOURCE_ALL){
    send_to(source, data, len);
  }

  return true;
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
  uint8_t  send_buf[PACK_PARSE_MAX_SIZE];
  uint16_t pack_len = 0;

  if ((source < EVENT_SOURCE_ALL) && !event_serial[source]->enable_sacp()) {
    return E_PARAM;
  }

  if (length + SACP_HEADER_LEN > PACK_PARSE_MAX_SIZE) {
    send_data(EVENT_SOURCE_ALL, (uint8_t *)STR_PACK_TOO_LARGE, sizeof(STR_PACK_TOO_LARGE));
  }

  // Package the data and call write_byte to emit the information
  pack_len = protocol_sacp.package(sacp, data, length, send_buf);

  send_data(source, send_buf, pack_len);

  if (!event_serial[EVENT_SOURCE_MARLIN]->enable_sacp()) {
    // char debug_buf[100];
    // sprintf(debug_buf, "MC:source:0x%x ,cmd_set:0x%x,cmd_id:0x%x, sequence:%d, len:%d", source, sacp.command_set, sacp.command_id, sacp.sequence, length);
    // SERIAL_ECHOLN(debug_buf);
  }

  return E_SUCCESS;
}

ErrCode send_event(event_source_e source, uint8_t recever_id, uint8_t attribute,
                   uint8_t command_set, uint8_t command_id, uint8_t *data, uint16_t length, uint16_t sequence) {
  SACP_head_base_t sacp = {recever_id, attribute, 0, command_set, command_id};
  if (!sequence) {
    sacp.sequence = protocol_sacp.sequence_pop();
  } else {
    sacp.sequence = sequence;
  }

  send_event(source, sacp, data, length);
  return E_SUCCESS;
}

ErrCode send_result(event_param_t &event, ErrCode result) {
  return send_event(event, (uint8_t *)&result, 1);
}
