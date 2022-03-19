#include "event.h"
#include "../J1/common_type.h"
#include "event_system.h"
#include "event_fdm.h"
#include "event_bed.h"
#include "event_adjust.h"
#include "event_printer.h"
#include "event_enclouser.h"

EventHandler event_handler;
static QueueHandle_t event_queue = NULL;

event_cb_info_t * get_event_info(uint8_t cmd_set, uint8_t cmd_id) {
  switch (cmd_set) {
    case COMMAND_SET_SYS:
      return get_evevt_info_by_id(cmd_id, system_cb_info, SYS_ID_CB_COUNT);
    case COMMAND_SET_FDM:
      return get_evevt_info_by_id(cmd_id, fdm_cb_info, FDM_ID_CB_COUNT);
    case COMMAND_SET_BED:
      return get_evevt_info_by_id(cmd_id, bed_cb_info, BED_ID_CB_COUNT);
    case COMMAND_SET_ADJUSTING:
      return get_evevt_info_by_id(cmd_id, adjust_cb_info, ADJUST_ID_CB_COUNT);
    case COMMAND_SET_PRINTER:
      return get_evevt_info_by_id(cmd_id, printer_cb_info, PRINTER_ID_CB_COUNT);
    case COMMAND_SET_ENCLOUSER:
      return get_evevt_info_by_id(cmd_id, enclouser_cb_info, ENCLOUSER_ID_CB_COUNT);
  }
  return NULL;
}

void EventHandler::parse_event_info(evevt_struct_t &data, event_cache_node_t *event) {
  event_param_t *param = &event->param;
  SACP_struct_t *info = data.info;
  param->info.attribute = SACP_ATTR_ACK;
  param->info.command_set = info->command_set;
  param->info.command_id = info->command_id;
  param->info.recever_id = info->sender_id;
  param->info.sequence = info->sequence;
  param->source = data.onwer;
  param->length = info->length;
  param->length -= 8;  // Effective data length
  SERIAL_ECHOLNPAIR("event data len:", param->length);
  for (uint32_t i = 0; i < param->length; i++) {
    param->data[i] = info->data[i];
  }
}

event_cache_node_t * EventHandler::get_event_cache() {
  for (uint8_t i = 0; i < EVENT_CACHE_COUNT; i++) {
    if (event_cache[i].block_status == EVENT_CACHT_STATUS_IDLE) {
      event_cache[i].block_status = EVENT_CACHT_STATUS_BUSY;
      return &event_cache[i];
    }
  }
  return NULL;
}

ErrCode EventHandler::parse(evevt_struct_t &data) {
  event_cache_node_t *event = get_event_cache();
  if (!event) {
    SERIAL_ECHO("ERROR:event no cache\n");
    return E_NO_MEM;
  }

  parse_event_info(data, event);
  // SERIAL_ECHOLNPAIR("event set:", data.info->command_set, " ,id:", data.info->command_id);
  event_cb_info_t * cb_info = get_event_info(data.info->command_set, data.info->command_id);
  if (!cb_info) {
    event->block_status = EVENT_CACHT_STATUS_IDLE;
    SERIAL_ECHOLN("ERROR: find no event cb");
    return E_PARAM;
  }

  event->cb = cb_info->cb;
  if (cb_info->type == EVENT_CB_DIRECT_RUN) {
    SERIAL_ECHOLN("run event cb");
    (event->cb)(event->param);
    event->block_status = EVENT_CACHT_STATUS_IDLE;
    return E_SUCCESS;
  } else {
    SERIAL_ECHOLN("wait event cb");
    event->block_status = EVENT_CACHT_STATUS_WAIT;
    if (xQueueSend(event_queue, (void *)&event, (TickType_t)0) != pdPASS ) {
      SERIAL_ECHOLN("event cacne full!!!");
      send_result(event->param, E_NO_MEM);
    } else {
    }
  }
  return E_PARAM;
}

void EventHandler::loop_task() {
  event_cache_node_t *event = NULL;
  while (true) {
    if (xQueueReceive(event_queue, &event, 1 ) == pdPASS) {
      if (event->block_status == EVENT_CACHT_STATUS_WAIT) {
        event->block_status = EVENT_CACHT_STATUS_BUSY;
        (event->cb)(event->param);
        event->block_status = EVENT_CACHT_STATUS_IDLE;
      }
    }
    printer_event_loop();
  }
}

static void event_task(void * arg) {
  event_handler.loop_task();
}

void event_init() {
  printer_event_init();
  event_queue = xQueueCreate(EVENT_CACHE_COUNT, sizeof(event_cache_node_t *));
  BaseType_t ret = xTaskCreate(event_task, "event_loop", 1000,NULL, 5, NULL);
  if (ret != pdPASS) {
    SERIAL_ECHO("Failed to create event_loop!\n");
  }
  else {
    SERIAL_ECHO("Created event_loop task!\n");
  }
}
