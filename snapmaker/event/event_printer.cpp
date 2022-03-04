#include "event_printer.h"
#include "../module/print_control.h"
#include "../module/power_loss.h"
#include "../module/system.h"

#define GCODE_MAX_PACK_SIZE 450
#define GCODE_REQ_TIMEOUT_MS 1000

#pragma pack(1)

typedef struct {
  uint16_t md5_length;
  uint8_t md5_str[GCODE_MD5_LENGTH];
  uint16_t name_size;
  uint8_t name[GCODE_FILE_NAME_SIZE];
} gcode_file_info_t;

typedef struct {
  uint32_t line_number;
  uint16_t buf_max_size;
} batch_gcode_req_info_t;

typedef struct {
  uint8_t flag;
  uint32_t start_line;
  uint32_t end_line;
  uint16_t  data_len;
  uint8_t data[];
} batch_gcode_t;

#pragma pack()

typedef enum {
  GCODE_PACK_REQ_IDLE,
  GCODE_PACK_REQ_WAIT,
  GCODE_PACK_REQ_RECV,
  GCODE_PACK_REQ_DONE,
} gcode_req_status_e;

event_source_e print_source = EVENT_SOURCE_HMI;
uint8_t source_recever_id = 0;
gcode_req_status_e gcode_req_status = GCODE_PACK_REQ_IDLE;
uint32_t gcode_req_timeout = 0;

static uint16_t load_gcode_file_info(uint8_t *buf) {
  uint16_t ret_len = 0;
  uint8_t temp_len = 0;
  uint8_t * addr = power_loss.get_file_md5(temp_len);

  if (addr) {
    memcpy(buf+2, addr, temp_len);
    *((uint16_t *)buf) = temp_len;
    ret_len += temp_len + 2;
  } else {
    SERIAL_ECHOLNPAIR("load gcode file MD5 failed");
  }

  buf += (temp_len + 2);
  addr = power_loss.get_file_name(temp_len);
  if (addr) {
    memcpy(buf+2, addr, temp_len);
    *((uint16_t *)buf) = temp_len;
    ret_len += temp_len + 2;
  } else {
    SERIAL_ECHOLNPAIR("load gcode file name failed");
  }
  return ret_len;
}

static ErrCode request_file_info(event_param_t& event) {
  uint16_t ret_len = load_gcode_file_info(event.data+1);
  SERIAL_ECHOLN("SC req file info");
  if (ret_len) {
    event.data[0] = E_SUCCESS;
    event.length = ret_len + 1;
  } else {
    event.data[0] = PRINT_RESULT_NO_FILE_INFO_E;
    event.length = 5;
  }
  event.length = sizeof(gcode_file_info_t) + 1;
  return send_event(event);
}

static ErrCode gcode_pack_deal(event_param_t& event) {
  batch_gcode_t *gcode = (batch_gcode_t *)event.data;
  print_control.push_gcode(gcode->start_line, gcode->end_line, gcode->data, gcode->data_len);
  if (gcode->flag == PRINT_RESULT_GCODE_RECV_DONE_E) {
    gcode_req_status = GCODE_PACK_REQ_DONE;
    SERIAL_ECHOLN("SC gcoce pack recv done");
  } else {
    gcode_req_status = GCODE_PACK_REQ_IDLE;
  }
  return E_SUCCESS;
}

static ErrCode request_start_work(event_param_t& event) {
  ErrCode result= print_control.start();
  batch_gcode_req_info_t *info = (batch_gcode_req_info_t *)(event.data + 1);
  SERIAL_ECHOLNPAIR("SC req start work, ret:", result);
  if (result == E_SUCCESS) {
    // set md5 and file name
    uint8_t data_len = event.data[0];
    uint8_t *data = event.data + 1;
    power_loss.set_file_md5(data, data_len);
    data = event.data + data_len + 2;
    data_len = event.data[data_len + 1];
    power_loss.set_file_name(data, data_len);

    // init print information
    print_source = event.source;
    source_recever_id = event.info.recever_id;
    gcode_req_status = GCODE_PACK_REQ_WAIT;
  }
  event.data[0] = result;
  info->buf_max_size = GCODE_MAX_PACK_SIZE;
  info->line_number = print_control.next_req_line();
  event.length = sizeof(batch_gcode_req_info_t) + 1;
  return send_event(event);
}

static ErrCode request_pause_work(event_param_t& event) {
  ErrCode result = print_control.pause();
  gcode_req_status = GCODE_PACK_REQ_IDLE;
  SERIAL_ECHOLNPAIR("SC req pause work, ret:", result);
  event.data[0] = result;
  event.length = 1;
  return send_event(event);
}

static ErrCode request_resume_work(event_param_t& event) {
  batch_gcode_req_info_t *info = (batch_gcode_req_info_t *)(event.data + 1);
  ErrCode result = print_control.resume();
  SERIAL_ECHOLNPAIR("SC req resume work, ret:", result);
  event.data[0] = result;
  info->buf_max_size = GCODE_MAX_PACK_SIZE;
  info->line_number = print_control.next_req_line();
  event.length = sizeof(batch_gcode_req_info_t) + 1;
  return send_event(event);
}

static ErrCode request_stop_work(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC req stop work");
  event.data[0] = print_control.stop();
  event.length = 1;
  return send_event(event);
}

static ErrCode request_power_loss_status(event_param_t& event) {
  event.data[0] = power_loss.power_loss_status();
  SERIAL_ECHOLNPAIR("SC req power loos status:", event.data[0]);
  if (event.data[0] == E_SUCCESS) {
    uint16_t ret_len = load_gcode_file_info(event.data+1);
    event.length = ret_len;
  } else {
    event.length = 5;
  }
  return send_event(event);
}

static ErrCode request_power_loss_resume(event_param_t& event) {
  batch_gcode_req_info_t *info = (batch_gcode_req_info_t *)(event.data + 1);
  SERIAL_ECHOLNPAIR("SC req power loos resume");
  event.data[0] = power_loss.power_loss_resume();
  if (event.data[0] == E_SUCCESS) {
    print_source = event.source;
    source_recever_id = event.info.recever_id;
    gcode_req_status = GCODE_PACK_REQ_WAIT;
  }
  info->buf_max_size = GCODE_MAX_PACK_SIZE;
  info->line_number = print_control.next_req_line();
  event.length = sizeof(batch_gcode_req_info_t) + 1;
  return E_SUCCESS;
}

static ErrCode request_clear_power_loss(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC req power loos clear");
  power_loss.clear();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode set_printer_mode(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC set print mode:", event.data[0]);
  event.data[0] = print_control.set_mode((print_mode_e)event.data[0]);
  event.length = 1;
  return send_event(event);
}

static ErrCode request_auto_pack_status(event_param_t& event) {
  bool status = print_control.is_auto_pack_mode();
  SERIAL_ECHOLNPAIR("SC req auto pack mode:", status);
  event.data[0] = E_SUCCESS;
  event.data[1] = status;
  event.length = 2;
  return send_event(event);
}

static ErrCode set_auto_pack_mode(event_param_t& event) {
  print_mode_e mode = PRINT_FULL_CONTROL_MODE;
  if (event.data[0])
    mode = PRINT_AUTO_PARK_MODE;
  SERIAL_ECHOLNPAIR("SC req auto pack mode:", mode);
  event.data[0] = print_control.set_mode(mode);
  event.length = 1;
  return send_event(event);
}

static ErrCode request_cur_line(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  uint32_t *cur_line = (uint32_t *)(event.data+1);
  *cur_line = print_control.get_cur_line();
  event.length = 5;
  return send_event(event);
}

event_cb_info_t printer_cb_info[PRINTER_ID_CB_COUNT] = {
  {PRINTER_ID_REQ_FILE_INFO       , EVENT_CB_DIRECT_RUN, request_file_info},
  {PRINTER_ID_REQ_GCODE           , EVENT_CB_TASK_RUN,   gcode_pack_deal},
  {PRINTER_ID_START_WORK          , EVENT_CB_TASK_RUN,   request_start_work},
  {PRINTER_ID_PAUSE_WORK          , EVENT_CB_TASK_RUN,   request_pause_work},
  {PRINTER_ID_RESUME_WORK         , EVENT_CB_TASK_RUN,   request_resume_work},
  {PRINTER_ID_STOP_WORK           , EVENT_CB_TASK_RUN,   request_stop_work},
  {PRINTER_ID_REQ_PL_STATUS       , EVENT_CB_DIRECT_RUN, request_power_loss_status},
  {PRINTER_ID_PL_RESUME           , EVENT_CB_TASK_RUN,   request_power_loss_resume},
  {PRINTER_ID_CLEAN_PL_DATA       , EVENT_CB_TASK_RUN,   request_clear_power_loss},
  {PRINTER_ID_SET_MODE            , EVENT_CB_TASK_RUN,   set_printer_mode},
  {PRINTER_ID_REQ_AUTO_PARK_STATUS, EVENT_CB_DIRECT_RUN, request_auto_pack_status},
  {PRINTER_ID_SET_AUTO_PARK_STATUS, EVENT_CB_TASK_RUN,   set_auto_pack_mode},
  {PRINTER_ID_REQ_LINE            , EVENT_CB_DIRECT_RUN, request_cur_line},
};

static void req_gcode_pack() {
  batch_gcode_req_info_t info;
  uint16_t free_buf = print_control.get_buf_free();
  // SERIAL_ECHOLNPAIR("gcode buf free:", free_buf);
  if (free_buf >= GCODE_MAX_PACK_SIZE) {
    info.line_number = print_control.next_req_line();
    info.buf_max_size = GCODE_MAX_PACK_SIZE;
    send_event(print_source, source_recever_id, SACP_ATTR_REQ,
        COMMAND_SET_PRINTER, PRINTER_ID_REQ_GCODE, (uint8_t *)&info, sizeof(info));
    gcode_req_status = GCODE_PACK_REQ_WAIT;
    gcode_req_timeout = millis() + GCODE_REQ_TIMEOUT_MS;
    SERIAL_ECHOLNPAIR("gcode requst start line:", info.line_number, ",size:", info.buf_max_size);
  }
}

static void gcode_req_timeout_deal() {
  if (gcode_req_timeout < millis()) {
    SERIAL_ECHOLNPAIR("requst gcode pack timeout!");
    gcode_req_status = GCODE_PACK_REQ_IDLE;
  }
}

static void report_status_info(ErrCode status) {
  send_event(print_source, source_recever_id, SACP_ATTR_REQ,
      COMMAND_SET_PRINTER, PRINTER_ID_REPORT_STATUS, &status, 1);
}

void wait_print_end(void) {
  if (print_control.buffer_is_empty()) {
    SERIAL_ECHOLNPAIR("print done and will stop");
    gcode_req_status = GCODE_PACK_REQ_IDLE;
    print_control.stop();
    report_status_info(0);
    SERIAL_ECHOLNPAIR("Active stop end");
  }
}

void printer_event_init(void) {
  power_loss.init();
}

void printer_event_loop(void) {
  if (system_service.get_status() == SYSTEM_STATUE_PRINTING) {
    switch (gcode_req_status) {
      case GCODE_PACK_REQ_IDLE:
        req_gcode_pack();
        break;
      case GCODE_PACK_REQ_WAIT:
        gcode_req_timeout_deal();
        break;
      case GCODE_PACK_REQ_DONE:
        wait_print_end();
        break;
      default:
        break;
    }
  }
}
