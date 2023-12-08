/*
 * Snapmaker 3D Printer Firmware
 * Copyright (C) 2023 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of SnapmakerController-IDEX
 * (see https://github.com/Snapmaker/SnapmakerController-IDEX)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "event_printer.h"
#include "../module/print_control.h"
#include "../module/power_loss.h"
#include "../module/filament_sensor.h"
#include "../module/system.h"
#include "../module/fdm.h"
#include "../module/bed_control.h"
#include "../module/motion_control.h"
#include "../../../src/module/AxisManager.h"
#include "../../Marlin/src/module/temperature.h"


#define GCODE_MAX_PACK_SIZE     (450)
#define GCODE_REQ_TIMEOUT_MS    (200)
#define GCODE_TIMEOUT_MAX_CNT   (8)  // 25.6 second

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

typedef struct {
  uint8_t key;
  uint8_t e_count;
  uint16_t percentage;
} feedrate_percentage_t;

typedef struct {
  uint8_t key;
  uint8_t e_index;
  uint16_t percentage;
} flow_percentage_t;

#pragma pack()

typedef enum {
  GCODE_PACK_REQ_IDLE,
  GCODE_PACK_REQ_WAIT_RECV,
  GCODE_PACK_REQ_WAIT_CACHE,
  GCODE_PACK_REQ_DONE,
} gcode_req_status_e;

typedef enum {
  STATUS_PRINT_DONE,
  STATUS_PAUSE_BE_GCODE,
  STATUS_PAUSE_BE_GCODE_FILAMENT,
  STATUS_PAUSE_BE_FILAMENT,
  STATUS_STALL_GUARD,
  STATUS_TEMPERATURE_ERR,
  STATUS_GCODE_LINES_ERR,
  STATUS_PAUSE_BE_EXCEPTION = 20,
} report_status_e;


// 这里的变量应该统一结构体管理
event_source_e print_source = EVENT_SOURCE_HMI;
event_source_e rep_gcode_source = EVENT_SOURCE_HMI;
uint8_t rep_gcode_recever_id = 0;
uint8_t source_recever_id = 0;
uint16_t source_sequence = 0;
gcode_req_status_e gcode_req_status = GCODE_PACK_REQ_IDLE;
uint32_t gcode_req_timeout = 0;
uint32_t gcode_req_timeout_times = 0;
uint32_t gcode_req_base_wait_ms = 0;

bool start_pause_record = false;
uint32_t start_pause_time_ms = 0;
bool pause_hotend_tmp_down = 0;


static void req_gcode_pack();
static void report_status_info(ErrCode status);
static void save_event_suorce_info(event_param_t& event, bool update_get_gcode_info=false);

static void save_event_suorce_info(event_param_t& event, bool update_get_gcode_info) {
  print_source = event.source;
  source_recever_id = event.info.recever_id;
  source_sequence = event.info.sequence;

  if (update_get_gcode_info) {
    rep_gcode_source = event.source;
    rep_gcode_recever_id = event.info.recever_id;
  }
}

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
  ErrCode ret;
  batch_gcode_t *gcode = (batch_gcode_t *)event.data;
  ret = print_control.push_gcode(gcode->start_line, gcode->end_line, gcode->data, gcode->data_len);
  if (gcode->flag == PRINT_RESULT_GCODE_RECV_DONE_E) {
    gcode_req_status = GCODE_PACK_REQ_DONE;
    SERIAL_ECHOLN("SC gcoce pack recv done");
  } else {
    if (E_SUCCESS == ret) {
      if (gcode_req_timeout_times) gcode_req_timeout_times--;
      gcode_req_base_wait_ms = 0;
      req_gcode_pack();
    }
  }
  return E_SUCCESS;
}

static ErrCode request_start_work(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC req start work");
  ErrCode result= print_control.start();
  SERIAL_ECHOLNPAIR("start work result:", result);
  if (result == E_SUCCESS) {
    // set md5 and file name
    uint16_t data_len = *((uint16_t *)event.data);
    uint8_t *data = event.data + 2;
    power_loss.set_file_md5(data, data_len);
    data = event.data + data_len + 4;
    data_len = *((uint16_t *)&event.data[data_len + 2]);
    power_loss.set_file_name(data, data_len);

    save_event_suorce_info(event, true);
  }
  event.data[0] = result;
  event.length = 1;
  send_event(event);
  if (result == E_SUCCESS) {
    gcode_req_timeout_times = 0;
    gcode_req_base_wait_ms = 2000;
    req_gcode_pack();
  }
  return result;
}

static ErrCode request_pause_work(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC req pause working...");
  if (E_SUCCESS == system_service.set_status(SYSTEM_STATUE_PAUSING, SYSTEM_STATUE_SCOURCE_SACP)) {
    gcode_req_status = GCODE_PACK_REQ_IDLE;
    save_event_suorce_info(event);
    return E_SUCCESS;
  }
  else {
    LOG_E("can NOT set to SYSTEM_STATUE_PAUSING\r\n");
    event.data[0] = PRINT_RESULT_PAUSE_ERR_E;
    event.length = 1;
    SERIAL_ECHOLNPAIR("SC req pause work failed");
    return send_event(event);
  }
}

static ErrCode request_resume_work(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC req resume work");
  if (E_SUCCESS == system_service.set_status(SYSTEM_STATUE_RESUMING, SYSTEM_STATUE_SCOURCE_SACP)) {
    save_event_suorce_info(event);
    return E_SUCCESS;
  }
  else {
    LOG_E("can NOT set to SYSTEM_STATUE_RESUMING\r\n");
    event.data[0] = PRINT_RESULT_RESUME_ERR_E;
    event.length = 1;
    SERIAL_ECHOLNPAIR("SC req pause work failed");
    return send_event(event);
  }
}

static ErrCode request_stop_work(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC req stop work");
  if (E_SUCCESS == system_service.set_status(SYSTEM_STATUE_STOPPING, SYSTEM_STATUE_SCOURCE_SACP)) {
    gcode_req_status = GCODE_PACK_REQ_IDLE;
    save_event_suorce_info(event);
    return E_SUCCESS;
  }
  else {
    LOG_E("can NOT set to SYSTEM_STATUE_STOPPING\r\n");
    event.data[0] = PRINT_RESULT_STOP_E;
    event.length = 1;
    SERIAL_ECHOLNPAIR("SC req stop work failed");
    return send_event(event);
  }
}

static ErrCode request_stop_single_extrude_work(event_param_t& event) {
  uint8_t e = MODULE_INDEX(event.data[0]);
  uint8_t en = event.data[1];
  SERIAL_ECHOLNPAIR("SC req extrude:", e, " enable:", en);

  if (system_service.get_status() == SYSTEM_STATUE_IDLE && !fdm_head.is_duplicating()) {
    event.data[0] = PRINT_RESULT_START_ERR_E;
    event.length = 1;
    SERIAL_ECHOLNPAIR("SC req single stop work failed");
    return send_event(event);
  }

  if (E_SUCCESS == system_service.set_status(SYSTEM_STATUE_PAUSING, SYSTEM_STATUE_SCOURCE_STOP_EXTRUDE)) {
    // Printing can only be stopped.
    // This function will be enabled again when printing ends
    save_event_suorce_info(event);
    // fdm_head.set_duplication_enabled(e, en);
    fdm_head.stop_single_extruder_e = e;
    fdm_head.stop_single_extruder_en = en;
    return E_SUCCESS;
  }
  else {
    LOG_E("can NOT set to SYSTEM_STATUE_PAUSING\r\n");
    power_loss.stash_data.extruder_dual_enable[e] = en;
    event.data[0] = E_SUCCESS;
    event.length = 1;
    return send_event(event);
  }
}

static ErrCode request_power_loss_status(event_param_t& event) {
  event.data[0] = power_loss.is_power_loss_data();
  SERIAL_ECHOLNPAIR("SC req power loss status:", event.data[0]);
  if (event.data[0] == E_SUCCESS) {
    uint16_t ret_len = load_gcode_file_info(event.data+1);
    event.length = ret_len + 1;
  } else {
    event.length = 5;
  }
  return send_event(event);
}

static ErrCode request_power_loss_resume(event_param_t& event) {
  SERIAL_ECHOLNPAIR("SC req power loss resume");
  ErrCode ret = power_loss.power_loss_resume();
  event.data[0] = E_SUCCESS;
  event.length = 1;
  send_event(event);
  if (ret == E_SUCCESS) {
    SERIAL_ECHOLNPAIR("power loss resume success");
    event.data[0] = E_SUCCESS;
    event.length = 1;
    send_event(event);
    save_event_suorce_info(event, true);
    req_gcode_pack();
  } else if (ret == E_SYSTEM_EXCEPTION) {
    SERIAL_ECHOLNPAIR("power loss resume success but lilament trigger");
    event.data[0] = E_SUCCESS;
    event.length = 1;
    send_event(event);
    save_event_suorce_info(event, true);
    report_status_info(STATUS_PAUSE_BE_FILAMENT);
  } else {
    event.data[0] = ret;
    event.length = 1;
    send_event(event);
    SERIAL_ECHOLNPAIR("power loss resume failed");
  }
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
  LOG_I("SC set print mode:%d\n", event.data[0]);
  event.data[0] = print_control.set_mode((print_mode_e)event.data[0]);
  event.length = 1;
  return send_event(event);
}

static ErrCode subscribe_print_mode(event_param_t& event) {
  uint8_t  mode = print_control.get_mode();
  LOG_V("SC subscribe print mode:%d\n", mode);
  event.data[0] = E_SUCCESS;
  event.data[1] = mode;
  event.length = 2;
  return send_event(event);
}

static ErrCode request_auto_pack_status(event_param_t& event) {
  bool status = print_control.is_backup_mode();
  SERIAL_ECHOLNPAIR("SC req auto pack mode:", status);
  event.data[0] = E_SUCCESS;
  event.data[1] = status;
  event.length = 2;
  return send_event(event);
}

static ErrCode set_print_offset(event_param_t& event) {
  float_to_int_t * val = (float_to_int_t *)event.data;
  float x = INT_TO_FLOAT(val[0]);
  float y = INT_TO_FLOAT(val[1]);
  float z = INT_TO_FLOAT(val[2]);
  SERIAL_ECHOLNPAIR("Sprint_controlC req print offset x:", x, " y:", y, " z:", z);
  event.data[0] = print_control.set_print_offset(x, y, z);
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

static ErrCode set_temperature_lock(event_param_t& event) {
  uint8_t e = MODULE_INDEX(event.data[0]);
  uint8_t lock = MODULE_INDEX(event.data[2]);
  SERIAL_ECHOLNPAIR("SC set temp lock T:", e," statue:", lock);
  print_control.temperature_lock(e, lock);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode get_temperature_lock(event_param_t& event) {
  uint8_t key = event.data[0];
  uint8_t e = MODULE_INDEX(key);
  uint8_t lock = print_control.temperature_lock(e);
  uint8_t index = 0;
  SERIAL_ECHOLNPAIR("SC get temp lock T:", e," statue:", lock);
  event.data[index++] = E_SUCCESS;
  event.data[index++] = key;
  event.data[index++] = 1;  //  extruder count
  event.data[index++] = lock;
  event.length = index;
  return send_event(event);
}

static ErrCode get_fdm_enable(event_param_t& event) {
  uint8_t index = 0;
  event.data[index++] = E_SUCCESS;
  event.data[index++] = fdm_head.is_duplication_enabled(0);
  event.data[index++] = fdm_head.is_duplication_enabled(1);
  LOG_V("SC get fdm enable: T0-%d T1-%d\n", event.data[1], event.data[2]);
  event.length = index;
  return send_event(event);
}

static ErrCode set_noise_mode(event_param_t& event) {
  LOG_V("SC set noise mode %d\n", event.data[0]);
  if (print_control.set_noise_mode(print_noise_mode_e(event.data[0]))) {
    event.data[0] = E_SUCCESS;
  }
  else {
    event.data[0] = E_PARAM;
  }
  event.length = 1;
  return send_event(event);
}

static ErrCode get_noise_mode(event_param_t& event) {
  print_noise_mode_e pnm = print_control.get_noise_mode();
  LOG_V("SC get noise mode %d\n", pnm);
  event.data[0] = E_SUCCESS;
  event.data[1] = (uint8_t)pnm;
  event.length = 2;
  return send_event(event);
}

static ErrCode get_work_feedrate(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  uint16_t *fr = (uint16_t *)&event.data[1];
  *fr = motion_control.get_feedrate();
  event.length = 3;
  LOG_V("SC get feedrate:%d\n", *fr);
  return send_event(event);
}

static ErrCode set_work_feedrate_percentage(event_param_t& event) {
  feedrate_percentage_t *info = (feedrate_percentage_t *)event.data;
  SERIAL_ECHOLNPAIR("SC set feedrate percentage:", info->percentage);
  print_control.set_feedrate_percentage(info->percentage);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode get_work_feedrate_percentage(event_param_t& event) {
  uint8_t key = event.data[0];
  feedrate_percentage_t *info = (feedrate_percentage_t *)(event.data + 1);
  info->key = key;
  info->e_count = 1;
  info->percentage = print_control.get_feedrate_percentage();
  SERIAL_ECHOLNPAIR("SC get feedrate percentage:", info->percentage);
  event.data[0] = E_SUCCESS;
  event.length = 1 + sizeof(feedrate_percentage_t);
  return send_event(event);
}

static ErrCode set_work_flow_percentage(event_param_t& event) {
  flow_percentage_t *info = (flow_percentage_t *)event.data;
  uint8_t e = MODULE_INDEX(info->key);
  SERIAL_ECHOLNPAIR("SC set T:", e, " flow percentage:", info->percentage);
  print_control.set_work_flow_percentage(e, info->percentage);
  event.data[0] = E_SUCCESS;
  event.length = 1;
  return send_event(event);
}

static ErrCode get_work_flow_percentage(event_param_t& event) {
  uint8_t key = event.data[0];
  uint8_t e = MODULE_INDEX(key);
  flow_percentage_t *info = (flow_percentage_t *)(event.data + 1);
  info->key = key;
  info->e_index = 1;  //  extruder count
  info->percentage = print_control.get_work_flow_percentage(e);
  SERIAL_ECHOLNPAIR("SC get T:", e, " flow percentage:", info->percentage);
  event.data[0] = E_SUCCESS;
  event.length = 1 + sizeof(flow_percentage_t);
  return send_event(event);
}

static ErrCode subscribe_flow_percentage(event_param_t& event) {
  HOTEND_LOOP() {
    flow_percentage_t *info = (flow_percentage_t *)(event.data + 1);
    info->key = fdm_head.get_key(e);
    info->e_index = 1;  //  extruder count
    info->percentage = print_control.get_work_flow_percentage(e);
    LOG_V("report T:%d flow percentage:%d\n", e , info->percentage);
    event.data[0] = E_SUCCESS;
    event.length = 1 + sizeof(flow_percentage_t);
    send_event(event);
  }
  return E_SUCCESS;
}

static ErrCode subscribe_work_feedrate_percentage(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  event.data[1] = 1;
  int16_t * percentage = (int16_t *)&event.data[2];
  *percentage = print_control.get_feedrate_percentage();
  LOG_V("SC get feedrate percentage:%d\n", *percentage);
  event.length = 4;
  return send_event(event);
}

static ErrCode subscribe_work_time(event_param_t& event) {
  event.data[0] = E_SUCCESS;
  uint32_t * time = (uint32_t *)&event.data[1];
  *time = print_control.get_work_time() / 1000;
  LOG_V("SC get : work time %d S\n", *time);
  event.length = 5;
  return send_event(event);
}

event_cb_info_t printer_cb_info[PRINTER_ID_CB_COUNT] = {
  {PRINTER_ID_REQ_FILE_INFO       , EVENT_CB_DIRECT_RUN, request_file_info},
  {PRINTER_ID_REQ_GCODE           , EVENT_CB_TASK_RUN,   gcode_pack_deal},
  {PRINTER_ID_START_WORK          , EVENT_CB_TASK_RUN,   request_start_work},
  {PRINTER_ID_PAUSE_WORK          , EVENT_CB_DIRECT_RUN,   request_pause_work},
  {PRINTER_ID_RESUME_WORK         , EVENT_CB_DIRECT_RUN,   request_resume_work},
  {PRINTER_ID_STOP_WORK           , EVENT_CB_DIRECT_RUN,   request_stop_work},
  {PRINTER_ID_REQ_PL_STATUS       , EVENT_CB_DIRECT_RUN, request_power_loss_status},
  {PRINTER_ID_PL_RESUME           , EVENT_CB_TASK_RUN,   request_power_loss_resume},
  {PRINTER_ID_CLEAN_PL_DATA       , EVENT_CB_TASK_RUN,   request_clear_power_loss},
  {PRINTER_ID_SET_MODE            , EVENT_CB_TASK_RUN,   set_printer_mode},
  {PRINTER_ID_REQ_AUTO_PARK_STATUS, EVENT_CB_DIRECT_RUN, request_auto_pack_status},
  {PRINTER_ID_SET_PRINT_OFFSET    , EVENT_CB_TASK_RUN,   set_print_offset},
  {PRINTER_ID_STOP_SINGLE_EXTRUDE , EVENT_CB_DIRECT_RUN,   request_stop_single_extrude_work},
  {PRINTER_ID_SET_WORK_PERCENTAGE , EVENT_CB_DIRECT_RUN,   set_work_feedrate_percentage},
  {PRINTER_ID_GET_WORK_PERCENTAGE , EVENT_CB_DIRECT_RUN,   get_work_feedrate_percentage},
  {PRINTER_ID_SET_FLOW_PERCENTAGE , EVENT_CB_DIRECT_RUN, set_work_flow_percentage},
  {PRINTER_ID_GET_FLOW_PERCENTAGE , EVENT_CB_DIRECT_RUN, get_work_flow_percentage},
  {PRINTER_ID_SET_TEMPERATURE_LOCK    , EVENT_CB_DIRECT_RUN, set_temperature_lock},
  {PRINTER_ID_GET_TEMPERATURE_LOCK    , EVENT_CB_DIRECT_RUN, get_temperature_lock},
  {PRINTER_ID_GET_FDM_ENABLE          , EVENT_CB_DIRECT_RUN, get_fdm_enable},
  {PRINTER_ID_SET_NOISE_MODE          , EVENT_CB_DIRECT_RUN, set_noise_mode},
  {PRINTER_ID_GET_NOISE_MODE          , EVENT_CB_DIRECT_RUN, get_noise_mode},
  {PRINTER_ID_REQ_LINE                , EVENT_CB_DIRECT_RUN, request_cur_line},
  {PRINTER_ID_SUBSCRIBE_PRINT_MODE    , EVENT_CB_DIRECT_RUN, subscribe_print_mode},
  {PRINTER_ID_GET_WORK_FEEDRATE       , EVENT_CB_DIRECT_RUN, get_work_feedrate},
  {PRINTER_ID_SUBSCRIBE_FLOW_PERCENTAGE    , EVENT_CB_DIRECT_RUN, subscribe_flow_percentage},
  {PRINTER_ID_SUBSCRIBE_WORK_PERCENTAGE    , EVENT_CB_DIRECT_RUN, subscribe_work_feedrate_percentage},
  {PRINTER_ID_SUBSCRIBE_WORK_TIME    , EVENT_CB_DIRECT_RUN, subscribe_work_time},
};

static void req_gcode_pack() {
  batch_gcode_req_info_t info;
  uint16_t free_buf = print_control.get_buf_free();
  // SERIAL_ECHOLNPAIR("gcode buf free:", free_buf);
  if (free_buf >= GCODE_MAX_PACK_SIZE) {
    info.line_number = print_control.next_req_line();
    info.buf_max_size = GCODE_MAX_PACK_SIZE;
    // send_event(print_source, source_recever_id, SACP_ATTR_REQ,
    send_event(rep_gcode_source, rep_gcode_recever_id, SACP_ATTR_REQ,
        COMMAND_SET_PRINTER, PRINTER_ID_REQ_GCODE, (uint8_t *)&info, sizeof(info));
    gcode_req_status = GCODE_PACK_REQ_WAIT_RECV;
    gcode_req_timeout = millis() + (GCODE_REQ_TIMEOUT_MS<<gcode_req_timeout_times) + gcode_req_base_wait_ms;
    LOG_V("gcode requst start line:%u ,size:%u, timeout:%d ms, try: %d count\n",
          info.line_number,
          info.buf_max_size,
          (GCODE_REQ_TIMEOUT_MS<<gcode_req_timeout_times) + gcode_req_base_wait_ms,
          gcode_req_timeout_times);
  } else {
    gcode_req_status = GCODE_PACK_REQ_WAIT_CACHE;
  }
}

static void gcode_req_timeout_deal() {
  if (gcode_req_timeout < millis()) {
    extern uint32_t statistics_gcode_timeout_cnt;
    statistics_gcode_timeout_cnt++;
    LOG_E("requst gcode pack timeout!\n");
    req_gcode_pack();
    gcode_req_timeout_times++;
    if (gcode_req_timeout_times > GCODE_TIMEOUT_MAX_CNT) {
      print_control.error_and_stop();
    }
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
    if (E_SUCCESS != system_service.set_status(SYSTEM_STATUE_STOPPING, SYSTEM_STATUE_SCOURCE_DONE)) {
      LOG_E("can NOT set to SYSTEM_STATUE_STOPPING\r\n");
      system_service.return_to_idle();
    }
  }
}

void pausing_status_deal() {
  ErrCode result = print_control.pause();
  switch (system_service.get_source()) {
    case SYSTEM_STATUE_SCOURCE_M600:
      report_status_info(STATUS_PAUSE_BE_GCODE);
      SERIAL_ECHOLNPAIR("M600 pause done");
      break;
    case SYSTEM_STATUE_SCOURCE_FILAMENT:
      report_status_info(STATUS_PAUSE_BE_FILAMENT);
      SERIAL_ECHOLNPAIR("flilament puase done");
      break;
    case SYSTEM_STATUE_SCOURCE_GCODE:
      report_status_info(STATUS_PAUSE_BE_GCODE);
      SERIAL_ECHOLNPAIR("gcode puase done");
      break;
    case SYSTEM_STATUE_SCOURCE_TOOL_CHANGE:
      SERIAL_ECHOLNPAIR("change tool head continue");
      power_loss.change_head();
      if (print_control.resume() != E_SUCCESS) {
        report_status_info(STATUS_PAUSE_BE_FILAMENT);
        SERIAL_ECHOLNPAIR("flilament puase done");
      } else {
        req_gcode_pack();
      }
      break;
    case SYSTEM_STATUE_SCOURCE_STOP_EXTRUDE:
      SERIAL_ECHOLNPAIR("stop single extrude done and continue");
      result = print_control.resume();
      if (result == E_SUCCESS) {
        send_event(print_source, source_recever_id, SACP_ATTR_ACK,
                    COMMAND_SET_PRINTER, PRINTER_ID_STOP_SINGLE_EXTRUDE, &result, 1, source_sequence);
        req_gcode_pack();
      } else {
        result = E_SUCCESS;
        send_event(print_source, source_recever_id, SACP_ATTR_ACK,
                    COMMAND_SET_PRINTER, PRINTER_ID_STOP_SINGLE_EXTRUDE, &result, 1, source_sequence);
       report_status_info(STATUS_PAUSE_BE_FILAMENT);
      }
      break;
    case SYSTEM_STATUE_SCOURCE_EXCEPTION:
      report_status_info(STATUS_PAUSE_BE_EXCEPTION);
      fdm_head.set_temperature(0,0, false);
      fdm_head.set_temperature(1,0, false);
      bed_control.set_temperature(0, false);
      SERIAL_ECHOLNPAIR("exception puase done");
      break;
    default:
      send_event(print_source, source_recever_id, SACP_ATTR_ACK,
                  COMMAND_SET_PRINTER, PRINTER_ID_PAUSE_WORK, &result, 1, source_sequence);
      SERIAL_ECHOLNPAIR("system puase done:", result);
      break;
  }
}

void printer_event_init(void) {
  filament_sensor.init();
  power_loss.init();
}

void printing_status_deal() {

  start_pause_record = false;
  pause_hotend_tmp_down = false;

  switch (gcode_req_status) {
    case GCODE_PACK_REQ_WAIT_CACHE:
      req_gcode_pack();
      break;
    case GCODE_PACK_REQ_WAIT_RECV:
      gcode_req_timeout_deal();
      break;
    case GCODE_PACK_REQ_DONE:
      wait_print_end();
      break;
    default:
      break;
  }
}

void paused_status_deal() {

  if (pause_hotend_tmp_down)
    return;

  if (!start_pause_record) {
    start_pause_time_ms = millis();
    start_pause_record = true;
  }

  if (ELAPSED(millis(), start_pause_time_ms + (2 * 60 * 1000))) {
    HOTEND_LOOP() {
      if (fdm_head.extraduer_enable(e)) {
        thermalManager.setTargetHotend(0, e);
      }
    }
    LOG_I("Hotend thermal set to 0 as paused for a long time\r\n");
    pause_hotend_tmp_down = true;
  }

}

void resuming_status_deal() {
  uint8_t data[7];
  batch_gcode_req_info_t *info = (batch_gcode_req_info_t *)(data + 1);
  ErrCode result = print_control.resume();
  if (system_service.get_status() == SYSTEM_STATUE_STOPPING) {
    LOG_E("system status is stopping resunimg failed\n");
    return;
  }
  SERIAL_ECHOLNPAIR("resume work success, ret:", result);
  data[0] = result;
  info->buf_max_size = GCODE_MAX_PACK_SIZE;
  info->line_number = print_control.next_req_line();
  send_event(print_source, source_recever_id, SACP_ATTR_ACK,
    COMMAND_SET_PRINTER, PRINTER_ID_RESUME_WORK, data, 7, source_sequence);
  if (result == E_SUCCESS) {
    gcode_req_base_wait_ms = (info->line_number / 10000) * 100;
    NOLESS(gcode_req_base_wait_ms, 2000U);
    NOMORE(gcode_req_base_wait_ms, 5000U);
    req_gcode_pack();
  } else {
    report_status_info(STATUS_PAUSE_BE_FILAMENT);
    SERIAL_ECHOLNPAIR("resume be flilament pause");
  }
}

void stopping_status_deal() {
  ErrCode result = E_SUCCESS;
  SERIAL_ECHOLNPAIR("stop working...");
  result = print_control.stop();

  HOTEND_LOOP() {
    fdm_head.set_duplication_enabled(e, true);
    print_control.temperature_lock(e, false);
  }
  print_control.set_feedrate_percentage(100);
  if (system_service.get_source() == SYSTEM_STATUE_SCOURCE_SACP) {
    send_event(print_source, source_recever_id, SACP_ATTR_ACK,
      COMMAND_SET_PRINTER, PRINTER_ID_STOP_WORK, &result, 1, source_sequence);
  } else {
    report_status_info(STATUS_PRINT_DONE);
  }
  SERIAL_ECHOLNPAIR("stop success");
}

void printer_event_loop(void) {
  switch (system_service.get_status()) {
    case SYSTEM_STATUE_PRINTING:
      printing_status_deal();
      break;
    case SYSTEM_STATUE_PAUSED:
      paused_status_deal();
      break;
    case SYSTEM_STATUE_PAUSING:
      pausing_status_deal();
      break;
    case SYSTEM_STATUE_RESUMING:
      resuming_status_deal();
      break;
    case SYSTEM_STATUE_STOPPING:
      stopping_status_deal();
      break;
    default:
      break;
  }
}
