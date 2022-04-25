#ifndef PRINT_CONTROL_H
#define PRINT_CONTROL_H
#include "../J1/common_type.h"
#include "src/core/types.h"

typedef enum {
  PRINT_RESULT_GCODE_RECV_DONE_E = 201,
  PRINT_RESULT_START_ERR_E = 202,
  PRINT_RESULT_NO_HOME_E = 203,
  PRINT_RESULT_INV_FILE_INFO_E = 204,
  PRINT_RESULT_PAUSE_ERR_E = 205,
  PRINT_RESULT_RESUME_ERR_E = 206,
  PRINT_RESULT_NO_PL_DATA_E = 207,
  PRINT_RESULT_PL_RESUME_ERR_E = 208,
  PRINT_RESULT_SER_MODE_ERR_E = 209,
  PRINT_RESULT_NO_FILE_INFO_E = 210,
} print_result_e;

typedef enum : uint8_t{
  PRINT_BACKUP_MODE,
  PRINT_AUTO_PARK_MODE,
  PRINT_DUPLICATION_MODE,
  PRINT_MIRRORED_MODE
} print_mode_e;

typedef struct {
  bool is_err;
  uint32_t err_line;
} print_err_info_t;

class PrintControl {
  public:
    void init();
    uint32_t get_buf_used();
    uint32_t get_buf_free();
    ErrCode push_gcode(uint32_t start_line, uint32_t end_line, uint8_t *data, uint16_t size);
    uint32_t get_cur_line();
    uint32_t next_req_line();
    bool buffer_is_empty();
    ErrCode start();
    ErrCode pause();
    ErrCode resume();
    ErrCode stop();
    ErrCode set_mode(print_mode_e mode);
    void set_feedrate_percentage(int16_t percentage);
    int16_t get_feedrate_percentage();
    void set_work_flow_percentage(uint8_t e, int16_t percentage);
    int16_t get_work_flow_percentage(uint8_t e);
    bool is_backup_mode();
    bool filament_check();
    bool get_commands(uint8_t *cmd, uint32_t &line, uint16_t max_len);
    void commands_lock() {commands_lock_ = true;}
    void commands_unlock() {commands_lock_ = false;}
    void loop();
    bool temperature_lock(uint8_t e) {
      return temperature_lock_status[e];
    }
    void temperature_lock(uint8_t e, bool enable) {temperature_lock_status[e] = enable;}
    void error_and_stop();

  public:
    print_mode_e mode_ = PRINT_BACKUP_MODE;
    bool temperature_lock_status[EXTRUDERS] = {false, false};
    bool commands_lock_ = false;
    print_err_info_t print_err_info = {0};
};

extern PrintControl print_control;

#endif
