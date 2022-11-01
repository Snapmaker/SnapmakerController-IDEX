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
  PRINT_FULL_MODE,
  PRINT_BACKUP_MODE,
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
    void clear_gcode_buf();
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
    print_mode_e get_mode() {return mode_;}
    ErrCode set_print_offset(float x, float y, float z);
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
    uint32_t get_work_time();
    void set_work_time(uint32_t time);
  private:
    void start_work_time();
    void pause_work_time();
    void resume_work_time();
    void stop_work_time();

  public:
    print_mode_e mode_ = PRINT_FULL_MODE;
    bool temperature_lock_status[EXTRUDERS] = {false, false};
    bool commands_lock_ = false;
    print_err_info_t print_err_info = {0};
    xyz_pos_t xyz_offset = {0, 0, 0};
    uint32_t work_time_ms = 0;
    bool req_clear_work_time = false;
    uint32_t work_start_time = 0;
    bool is_calibretion_mode = false;  // calibretion mode not save powerloss data
};

extern PrintControl print_control;

#endif
