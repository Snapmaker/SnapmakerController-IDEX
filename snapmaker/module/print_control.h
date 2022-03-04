#ifndef PRINT_CONTROL_H
#define PRINT_CONTROL_H
#include "../J1/common_type.h"
#include "src/core/types.h"
#define GCODE_MD5_LENGTH 64
#define GCODE_FILE_NAME_SIZE 128

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
  PRINT_FULL_CONTROL_MODE,
  PRINT_AUTO_PARK_MODE,
  PRINT_DUPLICATION_MODE,
  PRINT_MIRRORED_MODE
} print_mode_e;


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
    bool is_auto_pack_mode();
    bool get_commands(uint8_t *cmd, uint32_t &line, uint16_t max_len);
    void loop();
  private:

};

extern PrintControl print_control;

#endif
