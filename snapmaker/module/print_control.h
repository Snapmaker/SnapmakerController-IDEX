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

# pragma pack(1)
typedef struct {
  uint32_t file_position;
  uint16_t nozzle_temp[EXTRUDERS];
  uint16_t bed_temp;
  uint8_t fan[EXTRUDERS][2];
  xyze_pos_t position;
  uint8_t gcode_file_name_len = 0;
  uint8_t gcode_file_name[GCODE_FILE_NAME_SIZE];
  uint8_t gcode_file_md5_len = 0;
  uint8_t gcode_file_md5[GCODE_MD5_LENGTH];
  uint8_t dual_x_carriage_mode;
} power_loss_t;

#pragma pack()

class PrintControl {
  public:
    void init();
    uint32_t get_buf_used();
    uint32_t get_buf_free();
    ErrCode push_gcode(uint32_t start_line, uint32_t end_line, uint8_t *data, uint16_t size);
    uint32_t get_cur_line();
    void set_cur_line(uint32_t line) {cur_line = line;}
    uint32_t next_req_line();
    bool buffer_is_empty();
    ErrCode start();
    ErrCode pause();
    ErrCode resume();
    ErrCode stop();
    ErrCode power_loss_status();
    ErrCode power_loss_resume();
    uint8_t * get_file_name(uint8_t &len);
    uint8_t * get_file_md5(uint8_t &len);
    ErrCode set_file_name(uint8_t *name, uint8_t len);
    ErrCode set_file_md5(uint8_t *md5, uint8_t len);
    ErrCode set_mode(print_mode_e mode);
    bool is_auto_pack_mode();
    bool get_commands(uint8_t *cmd, uint32_t &line, uint16_t max_len);
    void loop();
  private:
    void stash_print_env();
    void resume_print_env();
  private:
    uint32_t cur_line = 0;
    uint32_t line_number_sum = 0;
    uint32_t next_req = 0;
};

extern PrintControl print_control;

#endif
