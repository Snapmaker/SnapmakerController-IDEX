#ifndef POWER_LOSS_H
#define POWER_LOSS_H
#include "../J1/common_type.h"
#include "src/core/types.h"

#define POWER_LOSS_TRIGGER_STATUS 1

#define GCODE_MD5_LENGTH 64
#define GCODE_FILE_NAME_SIZE 128

#define PL_NO_DATE      0xFF
#define PL_WORKING      0x55
#define PL_WAIT_RESUME  0xAA

# pragma pack(4)
typedef struct {
  uint16_t state;
  uint16_t nozzle_temp[EXTRUDERS];
  uint32_t file_position;
  uint16_t bed_temp;
  float print_feadrate;
  float travel_feadrate;
  uint8_t fan[EXTRUDERS][2];
  xyze_pos_t position;
  uint32_t accumulator;
  uint8_t axis_relative;
  uint8_t gcode_file_name_len = 0;
  uint8_t gcode_file_name[GCODE_FILE_NAME_SIZE];
  uint8_t gcode_file_md5_len = 0;
  uint8_t gcode_file_md5[GCODE_MD5_LENGTH];
  uint8_t dual_x_carriage_mode;
  uint32_t check_num;
} power_loss_t;

#pragma pack()

class PowerLoss {
  public:
    void init();
    ErrCode power_loss_status();
    ErrCode power_loss_resume();
    void stash_print_env();
    void resume_print_env();
    ErrCode   set_file_name(uint8_t *name, uint8_t len);
    ErrCode   set_file_md5(uint8_t *md5, uint8_t len);
    uint8_t * get_file_name(uint8_t &len);
    uint8_t * get_file_md5(uint8_t &len);
    void clear();
    void show_power_loss_info();
    void check();
  private:
    void write_flash(void);
  public:
    uint32_t cur_line = 0;
    uint32_t line_number_sum = 0;
    uint32_t next_req = 0;
    bool power_loss_en = true;
    bool is_power_loss = false;
    power_loss_t stash_data;
};

extern PowerLoss power_loss;

#endif
