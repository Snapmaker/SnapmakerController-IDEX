#ifndef POWER_LOSS_H
#define POWER_LOSS_H
#include "../J1/common_type.h"
#include "src/core/types.h"

#define POWER_LOSS_TRIGGER_STATUS 0

#define GCODE_MD5_LENGTH 64
#define GCODE_FILE_NAME_SIZE 128

#define PL_NO_DATE      0xFF
#define PL_WORKING      0x55
#define PL_WAIT_RESUME  0xAA

#define Z_DOWN_SAFE_DISTANCE 2  // mm
#define PRINT_RETRACK_DISTANCE 1  // mm
#define PRINT_RETRACK_SPEED 10*60  // mm/min
#define PRINT_TRAVEL_FEADRATE 6000

#define EXTRUDE_X_MOVE_DISTANCE 8  // mm
#define EXTRUDE_E_DISTANCE 30  // mm

typedef enum {
  POWER_LOSS_IDLE,
  POWER_LOSS_STOP_MOVE,
  POWER_LOSS_DONE,
} power_loss_status_e;

# pragma pack(4)
typedef struct {
  uint16_t state;
  uint16_t nozzle_temp[EXTRUDERS];
  uint32_t file_position;
  float print_feadrate;
  float travel_feadrate;
  uint8_t fan[EXTRUDERS][2];
  xyze_pos_t position;
  uint32_t accumulator;
  uint16_t bed_temp;
  uint8_t extruder_dual_enable[EXTRUDERS];
  uint8_t extruder_temperature_lock[EXTRUDERS];
  uint8_t active_extruder;
  uint8_t axis_relative;
  uint8_t gcode_file_name_len = 0;
  uint8_t gcode_file_name[GCODE_FILE_NAME_SIZE];
  uint8_t gcode_file_md5_len = 0;
  uint8_t gcode_file_md5[GCODE_MD5_LENGTH];
  uint8_t dual_x_carriage_mode;
  float duplicate_extruder_x_offset;
  uint8_t print_mode;
  float nozzle_distance;
  uint8_t light_staus;
  uint32_t check_num;
} power_loss_t;

#pragma pack()

class PowerLoss {
  public:
    void init();
    ErrCode is_power_loss_data();
    ErrCode power_loss_resume();
    bool change_head();
    void extrude_before_resume();
    void stash_print_env();
    void resume_print_env();
    ErrCode   set_file_name(uint8_t *name, uint8_t len);
    ErrCode   set_file_md5(uint8_t *md5, uint8_t len);
    uint8_t * get_file_name(uint8_t &len);
    uint8_t * get_file_md5(uint8_t &len);
    void clear();
    void show_power_loss_info();
    void check();
    bool is_power_pin_trigger();
  private:
    void write_flash(void);
  public:
    uint32_t cur_line = 0;
    uint32_t line_number_sum = 0;
    uint32_t next_req = 0;
    bool power_loss_en = true;
    power_loss_status_e power_loss_status = POWER_LOSS_IDLE;
    bool is_inited = false;
    power_loss_t stash_data;
};

extern PowerLoss power_loss;

#endif