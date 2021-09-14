#include <stdint.h>
#include "tmc_regs.h"

typedef struct {
  uint8_t RegAddress;
  uint32_t Value;
}tmc_configure_t;

enum SG_Mode{
  SG_MODE_NONE = 0,
  SG_MODE_NORMAL = 1,
  SG_MODE_X0_HOME,
  SG_MODE_X1_HOME,
  SG_MODE_Y_HOME,
  SG_MODE_Z_HOME,
  SG_MODE_CALIBRATION
};


class TMCDriver {
public:
  static void init();
  static void configure(uint8_t index, tmc_configure_t *configure);
  static void comm_test(uint8_t rw, uint8_t reg_address, uint32_t *value);
  static void select(uint8_t index);
  static void enable_stall_guard();
  static void disable_stall_guard();
  static void trigger_stall_guard();
  static void stall_guard_check_loop();
  static void set_stall_guard_mode(SG_Mode mode);
  static void configure_axis();
  static void configure_for_print();
  static void configure_for_xy_calibration(uint8_t sg_x, uint8_t sg_y);
  static void configure_for_platform_calibration(uint8_t sg_z);
  static void configure_for_idle();
  static void configure_for_aixs_test(uint8_t index, uint8_t sg_value);
  static void stall_guard_deinit(uint8_t index);
  static void set_sta_guar_level(uint8_t level);
  static void update_stepper_tick(uint32_t tick);
  static void set_stepper_tick_threshold(uint32_t tick);
  static bool check_stepper_tick();
  static void set_reg_value(uint8_t index, uint8_t reg, uint32_t value);
  static void enable_stall_guard_interrupt();
  static void disable_stall_guard_interrupt();
private:
  static void stall_guard_init(uint8_t index, uint8_t sg_value);
  static void cool_step_init(uint8_t index, bool enable, uint8_t low_limit, uint8_t high_limit);
  static uint32_t read_reg(uint8_t reg_address);
  static void write_reg(uint8_t reg_address, uint32_t value);
  static void cacul_crc(uint8_t* datagram, uint8_t datagramLength);

private:
  static bool stall_guard_dectected;
  static SG_Mode stall_guard_mode;
  static uint8_t stall_trigged_mode;
  static uint8_t select_index;
  static uint8_t sel_table[8][3];
  static uint8_t stall_guard_level_table[][4];
  static uint8_t slave_address[6];
  static uint8_t recv_buff[16];
  static uint8_t send_buff[8];
  static tmc_configure_t local_configures[];
  static uint8_t print_stall_guard_level;
  static uint32_t stepper_isr_tick_check_threshold;
  static uint32_t stepper_isr_tick;
};

extern TMCDriver tmc_driver;
