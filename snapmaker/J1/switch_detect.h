#include "stdint.h"

class SwitchDetect
{
public:
  void init();
  void check();
  void disable_all();
  void enable_filament();
  void disable_filament();
  void enable_probe();
  void disable_probe();
  void enable_power_lost();
  void disable_power_lost();
  void enable_stall_guard();
  void disable_stall_guard();
  void manual_trig_stop();
  void stall_guard_stop();
  void set_probe_detect_level(uint8_t Level);
  bool read_e0_probe_status();
  bool read_e1_probe_status();
private:
  void enable(uint8_t Item);
  void disable(uint8_t Item);
private:
  uint32_t enable_bits;
  uint32_t status_bits;
  uint8_t probe_detect_level = 0;
};

extern SwitchDetect switch_detect;
