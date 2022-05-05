#ifndef ENCLOSURE_H
#define ENCLOSURE_H
#include "../J1/common_type.h"
#include "module_base.h"

typedef enum {
  ENCLOSURE_FAN_OFF,
  ENCLOSURE_FAN_ON,
} enclosure_fan_status_e;

class Enclosure {
  public:
    void get_module_info(module_info_t &info);
    uint8_t get_key();
    uint8_t get_light_power();
    void set_light_power(uint8_t power);
    uint8_t get_fan_power();
    void set_fan_power(uint8_t power);
  private:
    enclosure_fan_status_e fan_status = ENCLOSURE_FAN_OFF;
};

extern Enclosure enclosure;

#endif
