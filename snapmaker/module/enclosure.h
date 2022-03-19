#ifndef ENCLOSURE_H
#define ENCLOSURE_H
#include "../J1/common_type.h"
#include "module_base.h"


class Enclosure {
  public:
    void get_module_info(module_info_t &info);
    uint8_t get_key();
    uint8_t get_light_power();
    void set_light_power(uint8_t power);
    uint8_t get_fan_power();
    void set_fan_power(uint8_t power);
  private:

};

extern Enclosure enclosure;

#endif
