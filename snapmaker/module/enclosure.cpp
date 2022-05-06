#include "enclosure.h"
#include "../../Marlin/src/feature/caselight.h"

Enclosure enclosure;

void Enclosure::get_module_info(module_info_t &info) {
  info.key = get_key();
  info.module_id = MODULE_ENCLOSURE;
  info.module_index = 0;
  info.module_state = 0;
  info.sn = 0;
  info.version_length = MODULE_VER_LEN;
  info.version[0] = 0;
}

uint8_t Enclosure::get_key() {
  return MODULE_KEY(MODULE_ENCLOSURE, 0);
}

uint8_t Enclosure::get_light_power() {
  return caselight.get_power();
}

void Enclosure::set_light_power(uint8_t power) {
  caselight.set_power(power);
}

uint8_t Enclosure::get_fan_power() {
  return fan_status;
}

void Enclosure::set_fan_power(uint8_t power) {
}
