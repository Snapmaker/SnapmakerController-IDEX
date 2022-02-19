#include "bed_control.h"
#include "src/module//temperature.h"

BedControl bed_control;

ErrCode BedControl::set_temperature(uint16_t temperature) {
  thermalManager.setTargetBed(temperature);
  return E_SUCCESS;
}

ErrCode BedControl::get_info(bed_control_info_t &info) {
  info.key = MODULE_KEY(MODULE_BED, 0);
  info.bed_count = 1;
  info.zone_index = 0;
  info.cur_temp = FLOAT_TO_INT(thermalManager.degBed());
  info.target_temp = (int16_t)thermalManager.degTargetBed();
  return E_SUCCESS;
}

ErrCode BedControl::get_module_info(module_info_t &info) {
  info.key = MODULE_KEY(MODULE_BED, 0);
  info.module_id = MODULE_BED;
  info.module_index = 0;
  info.module_state = 0;
  info.sn = 0;
  info.version_length = MODULE_VER_LEN;
  info.version[0] = 0;
  return E_SUCCESS;
}
