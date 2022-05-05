#include "bed_control.h"
#include "src/module//temperature.h"

BedControl bed_control;

bool BedControl::self_check() {
  uint32_t delay_time = 0;
  is_error = false;
  SET_INPUT(HEATER_BED_BACK_PIN);
  OUT_WRITE(HEATER_BED_PIN, HIGH);
  delay_time = millis() + 20;
  while(delay_time > millis());
  if (READ(HEATER_BED_BACK_PIN) == HIGH) {
    is_error = true;
    SERIAL_ECHOLN("BED open self check failed");
  }

  OUT_WRITE(HEATER_BED_PIN, LOW);
  delay_time = millis() + 20;
  while(delay_time > millis());
  if (READ(HEATER_BED_BACK_PIN) == LOW) {
    SERIAL_ECHOLN("BED close self check failed");
    is_error = true;
  }
  if (is_error) {
    OUT_WRITE(HEATER_BED_PWR_PIN, LOW);
  } else {
    SERIAL_ECHOLN("BED self check success");
  }
  return is_error;
}

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
  info.module_id = J_BED_MODULE;
  info.module_index = 0;
  info.module_state = 0;
  info.sn = 0;
  info.version_length = MODULE_VER_LEN;
  info.version[0] = 0;
  return E_SUCCESS;
}
