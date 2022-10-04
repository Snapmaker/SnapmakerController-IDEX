#include "bed_control.h"
#include "src/module//temperature.h"
#include "exception.h"
#include "./power_loss.h"

BedControl bed_control;

bool BedControl::self_check() {
  uint32_t delay_time = 0;
  is_error = false;
  SET_INPUT(HEATER_BED_BACK_PIN);
  OUT_WRITE(HEATER_BED_PIN, HIGH);
  delay_time = millis() + 20;
  while (PENDING(millis(), delay_time));
  if (READ(HEATER_BED_BACK_PIN) == HIGH) {
    is_error = true;
    LOG_E("BED open self check failed\n");
  }

  OUT_WRITE(HEATER_BED_PIN, LOW);
  delay_time = millis() + 20;
  while (PENDING(millis(), delay_time));
  if (READ(HEATER_BED_BACK_PIN) == LOW) {
    LOG_E("BED close self check failed\n");
    is_error = true;
    exception_server.trigger_exception(EXCEPTION_TYPE_BED_SELF_CHECK);
  }
  if (is_error) {
    OUT_WRITE(HEATER_BED_PWR_PIN, LOW);
  } else {
    LOG_E("BED self check success\n");
  }
  return is_error;
}

ErrCode BedControl::set_temperature(uint16_t temperature, bool is_save) {
  if (temperature > 0 && !exception_server.is_allow_heat_bed()) {
    return E_SYSTEM_EXCEPTION;
  }
  if (is_save) {
    power_loss.stash_data.bed_temp = temperature;
  }
  thermalManager.setTargetBed(temperature);
  return E_SUCCESS;
}

ErrCode BedControl::get_info(bed_control_info_t &info) {
  float temperature = thermalManager.degBed();
  info.key = MODULE_KEY(MODULE_BED, 0);
  info.bed_count = 1;
  info.zone_index = 0;
  info.cur_temp = FLOAT_TO_INT(temperature);
  info.target_temp = (int16_t)thermalManager.degTargetBed();
  if (temperature >= 300) {
    exception_server.trigger_exception(EXCEPTION_TYPE_BED_NOT_FIND);
  } else {
    exception_server.clean_exception(EXCEPTION_TYPE_BED_NOT_FIND);
  }
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
