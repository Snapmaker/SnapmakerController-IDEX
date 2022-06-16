#include "fdm.h"
#include "../../Marlin/src/gcode/gcode.h"
#include "../../Marlin/src/module/tool_change.h"
#include "../../Marlin/src/module/temperature.h"
#include "../../Marlin/src/module/motion.h"
#include "../module/filament_sensor.h"
#include "motion_control.h"
#include "exception.h"

FDM_Head fdm_head;

enum {
  FAN_TYPE_COLD_MODULE = 0,
  FAN_TYPE_COLD_EXTRUDER = 2,
};

enum {
  HEAD_STATUS_OK = 0,
  HEAD_STATUS_UPGRADE,
  HEAD_STATUS_INVALID,
  HEAD_STATUS_UPGRADE_FAILED,
};


const nozzle_type_t nozzle_type[] = {
  {BRASS_PT100_E, 0.4, MV_TO_ADC_VAL(0  ), MV_TO_ADC_VAL(400)},
  {BRASS_PT100_E, 0.2, MV_TO_ADC_VAL(400), MV_TO_ADC_VAL(1200)},
  {BRASS_PT100_E, 0.6, MV_TO_ADC_VAL(1200), MV_TO_ADC_VAL(1600)},
  {BRASS_PT100_E, 0.8, MV_TO_ADC_VAL(1600), MV_TO_ADC_VAL(2200)},
  {HARDENING_STEEL_PT100, 0.6, MV_TO_ADC_VAL(2200), MV_TO_ADC_VAL(3000)}
};


ErrCode FDM_Head::set_temperature(uint8_t e, uint16_t temperature) {
  if (!exception_server.is_allow_heat_nozzle()) {
    return E_SYSTEM_EXCEPTION;
  }
  thermalManager.setTargetHotend(temperature, e);
  return E_SUCCESS;
}

float FDM_Head::get_temperature(uint8_t e) {
  return E_SUCCESS;
}

ErrCode FDM_Head::set_work_speed(float speed) {
  return E_SUCCESS;
}

ErrCode FDM_Head::change_filamenter(uint8_t e, float feedrate, filamenter_change_status_e status) {
  LOG_I("change head[%d] filament status:%d\n", e, status);
  change_filamenter_status[e] = status;
  motion_control.quickstop();
  if (is_change_filamenter()) {
    // The extruded length is arbitrary, and this is just to rotate the e axis
    motion_control.extrude_e(50, feedrate);
  }
  return E_SUCCESS;
}

ErrCode FDM_Head::change_tool(uint8_t e) {
  if (is_change_filamenter()) {
    return E_COMMON_ERROR;
  }
  gcode.reset_stepper_timeout();
  tool_change(e, false);
  return E_SUCCESS;
}

ErrCode FDM_Head::set_fan_speed(uint8_t e, uint8_t fan_index, uint8_t speed) {
  if (e == 0) {
    if (fan_index == 0) {
      thermalManager.set_fan_speed(0, speed);
    } else {
      // thermalManager.set_fan_speed(2, speed);
    }
  } else {
    if (fan_index == 0) {
      thermalManager.set_fan_speed(1, speed);
    } else {
      // thermalManager.set_fan_speed(3, speed);
    }
  }
  return E_SUCCESS;
}

ErrCode FDM_Head::get_fan_speed(uint8_t e, uint8_t fan_index, uint8_t &speed) {
  speed = 0;
  if (e == 0) {
    if (fan_index == 0) {
      speed = thermalManager.fanSpeed(0);
    } else {
      // thermalManager.fanSpeed(2);
    }
  } else {
    if (fan_index == 0) {
      speed = thermalManager.fanSpeed(1);
    } else {
      // thermalManager.fanSpeed(3);
    }
  }
  return E_SUCCESS;
}

ErrCode FDM_Head::set_extruder_diff(uint8_t axis, float diff) {
  return E_SUCCESS;
}

ErrCode FDM_Head::get_fdm_info(uint8_t e, FDM_info *fdm) {
  fdm->key = MODULE_KEY(MODULE_PRINT, e);
  fdm->head_status = HEAD_STATUS_OK;
  fdm->head_active = e == active_extruder;
  fdm->extruder_count = 1;
  get_extruder_info(e, &fdm->extruder_info);
  
  uint8_t index=0, speed=0;
  fdm->fan_count = 2;
  get_fan_speed(e, index, speed);
  fdm->extruder_fan[index].index = index;
  fdm->extruder_fan[index].type = FAN_TYPE_COLD_EXTRUDER;
  fdm->extruder_fan[index].speed = speed;

  index = 1;
  get_fan_speed(e, index, speed);
  fdm->extruder_fan[index].index = index;
  fdm->extruder_fan[index].type = FAN_TYPE_COLD_MODULE;
  fdm->extruder_fan[index].speed = speed;
  
  return E_SUCCESS;
}

ErrCode FDM_Head::get_extruder_info(uint8_t e, extruder_info_t *info) {
  float diameter;
  nozzle_texture_type_e type;
  get_nozzle_type(e, &type, &diameter);
  info->index = 0;
  info->is_available = 1;
  info->type = type;
  info->filament_status = filament_sensor.is_trigger(e);
  info->filament_enable_status = filament_sensor.is_enable(e);
  info->diameter = FLOAT_TO_INT(diameter);
  info->cur_temp = FLOAT_TO_INT(thermalManager.degHotend(e));
  info->target_temp = FLOAT_TO_INT(thermalManager.degTargetHotend(e));
  if (type == UNKNOWN_NOZZLE_TYPE && (info->cur_temp >= FLOAT_TO_INT(350))) {
    exception_server.trigger_exception((exception_type_e)(EXCEPTION_TYPE_LEFT_NOZZLE_LOSS + e));
  } else {
    exception_server.clean_exception((exception_type_e)(EXCEPTION_TYPE_LEFT_NOZZLE_LOSS + e));
  }
  return E_SUCCESS;
}

uint8_t FDM_Head::get_key(uint8_t e) {
  return MODULE_KEY(MODULE_PRINT, e);
}

ErrCode FDM_Head::get_module_info(uint8_t e, module_info_t &info) {
  info.key = get_key(e);
  info.module_id = MODULE_PRINT;
  info.module_index = e;
  info.module_state = 0;
  info.sn = 0;
  info.version_length = MODULE_VER_LEN;
  info.version[0] = 0;
  return E_SUCCESS;
}

bool FDM_Head::is_duplicating() {
  return idex_is_duplicating();
}

uint16_t FDM_Head::get_nozzle_type(uint8_t e, nozzle_texture_type_e *texture, float *caliber) {
  uint16_t val = 0;
  if (e == 0) {
    pinMode(HEAD0_ID_PIN, INPUT_ANALOG);
    val = analogRead(HEAD0_ID_PIN);
  } else {
    pinMode(HEAD1_ID_PIN, INPUT_ANALOG);
    val = analogRead(HEAD1_ID_PIN);
  }
  uint8_t type_count = ARRAY_SIZE(nozzle_type);
  uint8_t i = 0;
  for (i = 0; i < type_count; i++) {
    if (nozzle_type[i].adc_min <= val && nozzle_type[i].adc_max >= val) {
      break;
    }
  }
  if (i == type_count) {
    *texture = UNKNOWN_NOZZLE_TYPE;
    *caliber = nozzle_type[0].caliber;
  } else {
    *texture = nozzle_type[i].texture;
    *caliber = nozzle_type[i].caliber;
  }
  return val;
}
