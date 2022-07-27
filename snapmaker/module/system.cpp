#include "system.h"
#include "../../Marlin/src/inc/Version.h"
#include "../../Marlin/src/module/motion.h"
#include "../../Marlin/src/module/settings.h"
#include "power_loss.h"
SystemService system_service;

static const uint16_t hw_version_table[] = {
    MV_TO_ADC_VAL(0 ),
    MV_TO_ADC_VAL(450 ),
    MV_TO_ADC_VAL(700 ),
    MV_TO_ADC_VAL(1400),
    MV_TO_ADC_VAL(2000),
    MV_TO_ADC_VAL(2700),
    MV_TO_ADC_VAL(3200)
  };

void SystemService::get_coordinate_system_info(coordinate_system_t * info, bool is_logical) {
  xyze_pos_t position = current_position;
  float x0 = x_position();
  float x1 = x2_position();
  info->homed = homing_needed();
  info->is_origin_offset = true;
  info->coordinate_system_id = 0;
  info->is_origin_offset = true;

  if (is_logical) {
    position = current_position.asLogical();
    x0 = NATIVE_TO_LOGICAL(x0, X_AXIS);
    x1 = NATIVE_TO_LOGICAL(x1, X_AXIS);
  }
  info->coordinate_axis_count = AXIS_COUNT;
  info->coordinate_axis_info[0].axis = AXIS_X1;
  info->coordinate_axis_info[0].position = FLOAT_TO_INT(x0);

  info->coordinate_axis_info[1].axis = AXIS_X2;
  info->coordinate_axis_info[1].position = FLOAT_TO_INT(x1);

  info->coordinate_axis_info[2].axis = AXIS_Y1;
  info->coordinate_axis_info[2].position = FLOAT_TO_INT(position[Y_AXIS]);
  info->coordinate_axis_info[3].axis = AXIS_Z1;
  info->coordinate_axis_info[3].position = FLOAT_TO_INT(position[Z_AXIS]);

  // No other coordinate system is supported, so fake data is used to complete the protocol
  info->origin_offset_count = AXIS_COUNT;
  info->origin_offset_info[0].axis = AXIS_X1;
  info->origin_offset_info[0].position = 0;
  info->origin_offset_info[1].axis = AXIS_X2;
  info->origin_offset_info[1].position = 0;
  info->origin_offset_info[2].axis = AXIS_Y1;
  info->origin_offset_info[2].position = 0;
  info->origin_offset_info[3].axis = AXIS_Z1;
  info->origin_offset_info[3].position = 0;
}

uint8_t SystemService::get_hw_version(bool is_refresh) {
  if (hw_version == 0xff || is_refresh) {
    pinMode(HW_VERSION_PIN, INPUT_ANALOG);
    uint16_t val = analogRead(HW_VERSION_PIN);
    uint8_t ver_count = ARRAY_SIZE(hw_version_table);
    uint8_t i = 1;
    for (; i < ver_count; i++) {
      if (hw_version_table[i - 1] <= val && hw_version_table[i] > val) {
        break;
      }
    }
    hw_version = i;
  }
  return hw_version;
}

void SystemService::get_machine_info(machine_info_t *info) {
  char *ver = (char *)J1_BUILD_VERSION;
  info->Ji_num = 4; // J1
  info->hw_version = get_hw_version();
  info->sn = 0;
  info->version_length = strlen(ver) + 1;
  for (uint16_t i = 0; i < info->version_length; i++) {
    info->version[i] = ver[i];
  }
}

ErrCode SystemService::set_origin(coordinate_info_t axis) {
  return E_SUCCESS;
}

void SystemService::set_status(system_status_e status, system_status_source_e source) {
  switch (status) {
    case SYSTEM_STATUE_PAUSING:
    case SYSTEM_STATUE_STOPPING:
    case SYSTEM_STATUE_FINISHING:
      wait_for_heatup = false;
      break;
    default:
      break;
  }
  status_ = status;
  if (source != SYSTEM_STATUE_SCOURCE_NONE) {
    source_ = source;
  }
}

void SystemService::get_machine_size(machine_size_t *size) {
  size->size_count = AXIS_COUNT;
  size->size[0].axis = AXIS_X1;
  size->size[0].position = FLOAT_TO_INT(X_MAX_POS - X_MIN_POS);
  size->size[1].axis = AXIS_X2;
  size->size[1].position = FLOAT_TO_INT(X2_MAX_POS - X2_MIN_POS);
  size->size[2].axis = AXIS_Y1;
  size->size[2].position = FLOAT_TO_INT(Y_MAX_POS - Y_MIN_POS);
  size->size[3].axis = AXIS_Z1;
  size->size[3].position = FLOAT_TO_INT(Z_MAX_POS - Z_MIN_POS);

  size->home_offset_count = 3;
  size->home_offset[0].axis = AXIS_X1;
  size->home_offset[0].position = home_offset[X_AXIS];
  size->home_offset[1].axis = AXIS_Y1;
  size->home_offset[1].position = home_offset[Y_AXIS];
  size->home_offset[2].axis = AXIS_Z1;
  size->home_offset[2].position = home_offset[Z_AXIS];
}

void SystemService::factory_reset() {
  power_loss.clear();
  settings.reset();
  settings.save();
}
