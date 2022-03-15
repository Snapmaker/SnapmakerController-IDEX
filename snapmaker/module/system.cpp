#include "system.h"
#include "../../Marlin/src/inc/Version.h"
#include "../../Marlin/src/module/motion.h"
SystemService system_service;

void SystemService::get_coordinate_system_info(coordinate_system_t * info) {
  float x0 = x_position();
  float x1 = x2_position();
  info->homed = homing_needed();
  info->is_origin_offset = true;
  info->coordinate_system_id = 0;
  info->is_origin_offset = true;

  info->coordinate_axis_count = AXIS_COUNT;
  info->coordinate_axis_info[0].axis = AXIS_X1;
  info->coordinate_axis_info[0].position = FLOAT_TO_INT(x0);

  info->coordinate_axis_info[1].axis = AXIS_X2;
  info->coordinate_axis_info[1].position = FLOAT_TO_INT(x1);

  info->coordinate_axis_info[2].axis = AXIS_Y1;
  info->coordinate_axis_info[2].position = FLOAT_TO_INT(current_position[Y_AXIS]);
  info->coordinate_axis_info[3].axis = AXIS_Z1;
  info->coordinate_axis_info[3].position = FLOAT_TO_INT(current_position[Z_AXIS]);

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

void SystemService::get_machine_info(machine_info_t *info) {
  char *ver = (char *)DETAILED_BUILD_VERSION;
  info->Ji_num = 4; // J1
  info->hw_version = 1;
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