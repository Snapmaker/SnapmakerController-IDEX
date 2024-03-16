/*
 * Snapmaker 3D Printer Firmware
 * Copyright (C) 2023 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of SnapmakerController-IDEX
 * (see https://github.com/Snapmaker/SnapmakerController-IDEX)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "system.h"
#include "../../Marlin/src/inc/Version.h"
#include "../../Marlin/src/module/motion.h"
#include "../../Marlin/src/module/settings.h"
#include "../../Marlin/src/module/temperature.h"
#include "power_loss.h"
#include "fdm.h"
#include "../module/motion_control.h"
#include "../module/print_control.h"

SystemService system_service;

#define SN_LENGHT 30
#define SN_BACKUP_COUNT 2
#define FLASH_SN_SIZE (sizeof(fastory_sn_t) * SN_BACKUP_COUNT)
#define FLASH_PAGE_SN_ADDR  (APP_FLASH_PAGE_SIZE - FLASH_SN_SIZE)
#define FLASH_SN_ADDR (FLASH_BASE + 1024 * 10 - APP_FLASH_PAGE_SIZE + FLASH_PAGE_SN_ADDR)

#pragma pack(1)

typedef struct {
    uint16_t check_num;
    uint8_t sn[SN_LENGHT];
} fastory_sn_t;

#pragma pack()

const char *default_sn_str = "SN not found";

static const uint16_t hw_version_table[] = {
    MV_TO_ADC_VAL(0 ),
    MV_TO_ADC_VAL(450 ),
    MV_TO_ADC_VAL(700 ),
    MV_TO_ADC_VAL(1400),
    MV_TO_ADC_VAL(2000),
    MV_TO_ADC_VAL(2700),
    MV_TO_ADC_VAL(3200)
  };

static uint16_t calc_checksum(uint8_t *buffer, uint16_t length) {
  uint32_t volatile checksum = 0;

  if (!length || !buffer)
    return 0;

  for (int j = 0; j < (length - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (length % 2)
    checksum += buffer[length - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return (uint16_t)checksum;
}

static bool flash_is_erase() {
  uint8_t * addr = (uint8_t *)FLASH_SN_ADDR;
  for (uint16_t i = 0; i < FLASH_SN_SIZE; i++) {
    if (addr[i] != 0xff) {
      return false;
    }
  }
  return true;
}

static bool is_has_sn(uint8_t &index) {
  fastory_sn_t * sn = (fastory_sn_t *)FLASH_SN_ADDR;
  for (uint8_t i = 0; i < SN_BACKUP_COUNT; i++) {
    uint16_t check = calc_checksum(sn[i].sn, SN_LENGHT);
    if (sn[i].check_num == check) {
      index = i;
      return true;
    }
  }
  return false;
}

uint8_t *SystemService::get_sn_addr(uint16_t *sn_len) {
  fastory_sn_t * sn = (fastory_sn_t *)FLASH_SN_ADDR;
  uint8_t index = 0;

  if (!flash_is_erase() && is_has_sn(index)) {
    *sn_len = SN_LENGHT;
    return sn[index].sn;
  } else {
    *sn_len = strlen((char *)default_sn_str);
    return (uint8_t *)default_sn_str;
  }
}

void SystemService::init() {
  lock_ = xSemaphoreCreateMutex();
  if (!lock_) {
    LOG_E("Can NOT create a mutex for system service\r\n");
    while(1) vTaskDelay(pdMS_TO_TICKS(100));
  }
}

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

ErrCode SystemService::set_status(system_status_e req_status, system_status_source_e source) {

  ErrCode ret = E_FAILURE;

  LOG_I("Current system status %d, request status %d\r\n", status_, req_status);
  if (req_status == status_) return E_SUCCESS;

  xSemaphoreTake(lock_, 0xFFFFFFFF);
  switch (req_status) {
    case SYSTEM_STATUE_IDLE:
      status_ = req_status;
      ret = E_SUCCESS;
      break;

    case SYSTEM_STATUE_STARTING:
      if (SYSTEM_STATUE_IDLE == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_PRINTING:
      if (SYSTEM_STATUE_STARTING == status_ ||
          SYSTEM_STATUE_RESUMING == status_ ||
          SYSTEM_STATUE_IDLE == status_ ||
          SYSTEM_STATUE_POWER_LOSS_RESUMING == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_PAUSING:
      if (SYSTEM_STATUE_PRINTING == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
        wait_for_heatup = false;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_PAUSED:
      if (SYSTEM_STATUE_PAUSING == status_ ||
          SYSTEM_STATUE_RESUMING == status_ ||
          SYSTEM_STATUE_PRINTING == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_ ||
          SYSTEM_STATUE_POWER_LOSS_RESUMING == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_STOPPING:
      if (SYSTEM_STATUE_PRINTING == status_ ||
          SYSTEM_STATUE_PAUSED == status_ ||
          SYSTEM_STATUE_PAUSING == status_ ||
          SYSTEM_STATUE_RESUMING == status_ ||
          SYSTEM_STATUE_FINISHING == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_) {
        status_ = req_status;
        wait_for_heatup = false;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_STOPPED:
      if (SYSTEM_STATUE_STOPPING == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_FINISHING:
      if (SYSTEM_STATUE_PRINTING == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
        wait_for_heatup = false;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_COMPLETED:
      wait_for_heatup = false;
      status_ = req_status;
      ret = E_SUCCESS;
      break;

    case SYSTEM_STATUE_RECOVERING:
      status_ = req_status;
      ret = E_SUCCESS;
      break;

    case SYSTEM_STATUE_RESUMING:
      if (SYSTEM_STATUE_PAUSED == status_ || SYSTEM_STATUE_RECOVERING == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_POWER_LOSS_RESUMING:
      if (SYSTEM_STATUE_IDLE == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_CAlIBRATION:
      if (SYSTEM_STATUE_IDLE == status_ ||
          SYSTEM_STATUE_CAlIBRATION_Z_PROBING == status_ ||
          SYSTEM_STATUE_CAlIBRATION_XY_PROBING == status_ ||
          SYSTEM_STATUE_PID_AUTOTUNE == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_CAlIBRATION_Z_PROBING:
      if (SYSTEM_STATUE_IDLE == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_ ||
          SYSTEM_STATUE_CAlIBRATION_XY_PROBING == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_CAlIBRATION_XY_PROBING:
      if (SYSTEM_STATUE_IDLE == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_ ||
          SYSTEM_STATUE_CAlIBRATION_Z_PROBING == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    case SYSTEM_STATUE_PID_AUTOTUNE:
      if (SYSTEM_STATUE_IDLE == status_ ||
          SYSTEM_STATUE_CAlIBRATION == status_) {
        status_ = req_status;
        ret = E_SUCCESS;
      }
      else {
        ret = E_BUSY;
      }
      break;

    default:
      status_ = req_status;
      ret = E_SUCCESS;
      break;
  }

  if (E_SUCCESS == ret) {
    if (source != SYSTEM_STATUE_SCOURCE_NONE) {
      source_ = source;
    }
  }

  xSemaphoreGive(lock_);
  return ret;
}


bool SystemService::is_working() {
  switch (status_)
  {
  case SYSTEM_STATUE_STARTING:
  case SYSTEM_STATUE_PRINTING:
  case SYSTEM_STATUE_PAUSING:
  case SYSTEM_STATUE_PAUSED:
  case SYSTEM_STATUE_RESUMING:
  case SYSTEM_STATUE_RECOVERING:
  case SYSTEM_STATUE_POWER_LOSS_RESUMING:
    return true;
    break;

  default:
    break;
  }

  return false;
}

bool SystemService::is_printing() {
  switch (status_) {
    case SYSTEM_STATUE_PRINTING:
      return true;

    default:
      break;
  }

  return false;
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

void SystemService::save_setting() {
  settings.save();
}

void SystemService::return_to_idle() {
  LOG_I("System return to idle\r\n");
  // stop hotends and bed, stop fan, disable motor
  HOTEND_LOOP() {
    thermalManager.setTargetHotend(0, e);
    fdm_head.set_fan_speed(e, 0, 0);
  }
  thermalManager.setTargetBed(0);

  // reset to normal
  print_control.set_noise_mode(NOISE_NOIMAL_MODE);

  /*
  for (uint8_t i = 0; i <= E_AXIS; i++) {
    motion_control.motor_disable(i);
    if (i == X_AXIS || i == E_AXIS) motion_control.motor_disable(i, 1);
  }
  */
}
