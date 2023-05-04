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

#include <stdio.h>
#include <stdint.h>
#include "factory_data.h"

factory_data_srv fd_srv;

uint32_t fd_calc_checksum(uint8_t *buffer, uint32_t length) {
  uint32_t volatile checksum = 0;

  if (!length || !buffer)
    return 0;

  for (uint32_t j = 0; j < (length - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (length % 2)
    checksum += buffer[length - 1];

  checksum = ~checksum;

  return checksum;
}

void fd_erase_flash_page(uint32_t addr, uint16_t page_count) {
  FLASH_Unlock();
  for (int i = 0; i < page_count; i++) {
    FLASH_ErasePage(addr);
    if (addr < DATA_FLASH_START_ADDR)
      addr += APP_FLASH_PAGE_SIZE;
    else
      addr += DATA_FLASH_PAGE_SIZE;
  }
  FLASH_Lock();
}

void fd_write_to_flash(uint32_t addr, uint8_t *data, uint32_t len) {
  uint16_t tmp;
  FLASH_Unlock();
  for (uint32_t i = 0; (i + 2) <= len; i = i + 2) {
    tmp = ((data[i + 1]<<8) | data[i]);
    FLASH_ProgramHalfWord(addr, tmp);
    addr = addr + 2;
  }
  if (len % 2) {
    tmp = ((data[len - 1]) | 0xFF00);
    FLASH_ProgramHalfWord(addr, tmp);
  }
  FLASH_Lock();
}

bool factory_data_srv::init() {
  if (!load()) {
    reset();
    if (!save()) {
      LOG_E("Factory data save SN_ERROR\n");
      return false;
    }
  }

  log();
  return true;
}

bool factory_data_srv::load() {

  uint8_t *p_fd_flash = (uint8_t *)FACTORY_DATA_FLASH_ADDR;
  uint8_t *p_fd_ram = (uint8_t *)&_data;
  for (uint32_t i = 0; i < FACTORY_DATA_SIZE; i++) {
    p_fd_ram[i] = p_fd_flash[i];
  }

  return check(_data);
}

void factory_data_srv::reset(){
  LOG_I("Factory data reset\n");
  _data.fd.build_plate_thickness = DEFAULT_BUILD_PLATE_THICKNESS;
}

void factory_data_srv::erase() {
  fd_erase_flash_page(FACTORY_DATA_FLASH_ADDR, FACTORY_DATA_PAGE_SIZE);
}

bool factory_data_srv::save() {

  strncpy((char *)_data.magic, FACTORY_DATA_MAGIC, 4);
  _data.len = sizeof(fd_info_t);
  _data.checksum = fd_calc_checksum((uint8_t *)(&_data.fd), _data.len);

  erase();
  fd_write_to_flash(FACTORY_DATA_FLASH_ADDR, (uint8_t *)(&_data), FACTORY_DATA_SIZE);

  return load();
}

bool factory_data_srv::setBuildPlateThickness(float bpt) {
  if (MIN_BUILD_PLATE_THICKNESS <= bpt && bpt <= MAX_BUILD_PLATE_THICKNESS) {
    _data.fd.build_plate_thickness = bpt;
    return true;
  }
  else {
    return false;
  }
}
float factory_data_srv::getBuildPlateThickness() {
  return _data.fd.build_plate_thickness;
}

void factory_data_srv::log(void) {
  LOG_I("\r\n========= factory data =========\r\n");
  LOG_I("build_plate_thickness: %.3f\r\n", _data.fd.build_plate_thickness);
}

bool factory_data_srv::check(fd_flash_t &fd) {
  if (0 != strncmp(FACTORY_DATA_MAGIC, (char *)fd.magic, 4)) {
    LOG_I("Factory data magic check failed\n");
    return false;
  }

  uint32_t cacl_check = fd_calc_checksum((uint8_t *)(&fd.fd), fd.len);
  if (cacl_check != _data.checksum) {
    LOG_I("Factory data checksum failed\n");
    return false;
  }
  else {
    return true;
  }
}

// uint32_t UpdateServer::update_packet_head_checksum(update_packet_info_t *head) {
//   uint32_t checksum = update_calc_checksum((uint8_t *)head, sizeof(update_packet_info_t) - sizeof(head->pack_head_checknum));
//   return checksum;
// }

// ErrCode  UpdateServer::update_info_check(update_packet_info_t *head) {
//   uint32_t checksum = update_packet_head_checksum(head);
//   if (checksum != head->pack_head_checknum) {
//     return E_PARAM;
//   }

//   if (head->app_flash_start_addr % APP_FLASH_PAGE_SIZE) {
//     return E_PARAM;
//   }

//   if (head->app_flash_start_addr < FLASH_BASE + BOOT_CODE_SIZE) {
//     return E_PARAM;
//   }

//   if (head->type != 3) {
//     return E_PARAM;
//   }

//   return E_SUCCESS;
// }

// void UpdateServer::set_update_status(uint16_t status) {
//   update_packet_info_t *flash_info = (update_packet_info_t *)UPDATE_DATA_FLASH_ADDR;
//   update_packet_info_t info;
//   memcpy((uint8_t *)&info, (uint8_t *)flash_info, sizeof(update_packet_info_t));
//   info.status_flag = status;
//   uint32_t checksum = update_packet_head_checksum(&info);
//   info.pack_head_checknum = checksum;

//   erase_flash_page(UPDATE_DATA_FLASH_ADDR, 1);
//   write_to_flash(UPDATE_DATA_FLASH_ADDR, (uint8_t*)&info, sizeof(update_packet_info_t));
// }

// void UpdateServer::save_update_info(update_packet_info_t * info, uint8_t usart_num, uint8_t receiver_id) {
//   info->status_flag = UPDATE_STATUS_START;
//   info->usart_num = usart_num;
//   info->receiver_id = receiver_id;
//   uint32_t checksum = update_packet_head_checksum(info);
//   info->pack_head_checknum = checksum;
//   erase_flash_page(UPDATE_DATA_FLASH_ADDR, 1);
//   write_to_flash(UPDATE_DATA_FLASH_ADDR, (uint8_t*)info, sizeof(update_packet_info_t));
// }

// ErrCode UpdateServer::is_allow_update(update_packet_info_t *head) {
//   ErrCode ret = update_info_check(head);
//   return ret;
// }

// void UpdateServer::just_to_boot() {
//   // WatchDogInit();
//   nvic_sys_reset();
// }

// void UpdateServer::init() {
//   update_packet_info_t *update_info =  (update_packet_info_t *)UPDATE_DATA_FLASH_ADDR;
//   if (update_info->status_flag != UPDATE_STATUS_APP_NORMAL) {
//     set_update_status(UPDATE_STATUS_APP_NORMAL);
//   }
// }
