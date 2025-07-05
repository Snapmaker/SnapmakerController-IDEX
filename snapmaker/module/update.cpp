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

#include "update.h"
#include "../../Marlin/src/core/serial.h"
#include "flash_stm32.h"
#include HAL_PATH(src/HAL, HAL_watchdog_STM32F1.h)

UpdateServer update_server;

uint32_t update_calc_checksum(uint8_t *buffer, uint32_t length) {
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

void erase_flash_page(uint32_t addr, uint16_t page_count) {
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

void write_to_flash(uint32_t addr, uint8_t *data, uint32_t len) {
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

uint32_t UpdateServer::update_packet_head_checksum(update_packet_info_t *head) {
  uint32_t checksum = update_calc_checksum((uint8_t *)head, sizeof(update_packet_info_t) - sizeof(head->pack_head_checknum));
  return checksum;
}

ErrCode  UpdateServer::update_info_check(update_packet_info_t *head) {
  uint32_t checksum = update_packet_head_checksum(head);
  if (checksum != head->pack_head_checknum) {
    return E_PARAM;
  }

  if (head->app_flash_start_addr % APP_FLASH_PAGE_SIZE) {
    return E_PARAM;
  }

  if (head->app_flash_start_addr < FLASH_BASE + BOOT_CODE_SIZE) {
    return E_PARAM;
  }

  if (head->type != 3) {
    return E_PARAM;
  }

  return E_SUCCESS;
}

void UpdateServer::set_update_status(uint16_t status) {
  update_packet_info_t *flash_info = (update_packet_info_t *)UPDATE_DATA_FLASH_ADDR;
  update_packet_info_t info;
  memcpy((uint8_t *)&info, (uint8_t *)flash_info, sizeof(update_packet_info_t));
  info.status_flag = status;
  uint32_t checksum = update_packet_head_checksum(&info);
  info.pack_head_checknum = checksum;

  erase_flash_page(UPDATE_DATA_FLASH_ADDR, 1);
  write_to_flash(UPDATE_DATA_FLASH_ADDR, (uint8_t*)&info, sizeof(update_packet_info_t));
}

void UpdateServer::save_update_info(update_packet_info_t * info, uint8_t usart_num, uint8_t receiver_id) {
  info->status_flag = UPDATE_STATUS_START;
  info->usart_num = usart_num;
  info->receiver_id = receiver_id;
  uint32_t checksum = update_packet_head_checksum(info);
  info->pack_head_checknum = checksum;
  erase_flash_page(UPDATE_DATA_FLASH_ADDR, 1);
  write_to_flash(UPDATE_DATA_FLASH_ADDR, (uint8_t*)info, sizeof(update_packet_info_t));
}

ErrCode UpdateServer::is_allow_update(update_packet_info_t *head) {
  ErrCode ret = update_info_check(head);
  return ret;
}

void UpdateServer::just_to_boot() {
  // WatchDogInit();
  nvic_sys_reset();
}

void UpdateServer::init() {
  update_packet_info_t *update_info =  (update_packet_info_t *)UPDATE_DATA_FLASH_ADDR;
  if (update_info->status_flag != UPDATE_STATUS_APP_NORMAL) {
    set_update_status(UPDATE_STATUS_APP_NORMAL);
  }
}
