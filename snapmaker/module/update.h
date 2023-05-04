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

#ifndef UPDATE_H
#define UPDATE_H
#include "../J1/common_type.h"

#define UPDATE_STATUS_START 0xAA02
#define UPDATE_STATUS_APP_NORMAL 0xAA05

#pragma pack(1)

typedef struct {
  uint32_t start_addr;
  uint32_t end_addr;
  uint8_t data[];
} packet_data_t;

typedef struct {
  uint8_t  file_flag[21];
  uint8_t  pack_version;
  uint16_t type;
  uint8_t is_force_update;
  uint16_t start_id;
  uint16_t end_id;
  uint8_t  app_version[32];
  uint8_t  pack_time[20];
  uint16_t status_flag;
  uint32_t app_length;
  uint32_t app_checknum;
  uint32_t app_flash_start_addr;
  uint8_t usart_num;
  uint8_t receiver_id;
  uint32_t pack_head_checknum;
} update_packet_info_t;

#pragma pack(1)


class UpdateServer {
  public:
    void      init();
    ErrCode   is_allow_update(update_packet_info_t *head);
    void save_update_info(update_packet_info_t * info, uint8_t usart_num, uint8_t receiver_id);
    void just_to_boot();
  private:
    ErrCode update_info_check(update_packet_info_t *head);
    uint32_t update_packet_head_checksum(update_packet_info_t *head);
    void set_update_status(uint16_t status);
};

extern UpdateServer update_server;
#endif
