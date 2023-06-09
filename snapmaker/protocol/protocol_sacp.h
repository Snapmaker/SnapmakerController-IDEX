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

#ifndef PROTOCOL_ASCP_H
#define PROTOCOL_ASCP_H

#include "../J1/common_type.h"
#include <functional>
// protocol relative macros
#define SACP_PDU_SOF_H   0xAA
#define SACP_PDU_SOF_L   0x55
#define SACP_VERSION     0x01
#define SACP_HEADER_LEN  (15)   // frame_length - length_paylod

#define SACP_ID_PC         0
#define SACP_ID_CONTROLLER 1
#define SACP_ID_HMI        2

#define PACK_PARSE_MAX_SIZE 512
#define PACK_PACKET_MAX_SIZE 1024

#define SACP_ATTR_REQ 0
#define SACP_ATTR_ACK 1

#pragma pack(1)
typedef struct {
  uint8_t sof_h;
  uint8_t sof_l;
  uint16_t length;
  uint8_t version;  // 0x01
  uint8_t recever_id;
  uint8_t crc8;
  uint8_t sender_id;
  uint8_t attr;
  uint16_t sequence;
  uint8_t command_set;
  uint8_t command_id;
  uint8_t data[0];
} SACP_struct_t;

typedef struct {
  uint8_t recever_id;
  uint8_t attribute;
  uint16_t sequence;
  uint8_t command_set;
  uint8_t command_id;
} SACP_head_base_t;

typedef struct {
  uint16_t lenght;  // The total length of data
  union {
    uint8_t buff[PACK_PARSE_MAX_SIZE];
    SACP_struct_t sacp;
  };
} SACP_param_t;



#pragma pack()

class ProtocolSACP {
  public:
    ErrCode parse(uint8_t *data, uint16_t len, SACP_param_t &out);
    // Package the incoming data
    uint16_t package(SACP_head_base_t head, uint8_t *in_data, uint16_t length, uint8_t *out_data);
    uint16_t sequence_pop() {return sequence++;}
  private:
    uint32_t sequence = 0;
};

extern ProtocolSACP protocol_sacp;
#endif
