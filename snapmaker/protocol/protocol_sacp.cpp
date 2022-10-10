#include "protocol_sacp.h"
#include <functional>
#include "HAL.h"
#include "../../Marlin/src/core/serial.h"

ProtocolSACP protocol_sacp;

static uint8_t sacp_calc_crc8(uint8_t *buffer, uint16_t len) {
  int crc = 0x00;
  int poly = 0x07;
  for (int i = 0; i < len; i++) {
    for (int j = 0; j < 8; j++) {
      bool bit = ((buffer[i] >> (7 - j) & 1) == 1);
      bool c07 = ((crc >> 7 & 1) == 1);
      crc <<= 1;
      if (c07 ^ bit) {
        crc ^= poly;
      }
    }
  }
  crc &= 0xff;
  return crc;
}

uint16_t calc_checksum(uint8_t *buffer, uint16_t length) {
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

ErrCode ProtocolSACP::parse(uint8_t *data, uint16_t len, SACP_param_t &out) {
  uint8_t *parse_buff = out.buff;
  if (parse_buff[0] != SACP_PDU_SOF_H) {
    out.lenght = 0;
  }
  for (uint16_t i = 0; i < len; i++) {
    uint8_t ch = data[i];
    if (out.lenght == 0) {
      if (ch == SACP_PDU_SOF_H) {
        parse_buff[out.lenght++] = ch;
      }
    } else if (out.lenght == 1) {
      if (ch == SACP_PDU_SOF_L) {
        parse_buff[out.lenght++] = ch;
      } else {
        out.lenght = 0;
      }
    } else {
      parse_buff[out.lenght++] = ch;
    }

    if (out.lenght < 7) {
      break;
    }
    else if (out.lenght == 7) {
      if (sacp_calc_crc8(parse_buff, 6) != parse_buff[6]) {
        out.lenght = 0;
      }
    }
    else {
      uint16_t data_len = (parse_buff[3] << 8 | parse_buff[2]);
      uint16_t total_len = data_len + 7;
      if (out.lenght == total_len) {
        uint16_t checksum = calc_checksum(&parse_buff[7], data_len - 2);
        uint16_t checksum1 = (parse_buff[total_len - 1] << 8) | parse_buff[total_len - 2];
        if (checksum == checksum1) {
          out.lenght = 0;
          return E_SUCCESS;
        } else {
          out.lenght = 0;
          return E_PARAM;
        }
      } else if (out.lenght > total_len) {
        out.lenght = 0;
        return E_PARAM;
      }
    }
  }
  return E_IN_PROGRESS;
}


uint16_t ProtocolSACP::package(SACP_head_base_t head, uint8_t *in_data, uint16_t length, uint8_t *out_data) {
  uint16_t data_len = (length + 8); // header 6 byte, checknum 2byte
  SACP_struct_t *out =  (SACP_struct_t *)out_data;
  out->sof_h = SACP_PDU_SOF_H;
  out->sof_l = SACP_PDU_SOF_L;
  out->length = data_len;
  out->version = SACP_VERSION;
  out->recever_id = head.recever_id;
  out->crc8 = sacp_calc_crc8(out_data, 6);
  out->sender_id = SACP_ID_CONTROLLER;
  out->attr = head.attribute;
  out->sequence = head.sequence;
  out->command_set = head.command_set;
  out->command_id = head.command_id;
  for (uint16_t i = 0; i < length; i++) {
    out->data[i] = in_data[i];
  }
  uint16_t checksum = calc_checksum(&out_data[7], data_len - 2);  // - checknum 2 byte
  length = sizeof(SACP_struct_t) + length;
  out_data[length++] = (uint8_t)(checksum & 0x00FF);
  out_data[length++] = (uint8_t)(checksum>>8);
  return length;
}
