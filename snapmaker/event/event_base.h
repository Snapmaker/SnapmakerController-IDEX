#ifndef EVENT_BASE_H
#define EVENT_BASE_H

#include <functional>
#include "../J1/common_type.h"
#include "../protocol/protocol_sacp.h"
#include "../../Marlin/src/core/serial.h"
#include "MapleFreeRTOS1030.h"

typedef std::function<size_t(unsigned char ch)> write_byte_f;
typedef std::function<int(void)> read_byte_f;

// System control command
#define COMMAND_SET_SYS 0x01
// Exception notification command
#define COMMAND_SET_EXCEPTION 0x04
// 3D printing head control
#define COMMAND_SET_FDM 0x10
// Linear control
#define COMMAND_SET_LINEAR 0x13
// bed control
#define COMMAND_SET_BED 0x14
// Enclouser control
#define COMMAND_SET_ENCLOUSER 0x15
// Machine calibration function
#define COMMAND_SET_CAlIBRATION 0xA0
// print control
#define COMMAND_SET_PRINTER 0xAC
// update
#define COMMAND_SET_UPDATE 0xAD

// Event Source
typedef enum {
  EVENT_SOURCE_MARLIN,
  EVENT_SOURCE_HMI,
  EVENT_SOURCE_ALL,
} event_source_e;

// Callback function parameters
typedef struct {
  SACP_head_base_t info;  // Contains basic information about the SACP for replying to messages
  event_source_e source;  // hmi or marlin, used to distinguish event trigger sources
  write_byte_f write_byte;  // Callback of the send data function of the event source
  uint16_t length;  // Length of data
  uint8_t data[PACK_PARSE_MAX_SIZE];
} event_param_t;

//Types of event function callbacks
typedef std::function<ErrCode(event_param_t&)> evevnt_cb_f;

// Used to specify the event callback handling method
typedef enum {
  EVENT_CB_DIRECT_RUN,  // Received data diameter execution
  EVENT_CB_TASK_RUN,  // Put into the event task to execute
} event_callback_mode_e;

// Event callback array nodes
typedef struct {
  uint8_t command_id;
  event_callback_mode_e type;
  evevnt_cb_f cb;
} event_cb_info_t;

#pragma pack(1)
// Generic return result type
typedef struct {
  uint8_t statue; // 0: success, Others: exception codes
} event_result_t;
#pragma pack(0)

extern HardwareSerial *event_serial[EVENT_SOURCE_ALL];
extern write_byte_f event_write_byte[EVENT_SOURCE_ALL];
extern read_byte_f event_read_byte[EVENT_SOURCE_ALL];

void event_base_init();
// Find the corresponding event callback by id
event_cb_info_t * get_evevt_info_by_id(uint8_t id, event_cb_info_t *array, uint8_t count);
// Pack the parameters and call the event source send callback to send the data
ErrCode send_event(event_param_t &event);
ErrCode send_event(event_param_t &event, uint8_t *data, uint16_t length);
ErrCode send_event(event_source_e source, SACP_head_base_t &sacp, uint8_t *data, uint16_t length);
ErrCode send_event(event_source_e source, uint8_t recever_id, uint8_t attribute, uint8_t command_set,
                   uint8_t command_id, uint8_t *data, uint16_t length, uint16_t sequence=0);
ErrCode send_result(event_param_t &event, ErrCode result);
ErrCode write_fun_register(event_source_e source, write_byte_f cb);
bool send_data(event_source_e source, uint8_t *data, uint16_t len);
#endif // EVENT_BASE_H
