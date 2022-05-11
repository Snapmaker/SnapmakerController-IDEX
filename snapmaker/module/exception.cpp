#include "exception.h"
#include "src/core/serial.h"
Exception exception_server;

typedef struct {
  uint32_t behavior;
  uint8_t level;
} exception_behavior_t;

const exception_behavior_t exception_behavior_map[] = {
  {(BIT(0)),                          EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_NONE
  {(BIT(EXCEPTION_BAN_HEAT_BED)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_TMC_FILED
  {(BIT(EXCEPTION_BAN_HEAT_BED)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_NOT_FIND
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_SELF_CHECK
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_LEFT_NOZZLE_LOSS
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BOTH_NOZZLE_LOSS
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_LEFT_NOZZLE_TEMP
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_RIGHT_NOZZLE_TEMP
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BOTH_NOZZLE_TEMP
  {(BIT(EXCEPTION_BAN_HEAT_BED)),     EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_TEMP
  {(BIT(EXCEPTION_BAN_MOVE)),         EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_MOVE
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_LEFT_NOZZLE_TEMP_TIMEOUT
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_RIGHT_NOZZLE_TEMP_TIMEOUT
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BOTH_NOZZLE_TEMP_TIMEOUT
  {(BIT(EXCEPTION_BAN_HEAT_NOZZLE)),  EXCEPTION_LEVLE_0},  // EXCEPTION_TYPE_BED_TEMP_TIMEOUT
};

bool Exception::is_exception() {
  return exception_status != 0;
}

uint32_t Exception::get_exception() {
  return exception_status;
}

uint32_t Exception::get_behavior() {
  return  exception_behavior;
}

uint8_t Exception::get_level(exception_type_e e) {
  return exception_behavior_map[e].level;
}

void Exception::trigger_exception(exception_type_e e) {
  SERIAL_ECHOLNPAIR("trigger exception:", e);
  EXCEPTION_TRIGGER(e);
  exception_behavior |= exception_behavior_map[e].behavior;
}

void Exception::clean_exception(exception_type_e e) {
  uint32_t exception = exception_status & (~BIT(e));
  SERIAL_ECHOLNPAIR("clear exception:", e);
  exception_status = exception_behavior = 0;
  for (uint8_t i = 0; i < EXCEPTION_TYPE_MAX_COUNT; i++) {
    if (exception & BIT(i)) {
      trigger_exception((exception_type_e)i);
    }
  }
}
