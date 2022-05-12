#ifndef COMMON_TYPE_H
#define COMMON_TYPE_H
#include <stdint.h>
#include "MapleFreeRTOS1030.h"
#include "../debug/debug.h"

#define FLOAT_TO_INT(f) (int)((f) * 1000)
#define INT_TO_FLOAT(i) ((float)(i) / 1000)
typedef int float_to_int_t;  // float * 1000 to int

#define MODULE_KEY(type, index) (type << 3 | index)
#define MODULE_INDEX(key) (key & (0x7))

#define MV_TO_ADC_VAL(mv) (mv * 4096 / 3300)

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))
#define BIT(shift)   (1UL << (shift))

typedef enum : uint8_t {
  E_SUCCESS = 0,      /* non error */ 
  E_IN_PROGRESS,      /*Notification receives instructions and starts work*/
  E_RESEND_FAILED,
  E_EXECUTE_FAILED,
  E_COMMAND_SET,
  E_COMMAND_ID,
  E_PARAM,            /* got a invalid parameter */   
  E_MODULE_KEY,
  E_NO_MEM,           /* apply memory failed */
  E_SYSTEM_EXCEPTION = 14,
  E_COMMON_ERROR = 15,
  E_ADJUST_PRIOBE = 200,
  E_ADJUST_XY = 201,
} ErrCode_e;

typedef uint8_t ErrCode;


#endif
