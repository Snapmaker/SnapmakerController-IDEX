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
#define SET_BIT(a, b, v) if (v) a |= BIT(b); else a &= ~BIT(b)
#define GET_BIT(a, b)  (!!(a & BIT(b)))


#define HW_1_2(p1, p2) (system_service.get_hw_version() == HW_VER_1 ? (p1) : (p2))

#define PRIVATE_ERROR_BASE  200

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
  E_NO_RESRC,         /* apply resource failed except memory */
  E_FAILURE,          /* common error code */
  E_BUSY,             /* resource is busy, for example, bus is busy,
                      * a mutex lock is busy
                      */
  E_HARDWARE,         /* hardware errors such as invalid bus state */
  E_INVALID_STATE,    /* state is invalid for current operation */
  E_SYSTEM_EXCEPTION, /* prevent action as system exceptions */
  E_COMMON_ERROR,
  E_NON_HOME,                   /* didn't home axes */
  E_MACHINE_PROCESS_BLOCK,      /* handling thread blocking*/

  E_CAlIBRATION_PRIOBE = 200,
  E_CAlIBRATION_XY = 201,
} ErrCode_e;

typedef uint8_t ErrCode;


#endif
