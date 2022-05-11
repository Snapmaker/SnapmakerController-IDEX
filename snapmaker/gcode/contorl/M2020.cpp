/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../../Marlin/src/gcode/gcode.h"
#include "../../J1/tmc_driver.h"
#include "../../module/filament_sensor.h"
#include "../../module/fdm.h"
#include "../../module/system.h"
#include "../../module/print_control.h"
#include "../../module/exception.h"

/**
 *  S0
 *  PWM duty cycle goes from 0 (off) to 255 (always on).
 */
void GcodeSuite::M2020() {
  uint32_t reg_value;
  uint32_t tmp_value;
  uint8_t tmp_u8_value;
  float tmp_float_value;
  uint8_t reg_addr = 0xff;
  uint8_t motor_index;
  uint8_t div;
  uint8_t hold = 15;
  if(parser.seen('S')) {
    switch(parser.value_ushort()) {
      case 0:
        if(parser.seenval('O')) {
          motor_index = (uint8_t)parser.value_byte();
          tmc_driver.select(motor_index);
        }
        if(parser.seenval('R'))
          reg_addr = (uint8_t)parser.value_ushort();
        if(parser.seenval('V'))
          reg_value = parser.value_ulong();
        if(reg_addr != 0xff)
          tmc_driver.comm_test(0, reg_addr, &reg_value);
      break;

      case 1:
        if(parser.seenval('O')) {
            motor_index = (uint8_t)parser.value_byte();
            tmc_driver.select(motor_index);
        }
        if(parser.seenval('R'))
          reg_addr = (uint8_t)parser.value_ushort();
        tmc_driver.comm_test(1, reg_addr, &reg_value);
        SERIAL_ECHOPAIR("Reg Addr:", reg_addr, " Reg Value:", reg_value);
      break;

      case 2:
        if(READ(X0_CAL_PIN))
          SERIAL_ECHOLN("X0_CAL: HIGH");
        else
          SERIAL_ECHOLN("X0_CAL: LOW");
        if(READ(X1_CAL_PIN))
          SERIAL_ECHOLN("X1_CAL: HIGH");
        else
          SERIAL_ECHOLN("X1_CAL: LOW");
      break;

      case 3:
        reg_value = fdm_head.get_nozzle_type(0, (nozzle_texture_type_e*)&tmp_u8_value, &tmp_float_value);
        SERIAL_ECHOLNPAIR("nozzle ", 0, " type:", tmp_u8_value, " caliber:", tmp_float_value, " adc:", reg_value);
        reg_value = fdm_head.get_nozzle_type(1, (nozzle_texture_type_e*)&tmp_u8_value, &tmp_float_value);
        SERIAL_ECHOLNPAIR("nozzle ", 1, " type:", tmp_u8_value, " caliber:", tmp_float_value, " adc:", reg_value);
        break;
      case 4:
        tmc_driver.init();
      break;

      case 5:
        if(parser.seenval('T')) {
          uint8_t e = (uint8_t)parser.value_byte();
          if(parser.seenval('D')) {
            uint8_t d = (uint8_t)parser.value_byte();
            fdm_head.change_filamenter(e, 600, (filamenter_change_status_e)d);
          }
        }
      break;

      case 6:
        div = 31;
        motor_index = 7;
        if(parser.seenval('O')) {
          motor_index = parser.value_byte();
          tmc_driver.select(motor_index);
        }
        if(parser.seenval('I'))
          div = parser.value_byte();
        if(div < 4)
          div = 4;
        if(div > 31)
          div = 31;
        if(parser.seenval('H')) {
          hold = parser.value_byte();
        }
        reg_value = (5 << 16) | (div << 8) | (hold);
        
        tmc_driver.set_reg_value(motor_index, R_GCONF, 0x141);
        tmc_driver.set_reg_value(motor_index, R_IHOLD_IRUN, reg_value);
        tmc_driver.comm_test(1, 0x00, &tmp_value);
        tmc_driver.comm_test(1, 0x10, &reg_value);
        tmc_driver.select(7);
      break;

      case 7:
        if(parser.seen('O')) {
          bool level = parser.byteval('O');
          if (level) {
            filament_sensor.enable_all();
          } else {
            filament_sensor.disable_all();
          }
        }
        filament_sensor.debug();
      break;

      case 8:
        SERIAL_ECHOLNPAIR("system status:", system_service.get_status());
        if (print_control.print_err_info.is_err == false) {
          SERIAL_ECHOLN("Print work normal");
        } else {
          SERIAL_ECHOLNPAIR("Print work timeout, last line:", print_control.print_err_info.err_line);
        }
        break;
      case 9:
        if(parser.seen('E') && parser.seen('D') && parser.seen('C')) {
          filament_sensor.test_adc(parser.byteval('E'), parser.longval('D'), parser.longval('C'));
        }
      break;

      case 10:
        if(parser.seen('L')) {
          uint8_t level = parser.byteval('L', 4);
          tmc_driver.set_stall_guard_level(level);
        }
      break;

      case 11:
        if(parser.seen('L')) {
          uint8_t level = parser.byteval('L', 32);
          uint8_t func = parser.byteval('F', 0);
          if(func == 0)
            tmc_driver.configure_for_platform_calibration(level);
          else
            tmc_driver.configure_for_xy_calibration(level, level);
        }
        if(parser.seen('H')) {
          uint32_t step_tick = parser.value_ushort();
          tmc_driver.set_stepper_tick_threshold(step_tick);
        }
        if(parser.seen('C')) {
          reg_value = parser.value_ushort();
          tmc_driver.set_reg_value(0, R_TCOOLTHRS, reg_value);
        }
        break;
      case 12:
        tmp_u8_value = system_service.get_hw_version();
        SERIAL_ECHOLNPAIR("hw version:", tmp_u8_value);
        break;
      case 13:
        if (parser.seen('E')) {
          exception_server.trigger_exception((exception_type_e)parser.byteval('E'));
        } else if (parser.seen('C')) {
          exception_server.clean_exception((exception_type_e)parser.byteval('C'));
        }
        break;
      default:

      break;
    }
  }
}

