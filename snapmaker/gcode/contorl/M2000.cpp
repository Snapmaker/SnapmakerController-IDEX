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

#include "../../event/event.h"
#include "../../debug/debug.h"
#include "../../../Marlin/src/core/macros.h"
#include "../../../Marlin/src/gcode/gcode.h"
#include "../../../Marlin/src/module/endstops.h"
#include "../../../Marlin/src/module/stepper.h"
#include "../../module/motion_control.h"
#include "../../module/print_control.h"
#include "../../module/system.h"
#include <EEPROM.h>

/**
 *  S5 P0/1
 */
void GcodeSuite::M2000() {
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0xFF);
  switch (s) {
    case 1:
      debug.show_all_status();
      extern void log_reset_source(void);
      log_reset_source();
      break;
    case 2:
      WRITE(E0_AUTO_FAN_PIN, 1);
      break;
    case 3:
      WRITE(E1_AUTO_FAN_PIN, 1);
      break;
    case 4:
      WRITE(CHAMBER_AUTO_FAN_PIN, 1);
      break;
    case 5:
      { // set pc protocol
        event_handler.recv_enable(EVENT_SOURCE_MARLIN);
      }
      break;
    case 6:
      WRITE(MOTOR_PWR_PIN, 1);
      WRITE(HEATER_PWR_PIN, 1);
      WRITE(HEATER_BED_PWR_PIN, 1);
      break;
    case 7:
      WRITE(MOTOR_PWR_PIN, 0);
      WRITE(HEATER_PWR_PIN, 0);
      WRITE(HEATER_BED_PWR_PIN, 0);
      break;

    case 10:
      // endstops.echo();
      LOG_I("X: %d\r\n", stepper.triggered_position(X_AXIS));
      LOG_I("Y: %d\r\n", stepper.triggered_position(Y_AXIS));
      LOG_I("Z: %d\r\n", stepper.triggered_position(Z_AXIS));
      LOG_I("X1: %d\r\n", axisManager.counts[18]);
      axisManager.counts[18] = 0;
    break;

    case 11:
    {
      quickstop_stepper();
      dual_x_carriage_mode = DXC_FULL_CONTROL_MODE;
      set_duplication_enabled(false);
    }
    break;

    case 12:
    {
      uint8_t e = (uint8_t)parser.byteval('E', (uint8_t)0);
      uint8_t dir = (uint8_t)parser.byteval('D', (uint8_t)1);
      float feedrate_mm_m = (float)parser.byteval('F', (float)20.0);
      if (fabs(feedrate_mm_m) > 0.000001)
        stepper.start_only_extrude(e, dir, 50, feedrate_mm_m);
      else
        stepper.stop_only_extrude(e);
    }
    break;

    case 100:
      LOG_I("test watch dog!\n");
      vTaskDelay(pdMS_TO_TICKS(1000));
      taskENTER_CRITICAL();
      while(1);
      taskEXIT_CRITICAL();
      break;

    case 101:
      LOG_I("test crash!\n");
      vTaskDelay(pdMS_TO_TICKS(10));
      *((uint32_t *)0) = 1234;
      break;

    case 102:
      {
        SERIAL_ECHOLNPGM("\r\n ========= dump start ========= \r\n");
        uint8_t *p_crash_data_char = (uint8_t *)CRASH_DATA_FLASH_ADDR;
        for (uint32_t i = 0; i < CRASH_DATA_SIZE; i++) {
          SERIAL_IMPL.write(p_crash_data_char[i]);
        }
        SERIAL_ECHOLNPGM("\r\n========= crash dump end ========= \r\n");
      }
      break;

    case 103:
      {
        SERIAL_ECHOLNPGM("\r\n ========= erase crash dump ========= \r\n");
        FLASH_Unlock();
        FLASH_ErasePage(CRASH_DATA_FLASH_ADDR);
        FLASH_ErasePage(CRASH_DATA_FLASH_ADDR + APP_FLASH_PAGE_SIZE);
        FLASH_Lock();
      }
      break;

    case 200:
    {
      if (print_control.get_mode() >= PRINT_DUPLICATION_MODE) {
        LOG_I("work mode do not support this command\r\n");
        return;
      }

      if (axisManager.T0_T1_simultaneously_move) {
        LOG_I("BUSY\r\n");
        return;
      }

      if (system_service.get_status() == SYSTEM_STATUE_PAUSING) {
        LOG_I("PAUSING, can not move T0 T1 now\r\n");
        return;
      }

      // uint8_t AXIS = (uint8_t)parser.byteval('X', (uint8_t)0);
      // float L = (float)parser.floatval('L', (float)100.0);
      axisManager.T0_T1_target_pos = x_home_pos(!active_extruder);
      float L = axisManager.T0_T1_target_pos - inactive_extruder_x;
      float V = (float)parser.floatval('V', (float)300.0);
      float A = (float)parser.floatval('A', (float)6000.0);

      float millimeters     = fabs(L);                    // mm
      float entry_speed     = 5 / 1000.0f;            // mm / s
      float leave_speed     = 5 / 1000.0f;            // mm / s
      float nominal_speed   = fabs(V) / 1000.0f;          // mm / ms
      float acceleration    = fabs(A) / 1000000.0f;     // mm / ms^2
      float i_acceleration  = 1.0f / acceleration;
      float i_nominal_speed = 1.0f / nominal_speed;

      float accelDistance = Planner::estimate_acceleration_distance(entry_speed, nominal_speed, acceleration);
      float decelDistance = Planner::estimate_acceleration_distance(nominal_speed, leave_speed, -acceleration);
      if (accelDistance < EPSILON) {
        accelDistance = 0;
      }
      if (decelDistance < EPSILON) {
        decelDistance = 0;
      }
      float plateau = millimeters - accelDistance - decelDistance;

      float accelClocks = (nominal_speed - entry_speed) * i_acceleration;
      float decelClocks = (nominal_speed - leave_speed) * i_acceleration;
      float plateauClocks = plateau * i_nominal_speed;

      if (plateau < 0) {

        float newAccelDistance = Planner::intersection_distance(entry_speed, leave_speed, acceleration, millimeters);
        if (newAccelDistance > millimeters) {
          newAccelDistance = millimeters;
        }
        if (newAccelDistance < EPSILON) {
          newAccelDistance = 0;
        }
        if ((millimeters - newAccelDistance) < EPSILON) {
          newAccelDistance = millimeters;
        }

        accelDistance = newAccelDistance;
        decelDistance = millimeters - accelDistance;
        if (decelDistance < EPSILON) {
          decelDistance = 0;
        }

        nominal_speed = SQRT(2 * acceleration * accelDistance + sq(entry_speed));
        if (nominal_speed < leave_speed) {
          nominal_speed = leave_speed;
        }

        accelClocks = (nominal_speed - entry_speed) * i_acceleration;
        decelClocks = (nominal_speed - leave_speed) * i_acceleration;
        plateauClocks = 0;
        plateau = 0;

      }

      Move move;
      axisManager.axis_t0_t1.reset();
      move.start_t = 0;
      move.axis_r[T0_T1_AXIS_INDEX] = L > 0.0 ? 80 : -80;

      // axisManager.axis_t0_t1.generateLineFuncParams(&move);
      if (accelDistance > 0) {
        move.accelerate = acceleration;
        move.t = accelClocks;
        move.end_t = move.start_t + move.t;
        move.start_pos[T0_T1_AXIS_INDEX] = axisManager.axis_t0_t1.func_manager.last_pos;
        move.end_pos[T0_T1_AXIS_INDEX] = move.start_pos[T0_T1_AXIS_INDEX] + accelDistance * move.axis_r[T0_T1_AXIS_INDEX];
        axisManager.axis_t0_t1.generateLineFuncParams(&move);
        // LOG_I("acc move time %f ms\r\n", move.t);
        // LOG_I("acc move len %f mm\r\n", (move.end_pos[T0_T1_AXIS_INDEX] - move.start_pos[T0_T1_AXIS_INDEX]) / 80);
      }
      if (plateau > 0.0) {
        move.accelerate = 0;
        move.start_t = move.end_t;
        move.t = plateauClocks;
        move.end_t = move.start_t + move.t;
        move.start_pos[T0_T1_AXIS_INDEX] = move.end_pos[T0_T1_AXIS_INDEX];
        move.end_pos[T0_T1_AXIS_INDEX] = move.start_pos[T0_T1_AXIS_INDEX] + plateau * move.axis_r[T0_T1_AXIS_INDEX];
        axisManager.axis_t0_t1.generateLineFuncParams(&move);
        // LOG_I("plat move time %f ms\r\n", move.t);
        // LOG_I("plat move len %f mm\r\n", (move.end_pos[T0_T1_AXIS_INDEX] - move.start_pos[T0_T1_AXIS_INDEX]) / 80);
      }
      if (decelDistance > 0) {
        move.accelerate = -acceleration;
        move.start_t = move.end_t;
        move.t = decelClocks;
        move.end_t = move.start_t + move.t;
        move.start_pos[T0_T1_AXIS_INDEX] = move.end_pos[T0_T1_AXIS_INDEX];
        move.end_pos[T0_T1_AXIS_INDEX] = move.start_pos[T0_T1_AXIS_INDEX] + decelDistance * move.axis_r[T0_T1_AXIS_INDEX];
        axisManager.axis_t0_t1.generateLineFuncParams(&move);
        // LOG_I("decl move time %f ms\r\n", move.t);
        // LOG_I("decl move len %f mm\r\n", (move.end_pos[T0_T1_AXIS_INDEX] - move.start_pos[T0_T1_AXIS_INDEX]) / 80);
      }

      axisManager.T0_T1_axis = !active_extruder;
      inactive_extruder_x = axisManager.T0_T1_target_pos;
      axisManager.T0_T1_last_print_time = 0;
      axisManager.axis_t0_t1.is_consumed = true;
      axisManager.T0_T1_simultaneously_move = true;

      // axisManager.getNextAxisStepper();
      // time_double_t last_time;
      // axisManager.axis_t0_t1.getNextStep();
      // last_time = axisManager.axis_t0_t1.print_time;
      // while (axisManager.axis_t0_t1.getNextStep()) {
      //   float delta_time_ms = axisManager.axis_t0_t1.print_time - last_time;
      //   LOG_I("interval: %.3f ms\r\n", delta_time_ms);
      //   last_time = axisManager.axis_t0_t1.print_time;
      // }

    }
    break;

    default:
      break;
  }

  uint8_t z = (uint8_t)parser.byteval('Z', (uint8_t)0xFF);
  if (z != 0xFF) {
    motion_control.move_to_z((float)z, 600);
  }
}

