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

#include "../../event/event.h"
#include "../../debug/debug.h"
#include "../../../Marlin/src/core/macros.h"
#include "../../../Marlin/src/gcode/gcode.h"
#include "../../../Marlin/src/module/endstops.h"
#include "../../../Marlin/src/module/stepper.h"
#include "../../module/motion_control.h"
#include "../../module/print_control.h"
#include "../../module/power_loss.h"
#include "../../module/system.h"
#include "../../J1/switch_detect.h"
#include "../../module/factory_data.h"
#include "../../module/calibtration.h"
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
    case 8:
      {
        uint8_t dl = parser.byteval('D', SNAP_DEBUG_LEVEL_INFO);
        debug.set_level((debug_level_e)dl);
      }
      break;

    case 10:
      // endstops.echo();
      LOG_I("X: %d\r\n", stepper.triggered_position(X_AXIS));
      LOG_I("Y: %d\r\n", stepper.triggered_position(Y_AXIS));
      LOG_I("Z: %d\r\n", stepper.triggered_position(Z_AXIS));
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
      if (fabs(feedrate_mm_m) > EPSILON)
        stepper.start_only_extrude(e, dir, 50, feedrate_mm_m);
      else
        stepper.stop_only_extrude(e);
    }
    break;

    case 13:
    {
      // stepper.report_a_position(planner.position);
      LOG_I("E_COUNT: %d\r\n", stepper.position(E_AXIS));
    }
    break;

    case 14:
    {
      print_control.z_home_sg = (uint8_t)parser.byteval('Z', (uint8_t)0);
      LOG_I("Z home sg set to %d\r\n", print_control.z_home_sg);
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

    case 104:
      {
        if(parser.seen('X')) {
          LOG_I("X speed %f mms\r\n", axisManager.axis[0].getCurrentSpeedMMs());
        }
        else if(parser.seen('Y')) {
          LOG_I("Y speed %f mms\r\n", axisManager.axis[1].getCurrentSpeedMMs());
        }
        else if(parser.seen('Z')) {
          LOG_I("Z speed %f mms\r\n", axisManager.axis[2].getCurrentSpeedMMs());
        }
      }
      break;

    case 105:
    {
      uint32_t gcode_jumper_line = parser.longval('J', 0x0);
      if (gcode_jumper_line) {
        power_loss.next_req = gcode_jumper_line;
      }
    }
    break;

    case 110:
    {
      LOG_I("Erase factory data\r\n");
      // fd_srv.reset();
      // fd_srv.save();
      // fd_srv.log();
      fd_srv.erase();
    }
    break;

    case 111:
    {
      LOG_I("Set factory build plate thickness data\r\n");
      float bpt = (float)parser.floatval('B', (float)5.0);
      if (fd_srv.setBuildPlateThickness(bpt)) {
        fd_srv.save();
        fd_srv.log();
        calibtration.updateBuildPlateThickness(fd_srv.getBuildPlateThickness());
      }
      else {
        LOG_W("Build plate thickness is not valid\n\n");
      }
    }
    break;

    case 112:
    {
      fd_srv.log();
    }
    break;

    case 113:
    {
      uint8_t pnm = parser.byteval('M', 0xFF);
      print_control.set_noise_mode(print_noise_mode_e(pnm));
    }
    break;

    case 114:
    {
      LOG_I("print noise mode %u\n", (uint8_t)print_control.get_noise_mode());
    }
    break;

    case 115:
    {
      LOG_I("trun on probe power\n");
      switch_detect.trun_on_probe_pwr();
    }
    break;

    case 116:
    {
      LOG_I("trun off probe power\n");
      switch_detect.trun_off_probe_pwr();
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

      if (SYSTEM_STATUE_PRINTING != system_service.get_status()) {
        LOG_I("Not printing, can not move T0 T1 now\r\n");
        return;
      }

      axisManager.T0_T1_simultaneously_move_req = true;
      axisManager.T0_T1_target_pos = x_home_pos(!active_extruder);
      float L = axisManager.T0_T1_target_pos - inactive_extruder_x;
      float V = (float)parser.floatval('V', (float)200.0);
      float A = (float)parser.floatval('A', (float)6000.0);

      int32_t target_steps = (!active_extruder) == 0 ? axisManager.X0_home_step_pos : axisManager.X1_home_step_pos;
      axisManager.T0_T1_calc_steps = target_steps - axisManager.inactive_x_step_pos;
      int32_t float_d_to_step_d = L * planner.settings.axis_steps_per_mm[X_AXIS];
      if (abs(float_d_to_step_d - axisManager.T0_T1_calc_steps) > 5) {
        // LOG_E("Differ of float_d_to_step_d and axisManager.T0_T1_calc_steps too large, the inavtive X may has been move unexpected\r\n");
        axisManager.T0_T1_calc_steps = L * planner.settings.axis_steps_per_mm[X_AXIS];
      }

      // LOG_I("### M2000 S200: target_step_pos %d, current X step pos\r\n", target_steps, axisManager.inactive_x_step_pos);
      // LOG_I("### M2000 S200: axisManager.T0_T1_calc_steps = %d, run float distance %f\r\n", axisManager.T0_T1_calc_steps, L);
      if (0 == axisManager.T0_T1_calc_steps){
        axisManager.T0_T1_simultaneously_move_req = false;
        // LOG_I("Cancel M2000 S200\r\n");
        return;
      }

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

      if (accelDistance > 0) {
        move.accelerate = acceleration;
        move.t = accelClocks;
        move.end_t = move.start_t + move.t;
        move.start_pos[T0_T1_AXIS_INDEX] = axisManager.axis_t0_t1.func_manager.last_pos;
        move.end_pos[T0_T1_AXIS_INDEX] = move.start_pos[T0_T1_AXIS_INDEX] + accelDistance * move.axis_r[T0_T1_AXIS_INDEX];
        axisManager.axis_t0_t1.generateLineFuncParams(&move);
      }
      if (plateau > 0.0) {
        move.accelerate = 0;
        move.start_t = move.end_t;
        move.t = plateauClocks;
        move.end_t = move.start_t + move.t;
        move.start_pos[T0_T1_AXIS_INDEX] = move.end_pos[T0_T1_AXIS_INDEX];
        move.end_pos[T0_T1_AXIS_INDEX] = move.start_pos[T0_T1_AXIS_INDEX] + plateau * move.axis_r[T0_T1_AXIS_INDEX];
        axisManager.axis_t0_t1.generateLineFuncParams(&move);
      }
      if (decelDistance > 0) {
        move.accelerate = -acceleration;
        move.start_t = move.end_t;
        move.t = decelClocks;
        move.end_t = move.start_t + move.t;
        move.start_pos[T0_T1_AXIS_INDEX] = move.end_pos[T0_T1_AXIS_INDEX];
        move.end_pos[T0_T1_AXIS_INDEX] = move.start_pos[T0_T1_AXIS_INDEX] + decelDistance * move.axis_r[T0_T1_AXIS_INDEX];
        axisManager.axis_t0_t1.generateLineFuncParams(&move);
      }

      axisManager.T0_T1_execute_steps = 0;
      axisManager.T0_T1_axis = !active_extruder;
      inactive_extruder_x = axisManager.T0_T1_target_pos;
      axisManager.T0_T1_last_print_time = 0;
      axisManager.axis_t0_t1.is_consumed = true;
      axisManager.T0_T1_simultaneously_move = true;
      axisManager.T0_T1_simultaneously_move_req = false;
    }
    break;

    case 50:
    {
      switch_detect.debug_probe_poweron_sw = parser.byteval('F', 0);
    }
    break;

    default:
      break;
  }

  // uint8_t z = (uint8_t)parser.byteval('Z', (uint8_t)0xFF);
  // if (z != 0xFF) {
  //   motion_control.move_to_z((float)z, 600);
  // }
}

