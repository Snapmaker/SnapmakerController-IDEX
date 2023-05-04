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

#include "src/inc/MarlinConfig.h"
#include "src/gcode/gcode.h"
#include "src/MarlinCore.h"
#include HAL_PATH(src/HAL, HAL_watchdog_STM32F1.h)


void GcodeSuite::M1999() {
  if (parser.seen("S")) {
    SERIAL_ECHOLN("will reboot screen");
    OUT_WRITE(SCREEN_PWR_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
    OUT_WRITE(SCREEN_PWR_PIN, HIGH);
  } else if (parser.seen("C")) {
    SERIAL_ECHOLN("will reboot machine");
    nvic_sys_reset();
  } else {
    SERIAL_ECHOLN("will reboot machine and screen");
    OUT_WRITE(SCREEN_PWR_PIN, LOW);
    nvic_sys_reset();
  }
}
