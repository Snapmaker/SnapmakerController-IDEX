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
#pragma once

#ifndef RUNOUT_H
#define RUNOUT_H

/**
 * feature/runout.h - Runout sensor support
 */

#include "../sd/cardreader.h"
#include "../module/printcounter.h"
#include "../module/planner.h"
#include "../module/stepper.h" // for block_t
#include "../gcode/queue.h"
#include "../feature/pause.h"
#include "../inc/MarlinConfig.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

#if HAS_FILAMENT_SENSOR

//#define FILAMENT_RUNOUT_SENSOR_DEBUG
#ifndef FILAMENT_RUNOUT_THRESHOLD
  #define FILAMENT_RUNOUT_THRESHOLD 5
#endif

class FilamentMonitorBase {
  public:
    static bool enabled, filament_ran_out;

    #if ENABLED(HOST_ACTION_COMMANDS)
      static bool host_handling;
    #else
      static constexpr bool host_handling = false;
    #endif
};

class FilamentMonitor : public FilamentMonitorBase {
  public:
    static void setup();
    static void reset();
    static void filament_present(const uint8_t extruder);
    static void runout_detected(const uint8_t extruder); // Fixed: Added extruder parameter
    static bool is_triggered(const uint8_t extruder) { return triggered[extruder]; }
    static void set_triggered(const uint8_t extruder, bool value) { triggered[extruder] = value; }

  private:
    static bool triggered[NUM_RUNOUT_SENSORS]; // Track runout state per extruder
};

extern FilamentMonitor runout;

void event_filament_runout(const uint8_t extruder);

#endif // HAS_FILAMENT_SENSOR

#endif // RUNOUT_H