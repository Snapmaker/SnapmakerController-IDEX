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

 #include "../../../inc/MarlinConfig.h"

 #include "../../gcode.h"
 #include "../../snapmaker/module/filament_sensor.h"  
 #include "../../../module/settings.h"
 
 /**
  * M412: Enable / Disable filament runout detection
  *
  * Parameters
  *  R         : Reset the runout sensor
  *  S<bool>   : Reset and enable/disable the runout sensor
  *  H<bool>   : Enable/disable host handling of filament runout
  *  D<linear> : Extra distance to continue after runout is triggered
  */
 void GcodeSuite::M412() {
 
   bool need_save = false;
 
   if (parser.seenval('S')) {
     bool enable = parser.value_bool();
     uint8_t e = 0;
     if (parser.seenval('T')) {
       e = parser.value_bool();
     }
     filament_sensor.filament_param.enabled[e] = enable;
     need_save = true;
   }
 
   if (parser.seenval('D')) {
     uint8_t d = parser.value_byte();
     if (d < 1) {
       d = 1;
     }
     filament_sensor.filament_param.distance = d;
     need_save = true;
   }
 
   if (parser.seen('R') || need_save) {
     if (need_save)
       (void)settings.save();
     filament_sensor.reset();
   }
 
   SERIAL_ECHOLNPAIR("filament check enable T[0]:", filament_sensor.filament_param.enabled[0],
                     ", T[1]:", filament_sensor.filament_param.enabled[1]);
   SERIAL_ECHOLNPAIR("filament check param - diatance:", filament_sensor.filament_param.distance);
   SERIAL_ECHOLNPAIR("filament sensor value T[0]:", filament_sensor.get_adc_val(0),
                     ", T[1]:", filament_sensor.get_adc_val(1));
 }