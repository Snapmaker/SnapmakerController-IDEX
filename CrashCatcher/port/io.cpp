/* Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <CrashCatcher.h>
#include <stdio.h>
#include "../../Marlin/src/core/serial.h"
#include <EEPROM.h>

uint8_t w_data[4];
uint32_t w_count;

extern "C" {
  void fault_protect_action(void) {
    // turn off hotends
    // #define HEATER_0_PIN       PE14   // EXTRUDER 1
    // #define HEATER_1_PIN       PE13   // EXTRUDER 2
    // #define HEATER_BED_PIN     PA10   // BED
    // thermalManager.setTargetHotend(0, 0);
    // thermalManager.setTargetHotend(0, 1);
    // thermalManager.setTargetBed(0)
    WRITE(HEATER_PWR_PIN, LOW);
    WRITE(PE14, 0);
    WRITE(PE13, 0);
    WRITE(PA10, 0);
  }

  void CrashCatcher_io_init(void) {
  fault_protect_action();
  w_count = 0;
  FLASH_Unlock();
  FLASH_ErasePage(CRASH_DATA_FLASH_ADDR);
  FLASH_ErasePage(CRASH_DATA_FLASH_ADDR + APP_FLASH_PAGE_SIZE);
  FLASH_Lock();
  SERIAL_IMPL.begin(115200);
  }

  void CrashCatcher_io_done(void) {
  }
}


/* The following functions must be provided by a hex dumping implementation. Such implementations will also have to
   implement the core CrashCatcher_GetMemoryRegions() API as well.  The HexDump version of CrashCatcher calls these
   routines to have an implementation query the user when they are ready for the dump to start and actually dump the
   hex data to the user a character at a time. */

/* Called to receive a character of data from the user.  Typically this is in response to a "Press any key" type of
   prompt to the user.  This function should be blocking. */
int CrashCatcher_getc(void) {
  return -1;
}

/* Called to send a character of hex dump data to the user. */
void CrashCatcher_putc(int c) {
  SERIAL_IMPL.write_byte_direct((uint8_t)c);

  w_data[w_count%4] = c;
  w_count++;
  if (w_count%4 == 0 && w_count < CRASH_DATA_SIZE) {
    uint32_t data = *(uint32_t *)w_data;
    FLASH_Unlock();
    FLASH_ProgramWord(CRASH_DATA_FLASH_ADDR + w_count, data);
    FLASH_Lock();
  }
}
