/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 *
 */

/**
 * M101 Task Free Memory Watcher
 *
 * This code watches the free memory defined for each task in the FreeRTOS environment
 * It is useful to determine how much memory each task has available.
 *
 * Initial version...  More Marlin-specific information to be added.
 */


#include "MapleFreeRTOS1030.h"
#include "src/gcode/gcode.h"
#include "../../debug/debug.h"

#define MAX_TASKS 12

void GcodeSuite::M101() {
  TaskStatus_t TaskStatArray[MAX_TASKS];

  unsigned n_tasks = uxTaskGetNumberOfTasks();
  if (n_tasks > MAX_TASKS) {
    LOG_I("?Too many tasks: %u\n", n_tasks);
    SERIAL_EOL();
    return;
  }

  LOG_I("M101 RTOS Task Info:\n");
  /* Generate raw status information about each task. */
  n_tasks = uxTaskGetSystemState( TaskStatArray, MAX_TASKS, NULL);

  LOG_I("Free Heap: %u Bytes\n", xPortGetFreeHeapSize());

  LOG_I("n_tasks: %u\n", n_tasks);

  /* For each populated position in the TaskStatArray array,
     format the raw data as human readable ASCII data. */
  for (unsigned x = 0; x < n_tasks; x++) {
    char t_name[configMAX_TASK_NAME_LEN + 1];         // Pad out the task name so everything lines up nicely
    strcpy(t_name, TaskStatArray[x].pcTaskName);
    while (strlen(t_name) < configMAX_TASK_NAME_LEN)
      strcat(t_name, "_");

    SERIAL_ECHO(x);
    LOG_I(": %s", t_name);
    LOG_I(" Task #: %d", TaskStatArray[x].xTaskNumber);
    LOG_I(" Current_Priority: %u", TaskStatArray[x].uxCurrentPriority);
    LOG_I(" Base_Priority: %u", TaskStatArray[x].uxBasePriority);

    LOG_I(" Stack: 0x%08x ", TaskStatArray[x].pxStackBase);
    LOG_I(" Free_Mem: %u Bytes", (unsigned int) TaskStatArray[x].usStackHighWaterMark * sizeof( StackType_t ));
    LOG_I(" State: ");
    switch( TaskStatArray[x].eCurrentState ) {
      case eRunning:   LOG_I("Running\n");   break;
      case eReady:     LOG_I("Ready\n");     break;
      case eBlocked:   LOG_I("Blocked\n");   break;
      case eSuspended: LOG_I("Suspended\n"); break;
      case eDeleted:   LOG_I("Deleted\n");   break;
      default:         LOG_I("Corrupted: %u\n", TaskStatArray[x].eCurrentState);
        break;
    }
  }
}
