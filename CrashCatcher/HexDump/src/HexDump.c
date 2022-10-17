/* Copyright (C) 2018  Adam Green (https://github.com/adamgreen)

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
#include <stdint.h>
#include <assert.h>
#include <CrashCatcher.h>
#include <stdio.h>
// #include "../../../Marlin/Configuration.h"
#include "../../../snapmaker/lib/GD32F1/libraries/FreeRTOS1030/MapleFreeRTOS1030.h"


CRASH_CATCHER_TEST_WRITEABLE CrashCatcherReturnCodes g_crashCatcherDumpEndReturn = CRASH_CATCHER_EXIT;
static                       CrashCatcherInfo        g_info;
static CrashCatcherMemoryRegion mri_region[2];

static void printString(const char* pString);
static void waitForUserInput(void);
static void dumpBytes(const uint8_t* pMemory, size_t elementCount);
static void dumpByteAsHex(uint8_t byte);
static void dumpHexDigit(uint8_t nibble);
static void dumpHalfwords(const uint16_t* pMemory, size_t elementCount);
static void dumpWords(const uint32_t* pMemory, size_t elementCount);

extern void CrashCatcher_io_init(void);
extern void CrashCatcher_io_done(void);

void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo)
{
    g_info = *pInfo;

    CrashCatcher_io_init();

    printString("\r\n\r\n");
    if (pInfo->isBKPT)
        printString("BREAKPOINT");
    else
        printString("CRASH");
    printString(" ENCOUNTERED\r\n"
                 "Enable logging and then press any key to start dump.\r\n");

    waitForUserInput();
    printString("\r\n");
}

/* Called to obtain an array of regions in memory that should be dumped as part of the crash.  This will typically
   be all RAM regions that contain volatile data.  For some crash scenarios, a user may decide to also add peripheral
   registers of interest (ie. dump some ethernet registers when you are encountering crashes in the network stack.)
   If NULL is returned from this function, the core will only dump the registers. */
const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void) {

  return NULL;

  // TaskHandle_t ct = xTaskGetCurrentTaskHandle();
  // mri_region[0].startAddress  = *(uint32_t *)(ct);
  // mri_region[0].endAddress    = mri_region[0].startAddress + 2048;
  // mri_region[0].elementSize   = CRASH_CATCHER_BYTE;

  // mri_region[1].startAddress  = 0xFFFFFFFF;
  // mri_region[1].endAddress    = 0xFFFFFFFF;

  // return mri_region;
}

static void printString(const char* pString)
{
    while (*pString)
        CrashCatcher_putc(*pString++);
}

static void waitForUserInput(void)
{
    CrashCatcher_getc();
}

void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount)
{
    switch (elementSize)
    {
    case CRASH_CATCHER_BYTE:
        dumpBytes(pvMemory, elementCount);
        break;
    case CRASH_CATCHER_HALFWORD:
        dumpHalfwords(pvMemory, elementCount);
        break;
    case CRASH_CATCHER_WORD:
        dumpWords(pvMemory, elementCount);
        break;
    }
    printString("\r\n");
}

static void dumpBytes(const uint8_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        /* Only dump 16 bytes to a single line before introducing a line break. */
        if (i != 0 && (i & 0xF) == 0)
            printString("\r\n");
        dumpByteAsHex(*pMemory++);
    }
}

static void dumpByteAsHex(uint8_t byte)
{
    dumpHexDigit(byte >> 4);
    dumpHexDigit(byte & 0xF);
}

static void dumpHexDigit(uint8_t nibble)
{
    static const char hexToASCII[] = "0123456789ABCDEF";

    assert( nibble < 16 );
    CrashCatcher_putc(hexToASCII[nibble]);
}

static void dumpHalfwords(const uint16_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        uint16_t val = *pMemory++;
        /* Only dump 8 halfwords to a single line before introducing a line break. */
        if (i != 0 && (i & 0x7) == 0)
            printString("\r\n");
        dumpBytes((uint8_t*)&val, sizeof(val));
    }
}

static void dumpWords(const uint32_t* pMemory, size_t elementCount)
{
    size_t i;
    for (i = 0 ; i < elementCount ; i++)
    {
        uint32_t val = *pMemory++;
        /* Only dump 4 words to a single line before introducing a line break. */
        if (i != 0 && (i & 0x3) == 0)
            printString("\r\n");
        dumpBytes((uint8_t*)&val, sizeof(val));
    }
}


CrashCatcherReturnCodes CrashCatcher_DumpEnd(void)
{
    printString("\r\nEnd of dump\r\n");
    CrashCatcher_io_done();
    if (g_crashCatcherDumpEndReturn == CRASH_CATCHER_TRY_AGAIN && g_info.isBKPT)
        return CRASH_CATCHER_EXIT;
    else
        return g_crashCatcherDumpEndReturn;
}
