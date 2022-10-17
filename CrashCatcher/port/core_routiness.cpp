#include <CrashCatcher.h>

/* The following functions must be provided by a specific dumping implementation.  The Core CrashCatcher calls these
   routines to have an implementation dump the bytes associated with the current crash. */

/* Called at the beginning of crash dump. You should provide an implementation which prepares for the dump by opening
   a dump file, prompting the user to begin a crash dump, or whatever makes sense for your scenario. */
// void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo) {

// }

/* Called to obtain an array of regions in memory that should be dumped as part of the crash.  This will typically
   be all RAM regions that contain volatile data.  For some crash scenarios, a user may decide to also add peripheral
   registers of interest (ie. dump some ethernet registers when you are encountering crashes in the network stack.)
   If NULL is returned from this function, the core will only dump the registers. */
// const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void) {

// }

/* Called to dump the next chunk of memory to the dump (this memory may point to register contents which has been copied
   to memory by CrashCatcher already.  The element size will be 8-bits, 16-bits, or 32-bits.  The implementation should
   use reads of the specified size since some memory locations may only support the indicated size. */
// void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount) {

// }

/* Called at the end of crash dump. You should provide an implementation which cleans up at the end of dump. This could
   include closing a dump file, blinking LEDs, infinite looping, and/or returning CRASH_CATCHER_TRY_AGAIN if
   CrashCatcher should prepare to dump again incase user missed the first attempt. */
// CrashCatcherReturnCodes CrashCatcher_DumpEnd(void) {
//   return CRASH_CATCHER_EXIT;
// }
