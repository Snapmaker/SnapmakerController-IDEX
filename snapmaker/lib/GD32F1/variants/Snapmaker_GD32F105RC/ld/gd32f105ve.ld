/*
 * libmaple linker script for "Flash" builds.
 *
 * A Flash build puts .text (and .rodata) in Flash, and
 * .data/.bss/heap (of course) in SRAM, but offsets the sections by
 * enough space to store the Maple bootloader, which lives in low
 * Flash and uses low memory.
 */

/*
 * This pulls in the appropriate MEMORY declaration from the right
 * subdirectory of stm32/mem/ (the environment must call ld with the
 * right include directory flags to make this happen). Boards can also
 * use this file to use any of libmaple's memory-related hooks (like
 * where the heap should live).
 */
MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
  /* rom (rx)  : ORIGIN = 0x08002800, LENGTH = 488K-46K */
  rom_main (rx)  : ORIGIN = 0x08002800, LENGTH = 256-10K
  rom_last (rx)  : ORIGIN = 0x08061000, LENGTH = 64K
}

/* Provide memory region aliases for common.inc */
REGION_ALIAS("REGION_TEXT", rom_main);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_BSS", ram);
REGION_ALIAS("REGION_RODATA", rom_main);
REGION_ALIAS("REGION_LAST", rom_last);

/* Let common.inc handle the real work. */
INCLUDE common.inc
