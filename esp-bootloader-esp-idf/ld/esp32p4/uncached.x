/* The CPU accesses these sections through the L1-bypass alias of RAM. The
   bootloader cannot load to the alias, so `.uncached.data` loads at the start
   of RAM, which is the same physical memory. `.uncached.hole` in esp32p4.x
   reserves the RAM that `.uncached.bss` uses. */

SECTIONS {
  .uncached.data : ALIGN(4) {
    *(.uncached.data .uncached.data.*)
    . = ALIGN(4);
    _uncached_data_end = ABSOLUTE(.);
  } > UNCACHE AT> RAM

  .uncached.bss (NOLOAD) : ALIGN(4) {
    _uncached_bss_start = ABSOLUTE(.);
    *(.uncached.bss .uncached.bss.*)
    . = ALIGN(4);
    _uncached_bss_end = ABSOLUTE(.);
  } > UNCACHE
}

/* lld gives an empty section the flags of the previous output section. That
   section can be a non-allocated section, for example `.defmt`. Then the empty
   section has address 0. Do not use the addresses of empty sections. */
_uncached_data_start = ORIGIN(UNCACHE);
_uncached_end = ORIGIN(UNCACHE) + SIZEOF(.uncached.data) + SIZEOF(.uncached.bss);

ASSERT(SIZEOF(.uncached.data) == 0 || ADDR(.uncached.data) == ORIGIN(UNCACHE), "
ERROR(esp-hal): .uncached.data must start at the origin of UNCACHE");

ASSERT(SIZEOF(.uncached.data) == 0 || LOADADDR(.uncached.data) == ORIGIN(RAM), "
ERROR(esp-hal): .uncached.data must load at the origin of RAM");

ASSERT(SIZEOF(.uncached.bss) == 0 || ADDR(.uncached.bss) == ORIGIN(UNCACHE) + SIZEOF(.uncached.data), "
ERROR(esp-hal): .uncached.bss must follow .uncached.data");
