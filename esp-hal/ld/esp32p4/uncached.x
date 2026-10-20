/* The CPU accesses these sections through the L1-bypass alias of RAM.
   `.uncached.hole` in esp32p4.x reserves the same physical memory in RAM. */

SECTIONS {
  .uncached.data : ALIGN(4) {
    _uncached_data_start = ABSOLUTE(.);
    *(.uncached.data .uncached.data.*)
    . = ALIGN(4);
    _uncached_data_end = ABSOLUTE(.);
  } > UNCACHE

  .uncached.bss (NOLOAD) : ALIGN(4) {
    _uncached_bss_start = ABSOLUTE(.);
    *(.uncached.bss .uncached.bss.*)
    . = ALIGN(4);
    _uncached_bss_end = ABSOLUTE(.);
  } > UNCACHE
}

ASSERT(_uncached_data_start == ORIGIN(UNCACHE), "
ERROR(esp-hal): .uncached.data must start at the origin of UNCACHE");
