/* The CPU accesses these sections through the L1-bypass alias of RAM. `.uncached.data` is
   copied to the start of RAM, which is the same physical memory, and `.uncached.hole` reserves
   the RAM both sections use. */

SECTIONS {
  .uncached.data ORIGIN(UNCACHE) : AT(_direct_boot_ram_lma)
  {
    *(.uncached.data .uncached.data.*)
    . = ALIGN(4);
    _uncached_data_end = ABSOLUTE(.);
  } > UNCACHE

  .uncached.bss (NOLOAD) : AT(ADDR(.uncached.bss)) ALIGN(4)
  {
    _uncached_bss_start = ABSOLUTE(.);
    *(.uncached.bss .uncached.bss.*)
    . = ALIGN(4);
    _uncached_bss_end = ABSOLUTE(.);
  } > UNCACHE
}

/* lld gives an empty section the flags of the previous output section. That section can be a
   non-allocated section, for example `.defmt`. Then the empty section has address 0. Do not use
   the addresses of empty sections. */
_uncached_data_start = ORIGIN(UNCACHE);
_uncached_end = ORIGIN(UNCACHE) + SIZEOF(.uncached.data) + SIZEOF(.uncached.bss);

SECTIONS {
  /* Round the uncached sections up to the 64-byte L1 cache line. */
  .uncached.hole (NOLOAD) : AT(ADDR(.uncached.hole))
  {
    . = . + ALIGN(_uncached_end - _uncached_data_start, 64);
  } > RAM
}

/* Copied through the cached alias. `early_init` writes the copied ranges back to L2MEM. */
_direct_boot_uncached_data_src = DROM_BASE + LOADADDR(.uncached.data);
_direct_boot_uncached_data_dst = ORIGIN(RAM);
_direct_boot_uncached_data_end = ORIGIN(RAM) + SIZEOF(.uncached.data);

_direct_boot_rwtext_lma = ALIGN(LOADADDR(.uncached.data) + SIZEOF(.uncached.data), 4);

ASSERT(SIZEOF(.uncached.hole) == 0 || ADDR(.uncached.hole) == ORIGIN(RAM), "
ERROR(esp-bootloader-direct-boot): .uncached.hole must start at the origin of RAM");
