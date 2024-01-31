

/* Do not exceed this mark in the error messages above                                    | */
ASSERT(ORIGIN(ROTEXT) % 4 == 0, "
ERROR(riscv-rt): the start of the ROTEXT must be 4-byte aligned");

ASSERT(ORIGIN(RODATA) % 4 == 0, "
ERROR(riscv-rt): the start of the RODATA must be 4-byte aligned");

ASSERT(ORIGIN(REGION_DATA) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_DATA must be 4-byte aligned");

ASSERT(ORIGIN(ROTEXT) % 4 == 0, "
ERROR(riscv-rt): the start of the ROTEXT must be 4-byte aligned");

ASSERT(ORIGIN(REGION_STACK) % 4 == 0, "
ERROR(riscv-rt): the start of the REGION_STACK must be 4-byte aligned");

ASSERT(_stext % 4 == 0, "
ERROR(riscv-rt): `_stext` must be 4-byte aligned");

ASSERT(_data_start % 4 == 0 && _data_end % 4 == 0, "
BUG(riscv-rt): .data is not 4-byte aligned");

ASSERT(_sidata % 4 == 0, "
BUG(riscv-rt): the LMA of .data is not 4-byte aligned");

ASSERT(_bss_start % 4 == 0 && _bss_end % 4 == 0, "
BUG(riscv-rt): .bss is not 4-byte aligned");

ASSERT(_stext + SIZEOF(.text) < ORIGIN(ROTEXT) + LENGTH(ROTEXT), "
ERROR(riscv-rt): The .text section must be placed inside the ROTEXT region.
Set _stext to an address smaller than 'ORIGIN(ROTEXT) + LENGTH(ROTEXT)'");

ASSERT(SIZEOF(.got) == 0, "
.got section detected in the input files. Dynamic relocations are not
supported. If you are linking to C code compiled using the `gcc` crate
then modify your build script to compile the C code _without_ the
-fPIC flag. See the documentation of the `gcc::Config.fpic` method for
details.");

/* Do not exceed this mark in the error messages above                                    | */