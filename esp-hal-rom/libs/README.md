The libs directory may contain `libesp_rom.a` (or any other static library) meant to patch ROM functions.

Remember the linker scripts usually define the ROM functions weakly which means we can patch them.
