The libs directory may contain `libesp_rom.a` (or any other static library) meant to patch ROM functions.

Additionally you can have a `add_rwtext` file which adds to the `.rwtext` section.

Remember the linker scripts usually define the ROM functions weakly which means we can patch them.

Patches for all chips should originate from one ESP-IDF version only and that should be documented in the crate's README.md.
