/* Marks the end of `ROM`, for `.bss.psram` in `psram.x`. It must be included after every section
   in `ROM` and every section whose load address is in `ROM`. */
SECTIONS {
  .psram_code_end (NOLOAD) :
  {
    _psram_code_end = ABSOLUTE(.);
  } > ROM
}
