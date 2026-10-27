/* The template of picolibc's thread-local variables: their initial values in .tdata, followed by
   the zeroed .tbss. The `tp` register points to a copy of the template, see src/speech.rs. */
SECTIONS {
  .tdata : ALIGN(4) {
    _tdata_start = .;
    *(.tdata .tdata.*)
    _tdata_end = .;
  } > RODATA

  .tbss : ALIGN(4) {
    *(.tbss .tbss.*)
    _tbss_end = .;
  } > RODATA
}
