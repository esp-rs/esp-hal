SECTIONS {
  /* must be last segment using RWDATA */
  .stack (NOLOAD) : ALIGN(4)
  {
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);

    . = ORIGIN(RWDATA) + LENGTH(RWDATA);

    . = ALIGN (4);
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA
}
