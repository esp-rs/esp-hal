#IF ESP_HAL_CONFIG_FLIP_LINK
/* no Xtensa chip is supported - so we can assume RISC-V */
SECTIONS {
  /* must be last segment using RWDATA */
  .stack (NOLOAD) : ALIGN(4)
  {
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);

    /* Since we cannot know how much the alignment padding of the sections will add we shrink the stack for "the worst case"
    */
    . = . + LENGTH(RWDATA) -  (SIZEOF(.trap) + SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi) + SIZEOF(.data) + SIZEOF(.bss) + SIZEOF(.noinit) + SIZEOF(.data.wifi)) - 304;

    . = ALIGN (4);
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA
}
INSERT BEFORE .trap;

#ELSE

SECTIONS {
  /* must be last segment using RWDATA */
  .stack (NOLOAD) : ALIGN(4)
  {
    . = ALIGN (4);
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);

    . = ORIGIN(RWDATA) + LENGTH(RWDATA);

    . = ALIGN (4);
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA
}

#ENDIF

/*
Provide the stack_guard for `stack-protector`

TODO: Ideally the offset should be configurable - should be done once we have https://github.com/esp-rs/esp-hal/issues/1111
*/
PROVIDE(__stack_chk_guard = _stack_end + 4096);
