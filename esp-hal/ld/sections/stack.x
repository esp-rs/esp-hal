SECTIONS {
  /* must be last segment using RWDATA */
  .stack (NOLOAD) : ALIGN(4)
  {
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);

    /* The stack_guard for `stack-protector` mitigation - https://doc.rust-lang.org/rustc/exploit-mitigations.html#stack-smashing-protection */
    __stack_chk_guard = ABSOLUTE(_stack_end) + ${ESP_HAL_CONFIG_STACK_GUARD_OFFSET};

/* no Xtensa chip is supported - so we can assume RISC-V */
#IF ESP_HAL_CONFIG_FLIP_LINK
    /* Since we cannot know how much the alignment padding of the sections will add we shrink the stack for "the worst case"
    */
    . = . + LENGTH(RWDATA) -  (SIZEOF(.trap) + SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi) + SIZEOF(.data) + SIZEOF(.bss) + SIZEOF(.noinit) + SIZEOF(.data.wifi)) - 304;
#ELSE
    . = ORIGIN(RWDATA) + LENGTH(RWDATA);
#ENDIF
    . = ALIGN (4);
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA
}
