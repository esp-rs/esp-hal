SECTIONS {
  /* must be last segment using RWDATA */
  .stack (NOLOAD) : ALIGN(4)
  {
    _stack_end = ABSOLUTE(.);
    _stack_end_cpu0 = ABSOLUTE(.);

    /* The stack_guard for `stack-protector` mitigation - https://doc.rust-lang.org/rustc/exploit-mitigations.html#stack-smashing-protection */
    __stack_chk_guard = ABSOLUTE(_stack_end) + ${ESP_HAL_CONFIG_STACK_GUARD_OFFSET};

    . = ORIGIN(RWDATA) + LENGTH(RWDATA);

    . = ALIGN (4);
    _stack_start = ABSOLUTE(.);
    _stack_start_cpu0 = ABSOLUTE(.);
  } > RWDATA
}
