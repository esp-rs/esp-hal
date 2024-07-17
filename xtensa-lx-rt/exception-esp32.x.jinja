/* exception vector for the ESP32, requiring high priority interrupts and register window support */

/* high level exception/interrupt routines, which can be override with Rust functions */
PROVIDE(__exception = __default_exception);
PROVIDE(__user_exception = __default_user_exception);
PROVIDE(__double_exception = __default_double_exception);
PROVIDE(__level_1_interrupt = __default_interrupt);
PROVIDE(__level_2_interrupt = __default_interrupt);
PROVIDE(__level_3_interrupt = __default_interrupt);
PROVIDE(__level_4_interrupt = __default_interrupt);
PROVIDE(__level_5_interrupt = __default_interrupt);
PROVIDE(__level_6_interrupt = __default_interrupt);
PROVIDE(__level_7_interrupt = __default_interrupt);

/* high level CPU interrupts */
PROVIDE(Timer0 = __default_user_exception);
PROVIDE(Timer1 = __default_user_exception);
PROVIDE(Timer2 = __default_user_exception);
PROVIDE(Timer3 = __default_user_exception);
PROVIDE(Profiling = __default_user_exception);
PROVIDE(NMI = __default_user_exception);
PROVIDE(Software0 = __default_user_exception);
PROVIDE(Software1 = __default_user_exception);

/* low level exception/interrupt, which must be overridden using naked functions */
PROVIDE(__naked_user_exception = __default_naked_exception);
PROVIDE(__naked_kernel_exception = __default_naked_exception);
PROVIDE(__naked_double_exception = __default_naked_double_exception);
PROVIDE(__naked_level_2_interrupt = __default_naked_level_2_interrupt);
PROVIDE(__naked_level_3_interrupt = __default_naked_level_3_interrupt);
PROVIDE(__naked_level_4_interrupt = __default_naked_level_4_interrupt);
PROVIDE(__naked_level_5_interrupt = __default_naked_level_5_interrupt);
PROVIDE(__naked_level_6_interrupt = __default_naked_level_6_interrupt);
PROVIDE(__naked_level_7_interrupt = __default_naked_level_7_interrupt);


/* needed to force inclusion of the vectors */
EXTERN(__default_exception);
EXTERN(__default_double_exception);
EXTERN(__default_interrupt);

EXTERN(__default_naked_exception);
EXTERN(__default_naked_double_exception);
EXTERN(__default_naked_level_2_interrupt);
EXTERN(__default_naked_level_3_interrupt);
EXTERN(__default_naked_level_4_interrupt);
EXTERN(__default_naked_level_5_interrupt);
EXTERN(__default_naked_level_6_interrupt);
EXTERN(__default_naked_level_7_interrupt);


/* Define output sections */
SECTIONS {

  .vectors :
  {
    /* 
      Each vector has 64 bytes that it must fit inside. For each vector we calculate the size of the previous one, 
      and subtract that from 64 and start the new vector there.
    */
    _init_start = ABSOLUTE(.);
    . = ALIGN(64);
    KEEP(*(.WindowOverflow4.text));
    . = ALIGN(64);
    KEEP(*(.WindowUnderflow4.text));
    . = ALIGN(64);
    KEEP(*(.WindowOverflow8.text));
    . = ALIGN(64);
    KEEP(*(.WindowUnderflow8.text));
    . = ALIGN(64);
    KEEP(*(.WindowOverflow12.text));
    . = ALIGN(64);
    KEEP(*(.WindowUnderflow12.text));
    . = ALIGN(64);
    KEEP(*(.Level2InterruptVector.text));
    . = ALIGN(64);
    KEEP(*(.Level3InterruptVector.text));
    . = ALIGN(64);
    KEEP(*(.Level4InterruptVector.text));
    . = ALIGN(64);
    KEEP(*(.Level5InterruptVector.text));
    . = ALIGN(64);
    KEEP(*(.DebugExceptionVector.text));
    . = ALIGN(64);
    KEEP(*(.NMIExceptionVector.text));
    . = ALIGN(64);
    KEEP(*(.KernelExceptionVector.text));
    . = ALIGN(64);
    KEEP(*(.UserExceptionVector.text));
    . = ALIGN(128);
    KEEP(*(.DoubleExceptionVector.text));
    . = ALIGN(64);
    . = ALIGN(0x400);
    _init_end = ABSOLUTE(.);
  } > vectors_seg
}
