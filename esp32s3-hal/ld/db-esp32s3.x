/* before memory.x to allow override */
ENTRY(Reset)

INCLUDE memory.x

/* map generic regions to output sections */
INCLUDE "alias.x"

_external_ram_start = ABSOLUTE(ORIGIN(psram_seg));
_external_ram_end = ABSOLUTE(ORIGIN(psram_seg)+LENGTH(psram_seg));

_heap_end = ABSOLUTE(ORIGIN(dram_seg))+LENGTH(dram_seg)+LENGTH(reserved_for_boot_seg) - 2*STACK_SIZE;
_text_heap_end = ABSOLUTE(ORIGIN(iram_seg)+LENGTH(iram_seg));
_external_heap_end = ABSOLUTE(ORIGIN(psram_seg)+LENGTH(psram_seg));

_stack_start_cpu1 = _heap_end;
_stack_end_cpu1 = _stack_start_cpu1 + STACK_SIZE;
_stack_start_cpu0 = _stack_end_cpu1;
_stack_end_cpu0 = _stack_start_cpu0 + STACK_SIZE;

EXTERN(DefaultHandler);

INCLUDE "device.x"

/* after memory.x to allow override */
PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);

/*INCLUDE exception.x*/
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
EXTERN(__default_naked_exception);
EXTERN(__default_naked_double_exception);
EXTERN(__default_naked_level_2_interrupt);
EXTERN(__default_naked_level_3_interrupt);
EXTERN(__default_naked_level_4_interrupt);
EXTERN(__default_naked_level_5_interrupt);
EXTERN(__default_naked_level_6_interrupt);
EXTERN(__default_naked_level_7_interrupt);

SECTIONS {
  .pre_header (NOLOAD) : AT(0)
  {
    . = . + 0x400;
  }

  .header ORIGIN(ROTEXT) : AT(0x400)
  {
    LONG(0xaedb041d)
    LONG(0xaedb041d)
  }

  .text ORIGIN(ROTEXT) + 0x408 : AT(0x408)
  {
    _stext = .;
    . = ALIGN (4);
    _text_start = ABSOLUTE(.);
    . = ALIGN (4);
    KEEP(*(.init));
    *(.literal .text .literal.* .text.*)
    . = ALIGN (4);
    _text_end = ABSOLUTE(.);
    _etext = .;
  }
  _text_size = _etext - _stext;

  .rodata ORIGIN(RODATA) + 0x408 + _text_size : AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header))
  {
    _rodata_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.rodata .rodata.*)
    . = ALIGN (4);
    _rodata_end = ABSOLUTE(.);
  }

  .rwtext ORIGIN(RWTEXT) + 0x408 + _text_size + SIZEOF(.rodata) : 
      AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata))
  {
    _irwtext = ORIGIN(RODATA) + 0x408 + _text_size + SIZEOF(.rodata);
    _srwtext = .;

    . = ALIGN (4);

    . = ALIGN(0x1000);
    _init_start = ABSOLUTE(.);
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
    . = ALIGN(0x400);

    _init_end = ABSOLUTE(.);

    *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)

    . = ALIGN (4);
    _erwtext = .;
  }

  .data ORIGIN(RWDATA) : 
      AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext))
  {
    _data_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.data .data.*)
    . = ALIGN (4);
    _data_end = ABSOLUTE(.);
  }
 

  /* LMA of .data */
  _sidata = ORIGIN(RODATA) + _text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext);

  .bss (NOLOAD) : ALIGN(4)
  {
    _bss_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.bss .bss.* COMMON)
    . = ALIGN (4);
    _bss_end = ABSOLUTE(.);
  } > RWDATA

  .noinit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    *(.noinit .noinit.*)
    . = ALIGN (4);
  } > RWDATA

  .rtc_fast.text ORIGIN(rtc_fast_iram_seg) : 
      AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) )
  {
   . = ALIGN(4);
   _rtc_fast_text_start = ABSOLUTE(.);
    *(.rtc_fast.literal .rtc_fast.text .rtc_fast.literal.* .rtc_fast.text.*)
   . = ALIGN(4);
   _rtc_fast_text_end = ABSOLUTE(.);
  }
  _irtc_fast_text = ORIGIN(RODATA) + _text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext);
 
  .rtc_fast.data ORIGIN(rtc_fast_dram_seg) + SIZEOF(.rtc_fast.text) : 
      AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) + SIZEOF(.rtc_fast.text) )
  {
    . = ALIGN(4);
    _rtc_fast_data_start = ABSOLUTE(.);
    *(.rtc_fast.data .rtc_fast.data.*)
   . = ALIGN(4);
    _rtc_fast_data_end = ABSOLUTE(.);
  }
  _irtc_fast_data = ORIGIN(RODATA) + _text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) + SIZEOF(.rtc_fast.text);

 .rtc_fast.bss ORIGIN(rtc_fast_dram_seg) + SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data) (NOLOAD) : 
    AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + 
      SIZEOF(.rwtext) + SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data))
  {
    . = ALIGN(4);
    _rtc_fast_bss_start = ABSOLUTE(.);
    *(.rtc_fast.bss .rtc_fast.bss.*)
    . = ALIGN (4);
    _rtc_fast_bss_end = ABSOLUTE(.);
  }

 .rtc_fast.noinit ORIGIN(rtc_fast_dram_seg) + SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data) + SIZEOF(.rtc_fast.bss) (NOLOAD)  :
  {
    . = ALIGN(4);
    *(.rtc_fast.noinit .rtc_fast.noinit.*)
    . = ALIGN (4);
  }

 .rtc_slow.text ORIGIN(rtc_slow_seg) : 
    AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) + 
      SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data) + SIZEOF(.rtc_fast.bss)) 
  {
   . = ALIGN(4);
   _rtc_slow_text_start = ABSOLUTE(.);
    *(.rtc_slow.literal .rtc_slow.text .rtc_slow.literal.* .rtc_slow.text.*)
   . = ALIGN(4);
   _rtc_slow_text_end = ABSOLUTE(.);
  }
  _irtc_slow_text = ORIGIN(RODATA) + _text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) + 
      SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data) + SIZEOF(.rtc_fast.bss);

  .rtc_slow.data ORIGIN(rtc_slow_seg) + SIZEOF(.rtc_slow.text) : 
      AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) + 
        SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data) + SIZEOF(.rtc_fast.bss) + SIZEOF(.rtc_slow.text))
  {
    . = ALIGN(4);
    _rtc_slow_data_start = ABSOLUTE(.);
    *(.rtc_slow.data .rtc_slow.data.*)
    . = ALIGN(4);
    _rtc_slow_data_end = ABSOLUTE(.);
  }
  _irtc_slow_data = ORIGIN(RODATA) + _text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) + 
        SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data) + SIZEOF(.rtc_fast.bss) + SIZEOF(.rtc_slow.text);

 .rtc_slow.bss ORIGIN(rtc_slow_seg) + SIZEOF(.rtc_slow.text) + SIZEOF(.rtc_slow.data) (NOLOAD) : 
    AT(_text_size + SIZEOF(.header) + SIZEOF(.pre_header) + SIZEOF(.rodata) + SIZEOF(.rwtext) + 
      SIZEOF(.rtc_fast.text) + SIZEOF(.rtc_fast.data) + SIZEOF(.rtc_fast.bss) + SIZEOF(.rtc_slow.text) + SIZEOF(.rtc_slow.data))
  {
    . = ALIGN(4);
    _rtc_slow_bss_start = ABSOLUTE(.);
    *(.rtc_slow.bss .rtc_slow.bss.*)
    . = ALIGN (4);
    _rtc_slow_bss_end = ABSOLUTE(.);
  }

 .rtc_slow.noinit ORIGIN(rtc_slow_seg) + SIZEOF(.rtc_slow.text) + SIZEOF(.rtc_slow.data) + SIZEOF(.rtc_slow.bss) (NOLOAD) :
  {
    . = ALIGN(4);
    *(.rtc_slow.noinit .rtc_slow.noinit.*)
    . = ALIGN (4);
  }

 .external.data :
  {
    _external_data_start = ABSOLUTE(.);
    . = ALIGN(4);
    *(.external.data .external.data.*)
    . = ALIGN (4);
    _external_data_end = ABSOLUTE(.);
  } > psram_seg AT > RODATA

 .external.bss (NOLOAD) :
  {
    _external_bss_start = ABSOLUTE(.);
    . = ALIGN(4);
    *(.external.bss .external.bss.*)
    . = ALIGN (4);
    _external_bss_end = ABSOLUTE(.);
  } > psram_seg

 .external.noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.external.noinit .external.noinit.*)
    . = ALIGN (4);
  } > psram_seg

  /* must be last segment using psram_seg */
  .external_heap_start (NOLOAD) :
  {
    . = ALIGN (4);
    _external_heap_start = ABSOLUTE(.);
    . = ALIGN (4);
  } > psram_seg 
}
