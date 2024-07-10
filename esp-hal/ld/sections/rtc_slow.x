

SECTIONS {
  .rtc_slow.text : {
   . = ALIGN(4);
   *(.rtc_slow.literal .rtc_slow.text .rtc_slow.literal.* .rtc_slow.text.*)
   . = ALIGN(4);
  } > rtc_slow_seg AT > RODATA

  .rtc_slow.data :
  {
    . = ALIGN(4);
    _rtc_slow_data_start = ABSOLUTE(.);
    *(.rtc_slow.data .rtc_slow.data.*)
    _rtc_slow_data_end = ABSOLUTE(.);
    . = ALIGN(4);
  } > rtc_slow_seg AT > RODATA

 .rtc_slow.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_slow_bss_start = ABSOLUTE(.);
    *(.rtc_slow.bss .rtc_slow.bss.*)
    _rtc_slow_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
  } > rtc_slow_seg

 .rtc_slow.persistent (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_slow_persistent_start = ABSOLUTE(.);
    *(.rtc_slow.persistent .rtc_slow.persistent.*)
    _rtc_slow_persistent_end = ABSOLUTE(.);
    . = ALIGN(4);
  } > rtc_slow_seg
}