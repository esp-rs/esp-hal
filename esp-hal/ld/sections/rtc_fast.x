

SECTIONS {
  .rtc_fast.text : {
   . = ALIGN(4);
   *(.rtc_fast.literal .rtc_fast.text .rtc_fast.literal.* .rtc_fast.text.*)
   . = ALIGN(4);
  } > RTC_FAST_RWTEXT AT > RODATA
  
  .rtc_fast.data :
  {
    . = ALIGN(4);
    _rtc_fast_data_start = ABSOLUTE(.);
    *(.rtc_fast.data .rtc_fast.data.*)
    _rtc_fast_data_end = ABSOLUTE(.);
    . = ALIGN(4);
  } > RTC_FAST_RWDATA AT > RODATA

 .rtc_fast.bss (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_fast_bss_start = ABSOLUTE(.);
    *(.rtc_fast.bss .rtc_fast.bss.*)
    _rtc_fast_bss_end = ABSOLUTE(.);
    . = ALIGN(4);
  } > RTC_FAST_RWDATA

 .rtc_fast.persistent (NOLOAD) :
  {
    . = ALIGN(4);
    _rtc_fast_persistent_start = ABSOLUTE(.);
    *(.rtc_fast.persistent .rtc_fast.persistent.*)
    _rtc_fast_persistent_end = ABSOLUTE(.);
    . = ALIGN(4);
  } > RTC_FAST_RWDATA
}