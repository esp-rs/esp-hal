/* RTC / LP fast memory. Its images follow `.data` in flash. */

SECTIONS {
  .rtc_fast.text ORIGIN(RTC_FAST) : AT(_direct_boot_data_end_lma)
  {
    . = ALIGN(4);
    *(.rtc_fast.literal .rtc_fast.text .rtc_fast.literal.* .rtc_fast.text.*)
    . = ALIGN(4);
  } > RTC_FAST

  .rtc_fast.data : AT(ALIGN(LOADADDR(.rtc_fast.text) + SIZEOF(.rtc_fast.text), 4))
  {
    . = ALIGN(4);
    _rtc_fast_data_start = ABSOLUTE(.);
    *(.rtc_fast.data .rtc_fast.data.*)
    . = ALIGN(4);
    _rtc_fast_data_end = ABSOLUTE(.);
  } > RTC_FAST

  .rtc_fast.bss (NOLOAD) : AT(ADDR(.rtc_fast.bss))
  {
    . = ALIGN(4);
    _rtc_fast_bss_start = ABSOLUTE(.);
    *(.rtc_fast.bss .rtc_fast.bss.*)
    . = ALIGN(4);
    _rtc_fast_bss_end = ABSOLUTE(.);
  } > RTC_FAST

  .rtc_fast.persistent (NOLOAD) : AT(ADDR(.rtc_fast.persistent))
  {
    . = ALIGN(4);
    _rtc_fast_persistent_start = ABSOLUTE(.);
    *(.rtc_fast.persistent .rtc_fast.persistent.*)
    . = ALIGN(4);
    _rtc_fast_persistent_end = ABSOLUTE(.);
  } > RTC_FAST
}

_direct_boot_rtc_fast_text_src = DROM_BASE + LOADADDR(.rtc_fast.text);
_direct_boot_rtc_fast_text_dst = ADDR(.rtc_fast.text);
_direct_boot_rtc_fast_text_end = _direct_boot_rtc_fast_text_dst + SIZEOF(.rtc_fast.text);

_direct_boot_rtc_fast_data_src = DROM_BASE + LOADADDR(.rtc_fast.data);
_direct_boot_rtc_fast_data_dst = ADDR(.rtc_fast.data);
_direct_boot_rtc_fast_data_end = _direct_boot_rtc_fast_data_dst + SIZEOF(.rtc_fast.data);

_direct_boot_flash_lma = ALIGN(LOADADDR(.rtc_fast.data) + SIZEOF(.rtc_fast.data), 4);
