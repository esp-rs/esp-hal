/*
 This fix up is required when the RTC fast memory is split across two address spaces.
 This fix up pads the _data bus_ address space by the size of the code accessed by the instruction bus.
*/

SECTIONS {
  .rtc_fast.dummy (NOLOAD) :
  {
    _rtc_dummy_start = ABSOLUTE(.); /* needed to make section proper size */
    . = . + SIZEOF(.rtc_fast.text);
    _rtc_dummy_end = ABSOLUTE(.); /* needed to make section proper size */
  } > RTC_FAST_RWDATA
}
INSERT BEFORE .rtc_fast.data;