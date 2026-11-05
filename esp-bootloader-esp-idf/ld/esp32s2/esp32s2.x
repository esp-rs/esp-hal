INCLUDE exception.x

/* This represents .rwtext but in .data */
SECTIONS {
  .rwdata_dummy (NOLOAD) : ALIGN(4)
  {
    . = . + SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi);
  } > RWDATA
}
INSERT BEFORE .data;

INCLUDE "fixups/rtc_fast_rwdata_dummy.x"
/* End of fixups for esp32s2 */

/* Shared sections - ordering matters */
SECTIONS {
  INCLUDE "rwtext.x"
  INCLUDE "rwdata.x"
}
INCLUDE "rodata.x"
INCLUDE "text.x"
INCLUDE "rtc_fast.x"
INCLUDE "rtc_slow.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
INCLUDE "metadata.x"
INCLUDE "eh_frame.x"
/* End of Shared sections */
