INCLUDE exception.x

/* ESP32 fixups */
INCLUDE "fixups/rtc_fast_rwdata_dummy.x"
/* END ESP32 fixups */

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
