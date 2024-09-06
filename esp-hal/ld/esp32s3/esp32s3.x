/* before memory.x to allow override */
ENTRY(ESP32Reset)

/* after memory.x to allow override */
PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);
PROVIDE(__post_init = default_post_init);

PROVIDE(__level_1_interrupt = handle_interrupts);
PROVIDE(__level_2_interrupt = handle_interrupts);
PROVIDE(__level_3_interrupt = handle_interrupts);

INCLUDE exception.x

/* ESP32S3 fixups */
SECTIONS {
  .rwdata_dummy (NOLOAD) :
  {
    /* This dummy section represents the .rwtext section but in RWDATA.
     * Thus, it must have its alignment and (at least) its size.
     */

    /* Start at the same alignment constraint than .flash.text */

    . = ALIGN(ALIGNOF(.rwtext));

    /*  Create an empty gap as big as .rwtext section - 32k (SRAM0) 
     *  because SRAM1 is available on the data bus and instruction bus 
     */
    . = . + MAX(SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi) + RESERVE_ICACHE + VECTORS_SIZE, 32k) - 32k;

    /* Prepare the alignment of the section above. */
    . = ALIGN(4);
    _rwdata_reserved_start = .;
  } > RWDATA
}
INSERT BEFORE .data;

INCLUDE "fixups/rodata_dummy.x"
/* End of ESP32S3 fixups */

/* Shared sections - ordering matters */
INCLUDE "text.x"
INCLUDE "rodata.x"
INCLUDE "rwtext.x"
INCLUDE "rwdata.x"
INCLUDE "rtc_fast.x"
INCLUDE "rtc_slow.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
/* End of Shared sections */

EXTERN(DefaultHandler);

INCLUDE "device.x"
