/* before memory.x to allow override */
ENTRY(ESP32Reset)

INCLUDE memory.x

/* after memory.x to allow override */
PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);

INCLUDE exception.x

SECTIONS {
  .text : ALIGN(4)
  {
    _stext = .;
    . = ALIGN (4);
    _text_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.literal .text .literal.* .text.*)
    _text_end = ABSOLUTE(.);
    _etext = .;
  } > ROTEXT

  .rodata_dummy (NOLOAD) :
  {
    /* This dummy section represents the .flash.text section but in RODATA.
     * Thus, it must have its alignment and (at least) its size.
     */

    /* Start at the same alignment constraint than .flash.text */

    . = ALIGN(ALIGNOF(.text));

    /* Create an empty gap as big as .text section */

    . = SIZEOF(.text);

    /* Prepare the alignment of the section above. Few bytes (0x20) must be
     * added for the mapping header.
     */

    . = ALIGN(0x10000) + 0x20;
    _rodata_reserved_start = .;
  } > RODATA

  .rodata : ALIGN(4)
  {
    _rodata_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.rodata .rodata.*)
    _rodata_end = ABSOLUTE(.);
  } > RODATA

  .rodata.wifi :
  {
    . = ALIGN(4);
    *( .rodata_wlog_*.* )
  } > RODATA AT > RODATA
  
  .rwtext : ALIGN(4)
  {
    . = ALIGN (4);
    *(.rwtext.literal .rwtext .rwtext.literal.* .rwtext.*)
  } > RWTEXT

  /* wifi data */
  .rwtext.wifi :
  {
    . = ALIGN(4);
    *( .wifi0iram  .wifi0iram.*)
    *( .wifirxiram  .wifirxiram.*)
    *( .wifislprxiram  .wifislprxiram.*)
    *( .wifislpiram  .wifislpiram.*)
    *( .phyiram  .phyiram.*)
    *( .iram1  .iram1.*)
  } > RWTEXT AT > RODATA

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
    . = MAX(SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi) + RESERVE_ICACHE + VECTORS_SIZE, 32k) - 32k;

    /* Prepare the alignment of the section above. */
    . = ALIGN(4);
    _rwdata_reserved_start = .;
  } > RWDATA

  .data : ALIGN(4)
  {
    _data_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.data .data.*)
    _data_end = ABSOLUTE(.);
  } > RWDATA AT > RODATA

  .data.wifi :
  {
    . = ALIGN(4);
    *( .dram1 .dram1.*)
  } > RWDATA AT > RODATA

  /* LMA of .data */
  _sidata = LOADADDR(.data);

  .bss (NOLOAD) : ALIGN(4)
  {
    _bss_start = ABSOLUTE(.);
    . = ALIGN (4);
    *(.bss .bss.* COMMON)
    _bss_end = ABSOLUTE(.);
  } > RWDATA

  .noinit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    *(.noinit .noinit.*)
  } > RWDATA

  /* must be last segment using RWDATA */
  .heap_start (NOLOAD) : ALIGN(4)
  {
    . = ALIGN (4);
    _heap_start = ABSOLUTE(.);
  } > RWDATA

}
