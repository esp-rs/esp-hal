


SECTIONS {
  .rodata_dummy (NOLOAD) :
  {
    /* This dummy section represents the .flash.text section but in RODATA.
     * Thus, it must have its alignment and (at least) its size.
     */

    /* Start at the same alignment constraint than .flash.text */

    . = ALIGN(ALIGNOF(.text));

    /* Create an empty gap as big as .text section */

    . = . + SIZEOF(.text);

    /* Prepare the alignment of the section above. Few bytes (0x20) must be
     * added for the mapping header.
     */

    . = ALIGN(0x10000) + 0x20;
    _rodata_reserved_start = .;
  } > RODATA
}
INSERT BEFORE .rodata;