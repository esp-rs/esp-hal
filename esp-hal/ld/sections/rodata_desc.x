SECTIONS {
    .rodata_desc : ALIGN(4)
    {
        KEEP(*(.rodata_desc));
    } > RODATA
}
