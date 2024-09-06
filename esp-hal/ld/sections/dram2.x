/* an uninitialized section of RAM otherwise not useable */
SECTIONS {
    .dram2_uninit (NOLOAD) : ALIGN(4) {
        *(.dram2_uninit)
    } > dram2_seg
}
