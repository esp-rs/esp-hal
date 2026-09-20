/* an uninitialized section of RAM otherwise not useable */
SECTIONS {
    .dram2_uninit.bss (NOLOAD) : ALIGN(4) {
        _dram2_uninit_bss_start = ABSOLUTE(.);
        *(.dram2_uninit.bss)
        _dram2_uninit_bss_end = ABSOLUTE(.);
    } > dram2_seg

    .dram2_uninit (NOLOAD) : ALIGN(4) {
        *(.dram2_uninit)
    } > dram2_seg
}
