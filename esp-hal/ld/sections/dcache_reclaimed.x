/* an uninitialized section of RAM otherwise not useable */
SECTIONS {
    .dcache_reclaimed_uninit (NOLOAD) : ALIGN(4) {
        *(.dcache_reclaimed_uninit)
    } > dcache_reclaimed_seg
}
