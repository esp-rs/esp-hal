CONFIG_ULP_COPROC_RESERVE_MEM = 8 * 1024;

MEMORY
{
    RAM(RW) : ORIGIN = 0, LENGTH = CONFIG_ULP_COPROC_RESERVE_MEM
}

REGION_ALIAS("REGION_TEXT", RAM);
REGION_ALIAS("REGION_RODATA", RAM);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

_stext = ORIGIN(REGION_TEXT) + 0x0;             /* Load .text region at 0x0 */
_heap_size = 0;                                 /* Disable heap */
_max_hart_id = 0;                               /* One harts present */
_hart_stack_size = SIZEOF(.stack);              
_stack_start = ORIGIN(REGION_STACK) + LENGTH(REGION_STACK);
