ENTRY(Reset)

PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);
PROVIDE(__init_persistent = default_mem_hook);
PROVIDE(__post_init = no_init_hook);

PROVIDE(DefaultHandler = EspDefaultHandler);

PROVIDE(level1_interrupt = DefaultHandler);
PROVIDE(level2_interrupt = DefaultHandler);
PROVIDE(level3_interrupt = DefaultHandler);
PROVIDE(level4_interrupt = DefaultHandler);
PROVIDE(level5_interrupt = DefaultHandler);
PROVIDE(level6_interrupt = DefaultHandler);
PROVIDE(level7_interrupt = DefaultHandler);

/* external interrupts (from PAC) */
INCLUDE "device.x"
