/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

ENTRY(reset_vector)

CONFIG_ULP_COPROC_RESERVE_MEM = 8 * 1024;

MEMORY
{
    ram(RW) : ORIGIN = 0, LENGTH = CONFIG_ULP_COPROC_RESERVE_MEM
}

/* Default exception/interrupt handlers,
 * which may be overridden by the user.
 * TODO: Supply proc-macro attributes to make it easier for users
 *       to override these symbols.
 */
PROVIDE(IllegalInstructionException = default_exception_handler);
PROVIDE(BusErrorException = default_exception_handler);
PROVIDE(GpioInterrupt = default_gpio_interrupt);

/* External interrupt handlers, all do nothing by default */
PROVIDE(TOUCH_DONE_INT = noop_interrupt_handler);
PROVIDE(TOUCH_INACTIVE_INT = noop_interrupt_handler);
PROVIDE(TOUCH_ACTIVE_INT = noop_interrupt_handler);
PROVIDE(SARADC1_DONE_INT = noop_interrupt_handler);
PROVIDE(SARADC2_DONE_INT = noop_interrupt_handler);
PROVIDE(TSENS_DONE_INT = noop_interrupt_handler);
PROVIDE(RISCV_START_INT = noop_interrupt_handler);
PROVIDE(SW_INT = noop_interrupt_handler);
PROVIDE(SWD_INT = noop_interrupt_handler);
PROVIDE(TOUCH_TIME_OUT_INT = noop_interrupt_handler);
PROVIDE(TOUCH_APPROACH_LOOP_DONE_INT = noop_interrupt_handler);
PROVIDE(TOUCH_SCAN_DONE_INT = noop_interrupt_handler);

/* Default GPIO interrupt handlers */
PROVIDE(GPIO0 = noop_interrupt_handler);
PROVIDE(GPIO1 = noop_interrupt_handler);
PROVIDE(GPIO2 = noop_interrupt_handler);
PROVIDE(GPIO3 = noop_interrupt_handler);
PROVIDE(GPIO4 = noop_interrupt_handler);
PROVIDE(GPIO5 = noop_interrupt_handler);
PROVIDE(GPIO6 = noop_interrupt_handler);
PROVIDE(GPIO7 = noop_interrupt_handler);
PROVIDE(GPIO8 = noop_interrupt_handler);
PROVIDE(GPIO9 = noop_interrupt_handler);
PROVIDE(GPIO10 = noop_interrupt_handler);
PROVIDE(GPIO11 = noop_interrupt_handler);
PROVIDE(GPIO12 = noop_interrupt_handler);
PROVIDE(GPIO13 = noop_interrupt_handler);
PROVIDE(GPIO14 = noop_interrupt_handler);
PROVIDE(GPIO15 = noop_interrupt_handler);
PROVIDE(GPIO16 = noop_interrupt_handler);
PROVIDE(GPIO17 = noop_interrupt_handler);
PROVIDE(GPIO18 = noop_interrupt_handler);
PROVIDE(GPIO19 = noop_interrupt_handler);
PROVIDE(GPIO20 = noop_interrupt_handler);
PROVIDE(GPIO21 = noop_interrupt_handler);

SECTIONS
{
  . = ORIGIN(ram);

  .text :
  {
    /* Power-on-reset must be placed at address 0x0 */
    KEEP(*(.reset));
    /* ULP will jump to 0x10 when an interrupt trap occurs */
    . = ALIGN(0x10);
    KEEP(*(.trap));
    KEEP(*(.init));
    KEEP(*(.init.rust));
    KEEP(*(.trap.rust));
    *(.text .text.*)
  } >ram

  .rodata ALIGN(4):
  {
    *(.rodata)
    *(.rodata*)
  } >ram

  .data ALIGN(4):
  {
    PROVIDE(__global_pointer$ = . + 0x800);
    *(.data)
    *(.data*)
    *(.sdata)
    *(.sdata*)
  } >ram

  .bss ALIGN(4):
  {
    *(.bss)
    *(.bss*)
    *(.sbss)
    *(.sbss*)
  } >ram

  __stack_top = ORIGIN(ram) + LENGTH(ram);
}
