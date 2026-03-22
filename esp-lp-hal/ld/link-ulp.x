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
PROVIDE(TimerInterrupt = default_timer_interrupt);
PROVIDE(IllegalInstructionException = default_exception_handler);
PROVIDE(BusErrorException = default_exception_handler);
PROVIDE(SensInterrupt = default_sens_interrupt);
PROVIDE(GpioInterrupt = default_gpio_interrupt);


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
