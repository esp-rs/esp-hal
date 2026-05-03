/* Start-up assembly for ESP32C6-LP chip */
.org 0x0
.section    .init.vector, "ax"

/* Place exception handler at 0x0 */
.global exception_vector
.type exception_vector, @function
exception_vector:
    j _start_trap

/* Place vector interrupt handler (number 30) at 0x78 */
.org 0x78
.global interrupt_vector
.type interrupt_vector, @function
interrupt_vector:
    j _start_trap

/* The reset vector, placed at 0x80, the default reset value. */
.org 0x80
.section .init, "ax"
.global reset_vector
reset_vector:
    j __start

__start:
    /* Setup the global pointer & thread pointer */
    .option push
    .option norelax
    la gp, __global_pointer$
    addi tp, gp, 0
    .option pop
    /* Setup the stack pointer & frame pointer */
    la sp, __stack_top
    add s0, sp, zero
    /* Begin the rust program */
    jal zero, rust_main

/* Abort loop */
abort:
    jal zero, abort

.section .trap, "ax"
.global _start_trap
_start_trap:
    /* abort immediately */
    j ulp_riscv_halt
    mret