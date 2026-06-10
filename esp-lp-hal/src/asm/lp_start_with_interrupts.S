/* Start-up assembly for ESP32C6-LP chip */
.org 0x0
.section    .init.vector, "ax"

/* This is the vector table. For LP core, only vector 30 is used.
 * Base of the vector table is set by 'mtvec', defaulting to 0x0.
 *
 * To enable interrupts globally, set the MIE bit of mstatus.
 * To enable Interrupt 30, set the 30th bit of mie CSR.
 * After the interrupt is triggered, the LP CPU jumps to mtvec + 4 ∗ 30
 * After enterring the interrupt handler, users need to read LPPERI_INTERRUPT_SOURCE_REG
 * to get the peripheral that triggered the interrupt and process the interrupt. 
 *
 * For exceptions, the handler address is the base address of the vector table in the mtvec CSR
 * The core fetches instructions from address 0x50000080 after reset.
 * When the core starts up, the base address of the vector table is initialized to the boot address 0x50000000.
 */


.equ SAVE_REGS, 16
.equ CONTEXT_SIZE, (SAVE_REGS * 4)

/* Macro which first allocates space on the stack to save general
 * purpose registers, and then save them. GP register is excluded.
 * The default size allocated on the stack is CONTEXT_SIZE, but it
 * can be overridden.
 *
 * Note: We don't save the callee-saved s0-s11 registers to save space
 * Note: This has been modified from ESP-IDF version, so that it matches
 *       the standard riscv_rt::TrapFrame.
 */
.macro save_general_regs cxt_size=CONTEXT_SIZE
    addi sp, sp, -\cxt_size
    sw   ra, 0(sp)
    sw   t0, 4(sp)
    sw   t1, 8(sp)
    sw   t2, 12(sp)
    sw   t3, 16(sp)
    sw   t4, 20(sp)
    sw   t5, 24(sp)
    sw   t6, 28(sp)
    sw   a0, 32(sp)
    sw   a1, 36(sp)
    sw   a2, 40(sp)
    sw   a3, 44(sp)
    sw   a4, 48(sp)
    sw   a5, 52(sp)
    sw   a6, 56(sp)
    sw   a7, 60(sp)
.endm

/* Restore the general purpose registers (excluding gp) from the context on
 * the stack. The context is then deallocated. The default size is CONTEXT_SIZE
 * but it can be overridden.
 * Note: This has been modified from ESP-IDF version, so that it matches
 *       the standard riscv_rt::TrapFrame.
 */
.macro restore_general_regs cxt_size=CONTEXT_SIZE
    lw   ra, 0(sp)
    lw   t0, 4(sp)
    lw   t1, 8(sp)
    lw   t2, 12(sp)
    lw   t3, 16(sp)
    lw   t4, 20(sp)
    lw   t5, 24(sp)
    lw   t6, 28(sp)
    lw   a0, 32(sp)
    lw   a1, 36(sp)
    lw   a2, 40(sp)
    lw   a3, 44(sp)
    lw   a4, 48(sp)
    lw   a5, 52(sp)
    lw   a6, 56(sp)
    lw   a7, 60(sp)
    addi sp,sp, \cxt_size
.endm

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

.section .trap, "ax"
.global _start_trap
_start_trap:
    /* Save the general gurpose register context before handling the interrupt */
    save_general_regs
    
    /* Default trap handler, delegates to _start_trap_rust. */
    jal ra,_start_trap_rust

    /* Restore the register context after returning from the C interrupt handler */
    restore_general_regs
    /* mret to exit */
    mret