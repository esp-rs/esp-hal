/* Much of this assembly was sourced from the following ESP-IDF files...
*  ...irq handler macros:
*    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_vectors.S
*  ...critical section assembly
*    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/include/ulp_riscv_utils.h
*  ...riscv halt code
*    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_utils.c
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

.section .reset, "ax"
.global reset_vector 
/* Power-on reset vector, placed at address 0x0 */
reset_vector:
	/* no more than 16 bytes here */
	jal zero, _start

/* Initialisation routine */
.section .init, "ax"
.global _start
_start:
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

/* Hardware trap vector, placed at address 0x10
 * Handles all interrupts and exceptions for the ULP core.
 * Saves registers ra,t0-6,a0-7, then calls _start_trap_rust.
 */
.section .trap, "ax"
.global _start_trap
_start_trap:
    /* Save the general gurpose register context before handling the interrupt */
    save_general_regs

    /* Store the return address (x1) and stack pointer (x2) in the q registers. */
    setq_insn q2, ra
    setq_insn q3, sp

    /* Store the pointer to the stored registers (TrapFrame) in a0.
     * This will be the first argument to _start_trap_rust.
     */
    addi a0, sp, 0

    /* Fetch the interrupt status from the custom q1 register into a1.
     * This will be the second argumet to _start_trap_rust.
     */
    getq_insn a1, q1

    /* Call the global interrupt handler. The interrupt status is passed as the argument in a0.
     * We do not re-enable interrupts before calling the handler as ULP RISC-V does not
     * support nested interrupts.
     */
    jal ra,_start_trap_rust

    /* Restore the return address and stack pointer from the q registers */
    getq_insn ra, q2
    getq_insn sp, q3

    /* Restore the register context after returning from the C interrupt handler */
    restore_general_regs

    /* Exit interrupt handler by executing the custom retirq instruction which will restore pc and re-enable interrupts */
    retirq_insn

