/* Much of this assembly was sourced from the following ESP-IDF files...
*  ...irq handler macros:
*    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_vectors.S
*  ...critical section assembly
*    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/include/ulp_riscv_utils.h
*  ...riscv halt code
*    https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_utils.c
*/

.equ SAVE_REGS, 17
.equ CONTEXT_SIZE, (SAVE_REGS * 4)

/* Macro which first allocates space on the stack to save general
 * purpose registers, and then save them. GP register is excluded.
 * The default size allocated on the stack is CONTEXT_SIZE, but it
 * can be overridden.
 *
 * Note: We don't save the callee-saved s0-s11 registers to save space
 */
.macro save_general_regs cxt_size=CONTEXT_SIZE
    addi sp, sp, -\cxt_size
    sw   ra, 0(sp)
    sw   tp, 4(sp)
    sw   t0, 8(sp)
    sw   t1, 12(sp)
    sw   t2, 16(sp)
    sw   a0, 20(sp)
    sw   a1, 24(sp)
    sw   a2, 28(sp)
    sw   a3, 32(sp)
    sw   a4, 36(sp)
    sw   a5, 40(sp)
    sw   a6, 44(sp)
    sw   a7, 48(sp)
    sw   t3, 52(sp)
    sw   t4, 56(sp)
    sw   t5, 60(sp)
    sw   t6, 64(sp)
.endm

/* Restore the general purpose registers (excluding gp) from the context on
 * the stack. The context is then deallocated. The default size is CONTEXT_SIZE
 * but it can be overridden. */
.macro restore_general_regs cxt_size=CONTEXT_SIZE
    lw   ra, 0(sp)
    lw   tp, 4(sp)
    lw   t0, 8(sp)
    lw   t1, 12(sp)
    lw   t2, 16(sp)
    lw   a0, 20(sp)
    lw   a1, 24(sp)
    lw   a2, 28(sp)
    lw   a3, 32(sp)
    lw   a4, 36(sp)
    lw   a5, 40(sp)
    lw   a6, 44(sp)
    lw   a7, 48(sp)
    lw   t3, 52(sp)
    lw   t4, 56(sp)
    lw   t5, 60(sp)
    lw   t6, 64(sp)
    addi sp,sp, \cxt_size
.endm

.section .init.vectors
.global reset_vector
.global ulp_riscv_rescue_from_monitor
.global _start
.global _start_trap
.global _start_trap_rust
.global _setup_interrupts

/* The reset vector, jumps to startup code */
reset_vector:
	/* no more than 16 bytes here ! */
	j _start

/* Interrupt handler */
.balign 0x10
_start_trap:
    /* Save the general gurpose register context before handling the interrupt */
    save_general_regs

    /* Fetch the interrupt status from the custom q1 register into a0 */
    getq_insn a0, q1

    /* Call the global interrupt handler. The interrupt status is passed as the argument in a0.
     * We do not re-enable interrupts before calling the handler as ULP RISC-V does not
     * support nested interrupts.
     */
    jal _start_trap_rust

    /* Restore the register context after returning from the C interrupt handler */
    restore_general_regs

    /* Exit interrupt handler by executing the custom retirq instruction which will restore pc and re-enable interrupts */
    retirq_insn

.section .init.rust
_setup_interrupts:
    /* Custom instruction to un-mask the interrupts */
    /* waitirq_insn zero */
    maskirq_insn zero, zero
    /* Allow ULP to run */
    call ulp_riscv_rescue_from_monitor
    ret
