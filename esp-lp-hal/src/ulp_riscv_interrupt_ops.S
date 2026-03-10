/**
 * Defines the assembly macros needed for custom instructions on the ULP RISCV core.
 * Adapted from the following ESP-IDF header:
 * https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/include/ulp_riscv_interrupt_ops.h
 * With additional waitirq_insn from PicoRV32:
 * https://github.com/YosysHQ/picorv32/blob/87c89acc18994c8cf9a2311e871818e87d304568/firmware/start.S
 * A quote from the ESP-IDF header file, is shown below.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This header file defines custom instructions for interrupt handling on the            *
 * ULP RISC-V. The architecture of the processor and therefore, the interrupt            *
 * handling is based on the PicoRV32 CPU. Details about the operations are               *
 * available at https://github.com/YosysHQ/picorv32#custom-instructions-for-irq-handling *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 */

.set regnum_zero,0
.set regnum_ra,  1
.set regnum_sp,  2
.set regnum_gp,  3
.set regnum_tp,  4
.set regnum_t0,  5
.set regnum_t1,  6
.set regnum_t2,  7
.set regnum_s0,  8
.set regnum_s1,  9
.set regnum_a0, 10
.set regnum_a1, 11
.set regnum_a2, 12
.set regnum_a3, 13
.set regnum_a4, 14
.set regnum_a5, 15
.set regnum_a6, 16
.set regnum_a7, 17
.set regnum_s2, 18
.set regnum_s3, 19
.set regnum_s4, 20
.set regnum_s5, 21
.set regnum_s6, 22
.set regnum_s7, 23
.set regnum_s8, 24
.set regnum_s9, 25
.set regnum_s10,26
.set regnum_s11,27
.set regnum_t3, 28
.set regnum_t4, 29
.set regnum_t5, 30
.set regnum_t6, 31

/* Define encoding for special interrupt handling registers, viz., q0, q1, q2 and q3 */
.set regnum_q0,  0
.set regnum_q1,  1
.set regnum_q2,  2
.set regnum_q3,  3

/* All custom interrupt handling instructions follow the standard R-type instruction format from RISC-V ISA
 * with the same opcode of custom0 (0001011).
 */
.macro r_type_insn _f7, _rs2, _rs1, _f3, _rd, _opc
  .word (((\_f7) << 25) | ((\_rs2) << 20) | ((\_rs1) << 15) | ((\_f3) << 12) | ((\_rd) << 7) | ((\_opc) << 0))
.endm

/**
 * Instruction: getq rd, qs
 * Description: This instruction copies the value of Qx into a general purpose register rd
 */
.macro getq_insn _rd _qs
  r_type_insn 0b0000000, 0, "regnum_\_qs", 0b100, "regnum_\_rd", 0b0001011
.endm

/**
 * Instruction: setq qd, rs
 * Description: This instruction copies the value of general purpose register rs to Qx
 */
.macro setq_insn _qd _rs
  r_type_insn 0b0000001, 0, "regnum_\_rs", 0b010, "regnum_\_qd", 0b0001011
.endm

/**
 * Instruction: retirq
 * Description: This instruction copies the value of Q0 to CPU PC, and renables interrupts
 */
.macro retirq_insn
  r_type_insn 0b0000010, 0, 0, 0b000, 0, 0b0001011
.endm

/**
 * Instruction: maskirq rd, rs
 * Description: This instruction copies the value of the register IRQ Mask to the register rd, and copies the value
 *              of register rs to to IRQ mask.
 */
.macro maskirq_insn _rd _rs
  r_type_insn 0b0000011, 0, "regnum_\_rs", 0b110, "regnum_\_rd", 0b0001011
.endm

/**
 * Instruction: waitirq rd
 * Description: Pause execution until any interrupt (masked or unmasked) becomes pending. Stores the pending IRQ bitmask into register rd.
 */
.macro waitirq_insn _rd
  r_type_insn 0b0000100, 0, 0, 0b100, "regnum_\_rd", 0b0001011
.endm