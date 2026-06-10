    /*       Disable compressed instructions */
    .option  norvc
    .section .reset, "ax"
    .global  reset_vector
    /*       Power-on reset vector, placed at address 0x0 */
reset_vector:
    /*       no more than 16 bytes here */
    /*       disable interrupts and jump to _start */
    /*       lui a0,0x80000 */
    /* addi     a0,a0,7 */
    /* .insn    4, 0x0605658b */
    jal      zero, _start

    /*       Initialisation routine */
    .section .init, "ax"
    .global  _start
_start:
    /*       Setup the global pointer & thread pointer */
    .option  push
    .option  norelax
    la       gp, __global_pointer$
    addi     tp, gp, 0
    .option  pop
    /*       Setup the stack pointer & frame pointer */
    la       sp, __stack_top
    add      s0, sp, zero
    /*       Begin the rust program */
    jal      zero, rust_main

    /*       Abort loop */
abort:
    jal      zero, abort

    /*       Hardware trap vector, placed at address 0x10
    *        This will abort
    */
    .section .trap, "ax"
    .global  _start_trap
_start_trap:
    ret