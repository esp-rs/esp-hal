/* Start-up assembly for ESP32-C6 chip */
.section    .init.vector, "ax"
/* This is the vector table. It is currently empty, but will be populated
 * with exception and interrupt handlers when this is supported
 */
.align  0x4, 0xff
.global _vector_table
.type _vector_table, @function
_vector_table:
    .option push
    .option norvc

    .rept 32
    nop
    .endr

    .option pop
    .size _vector_table, .-_vector_table

    .section .init, "ax"
    .global reset_vector

/* The reset vector, jumps to startup code */
reset_vector:
    j __start

__start:
    /* setup the stack pointer */
    la sp, __stack_top
    call rust_main
loop:
    j loop