#define STORE    sw
#define LOAD     lw
#define LOG_REGBYTES 2
#define REGBYTES (1 << LOG_REGBYTES)

/*
    Trap entry point (_start_trap)
    Saves registers and calls _start_trap_rust_hal,
    restores registers and then returns.
*/
.section .trap, "ax"
.global _start_trap_hal
.option norelax
.align 6

_start_trap_hal:
    addi sp, sp, -32*REGBYTES

    STORE ra, 0*REGBYTES(sp)
    STORE t0, 1*REGBYTES(sp)
    STORE t1, 2*REGBYTES(sp)
    STORE t2, 3*REGBYTES(sp)
    STORE t3, 4*REGBYTES(sp)
    STORE t4, 5*REGBYTES(sp)
    STORE t5, 6*REGBYTES(sp)
    STORE t6, 7*REGBYTES(sp)
    STORE a0, 8*REGBYTES(sp)
    STORE a1, 9*REGBYTES(sp)
    STORE a2, 10*REGBYTES(sp)
    STORE a3, 11*REGBYTES(sp)
    STORE a4, 12*REGBYTES(sp)
    STORE a5, 13*REGBYTES(sp)
    STORE a6, 14*REGBYTES(sp)
    STORE a7, 15*REGBYTES(sp)
    STORE s0, 16*REGBYTES(sp)
    STORE s1, 17*REGBYTES(sp)
    STORE s2, 18*REGBYTES(sp)
    STORE s3, 19*REGBYTES(sp)
    STORE s4, 20*REGBYTES(sp)
    STORE s5, 21*REGBYTES(sp)
    STORE s6, 22*REGBYTES(sp)
    STORE s7, 23*REGBYTES(sp)
    STORE s8, 24*REGBYTES(sp)
    STORE s9, 25*REGBYTES(sp)
    STORE s10, 26*REGBYTES(sp)
    STORE s11, 27*REGBYTES(sp)
    STORE gp, 28*REGBYTES(sp)
    STORE tp, 29*REGBYTES(sp)

    addi s0, sp, 32*REGBYTES
    STORE s0, 30*REGBYTES(sp)

    add a0, sp, zero
    jal ra, _start_trap_rust_hal

    LOAD ra, 0*REGBYTES(sp)
    LOAD t0, 1*REGBYTES(sp)
    LOAD t1, 2*REGBYTES(sp)
    LOAD t2, 3*REGBYTES(sp)
    LOAD t3, 4*REGBYTES(sp)
    LOAD t4, 5*REGBYTES(sp)
    LOAD t5, 6*REGBYTES(sp)
    LOAD t6, 7*REGBYTES(sp)
    LOAD a0, 8*REGBYTES(sp)
    LOAD a1, 9*REGBYTES(sp)
    LOAD a2, 10*REGBYTES(sp)
    LOAD a3, 11*REGBYTES(sp)
    LOAD a4, 12*REGBYTES(sp)
    LOAD a5, 13*REGBYTES(sp)
    LOAD a6, 14*REGBYTES(sp)
    LOAD a7, 15*REGBYTES(sp)
    LOAD s0, 16*REGBYTES(sp)
    LOAD s1, 17*REGBYTES(sp)
    LOAD s2, 18*REGBYTES(sp)
    LOAD s3, 19*REGBYTES(sp)
    LOAD s4, 20*REGBYTES(sp)
    LOAD s5, 21*REGBYTES(sp)
    LOAD s6, 22*REGBYTES(sp)
    LOAD s7, 23*REGBYTES(sp)
    LOAD s8, 24*REGBYTES(sp)
    LOAD s9, 25*REGBYTES(sp)
    LOAD s10, 26*REGBYTES(sp)
    LOAD s11, 27*REGBYTES(sp)
    LOAD gp, 28*REGBYTES(sp)
    LOAD tp, 29*REGBYTES(sp)
    LOAD sp, 30*REGBYTES(sp)

    # SP was restored from the original SP
    mret

.section .trap, "ax"
.balign 0x100
.global _vector_table
.type _vector_table, @function

_vector_table:
    .option push
    .option norvc
    .rept 31
    j _start_trap_hal
    .endr