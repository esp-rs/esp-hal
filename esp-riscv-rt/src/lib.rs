//! Minimal startup/runtime for RISC-V CPUs from Espressif.
//!
//! ## Features
//!
//! This crate provides:
//!
//! - Before main initialization of the `.bss` and `.data` sections controlled by features
//! - `#[entry]` to declare the entry point of the program
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![deny(missing_docs)]
#![no_std]

use core::arch::global_asm;

pub use riscv;
pub use riscv_rt::entry;

#[doc(hidden)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _dispatch_exception() {
    // never called but needed for riscv-rt to link
    panic!();
}

#[doc(hidden)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _dispatch_core_interrupt() {
    // never called but needed for riscv-rt to link
    panic!();
}

/// Registers saved in trap handler
#[derive(Debug, Default, Clone, Copy)]
#[repr(C)]
pub struct TrapFrame {
    /// Return address, stores the address to return to after a function call or
    /// interrupt.
    pub ra: usize,
    /// Temporary register t0, used for intermediate values.
    pub t0: usize,
    /// Temporary register t1, used for intermediate values.
    pub t1: usize,
    /// Temporary register t2, used for intermediate values.
    pub t2: usize,
    /// Temporary register t3, used for intermediate values.
    pub t3: usize,
    /// Temporary register t4, used for intermediate values.
    pub t4: usize,
    /// Temporary register t5, used for intermediate values.
    pub t5: usize,
    /// Temporary register t6, used for intermediate values.
    pub t6: usize,
    /// Argument register a0, typically used to pass the first argument to a
    /// function.
    pub a0: usize,
    /// Argument register a1, typically used to pass the second argument to a
    /// function.
    pub a1: usize,
    /// Argument register a2, typically used to pass the third argument to a
    /// function.
    pub a2: usize,
    /// Argument register a3, typically used to pass the fourth argument to a
    /// function.
    pub a3: usize,
    /// Argument register a4, typically used to pass the fifth argument to a
    /// function.
    pub a4: usize,
    /// Argument register a5, typically used to pass the sixth argument to a
    /// function.
    pub a5: usize,
    /// Argument register a6, typically used to pass the seventh argument to a
    /// function.
    pub a6: usize,
    /// Argument register a7, typically used to pass the eighth argument to a
    /// function.
    pub a7: usize,
    /// Saved register s0, used to hold values across function calls.
    pub s0: usize,
    /// Saved register s1, used to hold values across function calls.
    pub s1: usize,
    /// Saved register s2, used to hold values across function calls.
    pub s2: usize,
    /// Saved register s3, used to hold values across function calls.
    pub s3: usize,
    /// Saved register s4, used to hold values across function calls.
    pub s4: usize,
    /// Saved register s5, used to hold values across function calls.
    pub s5: usize,
    /// Saved register s6, used to hold values across function calls.
    pub s6: usize,
    /// Saved register s7, used to hold values across function calls.
    pub s7: usize,
    /// Saved register s8, used to hold values across function calls.
    pub s8: usize,
    /// Saved register s9, used to hold values across function calls.
    pub s9: usize,
    /// Saved register s10, used to hold values across function calls.
    pub s10: usize,
    /// Saved register s11, used to hold values across function calls.
    pub s11: usize,
    /// Global pointer register, holds the address of the global data area.
    pub gp: usize,
    /// Thread pointer register, holds the address of the thread-local storage
    /// area.
    pub tp: usize,
    /// Stack pointer register, holds the address of the top of the stack.
    pub sp: usize,
    /// Program counter, stores the address of the next instruction to be
    /// executed.
    pub pc: usize,
    /// Machine status register, holds the current status of the processor,
    /// including interrupt enable bits and privilege mode.
    pub mstatus: usize,
    /// Machine cause register, contains the reason for the trap (e.g.,
    /// exception or interrupt number).
    pub mcause: usize,
    /// Machine trap value register, contains additional information about the
    /// trap (e.g., faulting address).
    pub mtval: usize,
}

/// Parse cfg attributes inside a global_asm call.
macro_rules! cfg_global_asm {
    {@inner, [$($x:tt)*], } => {
        global_asm!{$($x)*}
    };
    (@inner, [$($x:tt)*], #[cfg($meta:meta)] $asm:literal, $($rest:tt)*) => {
        #[cfg($meta)]
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
        #[cfg(not($meta))]
        cfg_global_asm!{@inner, [$($x)*], $($rest)*}
    };
    {@inner, [$($x:tt)*], $asm:literal, $($rest:tt)*} => {
        cfg_global_asm!{@inner, [$($x)* $asm,], $($rest)*}
    };
    {$($asms:tt)*} => {
        cfg_global_asm!{@inner, [], $($asms)*}
    };
}

cfg_global_asm! {
    r#"
.section .init, "ax"
.weak __pre_init
__pre_init:"#,
    // Zero .rtc_fast.bss
#[cfg(feature = "rtc-ram")]
    r#"
    la a0, _rtc_fast_bss_start
    la a1, _rtc_fast_bss_end
    bge a0, a1, 2f
    mv a3, x0
    1:
    sw a3, 0(a0)
    addi a0, a0, 4
    blt a0, a1, 1b
    2:
"#,
     // Zero .rtc_fast.persistent if the chip just powered on
 #[cfg(feature = "rtc-ram")]
    r#"
    mv a0, zero
    mv t0, ra
    call rtc_get_reset_reason
    mv ra, t0
    addi a1, zero, 1
    bne a0, a1, 2f
    la a0, _rtc_fast_persistent_start
    la a1, _rtc_fast_persistent_end
    bge a0, a1, 2f
    mv a3, x0
    1:
    sw a3, 0(a0)
    addi a0, a0, 4
    blt a0, a1, 1b
    2:
"#,
r#"
    ret
"#,
r#"

/*
    Trap entry points (_start_trap, _start_trapN for N in 1..=31)

    The default implementation saves all registers to the stack and calls
    _start_trap_rust, then restores all saved registers before `mret`
*/
.section .trap, "ax"
.weak _start_trap  /* Exceptions call into _start_trap in vectored mode */
.weak _start_trap1
.weak _start_trap2
.weak _start_trap3
.weak _start_trap4
.weak _start_trap5
.weak _start_trap6
.weak _start_trap7
.weak _start_trap8
.weak _start_trap9
.weak _start_trap10
.weak _start_trap11
.weak _start_trap12
.weak _start_trap13
.weak _start_trap14
.weak _start_trap15
.weak _start_trap16
.weak _start_trap17
.weak _start_trap18
.weak _start_trap19
.weak _start_trap20
.weak _start_trap21
.weak _start_trap22
.weak _start_trap23
.weak _start_trap24
.weak _start_trap25
.weak _start_trap26
.weak _start_trap27
.weak _start_trap28
.weak _start_trap29
.weak _start_trap30
.weak _start_trap31
"#,
r#"
_start_trap:
    // Handle exceptions in vectored mode
    // move SP to some save place if it's pointing below the RAM
    // otherwise we won't be able to do anything reasonable
    // (since we don't have a working stack)
    //
    // most probably we will just print something and halt in this case
    // we actually can't do anything else
    csrw mscratch, t0
    la t0, _dram_origin
    bge sp, t0, 1f

    // use the reserved exception cause 14 to signal we detected a stack overflow
    li t0, 14
    csrw mcause, t0

    // set SP to the start of the stack
    la sp, _stack_start
    li t0, 4 // make sure stack start is in RAM
    sub sp, sp, t0
    andi sp, sp, -16 // Force 16-byte alignment

    1:
    csrr t0, mscratch
    // now SP is in RAM - continue

    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, _start_trap_rust_hal /* this runs on exception, use regular fault handler */
    j _start_trap_direct
_start_trap1:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt1
    j _start_trap_direct
_start_trap2:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt2
    j _start_trap_direct
_start_trap3:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt3
    j _start_trap_direct
_start_trap4:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt4
    j _start_trap_direct
_start_trap5:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt5
    j _start_trap_direct
_start_trap6:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt6
    j _start_trap_direct
_start_trap7:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt7
    j _start_trap_direct
_start_trap8:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt8
    j _start_trap_direct
_start_trap9:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt9
    j _start_trap_direct
_start_trap10:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt10
    j _start_trap_direct
_start_trap11:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt11
    j _start_trap_direct
_start_trap12:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt12
    j _start_trap_direct
_start_trap13:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt13
    j _start_trap_direct
_start_trap14:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt14
    j _start_trap_direct
_start_trap15:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt15
    j _start_trap_direct
_start_trap16:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt16
    j _start_trap_direct
_start_trap17:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt17
    j _start_trap_direct
_start_trap18:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt18
    j _start_trap_direct
_start_trap19:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt19
    j _start_trap_direct
_start_trap20:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt20
    j _start_trap_direct
_start_trap21:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt21
    j _start_trap_direct
_start_trap22:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt22
    j _start_trap_direct
_start_trap23:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt23
    j _start_trap_direct
_start_trap24:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt24
    j _start_trap_direct
_start_trap25:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt25
    j _start_trap_direct
_start_trap26:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt26
    j _start_trap_direct
_start_trap27:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt27
    j _start_trap_direct
_start_trap28:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt28
    j _start_trap_direct
_start_trap29:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt29
    j _start_trap_direct
_start_trap30:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt30
    j _start_trap_direct
_start_trap31:
    addi sp, sp, -40*4
    sw ra, 0(sp)
    la ra, interrupt31
_start_trap_direct:
"#,
r#"
    sw t0, 1*4(sp)
    sw t1, 2*4(sp)
    sw t2, 3*4(sp)
    sw t3, 4*4(sp)
    sw t4, 5*4(sp)
    sw t5, 6*4(sp)
    sw t6, 7*4(sp)
    sw a0, 8*4(sp)
    sw a1, 9*4(sp)
    sw a2, 10*4(sp)
    sw a3, 11*4(sp)
    sw a4, 12*4(sp)
    sw a5, 13*4(sp)
    sw a6, 14*4(sp)
    sw a7, 15*4(sp)
    sw s0, 16*4(sp)
    sw s1, 17*4(sp)
    sw s2, 18*4(sp)
    sw s3, 19*4(sp)
    sw s4, 20*4(sp)
    sw s5, 21*4(sp)
    sw s6, 22*4(sp)
    sw s7, 23*4(sp)
    sw s8, 24*4(sp)
    sw s9, 25*4(sp)
    sw s10, 26*4(sp)
    sw s11, 27*4(sp)
    sw gp, 28*4(sp)
    sw tp, 29*4(sp)
    csrrs t1, mepc, x0
    sw t1, 31*4(sp)
    csrrs t1, mstatus, x0
    sw t1, 32*4(sp)
    csrrs t1, mcause, x0
    sw t1, 33*4(sp)
    csrrs t1, mtval, x0
    sw t1, 34*4(sp)

    addi s0, sp, 40*4
    sw s0, 30*4(sp)

    add a0, sp, zero
    "#,
    // store current priority, set threshold, enable interrupts
    r#"
    addi sp, sp, -16 #build stack
    sw ra, 0(sp)
    jal ra, _handle_priority
    lw ra, 0(sp)
    sw a0, 0(sp) #reuse old stack, a0 is return of _handle_priority
    addi a0, sp, 16 #the proper stack pointer is an argument to the HAL handler
    "#,
    // jump to handler loaded in direct handler
    r#"
    jalr ra, ra #jump to label loaded in _start_trapx
    "#,
    // restore threshold
    r#"
    lw a0, 0(sp) #load stored priority
    jal ra, _restore_priority
    addi sp, sp, 16 #pop
    "#,
    r#"
    lw t1, 31*4(sp)
    csrrw x0, mepc, t1

    lw t1, 32*4(sp)
    csrrw x0, mstatus, t1

    lw ra, 0*4(sp)
    lw t0, 1*4(sp)
    lw t1, 2*4(sp)
    lw t2, 3*4(sp)
    lw t3, 4*4(sp)
    lw t4, 5*4(sp)
    lw t5, 6*4(sp)
    lw t6, 7*4(sp)
    lw a0, 8*4(sp)
    lw a1, 9*4(sp)
    lw a2, 10*4(sp)
    lw a3, 11*4(sp)
    lw a4, 12*4(sp)
    lw a5, 13*4(sp)
    lw a6, 14*4(sp)
    lw a7, 15*4(sp)
    lw s0, 16*4(sp)
    lw s1, 17*4(sp)
    lw s2, 18*4(sp)
    lw s3, 19*4(sp)
    lw s4, 20*4(sp)
    lw s5, 21*4(sp)
    lw s6, 22*4(sp)
    lw s7, 23*4(sp)
    lw s8, 24*4(sp)
    lw s9, 25*4(sp)
    lw s10, 26*4(sp)
    lw s11, 27*4(sp)
    lw gp, 28*4(sp)
    lw tp, 29*4(sp)
    lw sp, 30*4(sp)

    # SP was restored from the original SP
    mret

/*
    Interrupt vector table (_vector_table)
*/

.section .trap, "ax"
.weak _vector_table
.type _vector_table, @function

.option push
.balign 0x100
.option norelax
.option norvc

_vector_table:
    j _start_trap
    j _start_trap1
    j _start_trap2
    j _start_trap3
    j _start_trap4
    j _start_trap5
    j _start_trap6
    j _start_trap7
    j _start_trap8
    j _start_trap9
    j _start_trap10
    j _start_trap11
    j _start_trap12
    j _start_trap13
    j _start_trap14
    j _start_trap15
    j _start_trap16
    j _start_trap17
    j _start_trap18
    j _start_trap19
    j _start_trap20
    j _start_trap21
    j _start_trap22
    j _start_trap23
    j _start_trap24
    j _start_trap25
    j _start_trap26
    j _start_trap27
    j _start_trap28
    j _start_trap29
    j _start_trap30
    j _start_trap31
.option pop
"#,
r#"
#this is required for the linking step, these symbols for in-use interrupts should always be overwritten by the user.
.section .trap, "ax"
// See https://github.com/esp-rs/esp-hal/issues/1326 and https://reviews.llvm.org/D98762
// and yes, this all has to go on one line... *sigh*.
.lto_discard interrupt1, interrupt2, interrupt3, interrupt4, interrupt5, interrupt6, interrupt7, interrupt8, interrupt9, interrupt10, interrupt11, interrupt12, interrupt13, interrupt14, interrupt15, interrupt16, interrupt17, interrupt18, interrupt19, interrupt20, interrupt21, interrupt22, interrupt23, interrupt24, interrupt25, interrupt26, interrupt27, interrupt28, interrupt29, interrupt30, interrupt31
.weak interrupt1
.weak interrupt2
.weak interrupt3
.weak interrupt4
.weak interrupt5
.weak interrupt6
.weak interrupt7
.weak interrupt8
.weak interrupt9
.weak interrupt10
.weak interrupt11
.weak interrupt12
.weak interrupt13
.weak interrupt14
.weak interrupt15
.weak interrupt16
.weak interrupt17
.weak interrupt18
.weak interrupt19
.weak interrupt20
.weak interrupt21
.weak interrupt22
.weak interrupt23
.weak interrupt24
.weak interrupt25
.weak interrupt26
.weak interrupt27
.weak interrupt28
.weak interrupt29
.weak interrupt30
.weak interrupt31
"#,
}
