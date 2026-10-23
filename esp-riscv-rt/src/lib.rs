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
#![doc = document_features::document_features!(feature_label = r#"<span class="stab portability"><code>{feature}</code></span>"#)]
#![doc(html_logo_url = "https://docs.espressif.com/projects/rust/esp-rs-grey-bg.svg")]
#![deny(missing_docs)]
#![no_std]

use core::arch::global_asm;

pub use riscv;
pub use riscv_rt::{TrapFrame, entry};

#[doc(hidden)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _dispatch_exception(trap_frame: &TrapFrame, _code: usize) {
    unsafe extern "C" {
        fn _start_trap_rust_hal(trap_frame: &TrapFrame);
    }

    unsafe {
        _start_trap_rust_hal(trap_frame);
    }
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
    // Zero .dram2_uninit.bss
    r#"
    la a0, _dram2_uninit_bss_start
    la a1, _dram2_uninit_bss_end
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

/*
    Move SP to a valid memory region if needed.
*/

.section .trap.start, "ax"
.extern _pre_default_start_trap_ret
.global _pre_default_start_trap
_pre_default_start_trap:
    // move SP to some safe place if it's pointing below the RAM
    // or into .trap / .rwtext.
    //
    // Otherwise we won't be able to do anything reasonable
    // (since we don't have a working stack)
    //
    // most probably we will just print something and halt in this case
    // we actually can't do anything else
    csrw mscratch, t0
    la t0, _dram_data_start
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

    j _pre_default_start_trap_ret

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
    j _start_Trap1_trap
    j _start_Trap2_trap
    j _start_Trap3_trap
    j _start_Trap4_trap
    j _start_Trap5_trap
    j _start_Trap6_trap
    j _start_Trap7_trap
    j _start_Trap8_trap
    j _start_Trap9_trap
    j _start_Trap10_trap
    j _start_Trap11_trap
    j _start_Trap12_trap
    j _start_Trap13_trap
    j _start_Trap14_trap
    j _start_Trap15_trap
    j _start_Trap16_trap
    j _start_Trap17_trap
    j _start_Trap18_trap
    j _start_Trap19_trap
    j _start_Trap20_trap
    j _start_Trap21_trap
    j _start_Trap22_trap
    j _start_Trap23_trap
    j _start_Trap24_trap
    j _start_Trap25_trap
    j _start_Trap26_trap
    j _start_Trap27_trap
    j _start_Trap28_trap
    j _start_Trap29_trap
    j _start_Trap30_trap
    j _start_Trap31_trap
"#,
 #[cfg(feature = "clic-48")]
r#"
    j _start_Trap32_trap
    j _start_Trap33_trap
    j _start_Trap34_trap
    j _start_Trap35_trap
    j _start_Trap36_trap
    j _start_Trap37_trap
    j _start_Trap38_trap
    j _start_Trap39_trap
    j _start_Trap40_trap
    j _start_Trap41_trap
    j _start_Trap42_trap
    j _start_Trap43_trap
    j _start_Trap44_trap
    j _start_Trap45_trap
    j _start_Trap46_trap
    j _start_Trap47_trap
"#,
r#"
    .option pop
"#,
}

// The vector tables jump to `_start_Trap<N>_trap`. Handlers run on the ISR stack of the current
// CPU, or below the current stack pointer if the CPU already runs on the ISR stack.
//
// Trap frame, in words: ra, t0-t6, a0-a7, then the stack pointer to return to.
global_asm!(
    r#"
.section .trap.start, "ax"
.balign 4
.global _esp_riscv_rt_interrupt_trap
_esp_riscv_rt_interrupt_trap:
    // `_start_Trap<N>_trap` has stored the interrupted a0 at 0(sp), moved sp down by 16, and
    // loaded the interrupt number into a0.
    sw t0, 4(sp)

    csrr t0, mhartid
    bnez t0, 2f

    la t0, _isr_stack_bottom_cpu0
    bltu sp, t0, 1f
    la t0, _isr_stack_top_cpu0
    bltu sp, t0, 5f
    j 6f
1:
    la t0, _isr_stack_top_cpu0
    j 6f

2:
    la t0, _isr_stack_bottom_cpu1
    bltu sp, t0, 3f
    la t0, _isr_stack_top_cpu1
    bltu sp, t0, 5f
    j 6f
3:
    la t0, _isr_stack_top_cpu1
    j 6f

5:
    mv t0, sp
6:
    addi t0, t0, -80
    sw sp, 16*4(t0)
    mv sp, t0

    sw ra, 0*4(sp)
    sw t1, 2*4(sp)
    sw t2, 3*4(sp)
    sw t3, 4*4(sp)
    sw t4, 5*4(sp)
    sw t5, 6*4(sp)
    sw t6, 7*4(sp)
    sw a1, 9*4(sp)
    sw a2, 10*4(sp)
    sw a3, 11*4(sp)
    sw a4, 12*4(sp)
    sw a5, 13*4(sp)
    sw a6, 14*4(sp)
    sw a7, 15*4(sp)

    lw t1, 16*4(sp)
    lw t2, 0(t1)
    sw t2, 8*4(sp)
    lw t2, 4(t1)
    sw t2, 1*4(sp)

    call handle_interrupts

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
    lw sp, 16*4(sp)
    addi sp, sp, 16
    mret
"#
);

macro_rules! define_interrupt {
    ($($num:literal),*) => {
        $(
            global_asm!(concat!(
                r#"
.section .trap.start, "ax"
.balign 4
.global _start_Trap"#, $num, r#"_trap
_start_Trap"#, $num, r#"_trap:
    addi sp, sp, -16
    sw a0, 0(sp)
    li a0, "#, $num, r#"
    j _esp_riscv_rt_interrupt_trap
"#
            ));
        )*
    };
}

define_interrupt!(
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31
);

#[cfg(feature = "clic-48")]
define_interrupt!(
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47
);
