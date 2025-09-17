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
r#"
    ret

/*
    Move SP to a valid memory region if needed.
*/

.section .trap.start, "ax"
.extern _pre_default_start_trap_ret
.global _pre_default_start_trap
_pre_default_start_trap:
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
.option pop
"#,
}

macro_rules! define_interrupt {
    ($num:literal, $name:ident, $fname:ident) => {
        #[derive(Copy, Clone)]
        struct $name;

        unsafe impl riscv_rt::InterruptNumber for $name {
            const MAX_INTERRUPT_NUMBER: usize = 31;

            fn number(self) -> usize {
                $num
            }

            fn from_number(_value: usize) -> riscv_rt::result::Result<Self> {
                Ok($name)
            }
        }

        unsafe impl riscv_rt::CoreInterruptNumber for $name {}

        #[unsafe(naked)]
        #[unsafe(link_section = ".trap.start")]
        #[riscv_rt::core_interrupt($name)]
        extern "C" fn $fname() {
            core::arch::naked_asm! {
                concat!(
                "
                li a0,",$num,"
                j handle_interrupts
                "
                )
            }
        }
    };
}

define_interrupt!(1, Trap1, trap1);
define_interrupt!(2, Trap2, trap2);
define_interrupt!(3, Trap3, trap3);
define_interrupt!(4, Trap4, trap4);
define_interrupt!(5, Trap5, trap5);
define_interrupt!(6, Trap6, trap6);
define_interrupt!(7, Trap7, trap7);
define_interrupt!(8, Trap8, trap8);
define_interrupt!(9, Trap9, trap9);
define_interrupt!(10, Trap10, trap10);
define_interrupt!(11, Trap11, trap11);
define_interrupt!(12, Trap12, trap12);
define_interrupt!(13, Trap13, trap13);
define_interrupt!(14, Trap14, trap14);
define_interrupt!(15, Trap15, trap15);
define_interrupt!(16, Trap16, trap16);
define_interrupt!(17, Trap17, trap17);
define_interrupt!(18, Trap18, trap18);
define_interrupt!(19, Trap19, trap19);
define_interrupt!(20, Trap20, trap20);
define_interrupt!(21, Trap21, trap21);
define_interrupt!(22, Trap22, trap22);
define_interrupt!(23, Trap23, trap23);
define_interrupt!(24, Trap24, trap24);
define_interrupt!(25, Trap25, trap25);
define_interrupt!(26, Trap26, trap26);
define_interrupt!(27, Trap27, trap27);
define_interrupt!(28, Trap28, trap28);
define_interrupt!(29, Trap29, trap29);
define_interrupt!(30, Trap30, trap30);
define_interrupt!(31, Trap31, trap31);
