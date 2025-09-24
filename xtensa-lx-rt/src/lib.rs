//! Minimal startup/runtime for Xtensa LX CPUs.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register, named_asm_labels)]
#![feature(asm_experimental_arch)]
#![no_std]

use core::arch::global_asm;

pub use macros::{entry, exception, interrupt, pre_init};
pub use xtensa_lx;

pub mod exception;
pub mod interrupt;

#[doc(hidden)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn no_init_hook() {}

unsafe extern "C" {
    fn __pre_init();
    fn __post_init();

    fn __zero_bss() -> bool;
    fn __init_data() -> bool;

    fn main() -> !;

    static _bss_start: u32;
    static _bss_end: u32;

    static _data_start: u32;
    static _data_end: u32;
    static _sidata: u32;

    static _init_start: u32;

    static _stack_start_cpu0: u32;
}

global_asm!(
    "
    .section .rwtext,\"ax\",@progbits
    .literal sym__pre_init, {__pre_init}
    .literal sym__post_init, {__post_init}
    .literal sym__zero_bss, {__zero_bss}
    .literal sym_main, {main}

    .literal sym_stack_start_cpu0, {_stack_start_cpu0}

    .literal sym_init_start, {_init_start}
    .literal sym_bss_end, {_bss_end}
    .literal sym_bss_start, {_bss_start}
    .literal sym__init_data, {__init_data}
    .literal sym_data_start, {_data_start}
    .literal sym_data_end, {_data_end}
    .literal sym_sidata, {_sidata}
",
    __pre_init = sym __pre_init,
    __post_init = sym __post_init,
    __zero_bss = sym __zero_bss,

    _stack_start_cpu0 = sym _stack_start_cpu0,

    _bss_end =  sym _bss_end,
    _bss_start =  sym _bss_start,
    __init_data = sym __init_data,
    _data_start = sym _data_start,
    _data_end = sym _data_end,
    _sidata = sym _sidata,

    _init_start = sym _init_start,
    main = sym main,
);

global_asm!(
    "
    // _xtensa_lx_rt_zero_fill
    //
    // Input arguments:
    // a2: start address (used as a cursor)
    // a3: end address

    .section .rwtext,\"ax\",@progbits
    .global _xtensa_lx_rt_zero_fill
    .p2align 2
    .type _xtensa_lx_rt_zero_fill,@function
_xtensa_lx_rt_zero_fill:
    entry a1, 0
    bgeu   a2, a3, .Lfill_done    // If start >= end, skip zeroing
    movi.n a5, 0

.Lfill_loop:
    s32i.n a5, a2, 0              // Store the zero at the current cursor
    addi.n a2, a2, 4              // Increment the cursor by 4 bytes
    bltu   a2, a3, .Lfill_loop    // If cursor < end, repeat
.Lfill_done:
    retw.n

    // _xtensa_lx_rt_copy
    //
    // Input arguments:
    // a2: source address
    // a3: destination start address (used as a cursor)
    // a4: destination end address

    .section .rwtext,\"ax\",@progbits
    .global _xtensa_lx_rt_copy
    .p2align 2
    .type _xtensa_lx_rt_copy,@function
_xtensa_lx_rt_copy:
    entry a1, 0
    bgeu   a3, a4, .Lcopy_done   // If start >= end, skip copying
.Lcopy_loop:
    l32i.n a5, a2, 0             // Load word from source pointer
    s32i.n a5, a3, 0             // Store word at destination pointer
    addi.n a3, a3, 4             // Increment destination pointer by 4 bytes
    addi.n a2, a2, 4             // Increment source pointer by 4 bytes
    bltu   a3, a4, .Lcopy_loop   // If cursor < end, repeat
.Lcopy_done:
    retw.n

    .section .rwtext,\"ax\",@progbits
    .global Reset
    .p2align 2
    .type Reset,@function
Reset:
    entry  a1, 0
    movi   a0, 0                    // Trash the return address. Debuggers may use this to stop unwinding.

    wsr.intenable a0                // Disable interrupts

    l32r   a5, sym_stack_start_cpu0 // a5 is our temporary value register
    mov    sp, a5                   // Set the stack pointer.

    l32r   a5, sym__pre_init
    callx8 a5                       // Call the pre-initialization function.

.Linit_bss:
    l32r   a5, sym__zero_bss        // Do we need to zero-initialize memory?
    callx8 a5
    beqz   a10, .Linit_data         // No -> skip to copying initialized data

    l32r   a10, sym_bss_start        // Set input range to .bss
    l32r   a11, sym_bss_end          //
    call8  _xtensa_lx_rt_zero_fill  // Zero-fill

.Linit_data:
    l32r   a5, sym__init_data       // Do we need to initialize data sections?
    callx8 a5
    beqz   a10, .Linit_data_done    // If not, skip initialization

    l32r   a10, sym_sidata           // Arguments - source data pointer
    l32r   a11, sym_data_start       //           - destination pointer
    l32r   a12, sym_data_end         //           - destination end pointer
    call8  _xtensa_lx_rt_copy       // Copy .data section

.Linit_data_done:
    memw    // Make sure all writes are completed before proceeding. At this point, all static variables have been initialized.
"
);

// According to 4.4.7.2 of the xtensa isa, ccount and compare are undefined on
// reset, set all values to zero to disable. ("timer interupts are cleared by writing CCOMPARE[i]")
#[cfg(any(
    XCHAL_HAVE_TIMER0,
    XCHAL_HAVE_TIMER1,
    XCHAL_HAVE_TIMER2,
    XCHAL_HAVE_TIMER3
))]
cfg_global_asm!(
    #[cfg(XCHAL_HAVE_TIMER0)]
    "wsr.ccompare0 a0",
    #[cfg(XCHAL_HAVE_TIMER1)]
    "wsr.ccompare1 a0",
    #[cfg(XCHAL_HAVE_TIMER2)]
    "wsr.ccompare2 a0",
    #[cfg(XCHAL_HAVE_TIMER3)]
    "wsr.ccompare3 a0",
    "isync",
);

global_asm!(
    "
    l32r   a5, sym_init_start // vector table address
    wsr.vecbase a5

    l32r   a5, sym__post_init
    callx8 a5

    l32r   a5, sym_main       // program entry point
    callx8 a5
    ",
);

// We redefine these functions to avoid pulling in `xtensa-lx` as a dependency:

// CPU Interrupts
unsafe extern "C" {
    #[cfg(XCHAL_HAVE_TIMER0)]
    pub fn Timer0(save_frame: &mut crate::exception::Context);
    #[cfg(XCHAL_HAVE_TIMER1)]
    pub fn Timer1(save_frame: &mut crate::exception::Context);
    #[cfg(XCHAL_HAVE_TIMER2)]
    pub fn Timer2(save_frame: &mut crate::exception::Context);
    #[cfg(XCHAL_HAVE_TIMER3)]
    pub fn Timer3(save_frame: &mut crate::exception::Context);

    #[cfg(XCHAL_HAVE_PROFILING)]
    pub fn Profiling(save_frame: &mut crate::exception::Context);

    #[cfg(XCHAL_HAVE_SOFTWARE0)]
    pub fn Software0(save_frame: &mut crate::exception::Context);
    #[cfg(XCHAL_HAVE_SOFTWARE1)]
    pub fn Software1(save_frame: &mut crate::exception::Context);

    #[cfg(XCHAL_HAVE_NMI)]
    pub fn NMI(save_frame: &mut crate::exception::Context);
}

#[doc(hidden)]
#[unsafe(no_mangle)]
pub extern "C" fn default_mem_hook() -> bool {
    true // default to zeroing bss & initializing data
}

#[doc(hidden)]
#[macro_export]
macro_rules! cfg_asm {
    (@inner, [$($x:tt)*], [$($opts:tt)*], ) => {
        asm!($($x)* $($opts)*)
    };
    (@inner, [$($x:tt)*], [$($opts:tt)*], #[cfg($meta:meta)] $asm:literal, $($rest:tt)*) => {
        #[cfg($meta)]
        cfg_asm!(@inner, [$($x)* $asm,], [$($opts)*], $($rest)*);
        #[cfg(not($meta))]
        cfg_asm!(@inner, [$($x)*], [$($opts)*], $($rest)*)
    };
    (@inner, [$($x:tt)*], [$($opts:tt)*], $asm:literal, $($rest:tt)*) => {
        cfg_asm!(@inner, [$($x)* $asm,], [$($opts)*], $($rest)*)
    };
    ({$($asms:tt)*}, $($opts:tt)*) => {
        cfg_asm!(@inner, [], [$($opts)*], $($asms)*)
    };
}

#[doc(hidden)]
#[macro_export]
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
