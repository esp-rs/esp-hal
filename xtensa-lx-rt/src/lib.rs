//! Minimal startup/runtime for Xtensa LX CPUs.
//!
//! ## Minimum Supported Rust Version (MSRV)
//!
//! This crate is guaranteed to compile on stable Rust 1.65 and up. It might
//! compile with older versions but that may change in any new patch release.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register, named_asm_labels)]
#![feature(asm_experimental_arch, naked_functions)]
#![no_std]

use core::{
    arch::asm,
    ptr::{addr_of, addr_of_mut},
};

pub use macros::{entry, exception, interrupt, pre_init};
pub use r0::{init_data, zero_bss};
pub use xtensa_lx;

pub mod exception;
pub mod interrupt;

#[doc(hidden)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn DefaultPreInit() {}

#[doc(hidden)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn Reset() -> ! {
    unsafe {
        // These symbols come from `link.x`
        unsafe extern "C" {
            static mut _bss_start: u32;
            static mut _bss_end: u32;

            static mut _data_start: u32;
            static mut _data_end: u32;
            static _sidata: u32;

            static mut _init_start: u32;

        }

        unsafe extern "Rust" {
            // This symbol will be provided by the user via `#[entry]`
            fn main() -> !;

            // This symbol will be provided by the user via `#[pre_init]`
            fn __pre_init();

            fn __post_init();

            fn __zero_bss() -> bool;

            fn __init_data() -> bool;
        }

        __pre_init();

        if __zero_bss() {
            r0::zero_bss(addr_of_mut!(_bss_start), addr_of_mut!(_bss_end));
        }

        if __init_data() {
            r0::init_data(addr_of_mut!(_data_start), addr_of_mut!(_data_end), &_sidata);
        }

        // Copy of data segment is done by bootloader

        // According to 4.4.6.2 of the xtensa isa, ccount and compare are undefined on
        // reset, set all values to zero to disable
        reset_internal_timers();

        // move vec table
        set_vecbase(addr_of!(_init_start));

        __post_init();

        main();
    }
}

#[doc(hidden)]
#[unsafe(no_mangle)]
#[rustfmt::skip]
pub unsafe extern "Rust" fn default_post_init() {}

// We redefine these functions to avoid pulling in `xtensa-lx` as a dependency:

#[doc(hidden)]
#[inline]
unsafe fn reset_internal_timers() {
    unsafe {
        #[cfg(any(
            XCHAL_HAVE_TIMER0,
            XCHAL_HAVE_TIMER1,
            XCHAL_HAVE_TIMER2,
            XCHAL_HAVE_TIMER3
        ))]
        {
            let value = 0;
            cfg_asm!(
        {
            #[cfg(XCHAL_HAVE_TIMER0)]
            "wsr.ccompare0 {0}",
            #[cfg(XCHAL_HAVE_TIMER1)]
            "wsr.ccompare1 {0}",
            #[cfg(XCHAL_HAVE_TIMER2)]
            "wsr.ccompare2 {0}",
            #[cfg(XCHAL_HAVE_TIMER3)]
            "wsr.ccompare3 {0}",
            "isync",
        }, in(reg) value, options(nostack));
        }
    }
}

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
#[inline]
unsafe fn set_vecbase(base: *const u32) {
    unsafe {
        asm!("wsr.vecbase {0}", in(reg) base, options(nostack));
    }
}

#[doc(hidden)]
#[unsafe(no_mangle)]
#[rustfmt::skip]
pub extern "Rust" fn default_mem_hook() -> bool {
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
