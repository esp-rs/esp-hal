//! Low-level access to Xtensa LX processors and peripherals.
//!
//! ## Minimum Supported Rust Version (MSRV)
//!
//! This crate is guaranteed to compile on stable Rust 1.65 and up. It might
//! compile with older versions but that may change in any new patch release.
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]
#![allow(asm_sub_register)]
#![feature(asm_experimental_arch)]
#![no_std]

use core::arch::asm;

pub mod interrupt;
pub mod timer;

#[macro_use]
mod macros;

const DCR_ENABLEOCD: u32 = 0x01;
const XDM_OCD_DCR_SET: u32 = 0x10200C;

/// Move the vector base
///
/// # Safety
///
/// *This is highly unsafe!*
/// It should be used with care, `base` MUST be a valid pointer
#[inline(always)]
pub unsafe fn set_vecbase(base: *const u32) { unsafe {
    asm!("wsr.vecbase {0}", in(reg) base, options(nostack));
}}

/// Get the core stack pointer
#[inline(always)]
pub fn get_stack_pointer() -> *const u32 {
    let x: *const u32;
    unsafe { asm!("mov {0}, sp", out(reg) x, options(nostack)) };
    x
}

/// Set the core stack pointer
///
/// `stack` pointer to the non-inclusive end of the stack (must be 16-byte
/// aligned)
///
/// # Safety
///
/// *This is highly unsafe!*
/// It should be used with care at e.g. program start or when building a task
/// scheduler
#[inline(always)]
pub unsafe fn set_stack_pointer(stack: *mut u32) { unsafe {
    // FIXME: this function relies on it getting inlined - if it doesn't inline it
    // will try and return from this function using the adress in `a0` which has
    // just been trashed... According to https://nnethercote.github.io/perf-book/inlining.html:
    // "Inline attributes do not guarantee that a function is inlined or not
    // inlined, but in practice, #[inline(always)] will cause inlining in all but
    // the most exceptional cases." Is this good enough? Should we rewrite these
    // as a macro to guarentee inlining?

    // NOTE: modification of the `sp` & `a0` is not typically allowed inside inline
    // asm!, but because we *need* to modify it we can do so by ommiting it from
    // the clobber
    asm!(
        "movi a0, 0", // trash return register
        "mov sp, {0}", // move stack pointer
        in(reg) stack, options(nostack)
    );
}}

/// Get the core current program counter
#[inline(always)]
pub fn get_program_counter() -> *const u32 {
    let x: *const u32;
    unsafe {
        asm!("
            mov {1}, {2}
            call0 2f
            .align 4
            2:
            mov {0}, {2}
            mov {2}, {1}
            ", out(reg) x, out(reg) _, out(reg) _, options(nostack))
    };
    x
}

/// Get the id of the current core
#[inline(always)]
pub fn get_processor_id() -> u32 {
    let mut x: u32;
    unsafe { asm!("rsr.prid {0}", out(reg) x, options(nostack)) };
    x
}

/// Returns true if a debugger is attached
#[inline(always)]
pub fn is_debugger_attached() -> bool {
    let mut x: u32;
    unsafe { asm!("rer {0}, {1}", out(reg) x, in(reg) XDM_OCD_DCR_SET, options(nostack)) };
    (x & DCR_ENABLEOCD) != 0
}

/// Insert debug breakpoint
#[inline(always)]
pub fn debug_break() {
    unsafe { asm!("break 1, 15", options(nostack)) };
}

/// Used to reexport items for use in macros. Do not use directly.
/// Not covered by semver guarantees.
#[doc(hidden)]
pub mod _export {
    pub use critical_section;
}
