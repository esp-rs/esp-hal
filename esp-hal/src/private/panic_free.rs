//! Operations for code that cannot panic.
//!
//! Some code runs where the panic path cannot run, such as with the caches suspended or before
//! `main`. With debug assertions or overflow checks, `read_volatile`, `write_volatile`, and
//! integer arithmetic have a panic path in flash. The functions in this module do the same work
//! without a panic path.

use procmacros::ram;

/// Reads one word with `lw`.
///
/// # Safety
///
/// `addr` must be a readable, aligned word.
#[inline(always)]
pub(crate) unsafe fn load_word(addr: *const u32) -> u32 {
    let value;
    unsafe {
        core::arch::asm!(
            "lw {0}, 0({1})",
            out(reg) value,
            in(reg) addr,
            options(nostack, readonly, preserves_flags),
        );
    }
    value
}

/// Writes one word with `sw`.
///
/// # Safety
///
/// `addr` must be a writable, aligned word.
#[inline(always)]
pub(crate) unsafe fn store_word(addr: *mut u32, value: u32) {
    unsafe {
        core::arch::asm!(
            "sw {0}, 0({1})",
            in(reg) value,
            in(reg) addr,
            options(nostack, preserves_flags),
        );
    }
}

/// Stops the program.
///
/// With a debugger connected, the program exits through semihosting with exit code 1. Otherwise
/// the function spins. If the bootloader started the RTC watchdog, the spin ends in its reset.
#[ram(unstable(checked))]
pub(crate) fn stop() -> ! {
    if crate::debugger::debugger_connected() {
        // `SYS_EXIT_EXTENDED` with `ADP_Stopped_ApplicationExit`.
        let block = [0x20026, 1];
        unsafe {
            core::arch::asm!(
                ".option push",
                ".option norvc",
                "slli x0, x0, 0x1f",
                "ebreak",
                "srai x0, x0, 7",
                ".option pop",
                inout("a0") 0x20 => _,
                in("a1") &block,
            )
        };
    }
    unsafe { core::arch::asm!("2: j 2b", options(noreturn)) }
}

/// Returns `a + b`, or stops if the sum overflows.
#[inline(always)]
pub(crate) fn add(a: u32, b: u32) -> u32 {
    match a.checked_add(b) {
        Some(value) => value,
        None => stop(),
    }
}

/// Returns `a - b`, or stops if `b` is greater than `a`.
#[inline(always)]
pub(crate) fn sub(a: u32, b: u32) -> u32 {
    match a.checked_sub(b) {
        Some(value) => value,
        None => stop(),
    }
}

/// Returns `a << b`, or stops if `b` is 32 or more.
///
/// The bits shifted out of `a` do not stop the program.
#[inline(always)]
pub(crate) fn shl(a: u32, b: u32) -> u32 {
    match a.checked_shl(b) {
        Some(value) => value,
        None => stop(),
    }
}

/// Returns `a >> b`, or stops if `b` is 32 or more.
///
/// The bits shifted out of `a` do not stop the program.
#[inline(always)]
pub(crate) fn shr(a: u32, b: u32) -> u32 {
    match a.checked_shr(b) {
        Some(value) => value,
        None => stop(),
    }
}
