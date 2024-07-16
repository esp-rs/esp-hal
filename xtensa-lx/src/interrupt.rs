//! Interrupts

use core::arch::asm;

pub use bare_metal::CriticalSection;

/// Trait for enums of external interrupt numbers.
///
/// This trait should be implemented by a peripheral access crate (PAC)
/// on its enum of available external interrupts for a specific device.
/// Each variant must convert to a u16 of its interrupt number,
/// which is its exception number - 16.
///
/// # Safety
///
/// This trait must only be implemented on enums of device interrupts. Each
/// enum variant must represent a distinct value (no duplicates are permitted),
/// and must always return the same value (do not change at runtime).
///
/// These requirements ensure safe nesting of critical sections.
pub unsafe trait InterruptNumber: Copy {
    /// Return the interrupt number associated with this variant.
    ///
    /// See trait documentation for safety requirements.
    fn number(self) -> u16;
}

/// Disables all interrupts and return the previous settings
#[inline]
pub fn disable() -> u32 {
    unsafe { set_mask(0) }
}

/// Enables all the interrupts
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section
#[inline]
pub unsafe fn enable() -> u32 {
    set_mask(!0)
}

/// Enables specific interrupts and returns the previous setting
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section
#[inline]
pub unsafe fn set_mask(mut mask: u32) -> u32 {
    asm!("
        xsr {0}, intenable
        rsync
        ",
        inout(reg) mask, options(nostack)
    );
    mask
}

/// Disables specific interrupts and returns the previous settings
#[inline]
pub fn disable_mask(mask: u32) -> u32 {
    let mut prev: u32 = 0;
    let _dummy: u32;
    unsafe {
        asm!("
        xsr.intenable {0}  // get mask and temporarily disable interrupts 
        and {1}, {1}, {0}
        rsync
        wsr.intenable {1}
        rsync
        ", inout(reg) prev, inout(reg) !mask => _dummy, options(nostack)
        );
    }
    prev
}

/// Enables specific interrupts and returns the previous setting
///
/// # Safety
///
/// - Do not call this function inside an `interrupt::free` critical section
#[inline]
pub unsafe fn enable_mask(mask: u32) -> u32 {
    let mut prev: u32 = 0;
    let _dummy: u32;
    asm!("
        xsr.intenable {0} // get mask and temporarily disable interrupts
        or {1}, {1}, {0}
        rsync
        wsr.intenable {1}
        rsync
    ", inout(reg) prev, inout(reg) mask => _dummy, options(nostack));
    prev
}

/// Get current interrupt mask
#[inline]
pub fn get_mask() -> u32 {
    let mask: u32;
    unsafe { asm!("rsr.intenable {0}", out(reg) mask) };
    mask
}

/// Get currently active interrupts
#[inline]
pub fn get() -> u32 {
    let mask: u32;
    unsafe {
        asm!("rsr.interrupt {0}", out(reg) mask, options(nostack));
    }
    mask
}

/// Set interrupt
///
/// # Safety
///
/// Only valid for software interrupts
#[inline]
pub unsafe fn set(mask: u32) {
    asm!("
         wsr.intset {0}
         rsync
         ",
         in(reg) mask, options(nostack)
    );
}

/// Clear interrupt
///
/// # Safety
///
/// Only valid for software and edge-triggered interrupts
#[inline]
pub unsafe fn clear(mask: u32) {
    asm!("
         wsr.intclear {0}
         rsync
         ",
         in(reg) mask, options(nostack)
    );
}

/// Get current interrupt level
#[inline]
pub fn get_level() -> u32 {
    let ps: u32;
    unsafe {
        asm!("rsr.ps {0}", out(reg) ps, options(nostack));
    };
    ps & 0xf
}

/// Execute closure `f` in an interrupt-free context.
///
/// This as also known as a "critical section".
#[inline]
pub fn free<F, R>(f: F) -> R
where
    F: FnOnce(&CriticalSection) -> R,
{
    // disable interrupts and store old mask
    let old_mask = disable();

    let r = f(unsafe { &CriticalSection::new() });

    // enable previously disable interrupts
    unsafe {
        enable_mask(old_mask);
    }

    r
}
