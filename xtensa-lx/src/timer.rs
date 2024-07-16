//! Xtensa internal timers

use core::arch::asm;

#[inline]
pub fn get_ccompare0() -> u32 {
    let x: u32;
    unsafe { asm!("rsr.ccompare0 {0}", out(reg) x, options(nostack)) };
    x
}

#[inline]
pub fn get_ccompare1() -> u32 {
    let x: u32;
    unsafe { asm!("rsr.ccompare1 {0}", out(reg) x, options(nostack)) };
    x
}

#[inline]
pub fn get_ccompare2() -> u32 {
    let x: u32;
    unsafe { asm!("rsr.ccompare2 {0}", out(reg) x, options(nostack)) };
    x
}

#[inline]
pub fn get_ccompare3() -> u32 {
    let x: u32;
    unsafe { asm!("rsr.ccompare3 {0}", out(reg) x, options(nostack)) };
    x
}

#[inline]
pub fn set_ccompare0(val: u32) {
    unsafe {
        asm!("
        wsr.ccompare0 {0}
        isync
        ", in(reg) val, options(nostack))
    };
}

#[inline]
pub fn set_ccompare1(val: u32) {
    unsafe {
        asm!("
        wsr.ccompare1 {0}
        isync
        ", in(reg) val, options(nostack))
    };
}

#[inline]
pub fn set_ccompare2(val: u32) {
    unsafe {
        asm!("
        wsr.ccompare2 {0}
        isync
        ", in(reg) val, options(nostack))
    };
}

#[inline]
pub fn set_ccompare3(val: u32) {
    unsafe {
        asm!("
        wsr.ccompare3 {0}
        isync
        ", in(reg) val, options(nostack))
    };
}

/// Get the core cycle count
#[inline]
pub fn get_cycle_count() -> u32 {
    let x: u32;
    unsafe { asm!("rsr.ccount {0}", out(reg) x, options(nostack)) };
    x
}

/// cycle accurate delay using the cycle counter register
#[inline]
pub fn delay(clocks: u32) {
    let start = get_cycle_count();
    loop {
        if get_cycle_count().wrapping_sub(start) >= clocks {
            break;
        }
    }
}
