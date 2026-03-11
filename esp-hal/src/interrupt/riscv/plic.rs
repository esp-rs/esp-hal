use super::{InterruptKind, Priority, RunLevel};
use crate::peripherals::PLIC_MX;

#[cfg(feature = "rt")]
pub(super) fn init() {}

pub(super) fn enable_cpu_interrupt_raw(cpu_interrupt: u32) {
    PLIC_MX::regs().mxint_enable().modify(|r, w| unsafe {
        w.cpu_mxint_enable()
            .bits(r.cpu_mxint_enable().bits() | (1 << cpu_interrupt))
    });
}

pub(super) fn set_kind_raw(cpu_interrupt: u32, kind: InterruptKind) {
    let interrupt_type = match kind {
        InterruptKind::Level => 0,
        InterruptKind::Edge => 1,
    };

    PLIC_MX::regs().mxint_type().modify(|r, w| unsafe {
        w.cpu_mxint_type().bits(
            r.cpu_mxint_type().bits() & !(1 << cpu_interrupt) | (interrupt_type << cpu_interrupt),
        )
    });
}

pub(super) fn set_priority_raw(cpu_interrupt: u32, priority: Priority) {
    PLIC_MX::regs()
        .mxint_pri(cpu_interrupt as usize)
        .modify(|_, w| unsafe { w.cpu_mxint_pri().bits(priority as u8) });
}

pub(super) fn clear_raw(cpu_interrupt: u32) {
    PLIC_MX::regs().mxint_clear().modify(|r, w| unsafe {
        w.cpu_mxint_clear()
            .bits(r.cpu_mxint_clear().bits() | (1 << cpu_interrupt))
    });
}

pub(super) fn cpu_interrupt_priority_raw(cpu_interrupt: u32) -> u8 {
    PLIC_MX::regs()
        .mxint_pri(cpu_interrupt as usize)
        .read()
        .cpu_mxint_pri()
        .bits()
}

pub(super) fn current_runlevel() -> u8 {
    PLIC_MX::regs()
        .mxint_thresh()
        .read()
        .cpu_mxint_thresh()
        .bits()
        .saturating_sub(1)
}

/// Changes the current run level (the level below which interrupts are
/// masked), and returns the previous run level.
///
/// # Safety
///
/// This function must only be used to raise the runlevel and to restore it
/// to a previous value. It must not be used to arbitrarily lower the
/// runlevel.
pub(crate) fn change_current_runlevel(level: RunLevel) -> u8 {
    let prev_interrupt_priority = current_runlevel();

    // The CPU responds to interrupts `>= level`, but we want to also disable
    // interrupts at `level` so we set the threshold to `level + 1`.
    PLIC_MX::regs()
        .mxint_thresh()
        .write(|w| unsafe { w.cpu_mxint_thresh().bits(u32::from(level) as u8 + 1) });

    prev_interrupt_priority
}
