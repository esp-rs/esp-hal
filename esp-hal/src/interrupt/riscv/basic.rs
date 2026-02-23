use super::{InterruptKind, Priority, RunLevel};
use crate::peripherals::INTERRUPT_CORE0;

#[cfg(feature = "rt")]
pub(super) fn init() {}

pub(super) fn enable_cpu_interrupt_raw(cpu_interrupt: u32) {
    INTERRUPT_CORE0::regs()
        .cpu_int_enable()
        .modify(|r, w| unsafe { w.bits((1 << cpu_interrupt) | r.bits()) });
}

pub(super) fn set_kind_raw(cpu_interrupt_number: u32, kind: InterruptKind) {
    let interrupt_type = match kind {
        InterruptKind::Level => 0,
        InterruptKind::Edge => 1,
    };
    INTERRUPT_CORE0::regs()
        .cpu_int_type()
        .modify(|r, w| unsafe {
            w.bits(
                r.bits() & !(1 << cpu_interrupt_number) | (interrupt_type << cpu_interrupt_number),
            )
        });
}

pub(super) fn set_priority_raw(cpu_interrupt: u32, priority: Priority) {
    INTERRUPT_CORE0::regs()
        .cpu_int_pri(cpu_interrupt as usize)
        .write(|w| unsafe { w.map().bits(priority as u8) });
}

pub(super) fn clear_raw(cpu_interrupt: u32) {
    INTERRUPT_CORE0::regs()
        .cpu_int_clear()
        .write(|w| unsafe { w.bits(1 << cpu_interrupt) });
}

pub(super) fn cpu_interrupt_priority_raw(cpu_interrupt: u32) -> u8 {
    INTERRUPT_CORE0::regs()
        .cpu_int_pri(cpu_interrupt as usize)
        .read()
        .map()
        .bits()
}

pub(super) fn current_runlevel() -> u8 {
    INTERRUPT_CORE0::regs()
        .cpu_int_thresh()
        .read()
        .bits()
        .saturating_sub(1) as u8
}

pub(super) fn change_current_runlevel(level: RunLevel) -> u8 {
    let prev_interrupt_priority = current_runlevel();

    // The CPU responds to interrupts `>= level`, but we want to also disable
    // interrupts at `level` so we set the threshold to `level + 1`.
    INTERRUPT_CORE0::regs()
        .cpu_int_thresh()
        .write(|w| unsafe { w.bits(u32::from(level) + 1) });

    prev_interrupt_priority
}
