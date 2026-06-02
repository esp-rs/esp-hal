use super::{GpioBank, InterruptStatusRegisterAccess};
use crate::{gpio::AnyPin, peripherals::GPIO};

pub(super) fn read_bank_interrupt_status(bank: GpioBank) -> u32 {
    match bank {
        GpioBank::_0 => GPIO::regs().status().read().bits(),
        #[cfg(gpio_has_bank_1)]
        GpioBank::_1 => GPIO::regs().status1().read().bits(),
    }
}

pub(super) fn read_interrupt_status(access: InterruptStatusRegisterAccess) -> u32 {
    match access {
        InterruptStatusRegisterAccess::Bank0 => GPIO::regs().status().read().bits(),
        #[cfg(gpio_has_bank_1)]
        InterruptStatusRegisterAccess::Bank1 => GPIO::regs().status1().read().bits(),
    }
}

pub(super) fn prepare_pin_pull(pin: &AnyPin<'_>, pull_up: bool, pull_down: bool) {
    crate::soc::gpio::errata36(unsafe { pin.clone_unchecked() }, pull_up, pull_down);
}

pub(super) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    match crate::system::Cpu::current() {
        crate::system::Cpu::AppCpu => int_enable as u8 | ((nmi_enable as u8) << 1),
        crate::system::Cpu::ProCpu => ((int_enable as u8) << 2) | ((nmi_enable as u8) << 3),
    }
}

pub(super) fn for_each_interrupt_core(mut f: impl FnMut(crate::system::Cpu)) {
    for cpu in crate::system::Cpu::all() {
        f(cpu);
    }
}

#[cfg(feature = "rt")]
pub(super) fn enable_additional_default_interrupts(
    interrupt: crate::peripherals::Interrupt,
    priority: crate::interrupt::Priority,
) {
    crate::interrupt::enable_on_cpu(crate::system::Cpu::AppCpu, interrupt, priority);
}
