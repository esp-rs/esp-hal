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
        InterruptStatusRegisterAccess::Bank0 => GPIO::regs().intr_0().read().bits(),
        #[cfg(gpio_has_bank_1)]
        InterruptStatusRegisterAccess::Bank1 => GPIO::regs().intr1_0().read().bits(),
    }
}

pub(super) fn prepare_pin_pull(_pin: &AnyPin<'_>, _pull_up: bool, _pull_down: bool) {}

pub(super) fn gpio_intr_enable(int_enable: bool, nmi_enable: bool) -> u8 {
    int_enable as u8 | ((nmi_enable as u8) << 1)
}

pub(super) fn for_each_interrupt_core(mut f: impl FnMut(crate::system::Cpu)) {
    f(crate::system::Cpu::current());
}

#[cfg(feature = "rt")]
pub(super) fn enable_additional_default_interrupts(
    _interrupt: crate::peripherals::Interrupt,
    _priority: crate::interrupt::Priority,
) {
}
