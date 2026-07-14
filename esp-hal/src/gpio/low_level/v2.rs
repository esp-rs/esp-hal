use super::GpioBank;
use crate::{
    gpio::AnyPin,
    interrupt::{self, InterruptHandler, Priority},
    peripherals::{GPIO, Interrupt},
};

pub(crate) fn read_bank_interrupt_status(bank: GpioBank) -> u32 {
    match bank {
        GpioBank::_0 => GPIO::regs().status().read().bits(),
        #[cfg(gpio_has_bank_1)]
        GpioBank::_1 => GPIO::regs().status1().read().bits(),
    }
}

pub(crate) fn read_interrupt_status_of_current_cpu(bank: GpioBank) -> u32 {
    match bank {
        GpioBank::_0 => GPIO::regs().pcpu_int().read().bits(),
        #[cfg(gpio_has_bank_1)]
        GpioBank::_1 => GPIO::regs().pcpu_int1().read().bits(),
    }
}

pub(crate) fn prepare_pin_pull(_pin: &AnyPin<'_>, _pull_up: bool, _pull_down: bool) {}

pub(crate) fn gpio_intr_enable(int_enable: bool) -> u8 {
    int_enable as u8
}

pub(crate) fn enable_interrupt(handler: InterruptHandler) {
    interrupt::bind_handler(Interrupt::GPIO, handler);
}

pub(crate) fn set_interrupt_priority(priority: Priority) {
    interrupt::enable(Interrupt::GPIO, priority);
}
