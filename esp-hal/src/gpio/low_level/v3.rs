use super::GpioBank;
use crate::{
    gpio::AnyPin,
    interrupt::{self, InterruptHandler, Priority},
    peripherals::{GPIO, Interrupt},
    system::Cpu,
};

pub(crate) fn read_bank_interrupt_status(bank: GpioBank) -> u32 {
    match bank {
        GpioBank::_0 => GPIO::regs().status().read().bits(),
        #[cfg(gpio_has_bank_1)]
        GpioBank::_1 => GPIO::regs().status1().read().bits(),
    }
}

pub(crate) fn read_interrupt_status_of_current_cpu(bank: GpioBank) -> u32 {
    match (Cpu::current(), bank) {
        (Cpu::ProCpu, GpioBank::_0) => GPIO::regs().intr_0().read().bits(),
        (Cpu::AppCpu, GpioBank::_0) => GPIO::regs().intr_1().read().bits(),

        #[cfg(gpio_has_bank_1)]
        (Cpu::ProCpu, GpioBank::_1) => GPIO::regs().intr1_0().read().bits(),
        #[cfg(gpio_has_bank_1)]
        (Cpu::AppCpu, GpioBank::_1) => GPIO::regs().intr1_1().read().bits(),
    }
}

pub(crate) fn prepare_pin_pull(_pin: &AnyPin<'_>, _pull_up: bool, _pull_down: bool) {}

pub(crate) fn gpio_intr_enable(int_enable: bool) -> u8 {
    match Cpu::current() {
        Cpu::ProCpu => int_enable as u8,
        Cpu::AppCpu => (int_enable as u8) << 1,
    }
}

pub(crate) fn enable_interrupt(handler: InterruptHandler) {
    interrupt::bind_handler(Interrupt::GPIO, handler);
    interrupt::bind_handler(Interrupt::GPIO_INT1, handler);

    interrupt::disable(Cpu::AppCpu, Interrupt::GPIO);
    interrupt::disable(Cpu::ProCpu, Interrupt::GPIO_INT1);

    set_interrupt_priority(handler.priority());
}

pub(crate) fn set_interrupt_priority(priority: Priority) {
    interrupt::enable_on_cpu(Cpu::ProCpu, Interrupt::GPIO, priority);
    interrupt::enable_on_cpu(Cpu::AppCpu, Interrupt::GPIO_INT1, priority);
}
