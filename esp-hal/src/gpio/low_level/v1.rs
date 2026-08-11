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
        GpioBank::_1 => GPIO::regs().status1().read().bits(),
    }
}

pub(crate) fn read_interrupt_status_of_current_cpu(bank: GpioBank) -> u32 {
    match (Cpu::current(), bank) {
        (Cpu::AppCpu, GpioBank::_0) => GPIO::regs().acpu_int().read().bits(),
        (Cpu::AppCpu, GpioBank::_1) => GPIO::regs().acpu_int1().read().bits(),
        (Cpu::ProCpu, GpioBank::_0) => GPIO::regs().pcpu_int().read().bits(),
        (Cpu::ProCpu, GpioBank::_1) => GPIO::regs().pcpu_int1().read().bits(),
    }
}

pub(crate) fn prepare_pin_pull(pin: &AnyPin<'_>, pull_up: bool, pull_down: bool) {
    errata36(pin, pull_up, pull_down);
}

pub(crate) fn gpio_intr_enable(int_enable: bool) -> u8 {
    match Cpu::current() {
        Cpu::AppCpu => int_enable as u8,
        Cpu::ProCpu => (int_enable as u8) << 2,
    }
}

pub(crate) fn enable_interrupt(handler: InterruptHandler) {
    interrupt::bind_handler(Interrupt::GPIO, handler);

    set_interrupt_priority(handler.priority());
}

pub(crate) fn set_interrupt_priority(priority: Priority) {
    interrupt::enable_on_cpu(Cpu::ProCpu, Interrupt::GPIO, priority);
    interrupt::enable_on_cpu(Cpu::AppCpu, Interrupt::GPIO, priority);
}

fn errata36(pin: &AnyPin<'_>, pull_up: bool, pull_down: bool) {
    use crate::gpio::{Pin, lp_io::low_level};

    for_each_lp_function! {
        (LP_GPIOn $( (($_sig:ident, LP_GPIOn, $n:literal), $gpio:ident, $_af:ident, $_lp_in:tt $_lp_out:tt) ),* ) => {
            // The digital pin number, and the number that the low-power registers use for the pad.
            const LP_IO_PINS: &[(u8, u8)] = &[ $( ($crate::peripherals::$gpio::NUMBER, $n) ),* ];
        };
    };

    let lp = LP_IO_PINS
        .iter()
        .find(|(gpio, _)| *gpio == pin.number())
        .map(|(_, lp)| *lp);

    // Some low-power pads have no pull resistors. The two functions below log a warning for such a
    // pad, and write no register.
    if let Some(lp) = lp
        && pin.is_output()
    {
        low_level::pullup_enable(lp, pull_up);
        low_level::pulldown_enable(lp, pull_down);
    }
}
