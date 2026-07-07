use super::GpioBank;
use crate::{
    gpio::AnyPin,
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

#[cfg(feature = "rt")]
pub(crate) fn enable_interrupt_on_second_core(priority: crate::interrupt::Priority) {
    crate::interrupt::enable_on_cpu(Cpu::AppCpu, Interrupt::GPIO, priority);
}

fn errata36(pin: &AnyPin<'_>, pull_up: bool, pull_down: bool) {
    use crate::gpio::{Pin, RtcPinWithResistors};

    for_each_lp_function! {
        (all_expanded $( (($_sig:ident, RTC_GPIOn, $_n:literal), $gpio:ident) ),* ) => {
            const RTC_IO_PINS: &[u8] = &[ $( $crate::peripherals::$gpio::NUMBER ),* ];
        };
    };

    if RTC_IO_PINS.contains(&pin.number()) && pin.is_output() {
        pin.rtcio_pullup(pull_up);
        pin.rtcio_pulldown(pull_down);
    }
}
