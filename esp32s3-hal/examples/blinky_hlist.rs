//! Blinky, but manages the pins with an HList from the `frunk` crate
//!
//! This assumes that a LED is connected to the pin assigned in the `Blinker`s pin-field

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    gpio::{ Output, Plucker, PushPull, Unknown, IO},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
};
use esp_backtrace as _;
use esp_hal_common::gpio::GpioPin;

struct Blinker {
    pin: GpioPin<Output<PushPull>, 4>,
}


impl Blinker {
    fn initialize<T, Remaining>(io: IO<T>) -> (Self, IO<T::Remainder>)
    where
        T: Plucker<GpioPin<Unknown, 4>, Remaining>,
    {
        let (pin, io) = io.pluck_pin();
        let mut pin = pin.into_push_pull_output();
        pin.set_high().unwrap();
        (Self { pin }, io)
    }

    fn toggle(&mut self) {
        self.pin.toggle().unwrap();
    }

    fn blink_loop(mut self, delay: &mut Delay, rest_period: u16) -> ! {
        loop {
            self.toggle();
            delay.delay_ms(rest_period);
        }
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.SYSTEM.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable MWDT and RWDT (Watchdog) flash boot protection
    wdt.disable();
    rtc.rwdt.disable();

    // Initialize a blinky-pin using pin#4
    let io = IO::hl_new(peripherals.GPIO, peripherals.IO_MUX);
    let (blinker, _io) = Blinker::initialize(io);

    // One job of the clock system is to manage short delays...
    let mut delay = Delay::new(&clocks);

    blinker.blink_loop(&mut delay, 500u16)
}
