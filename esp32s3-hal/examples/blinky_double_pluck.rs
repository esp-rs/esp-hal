//! Blinky, but manages the pins with an HList from the `frunk` crate
//!
//! This assumes that a LED is connected to the pin assigned in the `Blinker`s
//! pin-field

#![no_std]
#![no_main]

use esp32s3_hal::{
    clock::ClockControl,
    gpio::{Output, PushPull, Unknown, IO},
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Delay,
    Rtc,
    frunk,
    frunk::ListBuild,
};
use esp_backtrace as _;
use esp_hal_common::gpio::GpioPin;

#[derive(frunk::ListBuild)]
struct PluckedBlinker {
    pin4: GpioPin<Unknown, 4>,
    pin5: GpioPin<Unknown, 5>,
    pin6: GpioPin<Unknown, 6>,
}
impl From<PluckedBlinker> for Blinker {
    fn from(value: PluckedBlinker) -> Self {
        let mut pin4 = value.pin4.into_push_pull_output();
        pin4.set_high().unwrap();
        let mut pin5 = value.pin5.into_push_pull_output();
        pin5.set_high().unwrap();
        let mut pin6 = value.pin6.into_push_pull_output();
        pin6.set_high().unwrap();
        Self {
            pin4,
            pin5,
            pin6
        }
    }
}
struct Blinker {
    pin4: GpioPin<Output<PushPull>, 4>,
    pin5: GpioPin<Output<PushPull>, 5>,
    pin6: GpioPin<Output<PushPull>, 6>,
}


impl Blinker {
    fn toggle(&mut self) {
        self.pin4.toggle().unwrap();
        self.pin5.toggle().unwrap();
        self.pin6.toggle().unwrap();
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
    let (blinker, _io) = PluckedBlinker::hl_new(io.pins);
    let blinker: Blinker = blinker.into();


    // One job of the clock system is to manage short delays...
    let mut delay = Delay::new(&clocks);

    blinker.blink_loop(&mut delay, 500u16)
}
