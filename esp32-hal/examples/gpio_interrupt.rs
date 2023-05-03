//! GPIO interrupt
//!
//! This prints "Interrupt" when the boot button is pressed.
//! It also blinks an LED like the blinky example.

#![no_std]
#![no_main]

use core::{borrow::BorrowMut, cell::RefCell};

use critical_section::Mutex;
use esp32_hal::{
    clock::ClockControl,
    gpio::{Event, Gpio0, Input, PullDown, IO},
    interrupt,
    macros::ram,
    peripherals::{self, Peripherals},
    prelude::*,
    timer::TimerGroup,
    xtensa_lx,
    Delay,
    Rtc,
};
use esp_backtrace as _;

static BUTTON: Mutex<RefCell<Option<Gpio0<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
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

    // Set GPIO15 as an output, and set its state high initially.
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio15.into_push_pull_output();
    let mut button = io.pins.gpio0.into_pull_down_input();
    button.listen(Event::FallingEdge);

    critical_section::with(|cs| BUTTON.borrow_ref_mut(cs).replace(button));

    interrupt::enable(peripherals::Interrupt::GPIO, interrupt::Priority::Priority2).unwrap();

    led.set_high().unwrap();

    // Initialize the Delay peripheral, and use it to toggle the LED state in a
    // loop.
    let mut delay = Delay::new(&clocks);

    loop {
        led.toggle().unwrap();
        delay.delay_ms(500u32);
    }
}

#[ram]
#[interrupt]
unsafe fn GPIO() {
    esp_println::println!(
        "GPIO Interrupt with priority {}",
        xtensa_lx::interrupt::get_level()
    );

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .borrow_mut()
            .as_mut()
            .unwrap()
            .clear_interrupt();
    });
}
