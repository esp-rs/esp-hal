//! This shows how to use RTC memory.
//!
//! RTC memory is retained during resets and during most sleep modes.
//!
//! Initialized memory is always re-initialized on startup.
//!
//! Persistent memory isn't initialized after software requested resets.
//! The boot button triggers such a reset.
//!
//! Zeroed memory is initialized to zero on startup.
//!
//! We can also run code from RTC memory.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    delay::Delay,
    gpio::{self, Io},
    macros::ram,
    peripherals::Peripherals,
    persistent::Persistent,
    prelude::*,
    reset::software_reset,
    system::SystemControl,
};
use esp_println::println;

#[ram(rtc_fast)]
static mut SOME_INITED_DATA: [u8; 2] = [0xaa, 0xbb];

#[ram(rtc_fast, zeroed)]
static mut SOME_ZEROED_DATA: [u8; 8] = [0; 8];

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let persistent = {
        #[ram(rtc_fast, persistent)]
        static SOME_PERSISTENT_DATA: Persistent<u8> = Persistent::new();

        // SAFETY:
        // - due to the restricted scope of the static, this is known to be the only place `.get()` is called
        // - nothing before this resets the chip
        unsafe { SOME_PERSISTENT_DATA.get(0) }
    };

    let delay = Delay::new(&clocks);

    let mut io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    io.set_interrupt_handler(handler);
    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let pin = io.pins.gpio0;
    #[cfg(not(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3")))]
    let pin = io.pins.gpio9;
    let mut button = gpio::Input::new(pin, gpio::Pull::Up);
    button.listen(gpio::Event::FallingEdge);

    println!(
        "IRAM function located at {:p}",
        function_in_ram as *const ()
    );
    unsafe {
        println!("SOME_INITED_DATA {:x?}", SOME_INITED_DATA);
        println!("SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA);

        SOME_INITED_DATA[0] = 0xff;
        SOME_ZEROED_DATA[0] = 0xff;

        println!("SOME_INITED_DATA {:x?}", SOME_INITED_DATA);
        println!("SOME_ZEROED_DATA {:x?}", SOME_ZEROED_DATA);
    }

    *persistent = persistent.wrapping_add(1);
    println!("Counter {}", persistent);

    println!(
        "RTC_FAST function located at {:p}",
        function_in_rtc_ram as *const ()
    );
    println!("Result {}", function_in_rtc_ram());

    loop {
        function_in_ram();
        delay.delay(1.secs());
    }
}

#[ram]
fn function_in_ram() {
    println!("Hello world!");
}

#[ram(rtc_fast)]
fn function_in_rtc_ram() -> u32 {
    42
}

#[handler]
#[ram]
fn handler() {
    software_reset();
}
