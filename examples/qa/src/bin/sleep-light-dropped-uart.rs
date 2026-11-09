//! QA test for light sleep after a UART driver is dropped.
//!
//! A dropped UART driver releases the clocks of its instance. The test enters light sleep
//! five times with a timer wakeup and prints a line after each wakeup. It must print `PASS`.
//!
//! Use the UART0 console. USB Serial/JTAG stops while the chip sleeps.

//% CHIP_FILTER: sleep_driver_supported && sleep_light_sleep && sleep_has_wakeup_source_timer
//% FEATURES: esp-println/uart

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    main,
    rtc_cntl::sleep::{LowPower, RtcSleepConfig},
    time::{Duration, Instant},
    uart::{Config, Uart},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

const ITERATIONS: u32 = 5;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut lpwr = LowPower::new(peripherals.LPWR);

    // Leave bytes in the transmit FIFO, so the dropped instance also has data it cannot send.
    let mut uart = Uart::new(peripherals.UART1, Config::default()).unwrap();
    uart.write(&[0x55; 32]).unwrap();
    drop(uart);

    for iteration in 1..=ITERATIONS {
        println!("Iteration {} - sleeping 1 s", iteration);
        lpwr.set_wakeup_deadline(Instant::now() + Duration::from_secs(1));
        lpwr.sleep_light(RtcSleepConfig::default());
        println!("Iteration {} - woke", iteration);
    }

    println!("PASS");
    loop {
        core::hint::spin_loop();
    }
}
