//! Example of sending a software break signal from a UART in
//! Async mode.
//!
//! The following wiring is assumed:
//! - TEST IO PIN => GPIO2
//! - TX => GPIO17
//! - RX => GPIO16
//% CHIPS: esp32
//% FEATURES: embassy embassy-generic-timers esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output},
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, DataBits, StopBits, Uart},
};

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let uart_config = UartConfig::default()
        .baudrate(19200)
        .data_bits(DataBits::DataBits8)
        .parity_none()
        .stop_bits(StopBits::Stop1);

    let mut uart = Uart::new(
        peripherals.UART1,
        uart_config,
        peripherals.GPIO16, // RX
        peripherals.GPIO17, // TX
    )
    .expect("Failed to initialize UART")
    .into_async();

    let delay = Delay::new();

    // Used to toggle an output pin for comparing its timing with
    // the TX line on an oscilloscope. It's also the LED pin.
    let mut test_io_pin = Output::new(peripherals.GPIO2, Level::Low);

    loop {
        test_io_pin.toggle();
        uart.send_break_async(19200).await; // 19200 bits at 19200bps = 1 second
        test_io_pin.toggle();
        Timer::after_millis(1000).await;
    }
}
