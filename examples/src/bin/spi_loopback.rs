//! SPI loopback test
//!
//! The following wiring is assumed:
//! - SCLK => GPIO0
//! - MISO/MOSI => GPIO2
//! - CS   => GPIO5
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.

//% CHIPS: esp32 esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: esp-hal/unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
};
use esp_println::println;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sclk = peripherals.GPIO0;
    let miso_mosi = peripherals.GPIO2;
    let cs = peripherals.GPIO5;

    let miso = unsafe { miso_mosi.clone_unchecked() };

    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_miso(miso) // order matters
    .with_mosi(miso_mosi) // order matters
    .with_cs(cs);

    let delay = Delay::new();

    loop {
        let mut data = [0xde, 0xca, 0xfb, 0xad];
        spi.transfer(&mut data).unwrap();
        println!("{:x?}", data);

        delay.delay_millis(250);
    }
}
