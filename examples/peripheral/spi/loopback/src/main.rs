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

//% CHIP_FILTER: spi_master_driver_supported

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

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32" => peripherals.GPIO1,
        feature = "esp32c2" => peripherals.GPIO20,
        feature = "esp32c3" => peripherals.GPIO21,
        any(feature = "esp32c5", feature = "esp32c61") => peripherals.GPIO11,
        feature = "esp32c6" => peripherals.GPIO16,
        any(feature = "esp32h2", feature = "esp32h4") => peripherals.GPIO24,
        feature = "esp32p4" => peripherals.GPIO37,
        any(feature = "esp32s2", feature = "esp32s3") => peripherals.GPIO43,
        feature = "esp32s31" => peripherals.GPIO58,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    let sclk = peripherals.GPIO0;
    let miso_mosi = peripherals.GPIO2;
    let cs = peripherals.GPIO5;

    let (miso, mosi) = unsafe { miso_mosi.split() };

    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(100))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sclk)
    .with_miso(miso)
    .with_mosi(mosi)
    .with_cs(cs);

    let delay = Delay::new();

    loop {
        let mut data = [0xde, 0xca, 0xfb, 0xad];
        spi.transfer(&mut data).unwrap();
        log::info!("{:x?}", data);

        delay.delay_millis(250);
    }
}
