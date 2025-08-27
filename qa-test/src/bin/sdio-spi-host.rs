//! SDIO SPI mode master driver example.

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Blocking,
    sdio::{
        command::Cmd52,
        response::spi::R5,
    },
    spi::master as spi,
    time::Rate,
};

esp_bootloader_esp_idf::esp_app_desc!();

// Represents one block length.
const BLOCK_LEN: usize = 512;

struct Context {
    spi: spi::Spi<'static, Blocking>,
}

impl Context {
    /// Creates a new context for the SDIO SPI master controller.
    pub fn new() -> Self {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                // Create SPI master for mock SDIO host
                // HSPI config
                let spi = spi::Spi::new(
                    peripherals.SPI2,
                    spi::Config::default().with_frequency(Rate::from_mhz(10)),
                )
                .unwrap()
                .with_sck(peripherals.GPIO14)
                .with_mosi(peripherals.GPIO13)
                .with_miso(peripherals.GPIO12)
                .with_cs(peripherals.GPIO15);

                // Create SPI master for mock SDIO host
                // VSPI config
                //let spi = spi::Spi::new(
                //    peripherals.SPI3,
                //    spi::Config::default().with_frequency(Rate::from_mhz(10)),
                //)
                //.unwrap()
                //.with_sck(peripherals.GPIO18)
                //.with_mosi(peripherals.GPIO23)
                //.with_miso(peripherals.GPIO19)
                //.with_cs(peripherals.GPIO5);
            } else if #[cfg(feature = "esp32c6")] {
                // Create SPI master for mock SDIO host
                let spi = spi::Spi::new(
                    peripherals.SPI2,
                    spi::Config::default().with_frequency(Rate::from_mhz(10)),
                )
                .unwrap()
                .with_sck(peripherals.GPIO6)
                .with_mosi(peripherals.GPIO7)
                .with_miso(peripherals.GPIO2)
                .with_cs(peripherals.GPIO16);
            } else {
                panic!("unsupported platform");
            }
        }

        Self { spi }
    }
}


#[esp_hal::main]
fn main() -> ! {
    let mut ctx = Context::new();

    loop {
        ctx.spi.write(&[0u8; 10]).expect("error sending SPI init cycles");


        let test_packet = Cmd52::new();
        ctx.spi
            .write(&test_packet.into_bytes())
            .expect("error writing SDIO from host");

        // expect R5 response from SDIO client
        let mut res_buffer = [0u8; BLOCK_LEN];
        ctx.spi
            .read(&mut res_buffer)
            .expect("error reading SDIO from host");

        // TODO: implement response types, and check for expected response
        assert_ne!(res_buffer, [0u8; BLOCK_LEN]);

        let exp_res = R5::new().into_bytes();
        assert_eq!(&res_buffer[..R5::LEN], exp_res.as_ref());
    }
}
