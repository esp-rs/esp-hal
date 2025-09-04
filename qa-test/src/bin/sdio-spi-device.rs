#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::sdio::{Config, Mode, Pins, Sdio, SpiMode};

esp_bootloader_esp_idf::esp_app_desc!();

struct Context {
    sdio: Sdio<'static>,
}

impl Context {
    /// Creates a new context for the SDIO SPI slave controller.
    pub fn new() -> Self {
        let peripherals = esp_hal::init(esp_hal::Config::default());

        cfg_if::cfg_if! {
            if #[cfg(feature = "esp32")] {
                // GPIO Slot 1 config
                let pins = Pins::new(
                    Mode::Spi,
                    peripherals.GPIO6,  // CLK/SCK
                    peripherals.GPIO11, // CMD/MOSI
                    peripherals.GPIO7,  // DAT0/MISO
                    peripherals.GPIO8,  // DAT1/IRQ
                    peripherals.GPIO9,  // DAT2
                    peripherals.GPIO10, // DAT3/#CS
                );

                // GPIO Slot 2 config
                //let pins = Pins::new(
                //    Mode::Spi,
                //    peripherals.GPIO14, // CLK/SCK
                //    peripherals.GPIO15, // CMD/MOSI
                //    peripherals.GPIO2,  // DAT0/MISO
                //    peripherals.GPIO4,  // DAT1/IRQ
                //    peripherals.GPIO12, // DAT2
                //    peripherals.GPIO13, // DAT3/#CS
                //);
            } else if #[cfg(feature = "esp32c6")] {
                let pins = Pins::new(
                    Mode::Spi,
                    peripherals.GPIO19, // CLK/SCLK
                    peripherals.GPIO18, // CMD/MOSI
                    peripherals.GPIO20, // DAT0/MISO
                    peripherals.GPIO21, // DAT1/IRQ
                    peripherals.GPIO22, // DAT2
                    peripherals.GPIO23, // DAT3/#CS
                );
            } else {
                panic!("unsupported platform");
            }
        }

        let config = Config::new().with_spi_mode(SpiMode::_2);

        let sdio = Sdio::new(
            peripherals.SLC,
            peripherals.SLCHOST,
            peripherals.HINF,
            pins,
            config,
        );

        Self { sdio }
    }
}

#[esp_hal::main]
fn main() -> ! {
    let ctx = Context::new();
    let _sdio = &ctx.sdio;

    // TODO: perform data transfer

    loop {
        core::hint::spin_loop()
    }
}
