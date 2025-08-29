#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::sdio::{
    Config,
    Mode,
    Pins,
    Sdio,
    command::Cmd52,
    response::spi::R5,
};

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

        let config = Config::new();

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
    let mut ctx = Context::new();

    loop {
        let cmd_buf = ctx.sdio
            .read_command_raw()
            .expect("error writing SDIO from host");

        let exp_packet = Cmd52::new();

        assert_eq!(Cmd52::try_from_bytes(&cmd_buf), Ok(exp_packet));

        let exp_res = R5::new().into_bytes();

        ctx.sdio.write_response_raw(&exp_res)
            .expect("error writing SDIO response to host");
    }
}
