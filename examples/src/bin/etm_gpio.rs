//! Control LED by the boot button via ETM without involving the CPU.

//! The following wiring is assumed:
//! - LED => GPIO1

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    etm::Etm,
    gpio::{
        etm::{GpioEtmChannels, GpioEtmInputConfig, GpioEtmOutputConfig},
        Io,
        Level,
        Output,
        Pull,
    },
    prelude::*,
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = Output::new(io.pins.gpio1, Level::Low);
    let button = io.pins.gpio9;

    led.set_high();

    // setup ETM
    let gpio_ext = GpioEtmChannels::new(peripherals.GPIO_SD);
    let led_task = gpio_ext.channel0_task.toggle(
        &mut led,
        GpioEtmOutputConfig {
            open_drain: false,
            pull: Pull::None,
            initial_state: Level::Low,
        },
    );
    let button_event = gpio_ext
        .channel0_event
        .falling_edge(button, GpioEtmInputConfig { pull: Pull::Down });

    let etm = Etm::new(peripherals.SOC_ETM);
    let channel0 = etm.channel0;

    // make sure the configured channel doesn't get dropped - dropping it will
    // disable the channel
    let _configured_channel = channel0.setup(&button_event, &led_task);

    // the LED is controlled by the button without involving the CPU
    loop {}
}
