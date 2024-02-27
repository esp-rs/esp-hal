//! Control LED on GPIO1 by the BOOT-BUTTON via ETM

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    etm::Etm,
    gpio::{etm::GpioEtmChannels, IO},
    peripherals::Peripherals,
    prelude::*,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio1.into_push_pull_output();
    let button = io.pins.gpio9.into_pull_down_input();

    led.set_high().unwrap();

    // setup ETM
    let gpio_ext = GpioEtmChannels::new(peripherals.GPIO_SD);
    let led_task = gpio_ext.channel0_task.toggle(&mut led);
    let button_event = gpio_ext.channel0_event.falling_edge(button);

    let etm = Etm::new(peripherals.SOC_ETM);
    let channel0 = etm.channel0;

    // make sure the configured channel doesn't get dropped - dropping it will
    // disable the channel
    let _configured_channel = channel0.setup(&button_event, &led_task);

    // the LED is controlled by the button without involving the CPU
    loop {}
}
