//! Control LED on GPIO 1 by the BOOT-BUTTON via ETM

#![no_std]
#![no_main]

use esp32c6_hal::{
    clock::ClockControl,
    etm::Etm,
    gpio::{etm::GpioEtmChannels, IO},
    peripherals::Peripherals,
    prelude::*,
};
use esp_backtrace as _;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let _clocks = ClockControl::boot_defaults(system.clock_control).freeze();

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
