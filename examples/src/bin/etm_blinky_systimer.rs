//! Control LED by the systimer via ETM without involving the CPU.

//! The following wiring is assumed:
//! - LED => GPIO1

//% CHIPS: esp32c6 esp32h2

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    etm::Etm,
    gpio::{
        etm::{Channels, OutputConfig},
        Io,
        Level,
        Pull,
    },
    prelude::*,
    timer::systimer::{etm::Event, Periodic, SystemTimer},
};
use fugit::ExtU32;

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let syst = SystemTimer::new(peripherals.SYSTIMER);
    let syst_alarms = syst.split::<Periodic>();
    let mut alarm0 = syst_alarms.alarm0;
    alarm0.set_period(1u32.secs());

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut led = io.pins.gpio1;

    // setup ETM
    let gpio_ext = Channels::new(peripherals.GPIO_SD);
    let led_task = gpio_ext.channel0_task.toggle(
        &mut led,
        OutputConfig {
            open_drain: false,
            pull: Pull::None,
            initial_state: Level::High,
        },
    );

    let timer_event = Event::new(&mut alarm0);

    let etm = Etm::new(peripherals.SOC_ETM);
    let channel0 = etm.channel0;

    // make sure the configured channel doesn't get dropped - dropping it will
    // disable the channel
    let _configured_channel = channel0.setup(&timer_event, &led_task);

    // the LED is controlled by the timer without involving the CPU
    loop {}
}
