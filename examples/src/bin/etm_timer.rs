//! Control LED by the boot button via ETM without involving the CPU.

//! The following wiring is assumed:
//! - LED => GPIO2

//% CHIPS: esp32c6 esp32h2
//% FEATURES: esp-hal/unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    etm::Etm,
    gpio::{
        etm::{Channels, OutputConfig},
        Level,
        Output,
        Pull,
    },
    main,
    time::ExtU64,
    timer::{
        systimer::{etm::Event, SystemTimer},
        PeriodicTimer,
    },
};

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let mut led = Output::new(peripherals.GPIO2, Level::Low);
    led.set_high();

    let syst = SystemTimer::new(peripherals.SYSTIMER);
    let alarm = syst.alarm0;

    let timer_event = Event::new(&alarm);

    // setup ETM
    let gpio_ext = Channels::new(peripherals.GPIO_SD);
    let led_task = gpio_ext.channel0_task.toggle(
        led,
        OutputConfig {
            open_drain: false,
            pull: Pull::None,
            initial_state: Level::Low,
        },
    );

    let etm = Etm::new(peripherals.SOC_ETM);
    let channel0 = etm.channel0;

    // make sure the configured channel doesn't get dropped - dropping it will
    // disable the channel
    let _configured_channel = channel0.setup(&timer_event, &led_task);

    let mut timer = PeriodicTimer::new(alarm);
    timer.start(1u64.secs()).unwrap();

    // the LED is controlled by the button without involving the CPU
    loop {}
}
