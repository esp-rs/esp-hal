//! Uses timer0 and operator0 of the MCPWM0 peripheral to output a 50% duty
//! signal at 20 kHz.
//!
//! The following wiring is assumed:
//! - Output pin => GPIO0

//% CHIPS: esp32 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, McPwm, PeripheralClockConfig},
    prelude::*,
};

#[entry]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let pin = peripherals.GPIO0;

    // initialize peripheral
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let freq = 40.MHz();
        } else {
            let freq = 32.MHz();
        }
    }

    let clock_cfg = PeripheralClockConfig::with_frequency(freq).unwrap();

    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);

    // connect operator0 to timer0
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    // connect operator0 to pin
    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, 20.kHz())
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // pin will be high 50% of the time
    pwm_pin.set_timestamp(50);

    loop {}
}
