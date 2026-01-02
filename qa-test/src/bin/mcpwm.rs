//! MCPWM Example
//!
//! Uses timer0 and operator0 of the MCPWM0 peripheral to output a 50% duty
//! signal at 20 kHz.
//!
//! The signal will be output to the pin assigned to `pin`. (GPIO0)

//% CHIPS: esp32 esp32c6 esp32h2 esp32s3

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    main,
    mcpwm::{McPwm, PeripheralClockConfig, operator::PwmPinConfig, timer::PwmWorkingMode},
    peripherals::Peripherals,
    time::Rate,
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals: Peripherals = esp_hal::init(esp_hal::Config::default());
    let pin = peripherals.GPIO0;

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32h2")] {
            let clock_cfg =
                PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
        } else if #[cfg(not(feature = "esp32h2"))] {
            let clock_cfg =
                PeripheralClockConfig::with_frequency(Rate::from_mhz(32)).unwrap();
        }
    }

    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);

    let mut pwm_pin = mcpwm
        .operator0
        .with_pin_a(pin, PwmPinConfig::UP_ACTIVE_HIGH);

    // start timer with timestamp values in the range of 0..=99 and a frequency of
    // 20 kHz
    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    // pin will be high 50% of the time
    pwm_pin.set_timestamp(50);

    loop {}
}
