//! Demonstrates waking from deep sleep with timer

//% CHIP_FILTER: sleep_driver_supported && sleep_deep_sleep

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    rtc_cntl::{
        SocResetReason,
        reset_reason,
        sleep::{LowPower, RtcSleepConfig},
        wakeup_cause,
    },
    system::Cpu,
    time::{Duration, Instant},
};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_log::init_uart!(peripherals);

    let delay = Delay::new();
    let mut lpwr = LowPower::new(peripherals.LPWR);

    log::info!("up and running!");
    let reason = reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    log::info!("reset reason: {:?}", reason);
    let wake_reason = wakeup_cause();
    log::info!("wake reason: {:?}", wake_reason);

    // The deadline is absolute, so the delay below does not shorten the sleep.
    lpwr.set_wakeup_deadline(Instant::now() + Duration::from_secs(5));

    log::info!("sleeping!");
    delay.delay_millis(100);
    lpwr.sleep_deep(RtcSleepConfig::deep());
}
