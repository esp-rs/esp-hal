//! Demonstrates clock setup and LP_TIMER's (lack of) accuracy

//% CHIPS: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    clock::{CpuClock, ll},
    delay::Delay,
    main,
    rtc_cntl::{Rtc, SocResetReason, reset_reason, sleep::TimerWakeupSource, wakeup_cause},
    system::Cpu,
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let mut cpu_clock_config: ll::ClockConfig = CpuClock::default().into();

    // TODO: aspirational example, not supported yet
    // cpu_clock_config.lp_slow_clk = Some(ll::LpSlowClkConfig::Xtal32k);

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(cpu_clock_config));

    let mut rtc = Rtc::new(peripherals.LPWR);

    let start = Instant::now();
    rtc.set_current_time_us(start.duration_since_epoch().as_micros());

    let mut prev = Instant::now();
    loop {
        let next = prev + Duration::from_millis(1000);
        while Instant::now() < next {}
        prev = next;

        let time_since_start = start.elapsed();
        let time_since_start_lp = Duration::from_micros(rtc.current_time_us());

        println!(
            "Micros since start: system = {}, LP = {}",
            time_since_start, time_since_start_lp
        );
        println!(
            "Difference: {}",
            Duration::from_micros(
                time_since_start_lp
                    .as_micros()
                    .abs_diff(time_since_start.as_micros())
            )
        );
    }
}
