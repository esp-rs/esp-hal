//! QA test for two deep-sleep timer bugs.
//!
//! Each iteration sleeps for `SLEEP_MS` and checks how much RTC time passed. The check fails when:
//!
//! - the chip oversleeps: <https://github.com/esp-rs/esp-hal/issues/5183>
//! - the RTC time jumps when esp-hal recalibrates the slow clock. See `make_calibration_stale`.
//!
//! On esp32, esp32c3, esp32s2 and esp32s3, also run the test with `RTC_SLOW_CLK` set to
//! `RtcSlowClkConfig::RcFast`. RC_FAST must keep running through the sleep.

// esp32c2, esp32c61 and esp32h4 have no rtc-fast memory, which is required by the persistent
// BOOT_COUNT variable.
//% CHIP_FILTER: sleep_driver_supported && sleep_deep_sleep && !esp32c2 && !esp32c61 && !esp32h4
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    rtc_cntl::{
        self,
        Rtc,
        sleep::{LowPower, RtcSleepConfig},
    },
    timer::timg::TimerGroup,
};
use esp_println::println;
use qa_test::DRAM2_UNINIT_SIZE;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// How long to sleep each iteration (matches original reproducer).
const SLEEP_MS: u64 = 30;
const MAX_ITERATIONS: u32 = 100;

const THRESHOLD_US: u64 = SLEEP_MS * SLEEP_MS * 1_000;

#[cfg(any(
    feature = "esp32",
    feature = "esp32c3",
    feature = "esp32s2",
    feature = "esp32s3"
))]
const RTC_SLOW_CLK: esp_hal::clock::ll::RtcSlowClkConfig =
    esp_hal::clock::ll::RtcSlowClkConfig::RcSlow;

// Each stale calibration moves the RTC time back by half the time since power-up, and the time
// keeps that offset. Start one day after 0, so the time does not wrap around.
const START_TIME_US: u64 = 24 * 60 * 60 * 1_000_000;

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut BOOT_COUNT: u32 = 0;

// RTC time just before the last sleep, read with the stale calibration.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut PRE_SLEEP_US: u64 = 0;

#[esp_hal::main]
async fn main(_spawner: Spawner) {
    cfg_select! {
        any(
            feature = "esp32",
            feature = "esp32c3",
            feature = "esp32s2",
            feature = "esp32s3"
        ) => {
            let mut clock_config: esp_hal::clock::ll::ClockConfig = CpuClock::max().into();
            clock_config.rtc_slow_clk = Some(RTC_SLOW_CLK);
        }
        _ => {
            let clock_config = CpuClock::max();
        }
    }
    let config = esp_hal::Config::default().with_cpu_clock(clock_config);
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 32768);
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: DRAM2_UNINIT_SIZE);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let rtc = Rtc::new(peripherals.RTC_TIMER);
    let mut lpwr = LowPower::new(peripherals.LPWR);

    let boot_count = unsafe { BOOT_COUNT };
    let pre_sleep_us = unsafe { PRE_SLEEP_US };

    if boot_count == 0 {
        rtc.set_current_time_us(START_TIME_US);
        println!(
            "Starting oversleep regression test ({} iterations, {} ms sleep)",
            MAX_ITERATIONS, SLEEP_MS
        );
    } else {
        let wake_reason = rtc_cntl::wakeup_cause();
        let now_us = rtc.current_time_us();

        let elapsed_us = now_us.saturating_sub(pre_sleep_us);
        let elapsed_ms = elapsed_us / 1_000;
        let pass = elapsed_us <= THRESHOLD_US;

        println!(
            "Iteration {} - woke after {} ms (wake={:?}) [{}]",
            boot_count,
            elapsed_ms,
            wake_reason,
            if pass { "PASS" } else { "FAIL" },
        );

        if !pass {
            println!(
                "FAIL: {} ms passed, threshold {} ms. The chip overslept, or the RTC time jumped at \
                 the slow clock recalibration.",
                elapsed_ms,
                THRESHOLD_US / 1_000,
            );
            loop {
                core::hint::spin_loop();
            }
        }

        if boot_count >= MAX_ITERATIONS {
            loop {
                core::hint::spin_loop();
            }
        }
    }

    let next_iteration = boot_count + 1;
    println!("Iteration {} - sleeping {} ms", next_iteration, SLEEP_MS);

    unsafe {
        BOOT_COUNT = next_iteration;
    }

    cfg_select! {
        any(feature = "esp32", feature = "esp32c3", feature = "esp32s2") => {
            let mut config = RtcSleepConfig::deep();
            config.set_rtc_fastmem_pd_en(false);
        }
        _ => {
            let config = RtcSleepConfig::deep();
        }
    }

    let delay = esp_hal::delay::Delay::new();

    // Send the console output before the code arms the deadline. The deadline is absolute, so a
    // delay after the arming makes the sleep 30 ms shorter.
    delay.delay_millis(100);

    lpwr.set_wakeup_deadline(
        esp_hal::time::Instant::now() + esp_hal::time::Duration::from_millis(SLEEP_MS),
    );

    make_calibration_stale();

    unsafe {
        PRE_SLEEP_US = rtc.current_time_us();
    }

    lpwr.sleep_deep(config);
}

/// Halves the stored slow clock period, so that the next recalibration changes it.
///
/// esp-hal recalibrates the slow clock at each boot, and before a deep sleep on chips with a PMU.
/// The RTC time must stay the same through the recalibration. If it does not, it jumps forward by
/// half the time since power-up. That grows with each iteration, and passes the threshold after a
/// few iterations.
///
/// Call this after the wakeup deadline is armed. The deadline is converted to ticks with the
/// stored period, so the stale period does not change the sleep length.
fn make_calibration_stale() {
    let reg = cfg_select! {
        any(
            feature = "esp32",
            feature = "esp32s2",
            feature = "esp32s3",
            feature = "esp32c3"
        ) => esp_hal::peripherals::LPWR::regs().store1(),
        feature = "esp32p4" => esp_hal::peripherals::LP_AON::regs().lp_store1(),
        feature = "esp32s31" => esp_hal::peripherals::LP_SYS::regs().lp_store(1),
        _ => esp_hal::peripherals::LP_AON::regs().store1(),
    };

    reg.modify(|r, w| unsafe { w.bits(r.bits() / 2) });
}
