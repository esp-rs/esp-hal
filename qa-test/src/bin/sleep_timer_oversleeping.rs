//! QA test for deep-sleep timer oversleeping bug.
//!
//! Issue: <https://github.com/esp-rs/esp-hal/issues/5183>

//% CHIPS: esp32 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3

// TODO: esp32c2
#![no_std]
#![no_main]
use embassy_executor::Spawner;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    rtc_cntl::{self, Rtc, sleep::RtcSleepConfig},
    timer::timg::TimerGroup,
};
use esp_println::println;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// How long to sleep each iteration (matches original reproducer).
const SLEEP_MS: u64 = 30;
const MAX_ITERATIONS: u32 = 100;

const THRESHOLD_US: u64 = SLEEP_MS * SLEEP_MS * 1_000;

#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut BOOT_COUNT: u32 = 0;

// LP-timer microseconds captured just before the last sleep.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut PRE_SLEEP_US: u64 = 0;

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 65536);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    let mut rtc = Rtc::new(peripherals.LPWR);

    let boot_count = unsafe { BOOT_COUNT };
    let pre_sleep_us = unsafe { PRE_SLEEP_US };

    if boot_count == 0 {
        // Set RTC timer to 0 so our arithmetic starts from a clean state
        rtc.set_current_time_us(0);
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
            if pass { "PASS" } else { "FAIL - oversleep!" },
        );

        if !pass {
            println!(
                "FAIL: slept {} ms, threshold {} ms",
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
        PRE_SLEEP_US = rtc.current_time_us();
    }

    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32c6", feature = "esp32h2", feature = "esp32s3"))] {
            let config = RtcSleepConfig::deep();
        } else { //esp32/c3/s2
            let mut config = RtcSleepConfig::deep();
            config.set_rtc_fastmem_pd_en(false);
        }
    }

    let delay = esp_hal::delay::Delay::new();

    let timer = esp_hal::rtc_cntl::sleep::TimerWakeupSource::new(
        core::time::Duration::from_millis(SLEEP_MS),
    );

    delay.delay_millis(100);

    rtc.sleep(&config, &[&timer]);
}
