//! QA test for the wakeup deadline through the slow clock calibration before deep sleep.
//!
//! The test halves the stored slow clock period, and then arms the wakeup deadline. The deadline
//! has twice the ticks that it needs. The calibration before the sleep must convert it again, so
//! the chip sleeps for `SLEEP_MS`, not for twice that time.

// esp32c2, esp32c61 and esp32h4 have no rtc-fast memory, which is required by the persistent
// variables.
//% CHIP_FILTER: sleep_driver_supported && sleep_deep_sleep && !esp32c2 && !esp32c61 && !esp32h4
#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    rtc_cntl::{
        Rtc,
        SocResetReason,
        reset_reason,
        sleep::{LowPower, RtcSleepConfig},
    },
    system::Cpu,
    time::{Duration, Instant},
};

esp_bootloader_esp_idf::esp_app_desc!();

const SLEEP_MS: u64 = 1_000;

// The wake boots the chip again, which takes less than this.
const BOOT_MS: u64 = 300;

// Time since power-up just before the sleep, read with the calibrated period.
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut PRE_SLEEP_US: u64 = 0;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    qa_test::init_logger!(peripherals);

    let rtc = Rtc::new(peripherals.RTC_TIMER);
    let mut lpwr = LowPower::new(peripherals.LPWR);

    if reset_reason(Cpu::ProCpu) == Some(SocResetReason::CoreDeepSleep) {
        let slept_ms = (rtc.time_since_power_up().as_micros() - unsafe { PRE_SLEEP_US }) / 1_000;
        let pass = slept_ms < SLEEP_MS + BOOT_MS;
        log::info!(
            "Slept {} ms, expected {} ms [{}]",
            slept_ms,
            SLEEP_MS,
            if pass { "PASS" } else { "FAIL" }
        );

        loop {
            core::hint::spin_loop();
        }
    }

    log::info!("Sleeping {} ms with a stale slow clock period", SLEEP_MS);
    Delay::new().delay_millis(100);

    cfg_select! {
        any(feature = "esp32", feature = "esp32c3", feature = "esp32s2") => {
            let mut config = RtcSleepConfig::deep();
            config.set_rtc_fastmem_pd_en(false);
        }
        _ => {
            let config = RtcSleepConfig::deep();
        }
    }

    unsafe {
        PRE_SLEEP_US = rtc.time_since_power_up().as_micros();
    }

    halve_slow_clk_period();
    lpwr.set_wakeup_deadline(Instant::now() + Duration::from_millis(SLEEP_MS));

    lpwr.sleep_deep(config);
}

/// Halves the stored slow clock period.
fn halve_slow_clk_period() {
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
