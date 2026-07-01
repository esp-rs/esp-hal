//! Timer-woken light sleep with the CPU/TOP power domains powered down, with a
//! built-in software proof that power was actually removed.
//!
//! The example walks three sleep policies for the same timer wakeup:
//!
//! - **clock-gated** ([`RtcSleepConfig::default`]): a plain light sleep. The CPU is only
//!   clock-gated - execution resumes in place and no register state is lost.
//! - **cpu-powerdown** ([`with_cpu_power_down`]): the CPU domain is powered off during sleep. Its
//!   state is saved/restored in software (the ROM wake stub, see `cpu_retention`), so execution
//!   still resumes in place.
//! - **top-powerdown** ([`with_top_power_down`]): the whole digital `TOP` domain is powered off.
//!   The core system peripherals (interrupt matrix, HP system, TEE/APM, IO MUX, flash SPI mem,
//!   SysTimer, PCR clocks, console UART) lose their state, so the regDMA/PAU engine backs them up
//!   to RAM before sleep and the PMU restores them on wakeup. On the C6 this also powers down the
//!   CPU domain.
//!
//! ## Software proof (no instruments)
//!
//! [`cpu_power_down_wake_count`] is incremented from *inside* the ROM wake-stub
//! restore path, so it advances **only** when the CPU domain genuinely lost
//! power. For each round the example prints the wall-clock time actually spent
//! asleep (measured with the always-on RTC, which no sleep can stop) and that
//! counter:
//!
//! ```text
//! clock-gated   #1: slept ~1000 ms (RTC), CPU power-downs = 0
//! cpu-powerdown #1: slept ~1000 ms (RTC), CPU power-downs = 1
//! top-powerdown #1: slept ~1000 ms (RTC), CPU power-downs = 4
//! ```
//!
//! The counter stays `0` for the clock-gated rounds and increments for every
//! power-down round - that is the definitive proof the CPU domain lost power and
//! resumed through the wake stub. Every mode sleeps for the full duration (the
//! RTC confirms the chip idled, it did not busy-wait), and there is no ROM
//! reboot banner between rounds, which shows peripheral state survived.
//!
//! The *system timer* ([`esp_hal::time::Instant`]) keeps counting through light
//! sleep on the C6 (it stays clocked for timekeeping), so it is deliberately not
//! used as the proof here.
//!
//! ## Measuring current (e.g. Nordic PPK2)
//!
//! GPIO5 is driven high while awake and low while asleep, so it brackets each
//! sleep window on a PPK2/logic-analyzer digital channel. Power the module's
//! 3V3 rail from the PPK2 in source-meter mode (USB unplugged), wire GPIO5 to a
//! logic input, and average the current over a sleep plateau. Because the three
//! modes run back-to-back you get their sleep floors in one capture: each
//! deeper power-down should show a lower floor, with small current shoulders at
//! the window edges from the regDMA / CPU-context save & restore.

//% CHIP_FILTER: esp32c6

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    rtc_cntl::{
        Rtc,
        cpu_retention::cpu_power_down_wake_count,
        sleep::{RtcSleepConfig, TimerWakeupSource},
    },
    time::Duration,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Sleep duration per round, in milliseconds. Long enough to give a current
/// meter a wide, flat sleep plateau to average over.
const EVENT_MS: u64 = 1000;
/// Awake window between sleeps, in milliseconds, so the sleep plateaus are
/// clearly separated on a current/logic trace.
const AWAKE_MS: u32 = 200;
/// Rounds per mode.
const ROUNDS: u32 = 3;

/// Sleep once for `EVENT_MS`, then report the wall-clock time spent asleep (from
/// the always-on RTC) and the CPU power-down wake counter. `marker` is driven
/// low for the sleep window (high while awake) so a PPK2/logic channel can
/// bracket each sleep.
fn sleep_round(
    rtc: &mut Rtc<'_>,
    marker: &mut Output<'_>,
    delay: &Delay,
    config: &RtcSleepConfig,
    label: &str,
    round: u32,
) {
    let timer = TimerWakeupSource::new(Duration::from_millis(EVENT_MS));

    let rtc_before = rtc.time_since_power_up().as_micros();
    marker.set_low();
    rtc.sleep(config, &[&timer]);
    marker.set_high();
    let slept_ms = (rtc.time_since_power_up().as_micros() - rtc_before) / 1000;

    println!(
        "{} #{}: slept ~{} ms (RTC), CPU power-downs = {}",
        label,
        round,
        slept_ms,
        cpu_power_down_wake_count()
    );

    delay.delay_millis(AWAKE_MS);
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut rtc = Rtc::new(peripherals.LPWR);
    let delay = Delay::new();

    // Awake = high, asleep = low. The IO domain stays powered through light
    // sleep, so the pin holds its level and a meter/scope sees a clean window.
    let mut marker = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());

    // Same timer wakeup, increasingly aggressive power policies.
    let modes: [(&str, RtcSleepConfig); 3] = [
        ("clock-gated  ", RtcSleepConfig::default()),
        (
            "cpu-powerdown",
            RtcSleepConfig::default().with_cpu_power_down(true),
        ),
        (
            "top-powerdown",
            RtcSleepConfig::default().with_top_power_down(true),
        ),
    ];

    println!("up and running!");

    for (label, config) in &modes {
        for round in 1..=ROUNDS {
            sleep_round(&mut rtc, &mut marker, &delay, config, label, round);
        }
    }

    // Keep going in the deepest mode so the counter can be watched climbing and
    // a meter has a steady stream of identical sleep windows to average.
    let top = RtcSleepConfig::default().with_top_power_down(true);
    let mut round = ROUNDS;
    loop {
        round += 1;
        sleep_round(&mut rtc, &mut marker, &delay, &top, "top-powerdown", round);
    }
}
