//! Light sleep on a chip with two cores.
//!
//! The program checks that either core can request a light sleep, without and with CPU retention,
//! and that each of the four sleeps happened: the call returned after at least 90 percent of the
//! time to the wakeup deadline, and the timer woke the chip.
//!
//! On a chip that retains the CPU in software, core 1 cannot sleep the chip itself. Its request
//! goes to core 0, which runs the sleep, parks core 1 for it and releases core 1 after the wake.
//! With CPU retention installed, every light sleep powers the CPU domain down, so each core saves
//! its own registers before the sleep and restores them after, and core 1 joins the sleep through
//! the rendezvous with core 0. Between the requests core 1 spins with interrupts enabled, so the
//! rendezvous reaches it. On the ESP32 and ESP32-S3, core 1 runs the sleep itself and parks core 0.
//!
//! The program fills PSRAM with a pattern at start. It runs the two requests without CPU
//! retention, then checks all of PSRAM. On a chip that can power the CPU down, it installs the
//! retention memory and runs both again. Each check prints its name before it runs, and panics on
//! a wrong result. If a check hangs, the watchdog resets the chip. The next boot prints the reset
//! reason, and the last check in the log is the one that hung. The program prints
//! `light-sleep-multi-core: done` after the last check.
//!
//! `light-sleep-stress` runs core 0's request many times under load.
//!
//! Use a UART port, with USB Serial/JTAG unplugged. esp-println prefers JTAG, and a sleep breaks
//! that connection.

//% CHIP_FILTER: multi_core && sleep_light_sleep && sleep_has_wakeup_source_timer

#![no_std]
#![no_main]

use core::{
    ptr::addr_of_mut,
    sync::atomic::{AtomicU32, Ordering},
};

use esp_backtrace as _;
use esp_hal::{
    main,
    rtc_cntl::{
        Rtc,
        RwdtStage,
        WakeupSource,
        reset_reason,
        sleep::{LightSleep, LowPower, RtcSleepConfig},
        wakeup_cause,
    },
    system::{Cpu, CpuControl, Stack},
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Prints a line and waits until the UART has sent it, because a light sleep loses what the UART
/// still holds.
macro_rules! report {
    ($($arg:tt)*) => {{
        println!($($arg)*);
        let end = Instant::now() + Duration::from_millis(20);
        while Instant::now() < end {}
    }};
}

/// How long a check may take before the watchdog resets the chip.
const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(5);

/// The time from each request to its wakeup deadline.
const SLEEP_LENGTH: Duration = Duration::from_millis(20);

/// The command from core 0 to core 1. Core 1 sets it back to `IDLE` when the command is done.
static COMMAND: AtomicU32 = AtomicU32::new(IDLE);

const IDLE: u32 = 0;
/// Core 1 requests a light sleep itself.
const SLEEP: u32 = 1;

/// The words of PSRAM that take one seed.
const SLICE_WORDS: usize = 16 * 1024;

/// The seed of the pattern in PSRAM. Each slice adds its number, so a slice that reads the words of
/// another slice fails.
const PSRAM_SEED: u32 = 0x6C07_8965;

fn pattern(index: usize, seed: u32) -> u32 {
    (index as u32).wrapping_mul(0x9E37_79B9) ^ seed
}

fn fill_psram(words: &mut [u32], rtc: &mut Rtc<'_>) {
    for (n, slice) in words.chunks_mut(SLICE_WORDS).enumerate() {
        rtc.rwdt.feed();
        for (i, word) in slice.iter_mut().enumerate() {
            unsafe {
                core::ptr::write_volatile(word, pattern(i, PSRAM_SEED.wrapping_add(n as u32)))
            };
        }
    }
}

/// Panics if a word of PSRAM lost its pattern.
fn check_psram(words: &[u32], rtc: &mut Rtc<'_>, with_retention: &str) {
    for (n, slice) in words.chunks(SLICE_WORDS).enumerate() {
        rtc.rwdt.feed();
        let seed = PSRAM_SEED.wrapping_add(n as u32);
        let bad = slice
            .iter()
            .enumerate()
            .filter(|&(i, word)| unsafe { core::ptr::read_volatile(word) } != pattern(i, seed))
            .count();
        if bad != 0 {
            panic!("slice {n} of PSRAM read {bad} bad words after the checks {with_retention}");
        }
    }
    report!("check: PSRAM ok, {with_retention}");
}

fn app_core_entry() -> ! {
    loop {
        match COMMAND.load(Ordering::Acquire) {
            SLEEP => {
                let mut lpwr = LowPower::new(unsafe { esp_hal::peripherals::LPWR::steal() });
                let _ = lpwr.sleep_light(RtcSleepConfig::default());
                COMMAND.store(IDLE, Ordering::Release);
            }
            _ => core::hint::spin_loop(),
        }
    }
}

/// Posts `command` to core 1, and waits up to `limit` for core 1 to finish it.
fn run_on_core1(command: u32, limit: Duration) -> bool {
    COMMAND.store(command, Ordering::Release);
    let end = Instant::now() + limit;
    while COMMAND.load(Ordering::Acquire) != IDLE {
        if Instant::now() > end {
            return false;
        }
    }
    true
}

/// Panics unless the sleep that started at `start` slept most of [`SLEEP_LENGTH`] and the timer
/// woke the chip. A request that returned without sleeping fails the first check.
fn check_slept(start: Instant, who: &str) {
    let slept = Instant::now() - start;
    if slept < SLEEP_LENGTH * 9 / 10 {
        panic!("{who}'s light sleep returned after {slept} of {SLEEP_LENGTH}");
    }
    if !wakeup_cause().contains(WakeupSource::Timer) {
        panic!("{who}'s light sleep woke on {:?}", wakeup_cause());
    }
}

/// Runs the two checks, core 0's request and then core 1's. Each panics on a wrong result.
fn checks(lpwr: &mut LowPower<'_>, with_retention: &str) {
    report!("check: core 0 requests a light sleep, {with_retention}");
    lpwr.set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
    let start = Instant::now();
    if lpwr.sleep_light(RtcSleepConfig::default()) == LightSleep::Refused {
        panic!("core 0's light sleep was refused");
    }
    check_slept(start, "core 0");
    report!("check: ok");

    report!("check: core 1 requests a light sleep, {with_retention}");
    lpwr.set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
    let start = Instant::now();
    if !run_on_core1(SLEEP, Duration::from_secs(1)) {
        panic!("core 1's light sleep did not return");
    }
    check_slept(start, "core 1");
    report!("check: ok");
}

#[cfg(supports_cpu_power_down)]
#[esp_hal::ram(reclaimed, unstable(zeroed))]
static CPU_RETENTION_MEMORY: esp_hal::rtc_cntl::CpuRetentionStorage =
    esp_hal::rtc_cntl::CpuRetentionStorage::new();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    report!(
        "light-sleep-multi-core: reset={:?}",
        reset_reason(Cpu::ProCpu)
    );

    let mut rtc = Rtc::new(peripherals.RTC_TIMER);
    rtc.rwdt.set_timeout(RwdtStage::Stage0, WATCHDOG_TIMEOUT);
    rtc.rwdt.enable();

    #[cfg(psram_driver_supported)]
    let psram_words: &'static mut [u32] = {
        let psram =
            esp_hal::psram::Psram::new(peripherals.PSRAM, esp_hal::psram::PsramConfig::default());
        let (start, size) = psram.raw_parts();
        if size == 0 {
            &mut []
        } else {
            unsafe { core::slice::from_raw_parts_mut(start.cast::<u32>(), size / 4) }
        }
    };
    #[cfg(not(psram_driver_supported))]
    let psram_words: &'static mut [u32] = &mut [];

    report!("PSRAM: {} KiB", psram_words.len() / 256);
    fill_psram(psram_words, &mut rtc);

    let mut lpwr = LowPower::new(peripherals.LPWR);

    static mut APP_CORE_STACK: Stack<8192> = Stack::new();
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, || {
            app_core_entry()
        })
        .unwrap();
    core::mem::forget(guard);

    rtc.rwdt.feed();
    checks(&mut lpwr, "without CPU retention");
    check_psram(psram_words, &mut rtc, "without CPU retention");

    #[cfg(supports_cpu_power_down)]
    {
        lpwr.install_cpu_retention_memory(CPU_RETENTION_MEMORY.take())
            .unwrap();
        rtc.rwdt.feed();
        checks(&mut lpwr, "with CPU retention");
        check_psram(psram_words, &mut rtc, "with CPU retention");
    }

    report!("light-sleep-multi-core: done");
    rtc.rwdt.disable();
    loop {}
}
