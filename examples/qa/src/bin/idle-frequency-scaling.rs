//! Checks the idle frequency scaling of `wait_for_interrupt`.
//!
//! The test wakes the CPU 100 times from `wait_for_interrupt` with an alarm, once with idle
//! frequency scaling and once while a `CpuFrequencyLock` is held. For each run, it prints the delay
//! from the alarm to the interrupt handler, and the CPU frequency that the handler measures. The
//! handler must always see the full CPU frequency.
//!
//! Then the test idles for 10 seconds with idle frequency scaling, and for 10 seconds while a
//! `CpuFrequencyLock` is held, so that the current consumption can be compared.
//!
//! On chips that switch PSRAM to a low speed during idle, the test fills 2 MiB of PSRAM with a
//! pattern, and checks the pattern after each run and each idle phase. The buffer is larger than
//! the cache, so most of the pattern comes back from PSRAM.

//% CHIP_FILTER: sleep_idle_frequency_scaling
//% ENV: ESP_HAL_CONFIG_IDLE_FREQUENCY_SCALING=true

#![no_std]
#![no_main]

use core::{
    cell::{Cell, RefCell},
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use embassy_sync::blocking_mutex::CriticalSectionMutex;
use esp_backtrace as _;
use esp_hal::{
    Blocking,
    clock::{self, CpuClock, CpuFrequencyLock},
    handler,
    interrupt::wait_for_interrupt,
    main,
    time::{Duration, Instant},
    timer::{OneShotTimer, systimer::SystemTimer},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

static TIMER: CriticalSectionMutex<RefCell<Option<OneShotTimer<'static, Blocking>>>> =
    CriticalSectionMutex::new(RefCell::new(None));
static HANDLER_ENTRY_US: CriticalSectionMutex<Cell<u64>> = CriticalSectionMutex::new(Cell::new(0));
static HANDLER_MHZ: AtomicU32 = AtomicU32::new(0);
static FIRED: AtomicBool = AtomicBool::new(false);

fn cpu_cycles() -> u32 {
    let cycles: u32;
    cfg_select! {
        any(feature = "esp32c6", feature = "esp32h2") => unsafe {
            core::arch::asm!("csrr {0}, 0x7e2", out(reg) cycles)
        },
        _ => unsafe { core::arch::asm!("csrr {0}, mcycle", out(reg) cycles) },
    }
    cycles
}

#[handler]
fn alarm_handler() {
    let entry = Instant::now();
    TIMER.lock(|timer| timer.borrow_mut().as_mut().unwrap().clear_interrupt());

    let start_cycles = cpu_cycles();
    let start = Instant::now();
    while start.elapsed() < Duration::from_micros(50) {}
    let mhz = cpu_cycles().wrapping_sub(start_cycles) / start.elapsed().as_micros() as u32;

    HANDLER_ENTRY_US.lock(|e| e.set(entry.duration_since_epoch().as_micros()));
    HANDLER_MHZ.store(mhz, Ordering::Relaxed);
    FIRED.store(true, Ordering::Release);
}

/// Waits in `wait_for_interrupt` until the alarm fires. Returns the delay from the alarm to the
/// handler in microseconds, and the CPU frequency that the handler measured in MHz.
fn wake_once(delay: Duration) -> (u64, u32) {
    FIRED.store(false, Ordering::Relaxed);
    let expected = Instant::now() + delay;
    TIMER.lock(|timer| {
        timer.borrow_mut().as_mut().unwrap().schedule(delay).unwrap();
    });

    while !FIRED.load(Ordering::Acquire) {
        wait_for_interrupt();
    }

    let entry = HANDLER_ENTRY_US.lock(|e| e.get());
    let latency = entry.saturating_sub(expected.duration_since_epoch().as_micros());
    (latency, HANDLER_MHZ.load(Ordering::Relaxed))
}

fn measure(label: &str) {
    const WAKES: u64 = 100;
    let full_mhz = clock::cpu_clock().as_mhz();

    let mut min = u64::MAX;
    let mut max = 0;
    let mut sum = 0;
    let mut min_mhz = u32::MAX;
    let mut slow_wakes = 0;
    for _ in 0..WAKES {
        let (latency, mhz) = wake_once(Duration::from_millis(2));
        min = min.min(latency);
        max = max.max(latency);
        sum += latency;
        min_mhz = min_mhz.min(mhz);
        if mhz < full_mhz * 9 / 10 {
            slow_wakes += 1;
        }
    }

    println!(
        "{}: latency min {} us, avg {} us, max {} us; lowest handler CPU clock {} MHz of {} MHz; {} slow wakes",
        label,
        min,
        sum / WAKES,
        max,
        min_mhz,
        full_mhz,
        slow_wakes
    );
}

#[cfg(psram_idle_low_speed_switch)]
struct PsramPattern(&'static mut [u32]);

#[cfg(psram_idle_low_speed_switch)]
impl PsramPattern {
    const WORDS: usize = 2 * 1024 * 1024 / 4;

    fn new(psram: &esp_hal::psram::Psram) -> Self {
        let (start, size) = psram.raw_parts();
        let words = Self::WORDS.min(size / 4);
        let buffer = unsafe { core::slice::from_raw_parts_mut(start.cast::<u32>(), words) };
        for (i, word) in buffer.iter_mut().enumerate() {
            *word = Self::value(i);
        }
        println!("PSRAM: {} bytes mapped, checking {} bytes", size, words * 4);
        Self(buffer)
    }

    fn value(index: usize) -> u32 {
        (index as u32).wrapping_mul(0x9E37_79B9) ^ 0xA5A5_5A5A
    }

    fn check(&self, label: &str) {
        let errors = self
            .0
            .iter()
            .enumerate()
            .filter(|&(i, &word)| word != Self::value(i))
            .count();
        println!("PSRAM check after {}: {} bad words", label, errors);
    }
}

fn idle_for(duration: Duration) {
    let end = Instant::now() + duration;
    while Instant::now() < end {
        wake_once(Duration::from_millis(100));
    }
}

#[main]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    let mut timer = OneShotTimer::new(systimer.alarm0);
    timer.set_interrupt_handler(alarm_handler);
    timer.listen();
    TIMER.lock(|t| t.replace(Some(timer)));

    println!("CPU clock: {} MHz", clock::cpu_clock().as_mhz());

    #[cfg(psram_idle_low_speed_switch)]
    let psram = esp_hal::psram::Psram::new(peripherals.PSRAM, Default::default());
    #[cfg(psram_idle_low_speed_switch)]
    let pattern = PsramPattern::new(&psram);

    measure("Scaling allowed");
    #[cfg(psram_idle_low_speed_switch)]
    pattern.check("the run with scaling");
    {
        let _lock = CpuFrequencyLock::new();
        measure("CpuFrequencyLock held");
    }

    loop {
        println!("Idle for 10 s, scaling allowed");
        idle_for(Duration::from_secs(10));
        #[cfg(psram_idle_low_speed_switch)]
        pattern.check("the idle phase with scaling");

        println!("Idle for 10 s, CpuFrequencyLock held");
        let _lock = CpuFrequencyLock::new();
        idle_for(Duration::from_secs(10));
    }
}
