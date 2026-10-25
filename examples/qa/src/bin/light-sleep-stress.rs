//! Light sleep under load.
//!
//! The program runs light sleeps of random length for [`RUN_FOR`], then prints the counts and
//! stops. The program checks that PSRAM keeps its data across each sleep and that each sleep
//! returns. On a chip with two cores, core 1 uses PSRAM and writes flash at the same time.
//!
//! The program prints the counts every 10 seconds. The program panics on the first bad word,
//! failed flash write, or failed check.
//!
//! Set [`CPU_POWER_DOWN`] to power the CPU domain down in each sleep.
//!
//! The flash writes erase the NVS partition of the default partition table, at offset `0x9000`.
//!
//! Use a UART port, with USB Serial/JTAG disconnected. esp-println prefers USB Serial/JTAG, and a
//! sleep breaks that connection.

//% CHIP_FILTER: sleep_light_sleep && sleep_has_wakeup_source_timer

#![no_std]
#![no_main]

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
    system::Cpu,
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

/// Prints a line and waits for the UART to send the line. A light sleep loses unsent UART data.
macro_rules! report {
    ($($arg:tt)*) => {{
        println!($($arg)*);
        let end = Instant::now() + Duration::from_millis(20);
        while Instant::now() < end {}
    }};
}

/// Whether the loop powers the CPU domain down in each light sleep.
///
/// The retention memory cannot be removed after installation, so one run tests one setting.
#[cfg(supports_cpu_power_down)]
const CPU_POWER_DOWN: bool = false;
/// Always `false` on a chip without CPU power-down.
#[cfg(not(supports_cpu_power_down))]
const CPU_POWER_DOWN: bool = false;

/// The duration of the run. Each flash write erases a sector, so the run stops to limit flash wear.
const RUN_FOR: Duration = Duration::from_secs(2 * 60 * 60);

/// The maximum duration of a loop step before the watchdog resets the chip.
const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(5);

const REPORT_EVERY: Duration = Duration::from_secs(10);

/// The minimum length of a long sleep.
const LONG_SLEEP: Duration = Duration::from_millis(500);

/// The number of PSRAM words that the loop writes before each sleep. 64 KiB is more than the cache.
const CORE0_WORDS: usize = 16 * 1024;

/// The number of words in one slice of the kept PSRAM. The loop checks one slice after each wake.
const SLICE_WORDS: usize = 16 * 1024;

/// The seed of the kept PSRAM pattern. Each slice adds the slice number to the seed. A slice that
/// reads the data of a different slice fails the check.
const KEPT_SEED: u32 = 0x6C07_8965;

fn pattern(index: usize, seed: u32) -> u32 {
    (index as u32).wrapping_mul(0x9E37_79B9) ^ seed
}

fn fill(words: &mut [u32], seed: u32) {
    for (i, word) in words.iter_mut().enumerate() {
        unsafe { core::ptr::write_volatile(word, pattern(i, seed)) };
    }
}

fn count_bad(words: &[u32], seed: u32) -> u32 {
    let mut bad = 0;
    for (i, word) in words.iter().enumerate() {
        if unsafe { core::ptr::read_volatile(word) } != pattern(i, seed) {
            bad += 1;
        }
    }
    bad
}

/// A xorshift generator. Each run uses the same sequence of sleep lengths.
struct Rng(u32);

impl Rng {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }
}

// The load that core 1 adds during the sleeps.
//
// Core 1 writes and checks a separate PSRAM region all the time. For every 16 sleeps, core 1 also
// writes one flash sector. `light-sleep-multi-core` checks that core 1 can request a sleep.
//
// Core 1 writes flash with the default `AutoPark` strategy. Each write stalls core 0. Core 0 enters
// the stall loop, which runs from RAM with interrupts masked. Core 1 starts the erase only after
// core 0 is in the stall loop. Core 0 masks interrupts for all of `sleep_light`. So core 0 is
// asleep or stalled, but never both, and the park never stops core 1 during an erase. The park
// stops core 1 in the PSRAM pass, or while core 1 waits for core 0 to enter the stall loop:
//
// ```text
// core 0  sets WRITE_FLASH   sleep_light: masks interrupts, parks core 1, sleeps, wakes,
//                            unparks core 1, unmasks interrupts
// core 1  PSRAM pass         sees the flag, posts the stall request, waits | parked | waits
//
// core 0  stall request: enters the stall loop ............... leaves it   checks, next sleep
// core 1  erases, writes, reads back, verifies, releases core 0            PSRAM pass
// ```
//
// The stall request is an interrupt. The stall request runs before core 0 masks interrupts, or
// waits until core 0 unmasks interrupts:
//
// ```text
// F  core 0 sets the flag        [S]  core 0 asleep, core 1 parked
// I  the stall request lands     [W]  core 0 in the stall loop, core 1 writing
//
// F[S]....[S]....[S] I [W]....[S]....   core 0 awake: the write runs before the next sleep
// F[S]....[S]....[S I S]....[W]..[S]    core 0 asleep: core 1 waits, parked, and writes after
// ```
//
// Core 1 waits with interrupts masked. With CPU power-down, a sleep that starts during this wait
// does not park core 1. The rendezvous gets no answer within 1 ms, and `sleep_light` returns
// `Ended` without a sleep. The flash write then runs before the next sleep.
#[cfg(multi_core)]
mod core1 {
    use core::sync::atomic::{AtomicU32, Ordering};

    use esp_hal::{
        flash::{Config, Flash},
        peripherals::{CPU_CTRL, FLASH},
        system::{CpuControl, Stack},
        time::{Duration, Instant},
    };
    use esp_println::println;

    use super::{count_bad, fill};

    /// The number of PSRAM words that core 1 writes and checks (512 KiB).
    pub const WORDS: usize = 128 * 1024;

    const FLASH_OFFSET: u32 = 0x9000;
    const FLASH_SECTOR: u32 = 4096;

    /// The number of sectors in the NVS partition. The writes rotate over these sectors to spread
    /// the wear.
    const FLASH_SECTORS: u32 = 6;

    /// Core 0 sets `WRITE_FLASH` to request one flash write. Core 1 clears `WRITE_FLASH` after the
    /// write.
    static WRITE_FLASH: AtomicU32 = AtomicU32::new(0);

    static BAD_WORDS: AtomicU32 = AtomicU32::new(0);
    static PASSES: AtomicU32 = AtomicU32::new(0);
    static FLASH_WRITES: AtomicU32 = AtomicU32::new(0);
    static FLASH_FAILURES: AtomicU32 = AtomicU32::new(0);

    /// Starts core 1. Core 1 writes and checks `words`, and writes flash on request from core 0.
    pub fn start(cpu_ctrl: CPU_CTRL<'static>, flash: FLASH<'static>, words: &'static mut [u32]) {
        static mut STACK: Stack<8192> = Stack::new();
        let flash = Flash::new(flash, Config::default()).unwrap();
        let mut cpu_control = CpuControl::new(cpu_ctrl);
        let guard = cpu_control
            .start_app_core(unsafe { &mut *core::ptr::addr_of_mut!(STACK) }, move || {
                run(flash, words)
            })
            .unwrap();
        core::mem::forget(guard);
    }

    fn run(mut flash: Flash<'static, esp_hal::Blocking>, words: &'static mut [u32]) -> ! {
        let mut sector = [0u32; FLASH_SECTOR as usize / 4];
        let mut seed = 1u32;
        loop {
            seed = seed.wrapping_add(1);
            if WRITE_FLASH.load(Ordering::Acquire) != 0 {
                fill(&mut sector, seed);
                let offset = FLASH_OFFSET
                    + FLASH_WRITES.load(Ordering::Relaxed) % FLASH_SECTORS * FLASH_SECTOR;
                let ok = unsafe {
                    flash.erase(offset, offset + FLASH_SECTOR).is_ok()
                        && flash.write(offset, &sector).is_ok()
                };
                sector.fill(0);
                let ok = ok && flash.read(offset, &mut sector).is_ok();
                if !ok || count_bad(&sector, seed) != 0 {
                    FLASH_FAILURES.fetch_add(1, Ordering::Relaxed);
                }
                FLASH_WRITES.fetch_add(1, Ordering::Relaxed);
                WRITE_FLASH.store(0, Ordering::Release);
            } else if !words.is_empty() {
                fill(words, seed);
                let bad = count_bad(words, seed);
                BAD_WORDS.fetch_add(bad, Ordering::Relaxed);
                PASSES.fetch_add(1, Ordering::Relaxed);
            } else {
                core::hint::spin_loop();
            }
        }
    }

    /// Requests one flash write from core 1.
    pub fn request_flash_write() {
        WRITE_FLASH.store(1, Ordering::Release);
    }

    /// Panics if core 1 read a bad word or a flash write failed.
    pub fn check(sleeps: u32) {
        let bad = BAD_WORDS.load(Ordering::Relaxed);
        let flash_failures = FLASH_FAILURES.load(Ordering::Relaxed);
        if bad != 0 || flash_failures != 0 {
            panic!(
                "sleep {sleeps}: core 1 read {bad} bad words, and {flash_failures} flash writes failed"
            );
        }
    }

    pub fn report() {
        report!(
            "stress: core 1 passes {}, flash writes {}",
            PASSES.load(Ordering::Relaxed),
            FLASH_WRITES.load(Ordering::Relaxed),
        );
    }
}

#[cfg(supports_cpu_power_down)]
#[esp_hal::ram(reclaimed, unstable(zeroed))]
static CPU_RETENTION_MEMORY: esp_hal::rtc_cntl::CpuRetentionStorage =
    esp_hal::rtc_cntl::CpuRetentionStorage::new();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    report!("light-sleep-stress: reset={:?}", reset_reason(Cpu::ProCpu));

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
    // The loop writes `core0_words` before each sleep, and checks the words after the wake. The
    // kept PSRAM keeps one pattern for the whole run. After each wake, the loop checks the next
    // slice of the kept PSRAM. The loop reads all of PSRAM every few hundred sleeps.
    let (core0_words, kept_words) = psram_words.split_at_mut(CORE0_WORDS.min(psram_words.len()));
    #[cfg(multi_core)]
    let (core1_words, kept_words) = kept_words.split_at_mut(core1::WORDS.min(kept_words.len()));
    let kept_words: &'static [u32] = {
        for (n, slice) in kept_words.chunks_mut(SLICE_WORDS).enumerate() {
            rtc.rwdt.feed();
            fill(slice, KEPT_SEED.wrapping_add(n as u32));
        }
        kept_words
    };
    let slices = kept_words.len().div_ceil(SLICE_WORDS);
    report!(
        "kept PSRAM: {} KiB in {slices} slices",
        kept_words.len() / 256
    );

    let mut lpwr = LowPower::new(peripherals.LPWR);

    #[cfg(supports_cpu_power_down)]
    if CPU_POWER_DOWN {
        lpwr.install_cpu_retention_memory(CPU_RETENTION_MEMORY.take())
            .unwrap();
    }

    #[cfg(multi_core)]
    core1::start(peripherals.CPU_CTRL, peripherals.FLASH, core1_words);

    report!("stress: start");

    let mut rng = Rng(0x2545_F491);
    let mut sleeps = 0u32;
    let mut short = 0u32;
    let mut long = 0u32;
    let mut refused = 0u32;
    let mut not_slept = 0u32;
    let mut next_report = Instant::now() + REPORT_EVERY;
    let end = Instant::now() + RUN_FOR;
    while Instant::now() < end {
        rtc.rwdt.feed();

        let seed = rng.next();
        fill(core0_words, seed);

        // Most sleeps last 1–10 ms. One in eight sleeps lasts less than 200 µs, so some wakeups
        // come during the sleep entry. One in 16 sleeps lasts 0.5–3 s.
        //
        // The sleep entry takes time for the cache write-back, the clock switches, and the park. A
        // deadline in that window makes the hardware reject the sleep or wake the chip at once. In
        // both cases, `sleep_light` returns `Ended`, so the loop cannot check the short sleeps. The
        // entry and the exit take an unknown part of a 1–10 ms sleep. At 0.5 s, the entry and the
        // exit take less than one percent, and the deadline cannot fall inside the entry.
        let random = rng.next();
        let length = match random % 16 {
            0 | 1 => {
                short += 1;
                Duration::from_micros(u64::from(random % 200))
            }
            2 => {
                long += 1;
                LONG_SLEEP + Duration::from_millis(u64::from(random % 2_500))
            }
            _ => Duration::from_micros(1_000 + u64::from(random % 9_000)),
        };

        #[cfg(multi_core)]
        if sleeps % 16 == 0 {
            core1::request_flash_write();
        }

        lpwr.set_wakeup_deadline(Instant::now() + length);
        let before = rtc.time_since_power_up();
        let result = lpwr.sleep_light(RtcSleepConfig::default());
        let slept = rtc.time_since_power_up() - before;
        if result == LightSleep::Refused {
            refused += 1;
        }
        sleeps += 1;

        // A sleep that returns at once passes every PSRAM check. So a long sleep must last at
        // least 90 percent of its length, and the timer must wake the chip.
        if result == LightSleep::Ended && length >= LONG_SLEEP {
            if CPU_POWER_DOWN && slept < length / 10 {
                // With CPU power-down on a two-core chip, core 1 can wait for a flash write with
                // interrupts masked. The rendezvous then gets no answer within 1 ms, and
                // `sleep_light` returns without a sleep. The count shows how often this occurs.
                // `wakeup_cause` returns the cause of the previous sleep, so the loop does not
                // check the cause.
                not_slept += 1;
            } else {
                let cause = wakeup_cause();
                if !cause.contains(WakeupSource::Timer) {
                    panic!("sleep {sleeps}: the wakeup cause is {cause:?}");
                }
                if slept < length * 9 / 10 {
                    panic!("sleep {sleeps}: slept {slept} of {length}");
                }
            }
        }

        let core0_bad = count_bad(core0_words, seed);
        if core0_bad != 0 {
            panic!("sleep {sleeps}: core 0 read {core0_bad} bad words");
        }
        if slices > 0 {
            let n = sleeps as usize % slices;
            let start = n * SLICE_WORDS;
            let end = kept_words.len().min(start + SLICE_WORDS);
            let kept_bad = count_bad(&kept_words[start..end], KEPT_SEED.wrapping_add(n as u32));
            if kept_bad != 0 {
                panic!("sleep {sleeps}: slice {n} of the kept PSRAM read {kept_bad} bad words");
            }
        }
        #[cfg(multi_core)]
        core1::check(sleeps);

        if Instant::now() > next_report {
            next_report += REPORT_EVERY;
            report!(
                "stress: sleeps {sleeps}, short {short}, long {long}, refused {refused}, not slept {not_slept}"
            );
            #[cfg(multi_core)]
            core1::report();
        }
    }

    report!(
        "stress: done, sleeps {sleeps}, short {short}, long {long}, refused {refused}, not slept {not_slept}"
    );
    #[cfg(multi_core)]
    core1::report();
    loop {
        rtc.rwdt.feed();
    }
}
