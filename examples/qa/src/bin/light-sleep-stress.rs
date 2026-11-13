//! Light sleep under load.
//!
//! The program runs a list of phases. Before each phase after the first, the program resets the
//! chip in software. The phase then starts from `esp_hal::init` at its own CPU clock, without the
//! CPU retention of an earlier phase. Each phase runs for [`PHASE_FOR`]. A phase sets three things:
//!
//! - How the program requests each sleep: `sleep_light` from the main loop, `sleep_light` from a
//!   high-priority esp-rtos thread while a low-priority thread works, or the automatic light sleep
//!   of esp-rtos. The main loop calls the driver without esp-rtos. The thread calls it through the
//!   esp-rtos scheduler, with its timer queue and its critical sections. The idle hook of esp-rtos
//!   sets the wakeup deadline from the timer queue. The automatic phases do not call `sleep_light`,
//!   so the program checks the length of each wait instead. At least one light sleep must wake on
//!   the timer in an automatic phase.
//! - The CPU clock: the default or the maximum. Each sleep switches the root clock to XTAL and
//!   back, and the two settings return to different PLL configurations.
//! - CPU retention: off or on, on a chip that can power the CPU domain down. With CPU retention,
//!   each sleep saves the CPU state, powers the CPU domain down, and restores the state after the
//!   wake. The PLLs then start again. On a chip with two cores, the other core joins the sleep
//!   through the rendezvous.
//!
//! The phases without CPU retention run first, so that a sleep with CPU retention that never ends
//! does not hang the test before they run.
//!
//! Before the first sleep of each phase, the program writes to UART1 and drops the driver without a
//! flush. The UART then has bytes in its transmit FIFO and no clocks. The sleep entry must not wait
//! for that UART.
//!
//! In each phase, each core runs these parts:
//!
//! - Sleeps of random length check the timer wakeup and the time update after the wake. One sleep
//!   in 16 is long, 0.5–3 s. A long sleep must end within [`TOLERANCE`] of its deadline, and the
//!   timer must be the wakeup cause. Most sleeps last 1–10 ms. The program does not check their
//!   length, because the entry and the exit take an unknown part of it.
//! - One sleep in eight has a deadline 0–200 µs ahead, which falls before or near the end of the
//!   sleep entry. The hardware then rejects the sleep or wakes the chip at once, and `sleep_light`
//!   returns `Ended` in both cases. The program cannot check the length of such a sleep, so it
//!   checks that the call returns and that the checks after it pass.
//! - Before each sleep, the core writes a new pattern to 128 KiB of PSRAM, or 1 MiB on the
//!   ESP32-P4. It checks the pattern after the wake. A wrong word shows a cache line that was not
//!   written back before the sleep, or PSRAM that lost data across the sleep.
//! - The rest of PSRAM keeps one pattern for the whole phase, and the core checks part of it after
//!   each wake. A wrong word there shows that PSRAM, or its clock, did not survive a sleep.
//! - Between sleeps, the core reads, checks, and writes back part of the kept PSRAM, so that cache
//!   lines are dirty when a sleep starts.
//! - About every 100 ms, a core that is not sleeping erases, writes, and reads back one flash
//!   sector of the free space after the image. The sector must read back what was written. On a
//!   chip with two cores, each flash operation first parks the other core, which masks its
//!   interrupts and waits in RAM until the operation ends. A core that wakes while an operation
//!   waits for it joins that park before its sleep call returns. So a sleep or a wait during a
//!   flash cycle does not get an upper bound.
//! - On a chip with two cores, a random core requests each sleep while the other core works. The
//!   working core must return from the park, and with CPU retention from the rendezvous, with its
//!   state intact. A core that makes no progress for [`STALL_REPORTS`] reports fails the phase.
//!
//! Each phase ends with `PASS <phase>` or `FAIL <phase>: <reason>`. When a check fails, the
//! program prints the wrong value at once, on a line that starts with `core <n>:`. To repeat a
//! phase, set [`SEED`] and [`FIRST_PHASE`] to the seed and the number that its `RUN` line printed.
//!
//! The progress of the run is kept in memory that a reset does not clear. When a phase hangs
//! outside a sleep or panics, the RTC watchdog resets the chip. The next boot reports that phase as
//! failed, with the reset reason, and runs the next phase. A reset does not clear all of the chip's
//! state, so a phase after a failed phase can fail because of it. A power-on reset starts the run
//! again at [`FIRST_PHASE`]. A sleep that never ends hangs the test, because the RTC watchdog is
//! not active during a light sleep.
//!
//! The console is on the UART, because USB Serial/JTAG stops across sleep.

//% CHIP_FILTER: sleep_light_sleep && sleep_has_wakeup_source_timer
//% FEATURES: esp-println/uart

#![no_std]
#![no_main]

use core::{
    fmt,
    ptr::{addr_of, addr_of_mut},
};

use esp_backtrace as _;
use esp_hal::{
    Blocking,
    clock::CpuClock,
    flash::{Config as FlashConfig, Flash},
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
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart},
};
use esp_rtos::{CurrentThreadHandle, thread::ThreadSpawner};
use portable_atomic::{AtomicBool, AtomicU32, Ordering};
use qa_test::{Progress, ProgressWord, free_flash, report};

esp_bootloader_esp_idf::esp_app_desc!();

/// Prints the reason for a failure, and makes core 0 fail the phase.
macro_rules! problem {
    ($($arg:tt)*) => {{
        report!("  {}", format_args!($($arg)*));
        FAILED.store(true, Ordering::Release);
    }};
}

/// The duration of each phase.
const PHASE_FOR: Duration = Duration::from_secs(10 * 60);

/// The seed of the random sequence, or `None` for a seed from the RNG. To repeat a failed phase,
/// set the seed that its `RUN` line printed.
const SEED: Option<u32> = None;

/// The phase that a run starts at. To run a phase first, set the number that its `RUN` line
/// printed.
const FIRST_PHASE: usize = 0;

/// The maximum duration of a loop step before the watchdog resets the chip.
const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(5);

const REPORT_EVERY: Duration = Duration::from_secs(10);

/// The number of reports in a row without progress after which a core fails the phase. The longest
/// pause of a working core is one long sleep of the other core, 3 s at most.
const STALL_REPORTS: u32 = 2;

/// The minimum length of a long sleep.
const LONG_SLEEP: Duration = Duration::from_millis(500);

/// The bound on a long sleep that returned without a sleep. Such a call returns within about 2 ms,
/// and a long sleep that sleeps lasts at least [`LONG_SLEEP`] less [`TOLERANCE`].
const NO_SLEEP: Duration = Duration::from_millis(5);

/// The longest time by which a long sleep or a wait can end before or after its deadline.
const TOLERANCE: Duration = Duration::from_millis(5);

/// The number of PSRAM words that a core writes before each sleep. The words must be more than the
/// cache can hold, so that some of them must be written back before the sleep. The ESP32-S3's data
/// cache is at most 64 KiB, and the ESP32-P4's L2 cache is at most 512 KiB.
const DIRTY_WORDS: usize = if cfg!(feature = "esp32p4") {
    256 * 1024
} else {
    32 * 1024
};

/// The number of words in one slice of the kept PSRAM. A core checks one slice after each wake.
const SLICE_WORDS: usize = 16 * 1024;

/// The number of kept words that one work pass reads and writes back.
const WINDOW_WORDS: usize = 4 * 1024;

/// The seed of the kept PSRAM pattern.
const KEPT_SEED: u32 = 0x6C07_8965;

/// The number of sleeps and waits between two checks of all of the kept PSRAM.
const FULL_CHECK_EVERY: u32 = 500;

/// The minimum time between two flash cycles. Each cycle erases a sector, so the period limits
/// flash wear.
const FLASH_PERIOD: Duration = Duration::from_millis(100);

const SECTOR: u32 = 4096;

const CORES: usize = if cfg!(multi_core) { 2 } else { 1 };

/// How a phase requests the sleeps.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Caller {
    Bare,
    Thread,
    Automatic,
}

#[derive(Clone, Copy)]
struct Phase {
    caller: Caller,
    max_clock: bool,
    retention: bool,
}

/// The number of phases, before [`Phase::runs`] removes the ones that do not apply to the chip.
const PHASES: usize = 12;

impl Phase {
    fn get(index: usize) -> Self {
        Self {
            caller: [Caller::Bare, Caller::Thread, Caller::Automatic][index % 6 / 2],
            max_clock: index % 2 == 1,
            retention: index >= 6,
        }
    }

    /// Returns whether the phase runs on this chip. A chip with its default CPU clock at the
    /// maximum does not run the phases at the maximum CPU clock, which repeat the others. A chip
    /// without CPU power-down does not run the phases with CPU retention.
    fn runs(self) -> bool {
        (!self.max_clock || CpuClock::default() != CpuClock::max())
            && (!self.retention || cfg!(supports_cpu_power_down))
    }

    fn cpu_clock(self) -> CpuClock {
        if self.max_clock {
            CpuClock::max()
        } else {
            CpuClock::default()
        }
    }
}

impl fmt::Display for Phase {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let caller = match self.caller {
            Caller::Bare => "sleep_light from the main loop",
            Caller::Thread => "sleep_light from an esp-rtos thread",
            Caller::Automatic => "esp-rtos automatic light sleep",
        };
        write!(
            f,
            "{caller} at the {} CPU clock, {} CPU retention",
            if self.max_clock { "maximum" } else { "default" },
            if self.retention { "with" } else { "without" },
        )
    }
}

/// Returns the index of the first phase at or after `index` that runs on this chip.
fn next_phase(index: usize) -> usize {
    (index..PHASES)
        .find(|&i| Phase::get(i).runs())
        .unwrap_or(PHASES)
}

/// Returns the number of phases from [`FIRST_PHASE`] that run on this chip.
fn phases_in_run() -> usize {
    (FIRST_PHASE..PHASES)
        .filter(|&i| Phase::get(i).runs())
        .count()
}

static PROGRESS: ProgressWord = ProgressWord::new(0xB);

/// Words of PSRAM that both cores read and write, through volatile accesses.
#[derive(Clone, Copy)]
struct Words {
    start: *mut u32,
    len: usize,
}

// SAFETY: every access is volatile, and the program gives each core its own words or writes the
// same value from both cores.
unsafe impl Send for Words {}
unsafe impl Sync for Words {}

impl Words {
    const EMPTY: Self = Self {
        start: core::ptr::null_mut(),
        len: 0,
    };

    fn split_at(self, n: usize) -> (Self, Self) {
        let n = n.min(self.len);
        let rest = Self {
            start: unsafe { self.start.add(n) },
            len: self.len - n,
        };
        (
            Self {
                start: self.start,
                len: n,
            },
            rest,
        )
    }

    fn read(self, i: usize) -> u32 {
        unsafe { self.start.add(i).read_volatile() }
    }

    fn write(self, i: usize, value: u32) {
        unsafe { self.start.add(i).write_volatile(value) }
    }
}

/// The PSRAM of the run: the words that each core writes before each sleep, and the kept words.
struct Memory {
    dirty: [Words; CORES],
    kept: Words,
}

static mut MEMORY: Memory = Memory {
    dirty: [Words::EMPTY; CORES],
    kept: Words::EMPTY,
};

/// Returns the PSRAM of the run. `main` sets it before any other core or thread starts.
fn memory() -> &'static Memory {
    unsafe { &*addr_of!(MEMORY) }
}

fn pattern(index: usize, seed: u32) -> u32 {
    (index as u32).wrapping_mul(0x9E37_79B9) ^ seed
}

fn fill(words: Words, seed: u32) {
    for i in 0..words.len {
        words.write(i, pattern(i, seed));
    }
}

/// The words that a check found wrong: their number, and the first of them.
struct Bad {
    count: u32,
    /// The address of the first wrong word, the value read, and the value expected.
    first: Option<(*const u32, u32, u32)>,
}

impl Bad {
    const NONE: Self = Self {
        count: 0,
        first: None,
    };

    fn add(&mut self, words: Words, i: usize, read: u32, expected: u32) {
        self.count += 1;
        if self.first.is_none() {
            self.first = Some((words.start.wrapping_add(i), read, expected));
        }
    }
}

impl fmt::Display for Bad {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let noun = if self.count == 1 { "word" } else { "words" };
        write!(f, "{} bad {noun}", self.count)?;
        if let Some((at, read, expected)) = self.first {
            write!(
                f,
                ", the first at {at:p}: {read:#010x}, expected {expected:#010x}"
            )?;
        }
        Ok(())
    }
}

/// Checks the words in `range` against the pattern of `seed`.
fn check_words(words: Words, range: core::ops::Range<usize>, seed: u32) -> Bad {
    let mut bad = Bad::NONE;
    for i in range {
        let (read, expected) = (words.read(i), pattern(i, seed));
        if read != expected {
            bad.add(words, i, read, expected);
        }
    }
    bad
}

/// Checks the slice `n` of the kept PSRAM.
fn check_kept_slice(core: usize, n: usize) {
    let kept = memory().kept;
    let start = n * SLICE_WORDS;
    let end = kept.len.min(start + SLICE_WORDS);
    let bad = check_words(kept, start..end, KEPT_SEED);
    if bad.count != 0 {
        problem!("core {core}: slice {n} of the kept PSRAM read {bad}");
    }
}

fn kept_slices() -> usize {
    memory().kept.len.div_ceil(SLICE_WORDS)
}

/// A xorshift generator.
struct Rng(u32);

impl Rng {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }

    fn below(&mut self, n: u32) -> u32 {
        self.next() % n
    }
}

/// The counts of the phase, over all cores.
struct Counts {
    sleeps: AtomicU32,
    short: AtomicU32,
    long: AtomicU32,
    refused: AtomicU32,
    /// The long sleeps that ended within their bounds.
    long_slept: AtomicU32,
    not_slept: AtomicU32,
    waits: AtomicU32,
    passes: AtomicU32,
    flash_cycles: AtomicU32,
    /// The work passes, sleeps, and waits of each core.
    progress: [AtomicU32; CORES],
    /// The longest time in microseconds by which a checked long sleep or wait ended before its
    /// deadline.
    longest_early_us: AtomicU32,
    /// The longest time in microseconds by which a checked long sleep or wait ended after its
    /// deadline, without a flash cycle during it.
    longest_overrun_us: AtomicU32,
    /// The same, for a long sleep or wait during a flash cycle.
    longest_flash_overrun_us: AtomicU32,
}

static COUNTS: Counts = Counts {
    sleeps: AtomicU32::new(0),
    short: AtomicU32::new(0),
    long: AtomicU32::new(0),
    long_slept: AtomicU32::new(0),
    refused: AtomicU32::new(0),
    not_slept: AtomicU32::new(0),
    waits: AtomicU32::new(0),
    passes: AtomicU32::new(0),
    flash_cycles: AtomicU32::new(0),
    progress: [const { AtomicU32::new(0) }; CORES],
    longest_early_us: AtomicU32::new(0),
    longest_overrun_us: AtomicU32::new(0),
    longest_flash_overrun_us: AtomicU32::new(0),
};

fn count(counter: &AtomicU32) -> u32 {
    counter.fetch_add(1, Ordering::Relaxed) + 1
}

/// Whether a check failed. Core 0 then fails the phase.
static FAILED: AtomicBool = AtomicBool::new(false);

/// Stops this core's work after a failure, until core 0 resets the chip.
fn stop_if_failed() {
    while FAILED.load(Ordering::Acquire) {
        core::hint::spin_loop();
    }
}

/// The free space that the flash cycles use, and the buffers of a cycle.
struct Scratch {
    flash: Flash<'static, Blocking>,
    first: u32,
    sectors: u32,
    start: u32,
    cycle: u32,
    next: Instant,
    data: [u32; SECTOR as usize / 4],
    read: [u32; SECTOR as usize / 4],
}

/// The core that runs a flash cycle holds this flag, so that only one core uses [`SCRATCH`] at a
/// time.
static FLASH_BUSY: AtomicBool = AtomicBool::new(false);

static mut SCRATCH: Option<Scratch> = None;

/// The number of times that a flash cycle started or ended. The value is odd while a cycle erases,
/// writes, or reads. A sleep or a wait during which a cycle ran does not get an upper bound,
/// because a flash operation can delay its end.
static FLASH_GEN: AtomicU32 = AtomicU32::new(0);

/// Erases, writes, and verifies the next sector of the free space. A cycle runs when
/// [`FLASH_PERIOD`] has passed since the last one and the other core is not in a cycle.
fn flash_cycle(core: usize) {
    if FLASH_BUSY.swap(true, Ordering::Acquire) {
        return;
    }
    // SAFETY: `FLASH_BUSY` gives this core the only access.
    if let Some(s) = unsafe { (*addr_of_mut!(SCRATCH)).as_mut() }
        && Instant::now() >= s.next
    {
        s.next = Instant::now() + FLASH_PERIOD;
        let sector = s.first + (s.start + s.cycle) % s.sectors * SECTOR;
        let seed = s.cycle.wrapping_mul(0x9E37_79B9);
        for (i, word) in s.data.iter_mut().enumerate() {
            *word = seed ^ i as u32;
        }
        s.read.fill(0);
        FLASH_GEN.fetch_add(1, Ordering::AcqRel);
        // SAFETY: the sector is in the booted partition, after the image.
        let written = unsafe {
            s.flash.erase(sector, sector + SECTOR).is_ok() && s.flash.write(sector, &s.data).is_ok()
        };
        let verified = written && s.flash.read(sector, &mut s.read).is_ok() && s.read == s.data;
        FLASH_GEN.fetch_add(1, Ordering::AcqRel);
        if !verified {
            problem!("core {core}: flash sector {sector:#x} did not read back what was written");
        }
        s.cycle += 1;
        count(&COUNTS.flash_cycles);
    }
    FLASH_BUSY.store(false, Ordering::Release);
}

/// Formats and sorts, so that the core runs code from other parts of the image between passes.
/// Code that the cache does not hold comes from flash, and fails at once when a sleep left the
/// flash MMU or the flash clock wrong.
#[inline(never)]
fn other_code(salt: u32) {
    struct Buf {
        data: [u8; 64],
        len: usize,
    }

    impl fmt::Write for Buf {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let n = s.len().min(self.data.len() - self.len);
            self.data[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
            self.len += n;
            Ok(())
        }
    }

    let mut buf = Buf {
        data: [0; 64],
        len: 0,
    };
    let _ = fmt::Write::write_fmt(
        &mut buf,
        format_args!("{salt:08x} {} {:?}", salt.rotate_left(7), salt % 1000),
    );
    let mut a = [0u32; 32];
    for (i, x) in a.iter_mut().enumerate() {
        *x = salt.wrapping_mul(i as u32 + 1).rotate_right(i as u32);
    }
    a.sort_unstable();
    core::hint::black_box((&buf.data, &a));
}

/// Runs a flash cycle when one is due, then reads, checks, and writes back a random window of the
/// kept PSRAM. The write leaves the cache lines dirty.
fn work_step(core: usize, rng: &mut Rng) {
    flash_cycle(core);
    let kept = memory().kept;
    if kept.len > WINDOW_WORDS {
        let offset = rng.below((kept.len - WINDOW_WORDS) as u32) as usize;
        let mut bad = Bad::NONE;
        for i in offset..offset + WINDOW_WORDS {
            let (read, expected) = (kept.read(i), pattern(i, KEPT_SEED));
            if read != expected {
                bad.add(kept, i, read, expected);
            }
            kept.write(i, expected);
        }
        if bad.count != 0 {
            problem!("core {core}: a work pass over the kept PSRAM read {bad}");
        }
    }
    other_code(rng.next());
    count(&COUNTS.passes);
    count(&COUNTS.progress[core]);
}

/// The core that requests a sleep holds this flag. The wakeup deadline and the wakeup cause belong
/// to one sleep at a time, so only one core requests a sleep at a time.
static SLEEP_TURN: AtomicBool = AtomicBool::new(false);

fn busy_wait(length: Duration) {
    let end = Instant::now() + length;
    while Instant::now() < end {}
}

/// Returns whether a flash cycle was running when the caller read `flash_gen` from
/// [`FLASH_GEN`], or ran after that.
fn flash_ran(flash_gen: u32) -> bool {
    flash_gen % 2 == 1 || FLASH_GEN.load(Ordering::Acquire) != flash_gen
}

/// Returns whether a long sleep or a wait ended within its bounds, and records by how much it ended
/// early or late. A sleep or wait of `length` that took `took` must not end more than [`TOLERANCE`]
/// early. When no flash cycle ran during it, it must also not end more than [`TOLERANCE`] late.
///
/// The bounds are wider than the error that the report shows, because a fault in the timer wakeup
/// or in the time update moves the end by much more.
fn length_ok(took: Duration, length: Duration, flash_gen: u32) -> bool {
    let during_flash = flash_ran(flash_gen);
    let early = length.as_micros().saturating_sub(took.as_micros()) as u32;
    COUNTS.longest_early_us.fetch_max(early, Ordering::Relaxed);
    let overrun = took.as_micros().saturating_sub(length.as_micros()) as u32;
    let longest = if during_flash {
        &COUNTS.longest_flash_overrun_us
    } else {
        &COUNTS.longest_overrun_us
    };
    longest.fetch_max(overrun, Ordering::Relaxed);
    took + TOLERANCE >= length && (during_flash || took <= length + TOLERANCE)
}

/// Requests one light sleep of random length, then checks the sleep and PSRAM. Returns at once when
/// another core is requesting a sleep.
fn sleep_round(core: usize, lpwr: &mut LowPower<'_>, rng: &mut Rng, retention: bool) {
    if SLEEP_TURN.swap(true, Ordering::Acquire) {
        return;
    }
    // A random delay varies where the other core is in its work when the sleep starts.
    busy_wait(Duration::from_micros(u64::from(rng.below(1_001))));
    let dirty = memory().dirty[core];
    let seed = rng.next();
    fill(dirty, seed);

    // The sleep entry takes time for the cache write-back, the clock switches, and the park. A
    // deadline during the entry makes the hardware reject the sleep or wake the chip at once. In
    // both cases, `sleep_light` returns `Ended`, so the program cannot check the short sleeps. At
    // 0.5 s, the entry and the exit take less than 1 %, and the deadline cannot fall inside the
    // entry.
    let random = rng.next();
    let length = match random % 16 {
        0 | 1 => {
            count(&COUNTS.short);
            Duration::from_micros(u64::from(random % 200))
        }
        2 => {
            count(&COUNTS.long);
            LONG_SLEEP + Duration::from_millis(u64::from(random % 2_500))
        }
        _ => Duration::from_micros(1_000 + u64::from(random % 9_000)),
    };

    let flash_gen = FLASH_GEN.load(Ordering::Acquire);
    // The deadline and the measured length of the sleep must count from the same `Instant`.
    // Otherwise a flash operation on the other core can stop this core between the two reads, for
    // longer than `TOLERANCE`.
    let before = Instant::now();
    lpwr.set_wakeup_deadline(before + length);
    let result = lpwr.sleep_light(RtcSleepConfig::default());
    let slept = Instant::now() - before;
    let n = count(&COUNTS.sleeps);
    if result == LightSleep::Refused {
        count(&COUNTS.refused);
    }

    if result == LightSleep::Ended && length >= LONG_SLEEP {
        if cfg!(all(cpu_retention = "software", multi_core))
            && (retention || core == 1)
            && slept < NO_SLEEP
        {
            // On a chip with two cores and software CPU retention, each sleep of core 1, and each
            // sleep of core 0 with CPU retention, waits for the other core in a rendezvous. A core
            // in a critical section, such as one that waits for a flash write, does not answer
            // within 1 ms, and `sleep_light` returns without a sleep. The count shows how often
            // this occurs. If such sleeps outnumber the long sleeps that slept, the phase tested
            // few sleeps, so it fails. `wakeup_cause` returns the cause of the previous sleep, so
            // the program does not check the cause.
            count(&COUNTS.not_slept);
        } else if !wakeup_cause().contains(WakeupSource::Timer) {
            problem!("core {core}: sleep {n} woke on {:?}", wakeup_cause());
        } else if !length_ok(slept, length, flash_gen) {
            problem!("core {core}: sleep {n} lasted {slept} of {length}");
        } else {
            count(&COUNTS.long_slept);
        }
    }
    SLEEP_TURN.store(false, Ordering::Release);
    count(&COUNTS.progress[core]);

    let bad = check_words(dirty, 0..dirty.len, seed);
    if bad.count != 0 {
        problem!("core {core}: sleep {n}: the PSRAM written before the sleep read {bad}");
    }
    let slices = kept_slices();
    if slices > 0 {
        check_kept_slice(core, n as usize % slices);
    }
}

/// Waits a random time in an esp-rtos thread, which lets the automatic light sleep run, then checks
/// the wait and PSRAM. The waits start at 1 ms, the default minimum residency of the idle hook. In
/// some waits the hook then waits for an interrupt instead of a sleep.
fn wait_round(core: usize, rng: &mut Rng) {
    let dirty = memory().dirty[core];
    let seed = rng.next();
    fill(dirty, seed);
    let length = Duration::from_micros(1_000 + u64::from(rng.below(29_001)));
    let flash_gen = FLASH_GEN.load(Ordering::Acquire);
    let before = Instant::now();
    CurrentThreadHandle::get().delay(length);
    let waited = Instant::now() - before;
    let n = count(&COUNTS.waits);
    count(&COUNTS.progress[core]);
    if !length_ok(waited, length, flash_gen) {
        problem!("core {core}: wait {n} lasted {waited} of {length}");
    }
    let bad = check_words(dirty, 0..dirty.len, seed);
    if bad.count != 0 {
        problem!("core {core}: wait {n}: the PSRAM written before the wait read {bad}");
    }
    let slices = kept_slices();
    if slices > 0 {
        check_kept_slice(core, n as usize % slices);
    }
}

/// The part of core 0 that controls the phase. It feeds the watchdog, prints the counts, checks all
/// of the kept PSRAM, and ends the phase.
struct Controller {
    rtc: Rtc<'static>,
    index: usize,
    progress: Progress,
    started: Instant,
    next_report: Instant,
    next_full_check: u32,
    /// Each core's progress count at the last report, and the reports in a row that it did not
    /// change.
    quiet: [(u32, u32); CORES],
}

impl Controller {
    fn poll(&mut self) {
        self.rtc.rwdt.feed();
        let done = COUNTS.sleeps.load(Ordering::Relaxed) + COUNTS.waits.load(Ordering::Relaxed);
        if done >= self.next_full_check {
            self.next_full_check = done + FULL_CHECK_EVERY;
            for n in 0..kept_slices() {
                self.rtc.rwdt.feed();
                check_kept_slice(0, n);
            }
        }
        if Instant::now() >= self.next_report {
            self.next_report += REPORT_EVERY;
            self.report();
            self.check_progress();
        }
        if FAILED.load(Ordering::Acquire) || Instant::now() - self.started >= PHASE_FOR {
            self.end();
        }
    }

    fn report(&self) {
        let c = &COUNTS;
        report!(
            "stress: {} s, sleeps {}, short {}, long {}, long slept {}, refused {}, not slept {}, waits {}, work passes {}, flash cycles {}",
            (Instant::now() - self.started).as_secs(),
            c.sleeps.load(Ordering::Relaxed),
            c.short.load(Ordering::Relaxed),
            c.long.load(Ordering::Relaxed),
            c.long_slept.load(Ordering::Relaxed),
            c.refused.load(Ordering::Relaxed),
            c.not_slept.load(Ordering::Relaxed),
            c.waits.load(Ordering::Relaxed),
            c.passes.load(Ordering::Relaxed),
            c.flash_cycles.load(Ordering::Relaxed),
        );
        if c.long_slept.load(Ordering::Relaxed) + c.waits.load(Ordering::Relaxed) > 0 {
            report!(
                "  bounds: at most {} µs early, longest overrun {} µs, during a flash cycle {} µs",
                c.longest_early_us.load(Ordering::Relaxed),
                c.longest_overrun_us.load(Ordering::Relaxed),
                c.longest_flash_overrun_us.load(Ordering::Relaxed),
            );
        }
    }

    /// Fails the phase when a core made no progress for [`STALL_REPORTS`] reports in a row.
    fn check_progress(&mut self) {
        for (core, (last, quiet)) in self.quiet.iter_mut().enumerate() {
            let count = COUNTS.progress[core].load(Ordering::Relaxed);
            *quiet = if count == *last { *quiet + 1 } else { 0 };
            *last = count;
            if *quiet == STALL_REPORTS {
                problem!(
                    "core {core}: no progress for {} s",
                    REPORT_EVERY.as_secs() * u64::from(STALL_REPORTS)
                );
            }
        }
    }

    /// Reports the phase, then resets the chip for the next phase, or reports the run and stops.
    fn end(&mut self) -> ! {
        self.report();
        let phase = Phase::get(self.index);
        let c = &COUNTS;
        let made = |counter: &AtomicU32| counter.load(Ordering::Relaxed) > 0;
        let missing = match phase.caller {
            _ if !made(&c.flash_cycles) => Some("no flash cycle"),
            Caller::Automatic if !made(&c.waits) => Some("no wait"),
            Caller::Automatic if !wakeup_cause().contains(WakeupSource::Timer) => {
                Some("no automatic light sleep")
            }
            Caller::Bare | Caller::Thread if !made(&c.sleeps) => Some("no sleep"),
            Caller::Bare | Caller::Thread if !made(&c.long_slept) => {
                Some("no long sleep ended within its bounds")
            }
            Caller::Bare | Caller::Thread
                if c.not_slept.load(Ordering::Relaxed) > c.long_slept.load(Ordering::Relaxed) =>
            {
                Some("more long sleeps returned without a sleep than ended within their bounds")
            }
            _ => None,
        };
        if FAILED.load(Ordering::Acquire) {
            report!("FAIL {phase}: see the core line above");
        } else if let Some(missing) = missing {
            report!("FAIL {phase}: {missing}");
        } else {
            report!("PASS {phase}");
            self.progress.passed += 1;
        }

        self.progress.next = next_phase(self.index + 1);
        if self.progress.next < PHASES {
            self.progress.reset_requested = true;
            PROGRESS.store(self.progress);
            // The reset drops what the UART has not sent yet.
            busy_wait(Duration::from_millis(100));
            esp_hal::system::software_reset();
        }
        PROGRESS.store(self.progress);
        report!(
            "light-sleep-stress: {} of {} phases passed",
            self.progress.passed,
            phases_in_run()
        );
        self.rtc.rwdt.disable();
        loop {
            core::hint::spin_loop();
        }
    }
}

/// Requests sleeps from the main loop, and works between them.
fn bare_loop(
    core: usize,
    mut lpwr: LowPower<'static>,
    mut rng: Rng,
    retention: bool,
    mut controller: Option<Controller>,
) -> ! {
    loop {
        match &mut controller {
            Some(controller) => controller.poll(),
            None => stop_if_failed(),
        }
        if rng.below(2) == 0 {
            sleep_round(core, &mut lpwr, &mut rng, retention);
        } else {
            work_step(core, &mut rng);
        }
    }
}

/// Requests sleeps from this thread, at a high priority, while a low-priority thread on the same
/// core works. The thread blocks for 1–3 ms after each sleep, so that the low-priority thread runs.
fn thread_loop(
    core: usize,
    mut lpwr: LowPower<'static>,
    mut rng: Rng,
    retention: bool,
    mut controller: Option<Controller>,
) -> ! {
    static WORKER_STACKS: [static_cell::ConstStaticCell<esp_rtos::thread::Stack<8192>>; CORES] =
        [const { static_cell::ConstStaticCell::new(esp_rtos::thread::Stack::new()) }; CORES];

    let mut worker_rng = Rng(rng.next() | 1);
    CurrentThreadHandle::get().set_priority(2);
    let worker = ThreadSpawner::from_static(WORKER_STACKS[core].take()).with_priority(1);
    #[cfg(multi_core)]
    let worker = worker.with_pinned_to(if core == 0 { Cpu::ProCpu } else { Cpu::AppCpu });
    worker
        .spawn(move || {
            loop {
                stop_if_failed();
                work_step(core, &mut worker_rng);
            }
        })
        .detach();

    loop {
        match &mut controller {
            Some(controller) => controller.poll(),
            None => stop_if_failed(),
        }
        sleep_round(core, &mut lpwr, &mut rng, retention);
        CurrentThreadHandle::get()
            .delay(Duration::from_micros(1_000 + u64::from(rng.below(2_001))));
    }
}

/// Works, then waits, so that the automatic light sleep runs when every core waits.
fn automatic_loop(core: usize, mut rng: Rng, mut controller: Option<Controller>) -> ! {
    loop {
        match &mut controller {
            Some(controller) => controller.poll(),
            None => stop_if_failed(),
        }
        work_step(core, &mut rng);
        wait_round(core, &mut rng);
    }
}

#[cfg(supports_cpu_power_down)]
#[esp_hal::ram(reclaimed, unstable(zeroed))]
static CPU_RETENTION_MEMORY: esp_hal::rtc_cntl::CpuRetentionStorage =
    esp_hal::rtc_cntl::CpuRetentionStorage::new();

#[main]
fn main() -> ! {
    let reset = reset_reason(Cpu::ProCpu);
    let loaded = PROGRESS.load(PHASES);
    // When the program did not request the reset, the stored phase did not finish.
    let hung = loaded
        .filter(|stored| !stored.reset_requested)
        .map(|stored| stored.next);
    let mut progress = loaded.unwrap_or(Progress {
        next: FIRST_PHASE,
        passed: 0,
        reset_requested: false,
    });
    if let Some(hung) = hung {
        progress.next = hung + 1;
    }
    progress.next = next_phase(progress.next);
    let index = progress.next;
    let phase = Phase::get(index);

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(phase.cpu_clock()));

    report!(
        "light-sleep-stress: reset={:?}, CPU clock {:?}",
        reset,
        phase.cpu_clock()
    );

    let mut rtc = Rtc::new(peripherals.RTC_TIMER);

    if let Some(hung) = hung {
        report!("FAIL {}: reset {:?}", Phase::get(hung), reset);
    }
    if index == PHASES {
        PROGRESS.store(progress);
        report!(
            "light-sleep-stress: {} of {} phases passed",
            progress.passed,
            phases_in_run()
        );
        rtc.rwdt.disable();
        loop {
            core::hint::spin_loop();
        }
    }
    progress.reset_requested = false;
    PROGRESS.store(progress);

    rtc.rwdt.set_timeout(RwdtStage::Stage0, WATCHDOG_TIMEOUT);
    rtc.rwdt.enable();

    #[cfg(psram_driver_supported)]
    let psram = {
        let psram =
            esp_hal::psram::Psram::new(peripherals.PSRAM, esp_hal::psram::PsramConfig::default());
        let (start, size) = psram.raw_parts();
        Words {
            start: start.cast::<u32>(),
            len: size / 4,
        }
    };
    #[cfg(not(psram_driver_supported))]
    let psram = Words::EMPTY;
    report!("PSRAM: {} KiB", psram.len / 256);

    let mut dirty = [Words::EMPTY; CORES];
    let mut kept = psram;
    for words in dirty.iter_mut() {
        (*words, kept) = kept.split_at(DIRTY_WORDS);
    }
    unsafe { MEMORY = Memory { dirty, kept } };
    for i in 0..kept.len {
        if i % SLICE_WORDS == 0 {
            rtc.rwdt.feed();
        }
        kept.write(i, pattern(i, KEPT_SEED));
    }

    let seed = SEED.unwrap_or_else(|| esp_hal::rng::Rng::new().random() | 1);
    let mut rng = Rng(seed);

    let mut flash = Flash::new(peripherals.FLASH, FlashConfig::default()).unwrap();
    let free = free_flash(&mut flash);
    let (first, sectors) = (free.start, free.len() as u32 / SECTOR);
    report!("flash: {sectors} free sectors from {first:#x}");
    unsafe {
        SCRATCH = Some(Scratch {
            flash,
            first,
            sectors,
            start: rng.below(sectors),
            cycle: 0,
            next: Instant::now(),
            data: [0; SECTOR as usize / 4],
            read: [0; SECTOR as usize / 4],
        })
    };

    let mut uart = Uart::new(peripherals.UART1, UartConfig::default()).unwrap();
    uart.write(&[0x55; 32]).unwrap();
    drop(uart);

    report!("RUN phase {index}: {phase}, seed {seed:#010x}");
    let now = Instant::now();
    let controller = Controller {
        rtc,
        index,
        progress,
        started: now,
        next_report: now + REPORT_EVERY,
        next_full_check: FULL_CHECK_EVERY,
        quiet: [(0, 0); CORES],
    };
    let retention = phase.retention;
    #[cfg(multi_core)]
    let core1_rng = Rng(rng.next() | 1);
    #[cfg(multi_core)]
    static mut CORE1_STACK: esp_hal::system::Stack<16384> = esp_hal::system::Stack::new();
    #[cfg(multi_core)]
    let core1_stack = unsafe { &mut *addr_of_mut!(CORE1_STACK) };

    if phase.caller == Caller::Automatic {
        #[cfg_attr(not(supports_cpu_power_down), expect(unused_mut))]
        let mut sleep = esp_rtos::sleep::configure(peripherals.LPWR);
        #[cfg(supports_cpu_power_down)]
        if retention {
            sleep
                .enable_cpu_powerdown(CPU_RETENTION_MEMORY.take())
                .unwrap();
        }
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start_with_idle_hook(timg0.timer0, sleep.light_sleep_hook);
        #[cfg(multi_core)]
        esp_rtos::start_second_core(peripherals.CPU_CTRL, core1_stack, move || {
            automatic_loop(1, core1_rng, None)
        });
        automatic_loop(0, rng, Some(controller))
    }

    #[cfg_attr(not(supports_cpu_power_down), expect(unused_mut))]
    let mut lpwr = LowPower::new(peripherals.LPWR);
    #[cfg(supports_cpu_power_down)]
    if retention {
        lpwr.install_cpu_retention_memory(CPU_RETENTION_MEMORY.take())
            .unwrap();
    }
    if phase.caller == Caller::Thread {
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        esp_rtos::start(timg0.timer0);
        #[cfg(multi_core)]
        esp_rtos::start_second_core(peripherals.CPU_CTRL, core1_stack, move || {
            thread_loop(
                1,
                LowPower::new(unsafe { esp_hal::peripherals::LPWR::steal() }),
                core1_rng,
                retention,
                None,
            )
        });
        thread_loop(0, lpwr, rng, retention, Some(controller))
    }

    #[cfg(multi_core)]
    {
        let mut cpu_control = esp_hal::system::CpuControl::new(peripherals.CPU_CTRL);
        let guard = cpu_control
            .start_app_core(core1_stack, move || {
                bare_loop(
                    1,
                    LowPower::new(unsafe { esp_hal::peripherals::LPWR::steal() }),
                    core1_rng,
                    retention,
                    None,
                )
            })
            .unwrap();
        core::mem::forget(guard);
    }
    bare_loop(0, lpwr, rng, retention, Some(controller))
}
