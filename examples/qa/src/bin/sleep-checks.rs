//! Quick checks of sleep.
//!
//! The program runs a list of checks, and prints `RUN <name>` before each check, then
//! `PASS <name>` or `FAIL <name>: <reason>`. At the end it prints
//! `sleep-checks: <n> of <m> checks passed`.
//!
//! Each check requests a light sleep. After the wake, the check verifies:
//!
//! 1. The length of the sleep. The sleep must last at least 90 percent of the time to the wakeup
//!    deadline.
//! 2. The wakeup cause.
//! 3. Every word of PSRAM, on a chip with PSRAM. With code in PSRAM, the check includes a PSRAM
//!    static.
//!
//! The checks run at the default and at the maximum CPU clock, without and with CPU retention. On a
//! chip with two cores, each core requests a light sleep.
//!
//! With code in PSRAM, more checks request a light sleep during a flash erase. A light sleep that
//! core 0 requests during an erase on core 1 must be refused at once, and so must a light sleep
//! that core 1 requests during an erase on core 0. After the erase, the same light sleep must not
//! be refused. On a chip with one core, a light sleep that an esp-rtos thread requests during an
//! erase in a lower-priority thread must be refused, and a light sleep after the erase must pass
//! the checks above. Without code in PSRAM, the program prints `SKIP` for these checks.
//!
//! The program keeps its progress in memory that a reset does not clear:
//!
//! - If a check hangs outside a sleep or panics, the RTC watchdog resets the chip. A CPU lockup
//!   also resets the chip. The next boot reports the check as failed, with the reset reason, and
//!   runs the next check. A reset does not clear all of the chip's state, so a check after a failed
//!   check can fail because of it. A sleep that never ends hangs the test, because the RTC watchdog
//!   is not active during a light sleep.
//! - The CPU clock is set at start-up. A check that needs a different CPU clock resets the chip in
//!   software first.
//! - A power-on reset starts the list again.
//!
//! The console is on the UART, because USB Serial/JTAG stops across sleep.

//% FEATURES: esp-println/uart
//% CHIP_FILTER: sleep_light_sleep && sleep_has_wakeup_source_timer

#![no_std]
#![no_main]

#[cfg(multi_core)]
use core::sync::atomic::AtomicU32;
use core::{
    ptr::addr_of_mut,
    sync::atomic::{AtomicBool, Ordering},
};

use esp_backtrace as _;
#[cfg(multi_core)]
use esp_hal::system::{CpuControl, Stack};
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
};
use qa_test::{Progress, ProgressWord, code_in_psram, free_flash, report};

esp_bootloader_esp_idf::esp_app_desc!();

/// The maximum duration of a check before the watchdog resets the chip.
const WATCHDOG_TIMEOUT: Duration = Duration::from_secs(5);

/// The time from a request to the wakeup deadline.
const SLEEP_LENGTH: Duration = Duration::from_millis(20);

/// The number of PSRAM words that use one seed.
const SLICE_WORDS: usize = 16 * 1024;

/// The seed of the PSRAM pattern. Each check puts its number in the high bits, and each slice adds
/// its number. A read that returns the data of a different check or slice fails the check.
const PSRAM_SEED: u32 = 0x6C07_8965;

/// The PSRAM static's seed is the check's seed XOR this value, so the static and the heap hold
/// different patterns.
#[cfg(code_in_psram)]
const STATIC_SEED: u32 = 0x5354_4154;

/// The number of words in [`PSRAM_STATIC`].
#[cfg(code_in_psram)]
const STATIC_WORDS: usize = 1024;

/// A static in `.bss.psram`, which lies outside the PSRAM that `Psram::raw_parts` returns. 4 KiB
/// spans many cache lines.
#[cfg(code_in_psram)]
#[esp_hal::psram(zeroed)]
static mut PSRAM_STATIC: [u32; STATIC_WORDS] = [0; STATIC_WORDS];

#[derive(Clone, Copy, PartialEq, Eq)]
enum Clock {
    Default,
    Max,
}

impl Clock {
    fn cpu_clock(self) -> CpuClock {
        match self {
            Clock::Default => CpuClock::default(),
            Clock::Max => CpuClock::max(),
        }
    }
}

/// One entry in [`CHECKS`].
struct Check {
    name: &'static str,
    clock: Clock,
    /// Whether the check needs code in PSRAM.
    code_in_psram: bool,
    run: fn(&mut Board<'_>) -> Result<(), &'static str>,
}

impl Check {
    /// Returns whether the check runs on this chip. A chip with its default CPU clock at the
    /// maximum runs each check once.
    fn runs_on_chip(&self) -> bool {
        self.clock == Clock::Default || CpuClock::default() != CpuClock::max()
    }

    /// Returns whether the check runs on this chip in this build.
    fn runs(&self) -> bool {
        self.runs_on_chip() && (!self.code_in_psram || code_in_psram())
    }
}

/// The checks, in the order they run.
///
/// CPU retention cannot be removed after installation. A check without CPU retention must come
/// before the checks with it at the same CPU clock. esp-rtos cannot be stopped, so a check that
/// starts esp-rtos must be the last.
const CHECKS: &[Check] = &[
    Check {
        name: "light sleep at the default CPU clock",
        clock: Clock::Default,
        code_in_psram: false,
        run: light_sleep,
    },
    #[cfg(multi_core)]
    Check {
        name: "light sleep from core 1 at the default CPU clock",
        clock: Clock::Default,
        code_in_psram: false,
        run: light_sleep_from_core1,
    },
    #[cfg(multi_core)]
    Check {
        name: "light sleep during an erase on core 1",
        clock: Clock::Default,
        code_in_psram: true,
        run: light_sleep_during_core1_erase,
    },
    #[cfg(multi_core)]
    Check {
        name: "light sleep from core 1 during an erase on core 0",
        clock: Clock::Default,
        code_in_psram: true,
        run: core1_light_sleep_during_erase,
    },
    #[cfg(supports_cpu_power_down)]
    Check {
        name: "light sleep with CPU retention at the default CPU clock",
        clock: Clock::Default,
        code_in_psram: false,
        run: light_sleep_with_retention,
    },
    #[cfg(all(multi_core, supports_cpu_power_down))]
    Check {
        name: "light sleep from core 1 with CPU retention at the default CPU clock",
        clock: Clock::Default,
        code_in_psram: false,
        run: light_sleep_from_core1_with_retention,
    },
    #[cfg(all(multi_core, supports_cpu_power_down))]
    Check {
        name: "light sleep with CPU retention during an erase on core 1",
        clock: Clock::Default,
        code_in_psram: true,
        run: light_sleep_with_retention_during_core1_erase,
    },
    #[cfg(all(multi_core, supports_cpu_power_down))]
    Check {
        name: "light sleep from core 1 with CPU retention during an erase on core 0",
        clock: Clock::Default,
        code_in_psram: true,
        run: core1_light_sleep_with_retention_during_erase,
    },
    Check {
        name: "light sleep at the maximum CPU clock",
        clock: Clock::Max,
        code_in_psram: false,
        run: light_sleep,
    },
    #[cfg(multi_core)]
    Check {
        name: "light sleep from core 1 at the maximum CPU clock",
        clock: Clock::Max,
        code_in_psram: false,
        run: light_sleep_from_core1,
    },
    #[cfg(supports_cpu_power_down)]
    Check {
        name: "light sleep with CPU retention at the maximum CPU clock",
        clock: Clock::Max,
        code_in_psram: false,
        run: light_sleep_with_retention,
    },
    #[cfg(all(multi_core, supports_cpu_power_down))]
    Check {
        name: "light sleep from core 1 with CPU retention at the maximum CPU clock",
        clock: Clock::Max,
        code_in_psram: false,
        run: light_sleep_from_core1_with_retention,
    },
    #[cfg(not(multi_core))]
    Check {
        name: "light sleep from an esp-rtos thread during an erase",
        clock: Clock::Default,
        code_in_psram: true,
        run: light_sleep_from_a_thread_during_an_erase,
    },
];

static PROGRESS: ProgressWord = ProgressWord::new(0xA);

/// The peripherals and the PSRAM that the checks use.
struct Board<'d> {
    rtc: Rtc<'d>,
    lpwr: LowPower<'d>,
    psram: &'static mut [u32],
    /// The seed of the current check's PSRAM pattern. PSRAM must hold other data before the fill,
    /// or a sleep that loses the writes still leaves the expected pattern.
    seed: u32,
    #[cfg(supports_cpu_power_down)]
    retention_installed: bool,
    #[cfg(multi_core)]
    cpu_control: CpuControl<'d>,
    #[cfg(multi_core)]
    core1_started: bool,
}

fn pattern(index: usize, seed: u32) -> u32 {
    (index as u32).wrapping_mul(0x9E37_79B9) ^ seed
}

fn fill_psram(board: &mut Board<'_>) {
    for (n, slice) in board.psram.chunks_mut(SLICE_WORDS).enumerate() {
        board.rtc.rwdt.feed();
        for (i, word) in slice.iter_mut().enumerate() {
            unsafe {
                core::ptr::write_volatile(word, pattern(i, board.seed.wrapping_add(n as u32)))
            };
        }
    }
    // The static must be written last. Only the lines that are still dirty at the sleep can lose
    // their data.
    #[cfg(code_in_psram)]
    for i in 0..STATIC_WORDS {
        let word = pattern(i, STATIC_SEED ^ board.seed);
        unsafe { core::ptr::write_volatile(addr_of_mut!(PSRAM_STATIC[i]), word) };
    }
}

fn check_psram(board: &mut Board<'_>) -> Result<(), &'static str> {
    let mut failed = false;
    for (n, slice) in board.psram.chunks(SLICE_WORDS).enumerate() {
        board.rtc.rwdt.feed();
        let seed = board.seed.wrapping_add(n as u32);
        let bad = slice
            .iter()
            .enumerate()
            .filter(|&(i, word)| unsafe { core::ptr::read_volatile(word) } != pattern(i, seed))
            .count();
        if bad != 0 {
            report!("  slice {n} of PSRAM read {bad} bad words");
            failed = true;
        }
    }
    #[cfg(code_in_psram)]
    let static_failed = {
        let bad = (0..STATIC_WORDS)
            .filter(|&i| {
                let word = unsafe { core::ptr::read_volatile(addr_of_mut!(PSRAM_STATIC[i])) };
                word != pattern(i, STATIC_SEED ^ board.seed)
            })
            .count();
        if bad != 0 {
            report!("  the PSRAM static read {bad} bad words");
        }
        bad != 0
    };
    #[cfg(not(code_in_psram))]
    let static_failed = false;
    if failed {
        Err("PSRAM lost data")
    } else if static_failed {
        Err("a PSRAM static changed across the sleep")
    } else {
        Ok(())
    }
}

/// Fills PSRAM, requests a light sleep with `request`, then verifies the sleep and PSRAM.
fn sleep_and_check(
    board: &mut Board<'_>,
    request: fn(&mut Board<'_>) -> Result<(), &'static str>,
) -> Result<(), &'static str> {
    fill_psram(board);
    board
        .lpwr
        .set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
    let start = Instant::now();
    request(board)?;
    let slept = Instant::now() - start;
    if slept < SLEEP_LENGTH * 9 / 10 {
        report!("  the light sleep returned after {slept} of {SLEEP_LENGTH}");
        return Err("the light sleep ended early");
    }
    if !wakeup_cause().contains(WakeupSource::Timer) {
        report!("  the light sleep woke on {:?}", wakeup_cause());
        return Err("the light sleep did not wake on the timer");
    }
    check_psram(board)
}

fn request_on_core0(board: &mut Board<'_>) -> Result<(), &'static str> {
    if board.lpwr.sleep_light(RtcSleepConfig::default()) == LightSleep::Refused {
        return Err("the light sleep was refused");
    }
    Ok(())
}

/// Returns an error if CPU retention is installed, so that the check does not sleep with it.
fn require_no_retention(_board: &Board<'_>) -> Result<(), &'static str> {
    #[cfg(supports_cpu_power_down)]
    if _board.retention_installed {
        return Err("CPU retention is installed, so the check would sleep with it");
    }
    Ok(())
}

#[cfg(supports_cpu_power_down)]
#[esp_hal::ram(reclaimed, unstable(zeroed))]
static CPU_RETENTION_MEMORY: esp_hal::rtc_cntl::CpuRetentionStorage =
    esp_hal::rtc_cntl::CpuRetentionStorage::new();

#[cfg(supports_cpu_power_down)]
fn install_retention(board: &mut Board<'_>) {
    if !board.retention_installed {
        board
            .lpwr
            .install_cpu_retention_memory(CPU_RETENTION_MEMORY.take())
            .unwrap();
        board.retention_installed = true;
    }
}

fn light_sleep(board: &mut Board<'_>) -> Result<(), &'static str> {
    require_no_retention(board)?;
    sleep_and_check(board, request_on_core0)
}

#[cfg(supports_cpu_power_down)]
fn light_sleep_with_retention(board: &mut Board<'_>) -> Result<(), &'static str> {
    install_retention(board);
    sleep_and_check(board, request_on_core0)
}

/// The command from core 0 to core 1. Core 1 sets `COMMAND` back to `IDLE` after the command.
#[cfg(multi_core)]
static COMMAND: AtomicU32 = AtomicU32::new(IDLE);

#[cfg(multi_core)]
const IDLE: u32 = 0;
/// The command that makes core 1 request a light sleep.
#[cfg(multi_core)]
const SLEEP: u32 = 1;
/// The command that makes core 1 erase the block of [`FLASH`].
#[cfg(multi_core)]
const ERASE: u32 = 2;
/// The command that makes core 1 request a light sleep after an erase on core 0 starts.
#[cfg(multi_core)]
const SLEEP_DURING_ERASE: u32 = 3;

/// Whether core 1's last light sleep returned `Refused`.
#[cfg(multi_core)]
static CORE1_REFUSED: AtomicBool = AtomicBool::new(false);
/// The length in microseconds of core 1's last light sleep call.
#[cfg(multi_core)]
static CORE1_TOOK_US: AtomicU32 = AtomicU32::new(0);

// On a chip that retains the CPU in software, core 1 cannot start a light sleep. Core 0 gets the
// request from core 1. Core 0 parks core 1, sleeps, and releases core 1 after the wake.
//
// With CPU retention installed, each light sleep powers the CPU domain down. Each core saves its
// registers before the sleep and restores them after the wake. Core 1 joins the sleep through the
// rendezvous with core 0. Core 1 spins with interrupts enabled, so the rendezvous reaches core 1.
//
// On the ESP32 and ESP32-S3, core 1 starts the sleep and parks core 0.
#[cfg(multi_core)]
fn app_core_entry() -> ! {
    loop {
        match COMMAND.load(Ordering::Acquire) {
            SLEEP => {
                let mut lpwr = LowPower::new(unsafe { esp_hal::peripherals::LPWR::steal() });
                let result = lpwr.sleep_light(RtcSleepConfig::default());
                CORE1_REFUSED.store(result == LightSleep::Refused, Ordering::Relaxed);
                COMMAND.store(IDLE, Ordering::Release);
            }
            ERASE => {
                ERASE_OK.store(erase(), Ordering::Release);
                COMMAND.store(IDLE, Ordering::Release);
            }
            SLEEP_DURING_ERASE => {
                wait_for_erase();
                let mut lpwr = LowPower::new(unsafe { esp_hal::peripherals::LPWR::steal() });
                let before = Instant::now();
                let result = lpwr.sleep_light(RtcSleepConfig::default());
                let took = Instant::now() - before;
                CORE1_TOOK_US.store(took.as_micros() as u32, Ordering::Relaxed);
                CORE1_REFUSED.store(result == LightSleep::Refused, Ordering::Relaxed);
                COMMAND.store(IDLE, Ordering::Release);
            }
            _ => core::hint::spin_loop(),
        }
    }
}

/// Starts core 1 the first time a check needs it.
#[cfg(multi_core)]
fn start_core1(board: &mut Board<'_>) {
    if !board.core1_started {
        static mut APP_CORE_STACK: Stack<8192> = Stack::new();
        let guard = board
            .cpu_control
            .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, || {
                app_core_entry()
            })
            .unwrap();
        core::mem::forget(guard);
        board.core1_started = true;
    }
}

/// Waits up to `limit` for core 1 to end its command.
#[cfg(multi_core)]
fn wait_for_core1(limit: Duration) -> Result<(), &'static str> {
    let end = Instant::now() + limit;
    while COMMAND.load(Ordering::Acquire) != IDLE {
        if Instant::now() > end {
            return Err("core 1 did not end its command");
        }
    }
    Ok(())
}

/// Sends core 1 the command to request a light sleep.
#[cfg(multi_core)]
fn request_on_core1(board: &mut Board<'_>) -> Result<(), &'static str> {
    start_core1(board);
    COMMAND.store(SLEEP, Ordering::Release);
    wait_for_core1(Duration::from_secs(1)).map_err(|_| "core 1's light sleep did not return")
}

#[cfg(multi_core)]
fn light_sleep_from_core1(board: &mut Board<'_>) -> Result<(), &'static str> {
    require_no_retention(board)?;
    sleep_and_check(board, request_on_core1)
}

#[cfg(all(multi_core, supports_cpu_power_down))]
fn light_sleep_from_core1_with_retention(board: &mut Board<'_>) -> Result<(), &'static str> {
    install_retention(board);
    sleep_and_check(board, request_on_core1)
}

const BLOCK: u32 = 0x1_0000;

/// The bound on a light sleep call that returns without a sleep. An erase runs for several hundred
/// milliseconds, so a call that waits for the erase exceeds it.
#[cfg(multi_core)]
const NO_SLEEP: Duration = Duration::from_millis(5);

/// The flash, and the block that the checks erase. `main` sets it when code is in PSRAM. Core 1
/// uses it only during its commands, and core 0 only at other times.
static mut FLASH: Option<(Flash<'static, Blocking>, u32)> = None;

static ERASING: AtomicBool = AtomicBool::new(false);
static ERASE_DONE: AtomicBool = AtomicBool::new(false);
static ERASE_OK: AtomicBool = AtomicBool::new(false);

/// Returns a random 64 KiB block after the image in the booted partition, so that each run wears
/// another.
fn scratch_block(flash: &mut Flash<'_, Blocking>) -> u32 {
    let free = free_flash(flash);
    let blocks = free.len() as u32 / BLOCK;
    assert!(blocks > 0, "no free 64 KiB block after the image");
    free.start + esp_hal::rng::Rng::new().random() % blocks * BLOCK
}

/// Erases the block of [`FLASH`], and returns whether the erase succeeded.
fn erase() -> bool {
    // SAFETY: the caller is the only user of `FLASH` until the erase ends.
    let Some((flash, block)) = (unsafe { (*addr_of_mut!(FLASH)).as_mut() }) else {
        return false;
    };
    let block = *block;
    ERASING.store(true, Ordering::Release);
    // SAFETY: the block is in the booted partition, after the image.
    let erased = unsafe { flash.erase(block, block + BLOCK) }.is_ok();
    ERASE_DONE.store(true, Ordering::Release);
    erased
}

/// Waits until an erase has started on the other core. The ROM call starts within microseconds of
/// `ERASING`, and the erase runs for several hundred milliseconds.
#[cfg(multi_core)]
fn wait_for_erase() {
    while !ERASING.load(Ordering::Acquire) {}
    let end = Instant::now() + Duration::from_millis(1);
    while Instant::now() < end {}
}

fn clear_erase_flags() {
    ERASING.store(false, Ordering::Relaxed);
    ERASE_DONE.store(false, Ordering::Relaxed);
    ERASE_OK.store(false, Ordering::Relaxed);
}

/// Core 1 erases, and core 0 requests a light sleep during the erase. The light sleep must be
/// refused at once, and the light sleep after the erase must not be.
#[cfg(multi_core)]
fn refused_during_core1_erase(board: &mut Board<'_>) -> Result<(), &'static str> {
    start_core1(board);
    clear_erase_flags();
    COMMAND.store(ERASE, Ordering::Release);
    wait_for_erase();

    board
        .lpwr
        .set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
    let before = Instant::now();
    let result = board.lpwr.sleep_light(RtcSleepConfig::default());
    let took = Instant::now() - before;
    let during = !ERASE_DONE.load(Ordering::Acquire);

    wait_for_core1(Duration::from_secs(2))?;
    if !ERASE_OK.load(Ordering::Acquire) {
        return Err("the erase on core 1 failed");
    }
    if result != LightSleep::Refused {
        return Err("the light sleep was not refused");
    }
    if took >= NO_SLEEP {
        report!("  the refused light sleep took {took}");
        return Err("the refused light sleep did not return at once");
    }
    if !during {
        return Err("the erase ended before the light sleep returned");
    }

    board
        .lpwr
        .set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
    if board.lpwr.sleep_light(RtcSleepConfig::default()) == LightSleep::Refused {
        return Err("the light sleep after the erase was refused");
    }
    Ok(())
}

/// Core 0 erases, and core 1 requests a light sleep during the erase. The light sleep must be
/// refused at once, and core 1's light sleep after the erase must not be.
#[cfg(multi_core)]
fn refused_during_core0_erase(board: &mut Board<'_>) -> Result<(), &'static str> {
    start_core1(board);
    clear_erase_flags();
    board
        .lpwr
        .set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
    COMMAND.store(SLEEP_DURING_ERASE, Ordering::Release);
    let erased = erase();
    let during = COMMAND.load(Ordering::Acquire) == IDLE;

    wait_for_core1(Duration::from_secs(1))?;
    if !erased {
        return Err("the erase on core 0 failed");
    }
    if !CORE1_REFUSED.load(Ordering::Relaxed) {
        return Err("core 1's light sleep was not refused");
    }
    let took = Duration::from_micros(u64::from(CORE1_TOOK_US.load(Ordering::Relaxed)));
    if took >= NO_SLEEP {
        report!("  core 1's refused light sleep took {took}");
        return Err("core 1's refused light sleep did not return at once");
    }
    if !during {
        return Err("core 1's light sleep returned after the erase");
    }

    board
        .lpwr
        .set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
    COMMAND.store(SLEEP, Ordering::Release);
    wait_for_core1(Duration::from_secs(1))?;
    if CORE1_REFUSED.load(Ordering::Relaxed) {
        return Err("core 1's light sleep after the erase was refused");
    }
    Ok(())
}

#[cfg(multi_core)]
fn light_sleep_during_core1_erase(board: &mut Board<'_>) -> Result<(), &'static str> {
    require_no_retention(board)?;
    refused_during_core1_erase(board)
}

#[cfg(multi_core)]
fn core1_light_sleep_during_erase(board: &mut Board<'_>) -> Result<(), &'static str> {
    require_no_retention(board)?;
    refused_during_core0_erase(board)
}

#[cfg(all(multi_core, supports_cpu_power_down))]
fn light_sleep_with_retention_during_core1_erase(
    board: &mut Board<'_>,
) -> Result<(), &'static str> {
    install_retention(board);
    refused_during_core1_erase(board)
}

#[cfg(all(multi_core, supports_cpu_power_down))]
fn core1_light_sleep_with_retention_during_erase(
    board: &mut Board<'_>,
) -> Result<(), &'static str> {
    install_retention(board);
    refused_during_core0_erase(board)
}

/// A low-priority esp-rtos thread erases, and this thread, at a higher priority, requests light
/// sleeps during the erase. Each light sleep must be refused. After the erase, a light sleep must
/// pass the checks of [`sleep_and_check`].
#[cfg(not(multi_core))]
fn light_sleep_from_a_thread_during_an_erase(board: &mut Board<'_>) -> Result<(), &'static str> {
    use esp_rtos::{
        CurrentThreadHandle,
        thread::{Stack, ThreadSpawner},
    };
    use static_cell::ConstStaticCell;

    static ERASER_STACK: ConstStaticCell<Stack<8192>> = ConstStaticCell::new(Stack::new());

    // SAFETY: nothing else in the program uses TIMG0.
    let timg0 =
        esp_hal::timer::timg::TimerGroup::new(unsafe { esp_hal::peripherals::TIMG0::steal() });
    esp_rtos::start(timg0.timer0);

    clear_erase_flags();
    // The erasing thread runs only while this thread blocks.
    CurrentThreadHandle::get().set_priority(2);
    ThreadSpawner::from_static(ERASER_STACK.take())
        .with_priority(1)
        .spawn(|| ERASE_OK.store(erase(), Ordering::Release))
        .detach();
    while !ERASING.load(Ordering::Acquire) {
        CurrentThreadHandle::get().delay(Duration::from_millis(1));
    }

    let mut refused = 0u32;
    loop {
        board
            .lpwr
            .set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
        let result = board.lpwr.sleep_light(RtcSleepConfig::default());
        if ERASE_DONE.load(Ordering::Acquire) {
            break;
        }
        if result != LightSleep::Refused {
            report!("  the light sleep after {refused} refused ones was not refused");
            return Err("a light sleep during the erase was not refused");
        }
        refused += 1;
        // A refused light sleep returns at once, so the erasing thread runs only in this delay.
        CurrentThreadHandle::get().delay(Duration::from_millis(5));
    }
    if !ERASE_OK.load(Ordering::Acquire) {
        return Err("the erase failed");
    }
    if refused == 0 {
        return Err("the erase ended before the first light sleep");
    }
    report!("  {refused} light sleeps were refused during the erase");
    sleep_and_check(board, request_on_core0)
}

/// Returns the index of the first check at or after `index` that runs on this chip.
fn next_check(index: usize) -> usize {
    (index..CHECKS.len())
        .find(|&i| CHECKS[i].runs())
        .unwrap_or(CHECKS.len())
}

#[main]
fn main() -> ! {
    let reset = reset_reason(Cpu::ProCpu);
    let loaded = PROGRESS.load(CHECKS.len());
    // When the program did not request the reset, the stored check did not finish.
    let hung = loaded
        .filter(|stored| !stored.reset_requested)
        .map(|stored| stored.next);
    let mut progress = loaded.unwrap_or(Progress {
        next: 0,
        passed: 0,
        reset_requested: false,
    });
    if let Some(hung) = hung {
        progress.next = hung + 1;
    }
    progress.next = next_check(progress.next);
    let clock = CHECKS
        .get(progress.next)
        .map_or(Clock::Default, |check| check.clock);

    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(clock.cpu_clock()));

    report!(
        "sleep-checks: reset={:?}, CPU clock {:?}",
        reset,
        clock.cpu_clock()
    );

    if let Some(hung) = hung {
        report!("FAIL {}: reset {:?}", CHECKS[hung].name, reset);
    }
    progress.reset_requested = false;

    let mut rtc = Rtc::new(peripherals.RTC_TIMER);
    rtc.rwdt.set_timeout(RwdtStage::Stage0, WATCHDOG_TIMEOUT);
    rtc.rwdt.enable();

    #[cfg(psram_driver_supported)]
    let psram: &'static mut [u32] = {
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
    let psram: &'static mut [u32] = &mut [];
    report!("PSRAM: {} KiB", psram.len() / 256);

    if code_in_psram() {
        let mut flash = Flash::new(peripherals.FLASH, FlashConfig::default()).unwrap();
        let block = scratch_block(&mut flash);
        unsafe { FLASH = Some((flash, block)) };
    }

    let mut board = Board {
        rtc,
        lpwr: LowPower::new(peripherals.LPWR),
        psram,
        seed: PSRAM_SEED,
        #[cfg(supports_cpu_power_down)]
        retention_installed: false,
        #[cfg(multi_core)]
        cpu_control: CpuControl::new(peripherals.CPU_CTRL),
        #[cfg(multi_core)]
        core1_started: false,
    };

    while progress.next < CHECKS.len() {
        let check = &CHECKS[progress.next];
        if check.clock != clock {
            progress.reset_requested = true;
            PROGRESS.store(progress);
            report!("reset for the {:?} CPU clock", check.clock.cpu_clock());
            esp_hal::system::software_reset();
        }

        PROGRESS.store(progress);
        board.rtc.rwdt.feed();
        // The slice number takes the low bits.
        board.seed = PSRAM_SEED ^ ((progress.next as u32 + 1) << 20);
        report!("RUN {}", check.name);
        match (check.run)(&mut board) {
            Ok(()) => {
                report!("PASS {}", check.name);
                progress.passed += 1;
            }
            Err(reason) => report!("FAIL {}: {}", check.name, reason),
        }
        progress.next = next_check(progress.next + 1);
    }

    PROGRESS.store(progress);
    for check in CHECKS
        .iter()
        .filter(|check| check.runs_on_chip() && !check.runs())
    {
        report!("SKIP {}: needs code in PSRAM", check.name);
    }
    report!(
        "sleep-checks: {} of {} checks passed",
        progress.passed,
        CHECKS.iter().filter(|check| check.runs()).count()
    );
    board.rtc.rwdt.disable();
    loop {
        core::hint::spin_loop();
    }
}
