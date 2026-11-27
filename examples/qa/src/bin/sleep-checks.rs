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
//! 4. On a chip with an FPU, the callee-saved FPU registers and `fcsr` of each core that took part
//!    in the sleep. They hold a pattern across the request, and a light sleep with CPU retention
//!    powers the FPU down.
//!
//! The checks run at the default and at the maximum CPU clock, without and with CPU retention. On a
//! chip with two cores, each core requests a light sleep.
//!
//! On a chip with two cores and with code in PSRAM, two more checks request a light sleep on one
//! core during a flash erase on the other. The light sleep must be refused, and the same light
//! sleep after the erase must not be refused. A check that misses the erase tries again.
//!
//! Set `ESP_HAL_CONFIG_CODE_IN_PSRAM=true` to build with code in PSRAM.
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

//% CHIP_FILTER: sleep_light_sleep && sleep_has_wakeup_source_timer

#![no_std]
#![no_main]

#[cfg(multi_core)]
use core::{
    ptr::addr_of_mut,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use esp_backtrace as _;
#[cfg(multi_core)]
use esp_hal::system::{CpuControl, Stack};
#[cfg(all(multi_core, code_in_psram))]
use esp_hal::{
    Blocking,
    flash::{Config as FlashConfig, Flash},
};
use esp_hal::{
    clock::CpuClock,
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
use qa_test::{Progress, ProgressWord, report};
#[cfg(all(multi_core, code_in_psram))]
use qa_test::{flash_operation, scratch_blocks};

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
    run: fn(&mut Board<'_>) -> Result<(), &'static str>,
}

impl Check {
    /// Returns whether the check runs on this chip. A chip with its default CPU clock at the
    /// maximum runs each check once.
    fn runs(&self) -> bool {
        self.clock == Clock::Default || CpuClock::default() != CpuClock::max()
    }
}

/// The checks, in the order they run.
///
/// CPU retention cannot be removed after installation. A check without CPU retention must come
/// before the checks with it at the same CPU clock.
const CHECKS: &[Check] = &[
    Check {
        name: "light sleep at the default CPU clock",
        clock: Clock::Default,
        run: light_sleep,
    },
    #[cfg(multi_core)]
    Check {
        name: "light sleep from core 1 at the default CPU clock",
        clock: Clock::Default,
        run: light_sleep_from_core1,
    },
    #[cfg(all(multi_core, code_in_psram))]
    Check {
        name: "light sleep during an erase on core 1",
        clock: Clock::Default,
        run: refused_during_core1_erase,
    },
    #[cfg(all(multi_core, code_in_psram))]
    Check {
        name: "light sleep from core 1 during an erase on core 0",
        clock: Clock::Default,
        run: refused_during_core0_erase,
    },
    #[cfg(supports_cpu_power_down)]
    Check {
        name: "light sleep with CPU retention at the default CPU clock",
        clock: Clock::Default,
        run: light_sleep_with_retention,
    },
    #[cfg(all(multi_core, supports_cpu_power_down))]
    Check {
        name: "light sleep from core 1 with CPU retention at the default CPU clock",
        clock: Clock::Default,
        run: light_sleep_from_core1_with_retention,
    },
    Check {
        name: "light sleep at the maximum CPU clock",
        clock: Clock::Max,
        run: light_sleep,
    },
    #[cfg(multi_core)]
    Check {
        name: "light sleep from core 1 at the maximum CPU clock",
        clock: Clock::Max,
        run: light_sleep_from_core1,
    },
    #[cfg(supports_cpu_power_down)]
    Check {
        name: "light sleep with CPU retention at the maximum CPU clock",
        clock: Clock::Max,
        run: light_sleep_with_retention,
    },
    #[cfg(all(multi_core, supports_cpu_power_down))]
    Check {
        name: "light sleep from core 1 with CPU retention at the maximum CPU clock",
        clock: Clock::Max,
        run: light_sleep_from_core1_with_retention,
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
        unsafe { core::ptr::write_volatile(&raw mut PSRAM_STATIC[i], word) };
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
    {
        let bad = (0..STATIC_WORDS)
            .filter(|&i| {
                let word = unsafe { core::ptr::read_volatile(&raw const PSRAM_STATIC[i]) };
                word != pattern(i, STATIC_SEED ^ board.seed)
            })
            .count();
        if bad != 0 {
            report!("  the PSRAM static read {bad} bad words");
            failed = true;
        }
    }
    if failed {
        Err("PSRAM lost data")
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
    with_fpu_pattern(|| request(board))?;
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

/// The values of `fs0`-`fs11` while a sleep request runs.
#[cfg(target_feature = "f")]
const FPU_PATTERN: [u32; 12] = [
    0x3F80_0001,
    0x4000_0002,
    0xC040_0003,
    0x4080_0004,
    0xC0A0_0005,
    0x40C0_0006,
    0xC0E0_0007,
    0x4100_0008,
    0xC110_0009,
    0x4120_000A,
    0xC130_000B,
    0x4140_000C,
];

/// The value of `fcsr` while a sleep request runs: every exception flag set, and the default
/// rounding mode. Code that runs in the request can only set flags, so the value stays.
#[cfg(target_feature = "f")]
const FCSR_PATTERN: u32 = 0x1F;

/// Runs `f` with [`FPU_PATTERN`] in `fs0`-`fs11` and [`FCSR_PATTERN`] in `fcsr`, and checks them
/// after `f` returns.
///
/// The registers are callee-saved, so a call must keep them. `f` runs from the assembly, because
/// the compiler would not keep a value in a chosen register across a Rust call.
#[cfg(target_feature = "f")]
fn with_fpu_pattern<F>(f: F) -> Result<(), &'static str>
where
    F: FnOnce() -> Result<(), &'static str>,
{
    struct Call<F> {
        f: Option<F>,
        result: Result<(), &'static str>,
    }

    extern "C" fn trampoline<F>(call: &mut Call<F>)
    where
        F: FnOnce() -> Result<(), &'static str>,
    {
        if let Some(f) = call.f.take() {
            call.result = f();
        }
    }

    let mut call = Call {
        f: Some(f),
        result: Ok(()),
    };
    let mut after = [0u32; 13];
    unsafe {
        core::arch::asm!(
            "frcsr s4",
            "flw fs0, 0(s2)",
            "flw fs1, 4(s2)",
            "flw fs2, 8(s2)",
            "flw fs3, 12(s2)",
            "flw fs4, 16(s2)",
            "flw fs5, 20(s2)",
            "flw fs6, 24(s2)",
            "flw fs7, 28(s2)",
            "flw fs8, 32(s2)",
            "flw fs9, 36(s2)",
            "flw fs10, 40(s2)",
            "flw fs11, 44(s2)",
            "fscsr {fcsr}",
            "call {trampoline}",
            "fsw fs0, 0(s3)",
            "fsw fs1, 4(s3)",
            "fsw fs2, 8(s3)",
            "fsw fs3, 12(s3)",
            "fsw fs4, 16(s3)",
            "fsw fs5, 20(s3)",
            "fsw fs6, 24(s3)",
            "fsw fs7, 28(s3)",
            "fsw fs8, 32(s3)",
            "fsw fs9, 36(s3)",
            "fsw fs10, 40(s3)",
            "fsw fs11, 44(s3)",
            "frcsr t0",
            "sw t0, 48(s3)",
            "fscsr s4",
            trampoline = sym trampoline::<F>,
            fcsr = in(reg) FCSR_PATTERN,
            in("a0") &mut call,
            in("s2") FPU_PATTERN.as_ptr(),
            in("s3") after.as_mut_ptr(),
            out("s4") _,
            out("fs0") _,
            out("fs1") _,
            out("fs2") _,
            out("fs3") _,
            out("fs4") _,
            out("fs5") _,
            out("fs6") _,
            out("fs7") _,
            out("fs8") _,
            out("fs9") _,
            out("fs10") _,
            out("fs11") _,
            clobber_abi("C"),
        );
    }
    call.result?;

    let mut lost = false;
    for (n, (&read, &expected)) in after.iter().zip(&FPU_PATTERN).enumerate() {
        if read != expected {
            report!("  fs{n} read {read:#010x}, expected {expected:#010x}");
            lost = true;
        }
    }
    if after[12] != FCSR_PATTERN {
        report!("  fcsr read {:#x}, expected {FCSR_PATTERN:#x}", after[12]);
        lost = true;
    }
    if lost {
        Err("the light sleep lost FPU registers")
    } else {
        Ok(())
    }
}

#[cfg(not(target_feature = "f"))]
fn with_fpu_pattern(f: impl FnOnce() -> Result<(), &'static str>) -> Result<(), &'static str> {
    f()
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
#[cfg(all(multi_core, code_in_psram))]
const ERASE: u32 = 2;
/// The command that makes core 1 request a light sleep after an erase on core 0 starts.
#[cfg(all(multi_core, code_in_psram))]
const SLEEP_DURING_ERASE: u32 = 3;

#[cfg(multi_core)]
static CORE1_REFUSED: AtomicBool = AtomicBool::new(false);
/// Whether core 1 saw core 0's erase in progress, and requested its light sleep.
#[cfg(all(multi_core, code_in_psram))]
static CORE1_REQUESTED: AtomicBool = AtomicBool::new(false);
/// The length in microseconds of core 1's last light sleep call.
#[cfg(all(multi_core, code_in_psram))]
static CORE1_TOOK_US: AtomicU32 = AtomicU32::new(0);

/// Whether core 1's last light sleep lost its FPU registers. Core 1 writes it before it sets
/// `COMMAND` back to `IDLE`.
#[cfg(multi_core)]
static CORE1_FPU_LOST: AtomicBool = AtomicBool::new(false);

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
                let mut refused = false;
                let result = with_fpu_pattern(|| {
                    refused = lpwr.sleep_light(RtcSleepConfig::default()) == LightSleep::Refused;
                    Ok(())
                });
                CORE1_FPU_LOST.store(result.is_err(), Ordering::Relaxed);
                CORE1_REFUSED.store(refused, Ordering::Relaxed);
                COMMAND.store(IDLE, Ordering::Release);
            }
            #[cfg(code_in_psram)]
            ERASE => {
                ERASE_OK.store(erase(), Ordering::Release);
                COMMAND.store(IDLE, Ordering::Release);
            }
            #[cfg(code_in_psram)]
            SLEEP_DURING_ERASE => {
                let requested = wait_for_erase(Cpu::ProCpu);
                if requested {
                    let mut lpwr = LowPower::new(unsafe { esp_hal::peripherals::LPWR::steal() });
                    lpwr.set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
                    let before = Instant::now();
                    let result = lpwr.sleep_light_if(RtcSleepConfig::default(), may_sleep);
                    let took = Instant::now() - before;
                    CORE1_TOOK_US.store(took.as_micros() as u32, Ordering::Relaxed);
                    CORE1_REFUSED.store(result == LightSleep::Refused, Ordering::Relaxed);
                }
                CORE1_REQUESTED.store(requested, Ordering::Relaxed);
                COMMAND.store(IDLE, Ordering::Release);
            }
            _ => core::hint::spin_loop(),
        }
    }
}

/// The registers that [`wait_with_caller_saved_pattern`] holds a pattern in, in the order of the
/// pattern.
#[cfg(all(multi_core, target_feature = "f"))]
const CALLER_SAVED: [&str; 35] = [
    "t0", "t1", "t2", "t3", "t4", "t5", "t6", "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7",
    "ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7", "ft8", "ft9", "ft10", "ft11", "fa0",
    "fa1", "fa2", "fa3", "fa4", "fa5", "fa6", "fa7",
];

/// Sends core 1 the command to sleep, and waits for its light sleep with a pattern in every
/// caller-saved register.
///
/// Core 1's request runs the sleep in the IPC handler of this core. The handler interrupts the
/// wait loop, so the loop's registers must come back as they were. The wait has no timeout: the
/// watchdog resets a sleep that does not return.
#[cfg(all(multi_core, target_feature = "f"))]
fn wait_with_caller_saved_pattern() -> Result<(), &'static str> {
    const _: () = assert!(IDLE == 0);

    let pattern: [u32; 35] =
        core::array::from_fn(|n| 0x5A00_0000 ^ (n as u32).wrapping_mul(0x0101_0101));
    let mut after = [0u32; 35];
    unsafe {
        core::arch::asm!(
            "lw t0, 0(s2)",
            "lw t1, 4(s2)",
            "lw t2, 8(s2)",
            "lw t3, 12(s2)",
            "lw t4, 16(s2)",
            "lw t5, 20(s2)",
            "lw t6, 24(s2)",
            "lw a0, 28(s2)",
            "lw a1, 32(s2)",
            "lw a2, 36(s2)",
            "lw a3, 40(s2)",
            "lw a4, 44(s2)",
            "lw a5, 48(s2)",
            "lw a6, 52(s2)",
            "lw a7, 56(s2)",
            "flw ft0, 60(s2)",
            "flw ft1, 64(s2)",
            "flw ft2, 68(s2)",
            "flw ft3, 72(s2)",
            "flw ft4, 76(s2)",
            "flw ft5, 80(s2)",
            "flw ft6, 84(s2)",
            "flw ft7, 88(s2)",
            "flw ft8, 92(s2)",
            "flw ft9, 96(s2)",
            "flw ft10, 100(s2)",
            "flw ft11, 104(s2)",
            "flw fa0, 108(s2)",
            "flw fa1, 112(s2)",
            "flw fa2, 116(s2)",
            "flw fa3, 120(s2)",
            "flw fa4, 124(s2)",
            "flw fa5, 128(s2)",
            "flw fa6, 132(s2)",
            "flw fa7, 136(s2)",
            "li s5, {sleep}",
            "fence rw, w",
            "sw s5, 0(s4)",
            "1:",
            "lw s5, 0(s4)",
            "bnez s5, 1b",
            "fence r, rw",
            "sw t0, 0(s3)",
            "sw t1, 4(s3)",
            "sw t2, 8(s3)",
            "sw t3, 12(s3)",
            "sw t4, 16(s3)",
            "sw t5, 20(s3)",
            "sw t6, 24(s3)",
            "sw a0, 28(s3)",
            "sw a1, 32(s3)",
            "sw a2, 36(s3)",
            "sw a3, 40(s3)",
            "sw a4, 44(s3)",
            "sw a5, 48(s3)",
            "sw a6, 52(s3)",
            "sw a7, 56(s3)",
            "fsw ft0, 60(s3)",
            "fsw ft1, 64(s3)",
            "fsw ft2, 68(s3)",
            "fsw ft3, 72(s3)",
            "fsw ft4, 76(s3)",
            "fsw ft5, 80(s3)",
            "fsw ft6, 84(s3)",
            "fsw ft7, 88(s3)",
            "fsw ft8, 92(s3)",
            "fsw ft9, 96(s3)",
            "fsw ft10, 100(s3)",
            "fsw ft11, 104(s3)",
            "fsw fa0, 108(s3)",
            "fsw fa1, 112(s3)",
            "fsw fa2, 116(s3)",
            "fsw fa3, 120(s3)",
            "fsw fa4, 124(s3)",
            "fsw fa5, 128(s3)",
            "fsw fa6, 132(s3)",
            "fsw fa7, 136(s3)",
            sleep = const SLEEP,
            in("s2") pattern.as_ptr(),
            in("s3") after.as_mut_ptr(),
            in("s4") COMMAND.as_ptr(),
            out("s5") _,
            clobber_abi("C"),
        );
    }

    let mut lost = false;
    for ((name, &read), &expected) in CALLER_SAVED.iter().zip(&after).zip(&pattern) {
        if read != expected {
            report!("  core 0 {name} read {read:#010x}, expected {expected:#010x}");
            lost = true;
        }
    }
    if lost {
        Err("core 0 lost caller-saved registers in core 1's light sleep")
    } else {
        Ok(())
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

/// Starts core 1 the first time a check needs it, then sends core 1 the command to request a light
/// sleep.
#[cfg(multi_core)]
fn request_on_core1(board: &mut Board<'_>) -> Result<(), &'static str> {
    start_core1(board);
    cfg_select! {
        target_feature = "f" => wait_with_caller_saved_pattern()?,
        _ => {
            COMMAND.store(SLEEP, Ordering::Release);
            wait_for_core1(Duration::from_secs(1))
                .map_err(|_| "core 1's light sleep did not return")?;
        }
    }
    if CORE1_FPU_LOST.load(Ordering::Relaxed) {
        return Err("core 1's light sleep lost FPU registers");
    }
    Ok(())
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

#[cfg(all(multi_core, code_in_psram))]
const BLOCK: u32 = 0x1_0000;

/// The hang bound on one block erase, 1 s less than the RTC watchdog's 5 s. A check feeds the
/// watchdog before each erase.
#[cfg(all(multi_core, code_in_psram))]
const ERASE_BOUND: Duration = Duration::from_secs(4);

/// The erases that a check tries before it fails.
#[cfg(all(multi_core, code_in_psram))]
const ATTEMPTS: u32 = 8;

/// The flash, and the block that the checks erase. Core 1 uses it only in its `ERASE` command, and
/// core 0 only while core 1 does not run that command.
#[cfg(all(multi_core, code_in_psram))]
static mut FLASH: Option<(Flash<'static, Blocking>, u32)> = None;

#[cfg(all(multi_core, code_in_psram))]
static ERASING: AtomicBool = AtomicBool::new(false);
#[cfg(all(multi_core, code_in_psram))]
static ERASE_DONE: AtomicBool = AtomicBool::new(false);
#[cfg(all(multi_core, code_in_psram))]
static ERASE_OK: AtomicBool = AtomicBool::new(false);
/// Whether [`may_sleep`] ran, and whether it found a flash operation in progress.
#[cfg(all(multi_core, code_in_psram))]
static CONDITION_RAN: AtomicBool = AtomicBool::new(false);
#[cfg(all(multi_core, code_in_psram))]
static HELD_IN_CONDITION: AtomicBool = AtomicBool::new(false);

/// The pattern for [`write_block`]. A nonzero `static mut` is in internal RAM, where a ROM flash
/// routine can read it.
#[cfg(all(multi_core, code_in_psram))]
static mut BLOCK_PATTERN: [u32; 4096] = [0x5AA5_C33C; 4096];

/// The condition of the checks' light sleeps. The light sleep calls it only after its flash check
/// passed, with the other core parked, so an operation in progress here shows a broken check.
#[cfg(all(multi_core, code_in_psram))]
fn may_sleep() -> bool {
    let held = flash_operation() != 0;
    HELD_IN_CONDITION.store(held, Ordering::Relaxed);
    CONDITION_RAN.store(true, Ordering::Release);
    !held
}

#[cfg(all(multi_core, code_in_psram))]
enum Outcome {
    Refused,
    /// The request missed the erase: the flash check let it through with no operation in progress.
    Missed,
}

#[cfg(all(multi_core, code_in_psram))]
fn outcome(result: LightSleep) -> Result<Outcome, &'static str> {
    match (CONDITION_RAN.load(Ordering::Acquire), result) {
        (true, _) if HELD_IN_CONDITION.load(Ordering::Relaxed) => {
            Err("the flash check let a light sleep through during a flash operation")
        }
        (true, _) => Ok(Outcome::Missed),
        (false, LightSleep::Refused) => Ok(Outcome::Refused),
        (false, _) => Err("a light sleep ended without the flash check or the condition"),
    }
}

/// Erases the block of [`FLASH`], and returns whether the erase succeeded.
#[cfg(all(multi_core, code_in_psram))]
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

/// Writes the pattern over the block of [`FLASH`]. A written block takes longer to erase.
#[cfg(all(multi_core, code_in_psram))]
fn write_block() {
    // SAFETY: the caller is the only user of `FLASH` until the write ends.
    let Some((flash, block)) = (unsafe { (*addr_of_mut!(FLASH)).as_mut() }) else {
        return;
    };
    // SAFETY: nothing writes `BLOCK_PATTERN`.
    let pattern = unsafe { &*core::ptr::addr_of!(BLOCK_PATTERN) };
    for offset in (0..BLOCK).step_by(core::mem::size_of_val(pattern)) {
        // SAFETY: the block is in the booted partition, after the image.
        unsafe { flash.write(*block + offset, pattern) }.unwrap();
    }
}

/// Waits until `eraser`, the other core, is inside its erase: `ERASING` is set, and the flash value
/// names `eraser`. Returns `false` if the erase ends first, or after 1 s.
#[cfg(all(multi_core, code_in_psram))]
fn wait_for_erase(eraser: Cpu) -> bool {
    let end = Instant::now() + Duration::from_secs(1);
    loop {
        if ERASING.load(Ordering::Acquire) && flash_operation() == eraser as u32 + 1 {
            return true;
        }
        if ERASE_DONE.load(Ordering::Acquire) || Instant::now() > end {
            return false;
        }
    }
}

#[cfg(all(multi_core, code_in_psram))]
fn clear_erase_flags() {
    ERASING.store(false, Ordering::Relaxed);
    ERASE_DONE.store(false, Ordering::Relaxed);
    ERASE_OK.store(false, Ordering::Relaxed);
    CONDITION_RAN.store(false, Ordering::Relaxed);
    HELD_IN_CONDITION.store(false, Ordering::Relaxed);
}

/// Core 0 requests a light sleep during an erase on core 1.
#[cfg(all(multi_core, code_in_psram))]
fn refused_during_core1_erase(board: &mut Board<'_>) -> Result<(), &'static str> {
    require_no_retention(board)?;
    start_core1(board);
    for _ in 0..ATTEMPTS {
        clear_erase_flags();
        // Before the command, so that the write is never under `ERASING`.
        write_block();
        board.rtc.rwdt.feed();
        COMMAND.store(ERASE, Ordering::Release);
        let requested = wait_for_erase(Cpu::AppCpu);
        let mut result = None;
        let mut took = Duration::ZERO;
        let mut during = false;
        if requested {
            board
                .lpwr
                .set_wakeup_deadline(Instant::now() + SLEEP_LENGTH);
            let before = Instant::now();
            result = Some(
                board
                    .lpwr
                    .sleep_light_if(RtcSleepConfig::default(), may_sleep),
            );
            took = Instant::now() - before;
            during = !ERASE_DONE.load(Ordering::Acquire);
        }

        wait_for_core1(ERASE_BOUND)?;
        if !ERASE_OK.load(Ordering::Acquire) {
            return Err("the erase on core 1 failed");
        }
        let Some(result) = result else { continue };
        if let Outcome::Refused = outcome(result)? {
            report!("  the refused light sleep took {took}, the erase still ran: {during}");
            return sleep_and_check(board, request_on_core0);
        }
    }
    Err("no light sleep was requested inside the erase")
}

/// Core 1 requests a light sleep during an erase on core 0.
#[cfg(all(multi_core, code_in_psram))]
fn refused_during_core0_erase(board: &mut Board<'_>) -> Result<(), &'static str> {
    require_no_retention(board)?;
    start_core1(board);
    for _ in 0..ATTEMPTS {
        clear_erase_flags();
        CORE1_REQUESTED.store(false, Ordering::Relaxed);
        write_block();
        board.rtc.rwdt.feed();
        COMMAND.store(SLEEP_DURING_ERASE, Ordering::Release);
        let erased = erase();
        let during = COMMAND.load(Ordering::Acquire) == IDLE;

        wait_for_core1(ERASE_BOUND)?;
        if !erased {
            return Err("the erase on core 0 failed");
        }
        if !CORE1_REQUESTED.load(Ordering::Relaxed) {
            continue;
        }
        let result = if CORE1_REFUSED.load(Ordering::Relaxed) {
            LightSleep::Refused
        } else {
            LightSleep::Ended
        };
        if let Outcome::Refused = outcome(result)? {
            let took = Duration::from_micros(u64::from(CORE1_TOOK_US.load(Ordering::Relaxed)));
            report!(
                "  core 1's refused light sleep took {took}, returned during the erase: {during}"
            );
            return sleep_and_check(board, request_on_core1);
        }
    }
    Err("core 1 requested no light sleep inside the erase")
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
    esp_log::init_uart!(peripherals);

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

    #[cfg(all(multi_core, code_in_psram))]
    {
        let mut flash = Flash::new(peripherals.FLASH, FlashConfig::default()).unwrap();
        let block = scratch_blocks(&mut flash, 1);
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
