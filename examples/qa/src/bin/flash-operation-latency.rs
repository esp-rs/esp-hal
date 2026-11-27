//! Measures how long interrupts and threads wait during flash operations, as the worst case in
//! microseconds:
//!
//! - interrupt and thread latency on the core that runs the operation;
//! - interrupt and thread latency on the other core;
//! - the longest stall of a bare loop on the other core.
//!
//! The program measures with no operation, and during a read, a write, a sector erase, and a block
//! erase. Part 1 runs a bare loop on the other core, and part 2 runs esp-rtos there.
//!
//! On a chip with two cores and with code in PSRAM, it also measures the first core's figures
//! while it stops the other core inside a sector erase. The stop waits for the end of the erase,
//! and the program checks that it did. Without code in PSRAM, an erase on the other core parks the
//! first core, so this row does not apply.
//!
//! The probes' handlers run at the lowest priority, so an interrupt figure includes the other
//! handlers and critical sections of esp-hal and esp-rtos.
//!
//! Build it with and without `ESP_HAL_CONFIG_CODE_IN_PSRAM=true` to compare. Without code in PSRAM,
//! the figures during an operation are about as long as the operation.
//!
//! The program prints a table for each part and ends with `DONE`.

//% CHIP_FILTER: psram_supports_code_in_psram

#![no_std]
#![no_main]

use core::{
    cell::{RefCell, UnsafeCell},
    ptr::addr_of_mut,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use esp_backtrace as _;
#[cfg(multi_core)]
use esp_hal::system::{CpuControl, Stack as CoreStack};
use esp_hal::{
    Blocking,
    flash::{Config as FlashConfig, Flash},
    handler,
    interrupt::InterruptHandler,
    system::Cpu,
    time::{Duration, Instant},
    timer::{
        PeriodicTimer,
        systimer::{SystemTimer, Unit},
        timg::TimerGroup,
    },
};
use esp_rtos::{
    CurrentThreadHandle,
    thread::{Stack, ThreadSpawner},
};
use esp_sync::raw::{RawLock, SingleCoreInterruptLock};
use qa_test::{code_in_psram, report, scratch_blocks};
use static_cell::ConstStaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

/// The period of the timer alarms and of the threads' delays.
const PERIOD: Duration = Duration::from_micros(200);
/// The wait after each operation, so that a late alarm or thread records its sample.
const SETTLE: Duration = Duration::from_millis(5);
const SECTOR: u32 = 0x1000;
const BLOCK: u32 = 0x1_0000;
const READ_WORDS: usize = 0x1000;

#[derive(Clone, Copy)]
enum Figure {
    InterruptSameCore,
    ThreadSameCore,
    InterruptOtherCore,
    ThreadOtherCore,
    BareLoopOtherCore,
}

const FIGURES: usize = 5;
const NAMES: [&str; FIGURES] = [
    "interrupt same core",
    "thread same core",
    "interrupt other core",
    "thread other core",
    "bare-loop stall other core",
];

/// The worst case of each figure, in microseconds, and its number of samples.
static WORST: [AtomicU32; FIGURES] = [const { AtomicU32::new(0) }; FIGURES];
static SAMPLES: [AtomicU32; FIGURES] = [const { AtomicU32::new(0) }; FIGURES];

/// The measured time: from `WINDOW_START` to `WINDOW_END`, or while `WINDOW_OPEN` is set. The times
/// are the low 32 bits of the microseconds since boot.
static WINDOW_START: AtomicU32 = AtomicU32::new(0);
static WINDOW_END: AtomicU32 = AtomicU32::new(0);
static WINDOW_OPEN: AtomicBool = AtomicBool::new(false);

fn now_us() -> u32 {
    Instant::now().duration_since_epoch().as_micros() as u32
}

fn in_window(time: u32) -> bool {
    let after_start = time.wrapping_sub(WINDOW_START.load(Ordering::Acquire)) as i32 >= 0;
    let before_end = WINDOW_OPEN.load(Ordering::Acquire)
        || WINDOW_END.load(Ordering::Acquire).wrapping_sub(time) as i32 >= 0;
    after_start && before_end
}

/// Records a sample of `figure` that was due at `due` and ran at `ran`.
fn record(figure: Figure, due: u32, ran: u32) {
    record_late(figure, due, ran.wrapping_sub(due));
}

fn record_late(figure: Figure, due: u32, late: u32) {
    if in_window(due) {
        WORST[figure as usize].fetch_max(late, Ordering::Relaxed);
        SAMPLES[figure as usize].fetch_add(1, Ordering::Relaxed);
    }
}

fn ticks() -> u64 {
    SystemTimer::unit_value(Unit::Unit0)
}

/// Ticks of SYSTIMER unit 0 in one microsecond. `Probe::start` sets it.
static TICKS_PER_US: AtomicU32 = AtomicU32::new(1);

/// Returns `REAL_TARGET` of comparator `n`: in period mode, the time of the next alarm, in ticks.
/// LO can change between the reads, so the function reads LO, HI, and LO until the LO values agree.
fn real_target(n: usize) -> u64 {
    let target = esp_hal::peripherals::SYSTIMER::regs().real_target(n);
    let mut lo = target.lo().read().bits();
    loop {
        let hi = target.hi().read().bits() & 0xF_FFFF;
        let again = target.lo().read().bits();
        if again == lo {
            return ((hi as u64) << 32) | lo as u64;
        }
        lo = again;
    }
}

/// A timer alarm in hardware period mode that records the latency of its handler. The due times
/// come from `REAL_TARGET`, so a late entry does not move them. The probe counts early entries,
/// alarms off the grid of the period, and alarms that the clear of the interrupt loses.
struct Probe {
    timer: UnsafeCell<Option<PeriodicTimer<'static, Blocking>>>,
    comparator: usize,
    /// The first due time, and the due time of the next entry, in ticks.
    first: UnsafeCell<u64>,
    due: UnsafeCell<u64>,
    early: AtomicU32,
    off_grid: AtomicU32,
    lost: AtomicU32,
    figure: Figure,
}

// SAFETY: `start` stores the timer and the due times with interrupts masked, before the first
// alarm, and then only the handler uses them.
unsafe impl Sync for Probe {}

impl Probe {
    const fn new(figure: Figure, comparator: usize) -> Self {
        Self {
            timer: UnsafeCell::new(None),
            comparator,
            first: UnsafeCell::new(0),
            due: UnsafeCell::new(0),
            early: AtomicU32::new(0),
            off_grid: AtomicU32::new(0),
            lost: AtomicU32::new(0),
            figure,
        }
    }

    fn start(&self, mut timer: PeriodicTimer<'static, Blocking>, handler: InterruptHandler) {
        TICKS_PER_US.store(
            (SystemTimer::ticks_per_second() / 1_000_000) as u32,
            Ordering::Relaxed,
        );
        let period = period_ticks();
        timer.set_interrupt_handler(handler);
        // Masked on this core only, so that the handler does not run before the due times are
        // stored.
        let token = unsafe { SingleCoreInterruptLock.enter() };
        timer.listen();
        // SAFETY: see the `Sync` implementation.
        let timer = unsafe { (*self.timer.get()).insert(timer) };
        let before = ticks();
        timer.start(PERIOD).unwrap();
        let next = real_target(self.comparator);
        // The hardware sets the first alarm one period after a count inside `start()`. So the
        // first alarm is the first time at or after `before + period` on the grid of `next`.
        let first = next - next.saturating_sub(before + period) / period * period;
        // SAFETY: see the `Sync` implementation.
        unsafe {
            *self.first.get() = first;
            *self.due.get() = first;
        }
        unsafe { SingleCoreInterruptLock.exit(token) };
    }

    fn run(&self) {
        let ran = ticks();
        // SAFETY: see the `Sync` implementation.
        unsafe { (*self.timer.get()).as_mut() }
            .unwrap()
            .clear_interrupt();
        // If the interrupt is pending again after the clear, the next entry serves an alarm that is
        // one period before `REAL_TARGET`.
        let period = period_ticks();
        let mut next = real_target(self.comparator);
        if esp_hal::peripherals::SYSTIMER::regs()
            .int_raw()
            .read()
            .target(self.comparator as u8)
            .bit_is_set()
        {
            next = real_target(self.comparator) - period;
        }
        // SAFETY: see the `Sync` implementation.
        let (first, due) = unsafe { (*self.first.get(), &mut *self.due.get()) };
        if (next - first) % period != 0 {
            self.off_grid.fetch_add(1, Ordering::Relaxed);
        }
        if ran < *due {
            self.early.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let per_us = TICKS_PER_US.load(Ordering::Relaxed) as u64;
        record_late(
            self.figure,
            (*due / per_us) as u32,
            ((ran - *due) / per_us) as u32,
        );
        // This entry serves the alarms from `due` to `ran`. A later alarm before the clear is lost.
        let served = (ran - *due) / period + 1;
        if next > *due + served * period {
            self.lost.fetch_add(1, Ordering::Relaxed);
        }
        *due = next;
    }
}

fn period_ticks() -> u64 {
    PERIOD.as_micros() * TICKS_PER_US.load(Ordering::Relaxed) as u64
}

static PROBE_SAME: Probe = Probe::new(Figure::InterruptSameCore, 0);
#[cfg(multi_core)]
static PROBE_OTHER: Probe = Probe::new(Figure::InterruptOtherCore, 1);

#[handler]
fn probe_same_handler() {
    PROBE_SAME.run();
}

#[cfg(multi_core)]
#[handler]
fn probe_other_handler() {
    PROBE_OTHER.run();
}

/// Delays the current thread by [`PERIOD`] in a loop, and records each delay's latency in `figure`.
fn measure_thread(figure: Figure) -> ! {
    loop {
        let due = Instant::now() + PERIOD;
        CurrentThreadHandle::get().delay_until(due);
        record(
            figure,
            due.duration_since_epoch().as_micros() as u32,
            now_us(),
        );
    }
}

const ROWS: [&str; 6] = [
    "no operation",
    "read 16 KiB",
    "write 4 KiB",
    "erase 4 KiB",
    "erase 64 KiB",
    "core stop inside erase",
];
const CORE_STOP: usize = 5;

const HEADS: [(&str, &str); FIGURES] = [
    ("interrupt", "same"),
    ("thread", "same"),
    ("interrupt", "other"),
    ("thread", "other"),
    ("bare loop", "other"),
];

/// One line of a table. The logger prints whole lines, so a row is written here first.
struct Line {
    text: [u8; 128],
    len: usize,
}

impl Line {
    const fn new() -> Self {
        Self {
            text: [0; 128],
            len: 0,
        }
    }

    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.text[..self.len]).unwrap_or("")
    }
}

impl core::fmt::Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let end = self.len + s.len();
        let Some(text) = self.text.get_mut(self.len..end) else {
            return Err(core::fmt::Error);
        };
        text.copy_from_slice(s.as_bytes());
        self.len = end;
        Ok(())
    }
}

/// The worst case of each figure in each row, and the longest operation, in one part.
struct Summary {
    worst: [[Option<u32>; FIGURES]; ROWS.len()],
    longest: [u32; ROWS.len()],
}

impl Summary {
    const fn new() -> Self {
        Self {
            worst: [[None; FIGURES]; ROWS.len()],
            longest: [0; ROWS.len()],
        }
    }

    fn add(&mut self, row: usize, worst: [Option<u32>; FIGURES], longest: u32) {
        for (cell, new) in self.worst[row].iter_mut().zip(worst) {
            *cell = (*cell).max(new);
        }
        self.longest[row] = self.longest[row].max(longest);
    }

    fn print(&self, title: &str, figures: &[Figure], core_stop: bool) {
        use core::fmt::Write;

        report!("{title}");
        for head in [0, 1] {
            let mut line = Line::new();
            let _ = write!(line, "  {:<24}", "");
            for figure in figures {
                let (top, bottom) = HEADS[*figure as usize];
                let text = if head == 0 { top } else { bottom };
                let _ = write!(line, "{text:>10}");
            }
            let text = if head == 0 { "operation" } else { "longest" };
            let _ = write!(line, "{text:>11}");
            report!("{}", line.as_str());
        }
        for (row, name) in ROWS.iter().enumerate() {
            if row == CORE_STOP {
                if !core_stop {
                    continue;
                }
                if !code_in_psram() {
                    report!("  {:<24}does not apply without code in PSRAM", name);
                    continue;
                }
            }
            let mut line = Line::new();
            let _ = write!(line, "  {:<24}", name);
            for figure in figures {
                let text: &str = match self.worst[row][*figure as usize] {
                    Some(value) => {
                        let _ = write!(line, "{value:>10}");
                        continue;
                    }
                    // The other core is the core that the row stops.
                    None if row == CORE_STOP
                        && *figure as usize >= Figure::InterruptOtherCore as usize =>
                    {
                        "stopped"
                    }
                    None => "none",
                };
                let _ = write!(line, "{text:>10}");
            }
            if row != CORE_STOP {
                let _ = write!(line, "{:>11}", self.longest[row]);
            }
            report!("{}", line.as_str());
        }
    }
}

/// Runs `operation` `times` times in the measured time, after `prepare` outside it, and prints the
/// figures. Returns the worst cases, and the longest and shortest operation in microseconds.
fn measure(
    part: &str,
    name: &str,
    times: u32,
    mut prepare: impl FnMut(),
    mut operation: impl FnMut(),
) -> ([Option<u32>; FIGURES], u32, u32) {
    for figure in 0..FIGURES {
        WORST[figure].store(0, Ordering::Relaxed);
        SAMPLES[figure].store(0, Ordering::Relaxed);
    }
    let mut longest = Duration::ZERO;
    let mut shortest = Duration::MAX;
    for _ in 0..times {
        prepare();
        WINDOW_START.store(now_us(), Ordering::Release);
        WINDOW_OPEN.store(true, Ordering::Release);
        let started = Instant::now();
        operation();
        let took = Instant::now() - started;
        longest = longest.max(took);
        shortest = shortest.min(took);
        WINDOW_END.store(now_us(), Ordering::Release);
        WINDOW_OPEN.store(false, Ordering::Release);
        busy_wait(SETTLE);
    }
    report!("{part} {name}: longest {} µs", longest.as_micros());
    let mut worst = [None; FIGURES];
    for figure in 0..FIGURES {
        let samples = SAMPLES[figure].load(Ordering::Relaxed);
        if samples != 0 {
            let value = WORST[figure].load(Ordering::Relaxed);
            report!("  {}: {} µs, {} samples", NAMES[figure], value, samples);
            worst[figure] = Some(value);
        }
    }
    (
        worst,
        longest.as_micros() as u32,
        shortest.as_micros() as u32,
    )
}

fn busy_wait(duration: Duration) {
    let end = Instant::now() + duration;
    while Instant::now() < end {}
}

/// Writes a word to each sector of `range`, so that an erase of the range has data to erase.
fn fill_sectors(flash: &mut Flash<'static, Blocking>, range: core::ops::Range<u32>) {
    for sector in range.step_by(SECTOR as usize) {
        // SAFETY: the range is in the booted partition, after the image.
        unsafe { flash.write(sector, &[0x5AA5_C33C]).unwrap() };
    }
}

fn assert_erased(flash: &mut Flash<'static, Blocking>, range: core::ops::Range<u32>) {
    for sector in range.step_by(SECTOR as usize) {
        let mut word = [0u32];
        flash.read(sector, &mut word).unwrap();
        assert_eq!(word[0], u32::MAX, "the erase left data at {sector:#x}");
    }
}

/// Measures during each operation. `block` is a free 64 KiB block of flash.
fn measure_operations(
    part: &str,
    flash: &mut Flash<'static, Blocking>,
    block: u32,
    buffer: &mut [u32; READ_WORDS],
    summary: &mut Summary,
) -> u32 {
    let data = [0x5AA5_C33Cu32; (SECTOR / 4) as usize];
    let flash = RefCell::new(flash);

    let (worst, longest, _) = measure(
        part,
        ROWS[0],
        5,
        || {},
        || busy_wait(Duration::from_millis(20)),
    );
    summary.add(0, worst, longest);
    let (worst, longest, _) = measure(
        part,
        ROWS[1],
        20,
        || {},
        || flash.borrow_mut().read(block, &mut buffer[..]).unwrap(),
    );
    summary.add(1, worst, longest);
    // SAFETY, for each erase and write: the block is in the booted partition, after the image.
    let (worst, longest, _) = measure(
        part,
        ROWS[2],
        5,
        || unsafe { flash.borrow_mut().erase(block, block + SECTOR).unwrap() },
        || unsafe { flash.borrow_mut().write(block, &data).unwrap() },
    );
    summary.add(2, worst, longest);
    let mut shortest_sector_erase = 0;
    for (row, len, times) in [(3, SECTOR, 5), (4, BLOCK, 2)] {
        let range = block..block + len;
        let (worst, longest, shortest) = measure(
            part,
            ROWS[row],
            times,
            || fill_sectors(&mut flash.borrow_mut(), range.clone()),
            || unsafe { flash.borrow_mut().erase(range.start, range.end).unwrap() },
        );
        summary.add(row, worst, longest);
        assert_erased(&mut flash.borrow_mut(), range);
        if row == 3 {
            shortest_sector_erase = shortest;
        }
    }
    shortest_sector_erase
}

/// What the other core runs in part 1.
#[cfg(multi_core)]
mod other_core {
    use super::*;

    pub const BARE_LOOP: u32 = 0;
    /// The other core writes and erases its sector in a loop.
    pub const ERASE: u32 = 1;

    pub static MODE: AtomicU32 = AtomicU32::new(BARE_LOOP);
    pub static IN_ERASE: AtomicBool = AtomicBool::new(false);
    /// The number of erases that the other core started, and the start time of the last one.
    pub static ERASES: AtomicU32 = AtomicU32::new(0);
    pub static ERASE_START: AtomicU32 = AtomicU32::new(0);
    pub static mut FLASH: Option<(Flash<'static, Blocking>, u32)> = None;

    pub fn run() -> ! {
        let mut last = now_us();
        while MODE.load(Ordering::Acquire) == BARE_LOOP {
            let now = now_us();
            if in_window(now) || in_window(last) {
                WORST[Figure::BareLoopOtherCore as usize]
                    .fetch_max(now.wrapping_sub(last), Ordering::Relaxed);
                SAMPLES[Figure::BareLoopOtherCore as usize].fetch_add(1, Ordering::Relaxed);
            }
            last = now;
        }
        // SAFETY: the first core does not use `FLASH` after it sets `ERASE`.
        let (flash, sector) = unsafe { (*addr_of_mut!(FLASH)).as_mut() }.unwrap();
        loop {
            fill_sectors(flash, *sector..*sector + SECTOR);
            ERASE_START.store(now_us(), Ordering::Relaxed);
            ERASES.fetch_add(1, Ordering::Release);
            IN_ERASE.store(true, Ordering::Release);
            // SAFETY: the sector is in the booted partition, after the image.
            unsafe { flash.erase(*sector, *sector + SECTOR).unwrap() };
            IN_ERASE.store(false, Ordering::Release);
        }
    }
}

#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_log::init_uart!(peripherals);
    report!(
        "flash-operation-latency: code in PSRAM {}",
        if code_in_psram() { "yes" } else { "no" }
    );

    let mut flash = Flash::new(peripherals.FLASH, FlashConfig::default()).unwrap();
    let block = scratch_blocks(&mut flash, 2);

    static BUFFER: ConstStaticCell<[u32; READ_WORDS]> = ConstStaticCell::new([0; READ_WORDS]);
    let buffer = BUFFER.take();

    let mut part_1 = Summary::new();
    #[cfg(multi_core)]
    let mut part_2 = Summary::new();

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    PROBE_SAME.start(PeriodicTimer::new(systimer.alarm0), probe_same_handler);

    static THREAD_STACK: ConstStaticCell<Stack<4096>> = ConstStaticCell::new(Stack::new());
    CurrentThreadHandle::get().set_priority(1);
    ThreadSpawner::from_static(THREAD_STACK.take())
        .with_priority(2)
        .with_pinned_to(Cpu::ProCpu)
        .spawn(|| measure_thread(Figure::ThreadSameCore));

    #[cfg(not(multi_core))]
    {
        let _ = measure_operations("part 1", &mut flash, block, buffer, &mut part_1);
        part_1.print(
            "worst case in µs",
            &[Figure::InterruptSameCore, Figure::ThreadSameCore],
            false,
        );
    }

    #[cfg(multi_core)]
    {
        // SAFETY: part 2 uses `CPU_CTRL` only after part 1 stopped the other core.
        unsafe {
            CPU_CONTROL = Some(CpuControl::new(peripherals.CPU_CTRL.clone_unchecked()));
        }
        // SAFETY: `other_core` uses this `Flash` only on the other core, and only in part 1.
        let other_flash =
            Flash::new(unsafe { flash_peripheral() }, FlashConfig::default()).unwrap();
        unsafe { other_core::FLASH = Some((other_flash, block + BLOCK)) };
        let guard = start_other_core();
        let sector_erase = measure_operations("part 1", &mut flash, block, buffer, &mut part_1);

        if code_in_psram() {
            measure_core_stop(&mut flash, guard, sector_erase, &mut part_1);
        } else {
            report!(
                "part 1 {}: does not apply without code in PSRAM",
                ROWS[CORE_STOP]
            );
            drop(guard);
        }

        static mut CORE1_RTOS_STACK: CoreStack<8192> = CoreStack::new();
        let alarm = PeriodicTimer::new(systimer.alarm1);
        esp_rtos::start_second_core(
            peripherals.CPU_CTRL,
            unsafe { &mut *addr_of_mut!(CORE1_RTOS_STACK) },
            move || {
                PROBE_OTHER.start(alarm, probe_other_handler);
                measure_thread(Figure::ThreadOtherCore)
            },
        );
        report!("part 2: the other core starts esp-rtos");
        let _ = measure_operations("part 2", &mut flash, block, buffer, &mut part_2);

        part_1.print(
            "part 1: worst case in µs, the other core runs a bare loop",
            &[
                Figure::InterruptSameCore,
                Figure::ThreadSameCore,
                Figure::BareLoopOtherCore,
            ],
            true,
        );
        part_2.print(
            "part 2: worst case in µs, both cores run esp-rtos",
            &[
                Figure::InterruptSameCore,
                Figure::ThreadSameCore,
                Figure::InterruptOtherCore,
                Figure::ThreadOtherCore,
            ],
            false,
        );
    }

    let probes: &[(&str, &Probe)] = &[
        ("same core", &PROBE_SAME),
        #[cfg(multi_core)]
        ("other core", &PROBE_OTHER),
    ];
    for (name, probe) in probes {
        let early = probe.early.load(Ordering::Relaxed);
        let off_grid = probe.off_grid.load(Ordering::Relaxed);
        if early != 0 || off_grid != 0 {
            report!(
                "the interrupt figures are not valid: the {} probe saw {} entries before the due \
                 time and {} alarms off the grid",
                name,
                early,
                off_grid
            );
        }
        let lost = probe.lost.load(Ordering::Relaxed);
        if lost != 0 {
            report!(
                "the {} probe lost {} alarms that fired before the clear of its interrupt",
                name,
                lost
            );
        }
    }
    report!("DONE");
    loop {}
}

/// Returns a second handle to the flash peripheral.
///
/// # Safety
///
/// The two `Flash` values must not run operations at the same time.
#[cfg(multi_core)]
unsafe fn flash_peripheral() -> esp_hal::peripherals::FLASH<'static> {
    unsafe { esp_hal::peripherals::FLASH::steal() }
}

/// The `CpuControl` of part 1. Only the first core's main thread uses it.
#[cfg(multi_core)]
static mut CPU_CONTROL: Option<CpuControl<'static>> = None;

/// Starts the other core in [`other_core::run`]. The guard of the last start must be dropped
/// before the call.
#[cfg(multi_core)]
fn start_other_core() -> esp_hal::system::AppCoreGuard<'static> {
    static mut CORE1_STACK: CoreStack<8192> = CoreStack::new();
    // SAFETY: no guard of an earlier start is alive, so nothing else borrows `CPU_CONTROL`, and
    // the other core is stopped, so it does not use the stack.
    let cpu_control = unsafe { (*addr_of_mut!(CPU_CONTROL)).as_mut() }.unwrap();
    cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(CORE1_STACK) }, || {
            other_core::run()
        })
        .unwrap()
}

/// Measures the first core during [`STOPS`] stops of the other core, each [`STOP_AFTER`] after an
/// erase starts, and checks that each stop waited for the end of the erase. `sector_erase` is the
/// shortest sector erase that the first core measured, in microseconds.
#[cfg(multi_core)]
fn measure_core_stop(
    flash: &mut Flash<'static, Blocking>,
    guard: esp_hal::system::AppCoreGuard<'static>,
    sector_erase: u32,
    summary: &mut Summary,
) {
    const STOPS: u32 = 5;
    const STOP_AFTER: Duration = Duration::from_millis(2);

    // SAFETY: the other core reads `FLASH`; this core reads only the sector number.
    let sector = unsafe { (*addr_of_mut!(other_core::FLASH)).as_ref() }
        .unwrap()
        .1;
    other_core::MODE.store(other_core::ERASE, Ordering::Release);
    let guard = RefCell::new(Some(guard));
    let erase = core::cell::Cell::new(None);
    let took = core::cell::Cell::new(0u32);
    let flash = RefCell::new(flash);
    // A stop that does not wait returns within microseconds.
    let floor = (sector_erase - STOP_AFTER.as_micros() as u32) / 2;
    let check = |erase: Option<u32>| {
        if let Some(erase) = erase {
            assert!(
                took.get() >= floor,
                "the stop took {} µs, less than {} µs, so it did not wait for the erase",
                took.get(),
                floor
            );
            assert!(
                !esp_hal::system::is_running(Cpu::AppCpu),
                "the other core runs after the stop"
            );
            assert_eq!(
                other_core::ERASES.load(Ordering::Acquire),
                erase,
                "the other core started an erase after the stop"
            );
            let mut word = [0u32];
            flash.borrow_mut().read(sector, &mut word).unwrap();
            assert_eq!(
                word[0],
                u32::MAX,
                "the stop did not wait for the end of the erase"
            );
            // The other core stopped inside its erase, before it cleared the flag.
            other_core::IN_ERASE.store(false, Ordering::Release);
        }
    };
    let (worst, _, _) = measure(
        "part 1",
        ROWS[CORE_STOP],
        STOPS,
        || {
            check(erase.get());
            if guard.borrow().is_none() {
                *guard.borrow_mut() = Some(start_other_core());
            }
            // An erase that started at least `STOP_AFTER` ago, and is still in progress.
            loop {
                // The number before and after the time and the flag, so that both are of that
                // erase.
                let number = other_core::ERASES.load(Ordering::Acquire);
                let started = other_core::ERASE_START.load(Ordering::Relaxed);
                if other_core::IN_ERASE.load(Ordering::Acquire)
                    && other_core::ERASES.load(Ordering::Acquire) == number
                    && Some(number) != erase.get()
                    && now_us().wrapping_sub(started) >= STOP_AFTER.as_micros() as u32
                {
                    erase.set(Some(number));
                    break;
                }
            }
        },
        || {
            let started = Instant::now();
            drop(guard.borrow_mut().take());
            took.set((Instant::now() - started).as_micros() as u32);
        },
    );
    check(erase.get());
    report!("  each of the {STOPS} stops took at least {floor} µs and left its sector erased");
    summary.add(CORE_STOP, worst, 0);
}
