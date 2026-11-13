//! Workaround for the cache live lock of ESP32 v3.0 and v3.1 ([errata WDT-3.15]).
//!
//! Three external memory requests of the two cores to the same cache set can stop both cores in
//! their memory accesses. An interrupt releases the cores. The stage 0 timeout of the TIMG1
//! watchdog interrupts both cores at priority 4, above the level that critical sections mask. The
//! handler then lets the cores finish their accesses one after the other.
//!
//! [errata WDT-3.15]: https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/watchdog-issue-caused-by-live-lock.html

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::{
    efuse::ChipRevision,
    interrupt::DISABLED_CPU_INTERRUPT,
    peripherals::{DPORT, Interrupt, TIMG1},
    system::{Cpu, Peripheral, PeripheralClockControl},
    time::Duration,
    timer::timg::{MwdtStage, MwdtStageAction, Wdt},
};

/// The level-triggered CPU interrupt with priority 4.
const CPU_INTERRUPT: u32 = 24;

const LEVEL_4_CPU_INTERRUPTS: u32 = (1 << 24) | (1 << 25) | (1 << 28) | (1 << 30);

const PERIOD: Duration = Duration::from_millis(20);

/// The time for one core to finish its cache access. ESP-IDF derives it from the flash and PSRAM
/// clocks. 9 µs is the longest value for all of them.
const ACCESS_US: u32 = 9;

/// The longest time that a core waits for the other core in the handler. A stalled core or a core
/// with priority 4 masked does not enter the handler.
const JOIN_US: u32 = 50;

const WDT_WKEY: u32 = 0x50D8_3AA1;

static PSRAM_MAPPED: AtomicBool = AtomicBool::new(false);
static APP_CORE_STARTED: AtomicBool = AtomicBool::new(false);
static STARTED: AtomicBool = AtomicBool::new(false);
static CYCLES_PER_US: AtomicU32 = AtomicU32::new(0);

/// Whether each core is in the handler.
static IN_HANDLER: [AtomicBool; 2] = [const { AtomicBool::new(false) }; 2];

/// Set while the PRO CPU keeps the APP CPU in the handler, and finishes its own access.
static HOLD_APP: AtomicBool = AtomicBool::new(false);

/// Enables the CPU interrupt of the workaround on the current core.
pub(crate) fn enable_cpu_interrupt() {
    unsafe { xtensa_lx::interrupt::enable_mask(1 << CPU_INTERRUPT) };
}

/// Starts the workaround if the APP CPU runs.
pub(crate) fn psram_mapped() {
    PSRAM_MAPPED.store(true, Ordering::Release);
    try_start();
}

/// Starts the workaround if PSRAM is mapped.
pub(crate) fn app_core_started() {
    APP_CORE_STARTED.store(true, Ordering::Release);
    try_start();
}

fn try_start() {
    if !PSRAM_MAPPED.load(Ordering::Acquire)
        || !APP_CORE_STARTED.load(Ordering::Acquire)
        || crate::efuse::chip_revision() < ChipRevision::from_combined(300)
        || STARTED.swap(true, Ordering::AcqRel)
    {
        return;
    }

    CYCLES_PER_US.store(crate::clock::cpu_clock().as_mhz(), Ordering::Relaxed);

    // A `TimerGroup` resets TIMG1 when no other user keeps its clock enabled.
    if PeripheralClockControl::enable(Peripheral::Timg1) {
        PeripheralClockControl::reset(Peripheral::Timg1);
    }

    let mut wdt = Wdt::<TIMG1<'static>>::new();
    wdt.set_timeout(MwdtStage::Stage0, PERIOD);
    wdt.set_stage_action(MwdtStage::Stage0, MwdtStageAction::Interrupt);

    let regs = TIMG1::regs();
    regs.wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });
    regs.wdtconfig0().modify(|_, w| unsafe {
        w.wdt_flashboot_mod_en().clear_bit();
        w.wdt_stg1().bits(MwdtStageAction::Off as u8);
        w.wdt_stg2().bits(MwdtStageAction::Off as u8);
        w.wdt_stg3().bits(MwdtStageAction::Off as u8);
        w.wdt_edge_int_en().clear_bit();
        w.wdt_level_int_en().set_bit();
        w.wdt_en().set_bit()
    });
    regs.wdtfeed().write(|w| unsafe { w.bits(1) });
    regs.wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) });
    regs.int_clr().write(|w| w.wdt().clear_bit_by_one());
    regs.int_ena().modify(|_, w| w.wdt().set_bit());

    map(Cpu::ProCpu, CPU_INTERRUPT);
    map(Cpu::AppCpu, CPU_INTERRUPT);
}

/// Stops the watchdog for a light sleep. The parked core waits in internal RAM, and cannot take
/// part in a live lock.
#[crate::ram]
pub(crate) fn pause() {
    if STARTED.load(Ordering::Acquire) {
        set_watchdog_enabled(false);
    }
}

/// Starts the watchdog again after [`pause`].
#[crate::ram]
pub(crate) fn resume() {
    if STARTED.load(Ordering::Acquire) {
        set_watchdog_enabled(true);
    }
}

#[inline(always)]
fn set_watchdog_enabled(enabled: bool) {
    let regs = TIMG1::regs();
    regs.wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });
    regs.wdtfeed().write(|w| unsafe { w.bits(1) });
    regs.wdtconfig0().modify(|_, w| w.wdt_en().bit(enabled));
    regs.wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) });
}

/// Handles the interrupt of the workaround. Returns `false` if another priority 4 interrupt is
/// pending.
///
/// The handler runs while a flash operation disables the cache. It must not access flash or PSRAM.
#[crate::ram]
pub(crate) fn handle_interrupt() -> bool {
    let pending: u32;
    unsafe { core::arch::asm!("rsr.interrupt {0}", out(reg) pending, options(nostack)) };
    if pending & (1 << CPU_INTERRUPT) == 0 {
        // The other core can clear the interrupt after this core takes it.
        return pending & LEVEL_4_CPU_INTERRUPTS == 0;
    }

    // ESP-IDF releases the cores in three stages. First, both cores wait in the handler until their
    // write buffers are empty. Then the APP CPU finishes its access while the PRO CPU waits. Last,
    // the PRO CPU finishes its access while the APP CPU waits.
    //
    // Only the PRO CPU changes the interrupt map of the APP CPU, and it restores the map before it
    // returns.
    match Cpu::current() {
        Cpu::ProCpu => {
            IN_HANDLER[0].store(true, Ordering::Release);
            if wait_until(JOIN_US, || IN_HANDLER[1].load(Ordering::Acquire)) {
                // The interrupt stays pending until the PRO CPU clears it. Without the map
                // change, the APP CPU takes the interrupt again when it returns.
                map(Cpu::AppCpu, DISABLED_CPU_INTERRUPT);
                wait_until(JOIN_US, || !IN_HANDLER[1].load(Ordering::Acquire));
                wait_until(ACCESS_US, || false);

                HOLD_APP.store(true, Ordering::Release);
                map(Cpu::AppCpu, CPU_INTERRUPT);
                wait_until(JOIN_US, || IN_HANDLER[1].load(Ordering::Acquire));
            }
            restart_watchdog();
            map(Cpu::AppCpu, CPU_INTERRUPT);
            HOLD_APP.store(false, Ordering::Release);
            IN_HANDLER[0].store(false, Ordering::Release);
        }
        Cpu::AppCpu => {
            IN_HANDLER[1].store(true, Ordering::Release);
            if HOLD_APP.load(Ordering::Acquire)
                || wait_until(JOIN_US, || IN_HANDLER[0].load(Ordering::Acquire))
            {
                wait_until(ACCESS_US, || false);
            } else {
                restart_watchdog();
            }
            IN_HANDLER[1].store(false, Ordering::Release);
        }
    }
    true
}

/// Waits until `done` returns `true`, for at most `limit_us`. Returns whether `done` returned
/// `true`.
#[inline(always)]
fn wait_until(limit_us: u32, done: impl Fn() -> bool) -> bool {
    let limit = limit_us * CYCLES_PER_US.load(Ordering::Relaxed);
    let start = cycle_count();
    loop {
        if done() {
            return true;
        }
        if cycle_count().wrapping_sub(start) >= limit {
            return false;
        }
    }
}

#[inline(always)]
fn cycle_count() -> u32 {
    let count: u32;
    unsafe { core::arch::asm!("rsr.ccount {0}", out(reg) count, options(nostack)) };
    count
}

#[inline(always)]
fn restart_watchdog() {
    let regs = TIMG1::regs();
    regs.wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(WDT_WKEY) });
    regs.wdtfeed().write(|w| unsafe { w.bits(1) });
    regs.wdtwprotect()
        .write(|w| unsafe { w.wdt_wkey().bits(0) });
    regs.int_clr().write(|w| w.wdt().clear_bit_by_one());
}

#[inline(always)]
fn map(cpu: Cpu, cpu_interrupt: u32) {
    let source = Interrupt::TG1_WDT_LEVEL as usize;
    match cpu {
        Cpu::ProCpu => {
            DPORT::regs()
                .core_0_intr_map(source)
                .write(|w| unsafe { w.map().bits(cpu_interrupt as u8) });
        }
        Cpu::AppCpu => {
            DPORT::regs()
                .core_1_intr_map(source)
                .write(|w| unsafe { w.map().bits(cpu_interrupt as u8) });
        }
    }
}
