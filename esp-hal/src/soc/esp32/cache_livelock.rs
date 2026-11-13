//! Workaround for the cache live lock of ESP32 v3.0 and v3.1 ([errata WDT-3.15]).
//!
//! Three external memory requests of the two cores to the same cache set can stop both cores in
//! their memory accesses. An interrupt releases the cores. The stage 0 timeout of the TIMG1
//! watchdog interrupts both cores at priority 4, above the level that critical sections mask. The
//! handler then lets the cores finish their accesses one after the other.
//!
//! [errata WDT-3.15]: https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/watchdog-issue-caused-by-live-lock.html

use core::{
    cell::UnsafeCell,
    mem::offset_of,
    sync::atomic::{AtomicBool, Ordering},
};

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

/// The other CPU interrupts with priority 4.
const OTHER_LEVEL_4_CPU_INTERRUPTS: [u32; 3] = [25, 28, 30];

const PERIOD: Duration = Duration::from_millis(20);

/// The time for one core to finish its cache access. ESP-IDF derives it from the flash and PSRAM
/// clocks. 9 µs is the longest value for all of them.
const ACCESS_US: u32 = 9;

/// The longest time that a core waits for the other core in the handler. A stalled core or a core
/// with priority 4 masked does not enter the handler.
const JOIN_US: u32 = 50;

const WDT_WKEY: u32 = 0x50D8_3AA1;

/// Set after the APP CPU enabled the CPU interrupt of the workaround.
static APP_CORE_STARTED: AtomicBool = AtomicBool::new(false);
static STARTED: AtomicBool = AtomicBool::new(false);

/// Enables the CPU interrupt of the workaround on the current core.
pub(crate) fn enable_cpu_interrupt() {
    unsafe { xtensa_lx::interrupt::enable_mask(1 << CPU_INTERRUPT) };
}

/// Starts the workaround if the APP CPU runs.
pub(crate) fn psram_mapped() {
    try_start();
}

/// Starts the workaround if PSRAM is mapped.
pub(crate) fn app_core_started() {
    APP_CORE_STARTED.store(true, Ordering::Release);
    try_start();
}

fn try_start() {
    if crate::psram::psram_range().is_empty()
        || !APP_CORE_STARTED.load(Ordering::Acquire)
        || crate::efuse::chip_revision() < ChipRevision::from_combined(300)
        || STARTED.swap(true, Ordering::AcqRel)
    {
        return;
    }

    init_handler_state(crate::clock::cpu_clock().as_mhz());

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

/// The data that the interrupt handler reads and writes.
#[repr(C)]
struct State {
    wdt_wprotect: *mut u32,
    wdt_feed: *mut u32,
    int_clr: *mut u32,
    app_map: *mut u32,
    wdt_wkey: u32,
    wdt_int: u32,
    mapped: u32,
    unmapped: u32,
    access_cycles: u32,
    join_cycles: u32,
    in_handler: [u32; 2],
    hold_app: u32,
}

#[repr(transparent)]
struct HandlerData<T>(UnsafeCell<T>);

// SAFETY: Rust code writes `STATE` before the handler runs. Only the handler accesses `SAVED`.
unsafe impl<T> Sync for HandlerData<T> {}

static STATE: HandlerData<State> = HandlerData(UnsafeCell::new(State {
    wdt_wprotect: core::ptr::null_mut(),
    wdt_feed: core::ptr::null_mut(),
    int_clr: core::ptr::null_mut(),
    app_map: core::ptr::null_mut(),
    wdt_wkey: WDT_WKEY,
    wdt_int: 1 << 2,
    mapped: CPU_INTERRUPT,
    unmapped: DISABLED_CPU_INTERRUPT,
    access_cycles: 0,
    join_cycles: 0,
    in_handler: [0; 2],
    hold_app: 0,
}));

/// The registers that the handler saves, for each core.
static SAVED: HandlerData<[[u32; 8]; 2]> = HandlerData(UnsafeCell::new([[0; 8]; 2]));

fn init_handler_state(cycles_per_us: u32) {
    let regs = TIMG1::regs();
    let source = Interrupt::TG1_WDT_LEVEL as usize;
    // SAFETY: CPU interrupt 24 is not mapped yet, so the handler does not run.
    let state = unsafe { &mut *STATE.0.get() };
    state.wdt_wprotect = regs.wdtwprotect().as_ptr();
    state.wdt_feed = regs.wdtfeed().as_ptr();
    state.int_clr = regs.int_clr().as_ptr();
    state.app_map = DPORT::regs().core_1_intr_map(source).as_ptr();
    state.access_cycles = ACCESS_US * cycles_per_us;
    state.join_cycles = JOIN_US * cycles_per_us;
}

// The handler interrupts code that runs with PS.EXCM set, for example the window overflow and
// underflow handlers. It must not cause an exception: it uses no windowed calls, no stack and no
// memory other than `STATE` and `SAVED`. It runs while a flash operation disables the cache, so it
// must be in internal RAM.
//
// ESP-IDF releases the cores in three stages. First, both cores wait in the handler until their
// write buffers are empty. Then the APP CPU finishes its access while the PRO CPU waits. Last, the
// PRO CPU finishes its access while the APP CPU waits. Only the PRO CPU changes the interrupt map
// of the APP CPU, and it restores the map before it returns.
//
// The vector saves a0 in EXCSAVE4. EPS4 restores PS at `rfi 4`.
core::arch::global_asm!(
    "
    .section .rwtext,\"ax\",@progbits
    .literal sym_livelock_state, {state}
    .literal sym_livelock_saved, {saved}

    .global __naked_level_4_interrupt
    .p2align 2
    .type __naked_level_4_interrupt,@function
__naked_level_4_interrupt:
    rsr.interrupt a0
    bbsi    a0, {cpu_interrupt}, .Llivelock
    bbsi    a0, {other0}, .Lother_level_4
    bbsi    a0, {other1}, .Lother_level_4
    bbsi    a0, {other2}, .Lother_level_4
    // The other core can clear the interrupt after this core takes it.
    rsr     a0, EXCSAVE4
    rfi     4
.Lother_level_4:
    call0   __default_naked_level_4_interrupt

.Llivelock:
    rsil    a0, 5
    // Level 5 is masked, so no other code uses EXCSAVE5.
    wsr     a2, EXCSAVE5
    rsr.prid a2
    extui   a2, a2, 13, 1
    slli    a2, a2, 5
    l32r    a0, sym_livelock_saved
    add     a0, a0, a2
    rsr     a2, EXCSAVE5
    s32i    a2, a0, 0
    s32i    a3, a0, 4
    s32i    a4, a0, 8
    s32i    a5, a0, 12
    s32i    a6, a0, 16

    rsr.prid a5
    extui   a5, a5, 13, 1
    l32r    a2, sym_livelock_state
    bnez    a5, .Lapp

    movi    a4, 1
    s32i    a4, a2, {in_pro}
    memw
    rsr.ccount a3
    l32i    a6, a2, {join}
.Lpro_join:
    l32i    a4, a2, {in_app}
    bnez    a4, .Lpro_joined
    rsr.ccount a4
    sub     a4, a4, a3
    bltu    a4, a6, .Lpro_join
    j       .Lpro_restart
.Lpro_joined:
    // The interrupt stays pending until the PRO CPU clears it. Without the map change, the APP CPU
    // takes the interrupt again when it returns.
    l32i    a3, a2, {app_map}
    l32i    a4, a2, {unmapped}
    s32i    a4, a3, 0
    memw
    rsr.ccount a3
.Lpro_app_leaves:
    l32i    a4, a2, {in_app}
    beqz    a4, .Lpro_app_left
    rsr.ccount a4
    sub     a4, a4, a3
    bltu    a4, a6, .Lpro_app_leaves
.Lpro_app_left:
    rsr.ccount a3
    l32i    a6, a2, {access}
.Lpro_access:
    rsr.ccount a4
    sub     a4, a4, a3
    bltu    a4, a6, .Lpro_access
    movi    a4, 1
    s32i    a4, a2, {hold_app}
    memw
    l32i    a3, a2, {app_map}
    l32i    a4, a2, {mapped}
    s32i    a4, a3, 0
    memw
    rsr.ccount a3
    l32i    a6, a2, {join}
.Lpro_rejoin:
    l32i    a4, a2, {in_app}
    bnez    a4, .Lpro_restart
    rsr.ccount a4
    sub     a4, a4, a3
    bltu    a4, a6, .Lpro_rejoin
.Lpro_restart:
    call0   .Lrestart_watchdog
    l32i    a3, a2, {app_map}
    l32i    a4, a2, {mapped}
    s32i    a4, a3, 0
    memw
    movi    a4, 0
    s32i    a4, a2, {hold_app}
    memw
    s32i    a4, a2, {in_pro}
    memw
    j       .Lreturn

.Lapp:
    movi    a4, 1
    s32i    a4, a2, {in_app}
    memw
    l32i    a4, a2, {hold_app}
    bnez    a4, .Lapp_access
    rsr.ccount a3
    l32i    a6, a2, {join}
.Lapp_join:
    l32i    a4, a2, {in_pro}
    bnez    a4, .Lapp_access
    rsr.ccount a4
    sub     a4, a4, a3
    bltu    a4, a6, .Lapp_join
    call0   .Lrestart_watchdog
    j       .Lapp_leave
.Lapp_access:
    rsr.ccount a3
    l32i    a6, a2, {access}
.Lapp_wait:
    rsr.ccount a4
    sub     a4, a4, a3
    bltu    a4, a6, .Lapp_wait
.Lapp_leave:
    movi    a4, 0
    s32i    a4, a2, {in_app}
    memw

.Lreturn:
    slli    a5, a5, 5
    l32r    a0, sym_livelock_saved
    add     a0, a0, a5
    l32i    a2, a0, 0
    l32i    a3, a0, 4
    l32i    a4, a0, 8
    l32i    a5, a0, 12
    l32i    a6, a0, 16
    rsr     a0, EXCSAVE4
    rfi     4

    // Feeds the watchdog and clears its interrupt. Reads a2, changes a0, a3 and a4.
    .p2align 2
.Lrestart_watchdog:
    l32i    a3, a2, {wdt_wprotect}
    l32i    a4, a2, {wdt_wkey}
    s32i    a4, a3, 0
    memw
    l32i    a3, a2, {wdt_feed}
    movi    a4, 1
    s32i    a4, a3, 0
    memw
    l32i    a3, a2, {wdt_wprotect}
    movi    a4, 0
    s32i    a4, a3, 0
    memw
    l32i    a3, a2, {int_clr}
    l32i    a4, a2, {wdt_int}
    s32i    a4, a3, 0
    memw
    ret
    ",
    state = sym STATE,
    saved = sym SAVED,
    cpu_interrupt = const CPU_INTERRUPT,
    other0 = const OTHER_LEVEL_4_CPU_INTERRUPTS[0],
    other1 = const OTHER_LEVEL_4_CPU_INTERRUPTS[1],
    other2 = const OTHER_LEVEL_4_CPU_INTERRUPTS[2],
    wdt_wprotect = const offset_of!(State, wdt_wprotect),
    wdt_feed = const offset_of!(State, wdt_feed),
    int_clr = const offset_of!(State, int_clr),
    app_map = const offset_of!(State, app_map),
    wdt_wkey = const offset_of!(State, wdt_wkey),
    wdt_int = const offset_of!(State, wdt_int),
    mapped = const offset_of!(State, mapped),
    unmapped = const offset_of!(State, unmapped),
    access = const offset_of!(State, access_cycles),
    join = const offset_of!(State, join_cycles),
    in_pro = const offset_of!(State, in_handler),
    in_app = const offset_of!(State, in_handler) + 4,
    hold_app = const offset_of!(State, hold_app),
);
