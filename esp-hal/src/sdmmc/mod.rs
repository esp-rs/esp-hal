//! # Secure Digital / MultiMedia Card host (SDMMC / SDIO)
//!
//! ## Overview
//!
//! Driver for the SDMMC/SDIO host controller (`SDHOST`). The controller exposes
//! up to two independent card slots that share a single transfer engine.

use core::{
    cell::UnsafeCell,
    future::poll_fn,
    marker::PhantomData,
    sync::atomic::Ordering,
    task::Poll,
};

use embassy_futures::yield_now;
use embassy_sync::{mutex::MutexGuard, waitqueue::WakerRegistration};
use esp_sync::{NonReentrantMutex, RawMutex};
use portable_atomic::AtomicBool;
use procmacros::{BuilderLite, handler};
use sdio::{self as _, MmcError};

#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
use crate::dma::aligned::InternalMemory;
#[cfg(sdmmc_has_gpio_matrix)]
use crate::gpio::{PinGuard, Pull, interconnect::PeripheralInput};
use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    dma::aligned::DmaAlignedMut,
    gpio::{
        InputSignal,
        OutputConfig,
        OutputSignal,
        interconnect::{self, PeripheralOutput},
    },
    peripherals::{Interrupt, SDHOST},
    private::DropGuard,
    soc::is_valid_ram_address,
    system::{Peripheral, PeripheralGuard},
};

/// Selects one of the controller's card slots.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SlotId {
    /// Slot 0.
    _0,
    /// Slot 1.
    _1,
}

impl SlotId {
    /// Zero-based slot index.
    #[cfg_attr(not(esp32s3), allow(dead_code))]
    fn index(self) -> u8 {
        match self {
            SlotId::_0 => 0,
            SlotId::_1 => 1,
        }
    }
}

/// Card clock source feeding the controller's divider.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ClockSource {
    /// 160 MHz PLL.
    Pll160m,
    /// Crystal oscillator.
    #[cfg(not(esp32p4))]
    Xtal,
}

/// Clock input sampling phase used for high-speed tuning.
#[cfg(esp32s3)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DelayPhase {
    /// 0°.
    #[default]
    _0,
    /// 90°.
    _1,
    /// 180°.
    _2,
    /// 270°.
    _3,
}

/// Card data bus width.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BusWidth {
    /// 1-bit bus (DAT0 only).
    #[default]
    Bit1,
    /// 4-bit bus (DAT0–DAT3).
    Bit4,
}

/// Controller-wide (engine) configuration.
///
/// These settings drive the shared module clock and therefore apply to the
/// whole controller, not an individual slot.
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub struct Config {
    clock_source: ClockSource,
    module_div: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            clock_source: ClockSource::Pll160m,
            module_div: 2,
        }
    }
}

impl Config {
    /// Sets the card clock source.
    pub fn with_clock_source(mut self, source: ClockSource) -> Self {
        self.clock_source = source;
        self
    }

    /// Sets the module-clock divider (valid range `2..=16`).
    pub fn with_module_divider(mut self, div: u8) -> Self {
        self.module_div = div;
        self
    }

    /// Validates field ranges.
    fn validate(&self) -> Result<(), ConfigError> {
        if !(2..=16).contains(&self.module_div) {
            return Err(ConfigError::InvalidModuleDivider);
        }
        Ok(())
    }
}

/// Per-slot configuration.
#[derive(Clone, Copy, Debug, BuilderLite)]
#[non_exhaustive]
pub struct SlotConfig {
    /// Input sampling delay phase used for high-speed tuning (esp32s3 only).
    #[cfg(esp32s3)]
    input_delay_phase: DelayPhase,

    /// Write-protect signal polarity (active-high or active-low).
    wp_active_high: bool,
}

impl Default for SlotConfig {
    fn default() -> Self {
        Self {
            #[cfg(esp32s3)]
            input_delay_phase: DelayPhase::_0,
            wp_active_high: false,
        }
    }
}

/// Length of the response a command expects.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ResponseLen {
    /// No response.
    None,
    /// Short 48-bit response (`resp[0]`).
    Short,
    /// Long 136-bit response (`resp[0..4]`).
    Long,
}

impl ResponseLen {
    /// CMD0 (`GO_IDLE_STATE`) never returns a response; the `sdio` crate
    /// declares it as `R1`, so force no-response by command index.
    #[cfg_attr(not(any(esp32, esp32s3)), allow(dead_code))]
    fn for_index(self, index: u8) -> Self {
        if index == 0 { ResponseLen::None } else { self }
    }
}

/// Per-command engine flags.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct CommandFlags {
    /// Wait for the data line to be free before issuing.
    pub wait_complete: bool,
    /// Stop/abort command (CMD12, CMD52 abort).
    pub stop_abort: bool,
    /// Poll DAT0 until the card releases busy (R1b).
    pub busy: bool,
}

/// Error returned by host operations.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[non_exhaustive]
pub enum Error {
    /// A hardware operation did not complete in time.
    Timeout,
    /// Response CRC check failed.
    ResponseCrc,
    /// Data CRC or end-bit error.
    DataCrc,
    /// Card did not respond to the command.
    ResponseTimeout,
    /// Data transfer timed out.
    DataTimeout,
    /// FIFO under- or overrun during a transfer.
    FifoOverrun,
    /// Data start-bit error.
    StartBitError,
    /// Command could not be loaded (hardware locked).
    HardwareLocked,
    /// Controller flagged a response error.
    ResponseError,
    /// IDMAC transfer error.
    DmaError,
    /// No card present in the slot.
    NoCard,
    /// Buffer lies in a region the IDMAC cannot reach.
    BufferNotDmaCapable,
    /// Operation not supported.
    Unsupported,
}

/// Error returned when applying a [`Config`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[non_exhaustive]
pub enum ConfigError {
    /// Module-clock divider is outside `2..=16`.
    InvalidModuleDivider,
    /// The slot is already in use.
    SlotInUse,
    /// The clock or command pin was not connected.
    MissingClkOrCmd,
    /// Data line 0 was not connected.
    NoData0,
}

impl From<Error> for MmcError {
    fn from(error: Error) -> Self {
        #[cfg(feature = "log-04")] // no defmt in sdio yet
        warn!("{:?}", error);
        match error {
            Error::ResponseTimeout | Error::DataTimeout | Error::Timeout | Error::NoCard => {
                MmcError::Timeout
            }
            Error::ResponseCrc | Error::DataCrc | Error::StartBitError => MmcError::Crc,
            Error::FifoOverrun
            | Error::DmaError
            | Error::ResponseError
            | Error::BufferNotDmaCapable => MmcError::Io,
            Error::HardwareLocked => MmcError::Other,
            Error::Unsupported => MmcError::Unsupported,
        }
    }
}

/// Spin-poll bound for self-clearing hardware bits.
const POLL_LIMIT: u32 = 1_000_000;

// `rintsts` event bits (see `sdmmc_ll.h` `SDMMC_LL_EVENT_*`).
const EVT_RESP_ERR: u32 = 1 << 1;
const EVT_CMD_DONE: u32 = 1 << 2;
const EVT_DATA_OVER: u32 = 1 << 3;
const EVT_RCRC: u32 = 1 << 6;
const EVT_DCRC: u32 = 1 << 7;
const EVT_RTO: u32 = 1 << 8;
const EVT_DTO: u32 = 1 << 9;
const EVT_HTO: u32 = 1 << 10;
const EVT_FRUN: u32 = 1 << 11;
const EVT_HLE: u32 = 1 << 12;
const EVT_SBE: u32 = 1 << 13;
const EVT_ACD: u32 = 1 << 14;
const EVT_EBE: u32 = 1 << 15;

// `ctrl` IDMAC-enable bits not modeled by the PAC (raw writes only).
const CTRL_DMA_ENABLE: u32 = 1 << 5;
const CTRL_USE_INTERNAL_DMA: u32 = 1 << 25;

/// IDMAC descriptor count in the driver-owned ring.
const RING_LEN: usize = 4;
/// Maximum bytes per descriptor (`SDMMC_DMA_MAX_BUF_LEN`).
const DMA_MAX_BUF_LEN: usize = 4096;

// `Desc.flags` bits (see `sdmmc_desc_t`).
const DESC_LAST: u32 = 1 << 2;
const DESC_FIRST: u32 = 1 << 3;
const DESC_CHAINED: u32 = 1 << 4;
const DESC_OWN: u32 = 1 << 31;

/// One 16-byte IDMAC linked-list descriptor.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct Desc {
    flags: u32,
    sizes: u32,
    buf1: u32,
    next: u32,
}

impl Desc {
    const ZERO: Self = Desc {
        flags: 0,
        sizes: 0,
        buf1: 0,
        next: 0,
    };
}

/// Tracks progress while filling the descriptor ring.
#[derive(Clone, Copy)]
struct Transfer {
    ptr: u32,
    remaining: usize,
    next_desc: usize,
}

/// Driver-owned descriptor ring in internal DMA-capable RAM.
#[cfg_attr(soc_internal_memory_cached, repr(align(64)))]
struct DescRing(UnsafeCell<[Desc; RING_LEN]>);
// Access is serialized by the single controller / per-slot mutex.
unsafe impl Sync for DescRing {}

fn ring() -> &'static mut [Desc; RING_LEN] {
    unsafe { &mut *STATE.desc_ring.0.get() }
}

// On SoCs with a data cache the IDMAC buffer must be cache-line aligned in
// both base address and length, otherwise a read invalidate would discard
// neighbouring data and a write would miss CPU-cached bytes. Control
// transfers (SCR = 8 B, SSR / SWITCH = 64 B) and single-block I/O (512 B)
// come from caller buffers we cannot align (e.g. `BufStream`), so they are
// bounced through an aligned scratch buffer. One block covers every case.
#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
const BOUNCE_LEN: usize = 512;

/// Per-transaction scratch owned by `engine_lock`. On SoCs with a data
/// cache it holds a DMA-aligned bounce buffer for cache-unaligned caller
/// buffers; `InternalMemory` provides the alignment and the lock provides
/// the exclusive access, so no `UnsafeCell` is needed. Elsewhere it is an
/// empty marker carried by the same lock.
struct Bounce {
    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    buf: InternalMemory<[u8; BOUNCE_LEN]>,
}

impl Bounce {
    const INIT: Self = Bounce {
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        buf: InternalMemory::new([0; BOUNCE_LEN]),
    };
}

// `rintsts` card-detect change bit.
const EVT_CD: u32 = 1 << 0;
// `rintsts` SDIO card-interrupt bits (one per slot).
const EVT_IO_SLOT0: u32 = 1 << 16;
const EVT_IO_SLOT1: u32 = 1 << 17;

// `idsts` fatal DMA bits (fatal bus error / descriptor unavailable).
const IDSTS_FBE: u32 = 1 << 2;
const IDSTS_DU: u32 = 1 << 4;

/// Interrupt mask kept armed while idle (card-detect only).
const INTMASK_IDLE: u32 = EVT_CD;
/// Command-phase interrupt mask.
const INTMASK_CMD: u32 = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR | EVT_HLE;
/// Data-phase interrupt mask (command bits plus data events).
const INTMASK_DATA: u32 = INTMASK_CMD
    | EVT_DATA_OVER
    | EVT_DCRC
    | EVT_DTO
    | EVT_HTO
    | EVT_SBE
    | EVT_EBE
    | EVT_FRUN
    | EVT_ACD;

// `idinten` bits: TX/RX done, fatal/unavailable, normal/abnormal summaries.
const IDINTEN_ALL: u32 = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 4) | (1 << 8) | (1 << 9);

/// Per-controller async engine state shared with the interrupt handler.
struct Engine {
    transfer: Option<Transfer>,
    result: Option<Result<(), Error>>,
    expect_data: bool,
    multiblock: bool,
    over_seen: bool,
    acd_seen: bool,
    wait_busy: bool,
    waker: WakerRegistration,
}

impl Engine {
    const IDLE: Self = Engine {
        transfer: None,
        result: None,
        expect_data: false,
        multiblock: false,
        over_seen: false,
        acd_seen: false,
        wait_busy: false,
        waker: WakerRegistration::new(),
    };

    /// Sets the bus width bits for a slot.
    fn set_card_width(&mut self, id: SlotId, width: BusWidth) {
        let slot = id.index();
        SDHOST::regs().ctype().modify(|rd, w| unsafe {
            let mut w4 = rd.card_width4().bits();
            let w8 = rd.card_width8().bits() & !(1 << slot);
            match width {
                BusWidth::Bit4 => w4 |= 1 << slot,
                BusWidth::Bit1 => w4 &= !(1 << slot),
            }
            w.card_width4().bits(w4);
            w.card_width8().bits(w8)
        });
    }

    /// Sets a slot's card clock divider and (re-)enables the card clock.
    fn set_card_clock(&mut self, id: SlotId, card_div: u8) -> Result<(), Error> {
        let slot = id.index();
        let r = SDHOST::regs();

        // Disable card clock before changing the divider.
        r.clkena().modify(|rd, w| unsafe {
            w.cclk_enable().bits(rd.cclk_enable().bits() & !(1 << slot))
        });
        self.apply_clock_update(id)?;

        // Select per-slot divider register and program the divider value.
        r.clksrc().modify(|rd, w| unsafe {
            let mut v = rd.clksrc().bits();
            match id {
                SlotId::_0 => v &= !0b11,
                SlotId::_1 => {
                    v &= !0b1100;
                    v |= 1 << 2;
                }
            }
            w.clksrc().bits(v)
        });
        r.clkdiv().modify(|_, w| unsafe {
            match id {
                SlotId::_0 => w.clk_divider0().bits(card_div),
                SlotId::_1 => w.clk_divider1().bits(card_div),
            }
        });

        // Re-enable card clock with low-power gating, then commit.
        r.clkena().modify(|rd, w| unsafe {
            w.cclk_enable().bits(rd.cclk_enable().bits() | (1 << slot));
            w.lp_enable().bits(rd.lp_enable().bits() | (1 << slot))
        });
        self.apply_clock_update(id)
    }

    /// Commits clock-register changes via an `update_clock_registers_only`
    /// command.
    fn apply_clock_update(&mut self, id: SlotId) -> Result<(), Error> {
        let r = SDHOST::regs();
        r.cmdarg().write(|w| unsafe { w.bits(0) });
        r.cmd().write(|w| unsafe {
            w.update_clock_registers_only().set_bit();
            w.wait_prvdata_complete().set_bit();
            w.card_number().bits(id.index() as u8);
            w.start_cmd().set_bit()
        });
        wait_command_accepted()
    }
}

/// Persistent controller/slot settings consulted on slot selection.
///
/// Kept separate from [`Engine`] (which is reset between transfers) so the
/// engine-wide module clock and per-slot tuning survive across operations.
struct Settings {
    module: Config,
    active_slot: Option<SlotId>,
    #[cfg(esp32s3)]
    slots: [SlotSettings; 2],
}

impl Settings {
    const INIT: Self = Settings {
        module: Config {
            clock_source: ClockSource::Pll160m,
            module_div: 2,
        },
        active_slot: None,
        #[cfg(esp32s3)]
        slots: [SlotSettings::INIT; 2],
    };
}

/// Per-slot settings needed to (re-)apply shared registers on selection.
#[cfg(esp32s3)]
#[derive(Clone, Copy)]
struct SlotSettings {
    hz: u32,
    input_delay_phase: DelayPhase,
}

#[cfg(esp32s3)]
impl SlotSettings {
    const INIT: Self = SlotSettings {
        hz: 0,
        input_delay_phase: DelayPhase::_0,
    };
}

/// Mutable controller state shared with the interrupt handler.
///
/// `engine_lock` serializes whole transactions over the single shared
/// transfer engine so a second slot cannot clobber
/// `blksiz`/`bytcnt`/`dbaddr`/`cmd` mid-transfer. Uncontended (and so
/// near-free) in the common single-slot case.
struct State {
    engine: NonReentrantMutex<Engine>,
    engine_lock: embassy_sync::mutex::Mutex<RawMutex, Bounce>,
    settings: NonReentrantMutex<Settings>,
    desc_ring: DescRing,
}
unsafe impl Sync for State {}

impl State {
    async fn select_slot(&self, slot_id: SlotId) -> MutexGuard<'_, RawMutex, Bounce> {
        let guard = self.engine_lock.lock().await;
        select_slot_sync(slot_id);
        guard
    }
}

static STATE: State = State {
    engine: NonReentrantMutex::new(Engine::IDLE),
    engine_lock: embassy_sync::mutex::Mutex::new(Bounce::INIT),
    settings: NonReentrantMutex::new(Settings::INIT),
    desc_ring: DescRing(UnsafeCell::new([Desc::ZERO; RING_LEN])),
};

/// Applies the active slot's shared-register settings when the active slot
/// changes. Serializes nothing; transaction ordering is handled separately.
fn select_slot_sync(slot_id: SlotId) {
    STATE.settings.with(|s| {
        if s.active_slot != Some(slot_id) {
            configure_for_slot(s, slot_id);
            s.active_slot = Some(slot_id);
        }
    });
}

/// Applies shared registers that depend on which slot is active. Only the
/// high-speed input-delay phase is slot-specific (esp32s3); other chips have
/// nothing to re-apply.
fn configure_for_slot(settings: &Settings, slot_id: SlotId) {
    #[cfg(esp32s3)]
    {
        let sl = &settings.slots[slot_id.index() as usize];
        // Keep the high-speed input-delay gate: only meaningful above SDR25.
        if sl.hz > 25_000_000 {
            set_input_delay_phase(sl.input_delay_phase);
        }
    }
    #[cfg(not(esp32s3))]
    let _ = (settings, slot_id);
}

/// Matrix-routed pin guards owned by the slot for its whole lifetime.
///
/// Held here (rather than leaked) so re-running a `with_*` builder method
/// drops the previous routing. IO_MUX chips drive fixed pads and need none.
#[cfg(sdmmc_has_gpio_matrix)]
struct SlotPins {
    clk: PinGuard,
    cmd: PinGuard,
    data: [PinGuard; 4],
}

#[cfg(sdmmc_has_gpio_matrix)]
impl SlotPins {
    const fn new() -> Self {
        Self {
            clk: PinGuard::new_unconnected(),
            cmd: PinGuard::new_unconnected(),
            data: [const { PinGuard::new_unconnected() }; 4],
        }
    }
}

/// Mutable per-slot runtime state.
struct SlotState {
    io_waker: AtomicWaker,
    cd_waker: AtomicWaker,
    io_armed: AtomicBool,
    #[cfg(sdmmc_has_gpio_matrix)]
    pins: UnsafeCell<SlotPins>,
}

// `pins` is only touched while building a slot (never by the ISR); the slot
// value owns its `State`, mirroring the SPI driver.
#[cfg(sdmmc_has_gpio_matrix)]
unsafe impl Sync for SlotState {}

impl SlotState {
    const fn new() -> Self {
        Self {
            io_waker: AtomicWaker::new(),
            cd_waker: AtomicWaker::new(),
            io_armed: AtomicBool::new(false),
            #[cfg(sdmmc_has_gpio_matrix)]
            pins: UnsafeCell::new(SlotPins::new()),
        }
    }
}

static SLOT_STATE: [SlotState; 2] = [SlotState::new(), SlotState::new()];

/// Mutable runtime state for a slot.
#[cfg(sdmmc_has_gpio_matrix)]
fn slot_state(id: SlotId) -> &'static SlotState {
    &SLOT_STATE[id.index() as usize]
}

/// Exclusive access to a slot's pin guards (builder-time only).
#[cfg(sdmmc_has_gpio_matrix)]
fn slot_pins(id: SlotId) -> &'static mut SlotPins {
    unsafe { &mut *slot_state(id).pins.get() }
}

/// Immutable per-slot routing: SDIO card-interrupt bit plus the input/output
/// signals the slot's pins connect to through the GPIO matrix.
///
/// Signals a slot does not route through the matrix (IO_MUX-routed bus
/// signals, or chips without a GPIO matrix at all) are `None`/empty, so one
/// table shape serves every chip. On IO_MUX-only chips none of the signal
/// fields are read, hence the conditional `allow(dead_code)`.
///
/// Populated from chip metadata by the `for_each_sdmmc!` invocation below.
#[cfg_attr(not(sdmmc_has_gpio_matrix), allow(dead_code))]
struct SlotInfo {
    io_event: u32,
    clk_out: Option<OutputSignal>,
    cmd_in: Option<InputSignal>,
    cmd_out: Option<OutputSignal>,
    data_in: &'static [InputSignal],
    data_out: &'static [OutputSignal],
    cd_in: Option<InputSignal>,
    wp_in: Option<InputSignal>,
}

// Wrap an optional metadata signal name in `Some(..)`, or yield `None` when
// the slot does not route that signal through the GPIO matrix.
macro_rules! opt_in {
    () => {
        None
    };
    ($signal:ident) => {
        Some(InputSignal::$signal)
    };
}
macro_rules! opt_out {
    () => {
        None
    };
    ($signal:ident) => {
        Some(OutputSignal::$signal)
    };
}

// The SDIO card-interrupt bit is positional (`rintsts` bit `16 + slot`); the
// signal names come straight from chip metadata.
for_each_sdmmc! {
    (all $( (
        $slot:ident, $idx:literal, $iomux:literal,
        [$($clk:ident)?], [$($cmd_in:ident)?], [$($cmd_out:ident)?],
        [$($data_in:ident),*], [$($data_out:ident),*],
        [$($cd:ident)?], [$($wp:ident)?], [$($card_int:ident)?],
        [$($data_strobe:ident)?], [$($rst:ident)?]
    ) ),*) => {
        static SLOT_INFO: [SlotInfo; 2] = [ $(
            SlotInfo {
                io_event: 1u32 << (16 + $idx),
                clk_out: opt_out!($($clk)?),
                cmd_in: opt_in!($($cmd_in)?),
                cmd_out: opt_out!($($cmd_out)?),
                data_in: &[ $(InputSignal::$data_in),* ],
                data_out: &[ $(OutputSignal::$data_out),* ],
                cd_in: opt_in!($($cd)?),
                wp_in: opt_in!($($wp)?),
            }
        ),* ];
    };
}

/// Static routing data for a slot.
fn slot_info(id: SlotId) -> &'static SlotInfo {
    &SLOT_INFO[id.index() as usize]
}

/// Interrupt mask kept armed while idle: card-detect plus any armed SDIO
/// card interrupts. Preserves IO arming across transfers.
fn idle_intmask() -> u32 {
    let mut m = INTMASK_IDLE;
    if SLOT_STATE[0].io_armed.load(Ordering::Acquire) {
        m |= EVT_IO_SLOT0;
    }
    if SLOT_STATE[1].io_armed.load(Ordering::Acquire) {
        m |= EVT_IO_SLOT1;
    }
    m
}

/// Arms the engine for a new operation and clears stale status.
fn arm_engine(expect_data: bool, multiblock: bool, transfer: Option<Transfer>) {
    STATE.engine.with(|eng| {
        *eng = Engine {
            transfer,
            result: None,
            expect_data,
            multiblock,
            over_seen: false,
            acd_seen: false,
            wait_busy: false,
            waker: WakerRegistration::new(),
        };
    });
}

/// SDMMC interrupt handler: refills the IDMAC ring and wakes the task.
#[handler]
fn on_interrupt() {
    let r = SDHOST::regs();
    let rint = r.rintsts().read().bits();
    let idsts = r.idsts().read().bits();

    STATE.engine.with(|eng| {
        if eng.wait_busy {
            // For an R1b/no-data command the SBE bit doubles as the Busy
            // Clear Interrupt, fired when the card releases DAT0.
            if rint & EVT_SBE != 0 {
                eng.result = Some(Ok(()));
            }
        } else {
            // Refill descriptors the engine has released mid-transfer.
            if let Some(t) = eng.transfer.as_mut()
                && t.remaining > 0
            {
                let ring = ring();
                let free = free_descriptors(ring, t.next_desc);
                if free > 0 {
                    fill_descriptors(ring, t, free);
                    r.pldmnd().write(|w| unsafe { w.bits(1) });
                }
            }

            if eng.result.is_none() {
                let err = map_rintsts(rint).err().or_else(|| {
                    if idsts & (IDSTS_FBE | IDSTS_DU) != 0 {
                        Some(Error::DmaError)
                    } else {
                        None
                    }
                });
                if let Some(e) = err {
                    eng.result = Some(Err(e));
                } else if eng.expect_data {
                    if rint & EVT_DATA_OVER != 0 {
                        eng.over_seen = true;
                    }
                    if rint & EVT_ACD != 0 {
                        eng.acd_seen = true;
                    }
                    if eng.over_seen && (!eng.multiblock || eng.acd_seen) {
                        eng.result = Some(Ok(()));
                    }
                } else if rint & EVT_CMD_DONE != 0 {
                    eng.result = Some(Ok(()));
                }
            }
        }

        if eng.result.is_some() {
            eng.wait_busy = false;
            eng.transfer = None;
            r.intmask().write(|w| unsafe { w.bits(idle_intmask()) });
            r.idinten().write(|w| unsafe { w.bits(0) });
            eng.waker.wake();
        }
    });

    // SDIO card interrupt (level-triggered): mask the bit and wake; the
    // caller re-arms after servicing the function via CMD52/53.
    if rint & EVT_IO_SLOT0 != 0 {
        SLOT_STATE[0].io_armed.store(false, Ordering::Release);
        r.intmask()
            .modify(|rd, w| unsafe { w.bits(rd.bits() & !EVT_IO_SLOT0) });
        SLOT_STATE[0].io_waker.wake();
    }
    if rint & EVT_IO_SLOT1 != 0 {
        SLOT_STATE[1].io_armed.store(false, Ordering::Release);
        r.intmask()
            .modify(|rd, w| unsafe { w.bits(rd.bits() & !EVT_IO_SLOT1) });
        SLOT_STATE[1].io_waker.wake();
    }

    // Card-detect change: wake both slot listeners.
    if rint & EVT_CD != 0 {
        SLOT_STATE[0].cd_waker.wake();
        SLOT_STATE[1].cd_waker.wake();
    }

    // Write-1-clear the bits handled this pass.
    r.rintsts().write(|w| unsafe { w.bits(rint) });
    r.idsts().write(|w| unsafe { w.bits(idsts) });
}

/// SDMMC / SDIO host controller driver.
///
/// Owns the shared transfer engine and hands out per-slot handles.
pub struct SdHostController<'d> {
    _peri: SDHOST<'d>,
    _guard: PeripheralGuard,
    taken: [AtomicBool; 2],
}

impl<'d> SdHostController<'d> {
    /// Creates the controller, enabling its bus clock and configuring the
    /// shared (engine-wide) module clock from `config`.
    pub fn new(peri: SDHOST<'d>, config: Config) -> Result<Self, ConfigError> {
        config.validate()?;
        // P4 powers the SD card / IO domain from an on-chip LDO that must be
        // up before any card communication.
        #[cfg(esp32p4)]
        enable_sd_io_ldo();
        let guard = PeripheralGuard::new(Peripheral::SdioHost);
        let this = Self {
            _peri: peri,
            _guard: guard,
            taken: [const { AtomicBool::new(false) }; 2],
        };

        // Module clock first, then the DesignWare reset, then quiesce
        // interrupts. The module clock is engine-wide, so it is programmed once
        // here and the config is stashed for slots to derive their card
        // dividers from. The clock must precede `reset_engine` because on P4
        // the controller's CIU has no functional clock until the module clock
        // (in `HP_SYS_CLKRST`) is running, so its reset would never complete.
        set_module_clock(config.clock_source, config.module_div);
        this.reset_engine();

        STATE.settings.with(|s| s.module = config);
        let r = SDHOST::regs();
        r.tmout().write(|w| unsafe {
            w.response_timeout().bits(0xFF);
            w.data_timeout().bits(0xFF_FFFF)
        });
        r.rintsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        r.ctrl().modify(|_, w| w.int_enable().clear_bit());

        Ok(this)
    }

    /// Resets the controller, FIFO and DMA blocks, waiting for self-clear.
    fn reset_engine(&self) {
        let r = SDHOST::regs();
        r.ctrl().modify(|_, w| {
            w.controller_reset().set_bit();
            w.fifo_reset().set_bit();
            w.dma_reset().set_bit()
        });

        // A reset timeout means the controller never left reset; there is no
        // matching `ConfigError` and the first card command would surface it
        // as a timeout regardless, so this is best-effort.
        let _ = poll_until(|| {
            let c = r.ctrl().read();
            !c.controller_reset().bit_is_set()
                && !c.fifo_reset().bit_is_set()
                && !c.dma_reset().bit_is_set()
        });
    }

    /// Returns a builder for the given slot in blocking mode.
    ///
    /// Both slots can be taken (once each) and driven independently; the
    /// shared engine serializes their transactions. Requesting the same slot
    /// twice returns [`ConfigError::SlotInUse`].
    ///
    /// The returned slot borrows the controller, so the controller cannot be
    /// dropped while any of its slots are alive.
    pub fn slot<const S: u8>(
        &self,
        config: SlotConfig,
    ) -> Result<Slot<'_, S, Blocking>, ConfigError> {
        const { ::core::assert!(S < 2, "SDMMC has only slots 0 and 1") };
        let idx = S as usize;
        if self.taken[idx].swap(true, Ordering::Relaxed) {
            return Err(ConfigError::SlotInUse);
        }
        // Publish this slot's tuning so the engine can re-apply it whenever
        // the slot becomes active.
        #[cfg(esp32s3)]
        STATE
            .settings
            .with(|s| s.slots[idx].input_delay_phase = config.input_delay_phase);
        Ok(Slot {
            id: slot_id(S),
            config,
            width: BusWidth::Bit1,
            data_pins: 0,
            clk_connected: false,
            cmd_connected: false,
            _guard: PeripheralGuard::new(Peripheral::SdioHost),
            power: None,
            cd: None,
            wp: None,
            _pd: PhantomData,
        })
    }
}

/// Maps a const slot index to its [`SlotId`].
const fn slot_id(s: u8) -> SlotId {
    if s == 0 { SlotId::_0 } else { SlotId::_1 }
}

/// Card-clock output pin for slot `S`.
pub trait SlotClk<'d, const S: u8> {
    #[doc(hidden)]
    fn configure(self);
}

/// Command (bidirectional) pin for slot `S`.
pub trait SlotCmd<'d, const S: u8> {
    #[doc(hidden)]
    fn configure(self);
}

/// Data line `L` (bidirectional) for slot `S`.
pub trait SlotData<'d, const S: u8, const L: u8> {
    #[doc(hidden)]
    fn configure(self);
}

// GPIO-matrix-routed slots (all of S3; slot 1 on P4): any pin can carry any of
// the slot's signals, routed through the interconnect matrix. The impls accept
// any GPIO and stash the routing guard in the slot's `State` for its lifetime.
//
// Generated per matrix slot (`iomux = false`) rather than blanket over the slot
// index: on P4 the IO_MUX slot's bus signals are implemented for fixed pins
// below, and a blanket `impl<const S>` would collide with those.
#[cfg(sdmmc_has_gpio_matrix)]
for_each_sdmmc! {
    (
        $slot:ident, $idx:literal, false,
        [$($clk:ident)?], [$($cmd_in:ident)?], [$($cmd_out:ident)?],
        [$($data_in:ident),*], [$($data_out:ident),*],
        [$($cd:ident)?], [$($wp:ident)?], [$($card_int:ident)?],
        [$($data_strobe:ident)?], [$($rst:ident)?]
    ) => {
        impl<'d, P: PeripheralOutput<'d>> SlotClk<'d, $idx> for P {
            fn configure(self) {
                let pin = self.into();
                pin.apply_output_config(&OutputConfig::default());
                pin.set_output_enable(true);
                slot_pins(slot_id($idx)).clk = interconnect::OutputSignal::connect_with_guard(
                    pin,
                    slot_info(slot_id($idx)).clk_out.unwrap(),
                );
            }
        }

        impl<'d, P: PeripheralInput<'d> + PeripheralOutput<'d>> SlotCmd<'d, $idx> for P {
            fn configure(self) {
                slot_pins(slot_id($idx)).cmd = connect_bidir(
                    self.into(),
                    slot_info(slot_id($idx)).cmd_in.unwrap(),
                    slot_info(slot_id($idx)).cmd_out.unwrap(),
                );
            }
        }

        impl<'d, const L: u8, P: PeripheralInput<'d> + PeripheralOutput<'d>>
            SlotData<'d, $idx, L> for P
        {
            fn configure(self) {
                slot_pins(slot_id($idx)).data[L as usize] = connect_bidir(
                    self.into(),
                    slot_info(slot_id($idx)).data_in[L as usize],
                    slot_info(slot_id($idx)).data_out[L as usize],
                );
            }
        }
    };

    // IO_MUX-routed slots (P4 slot 0) get their bus impls from the fixed-pin
    // block below; nothing to generate here.
    (
        $slot:ident, $idx:literal, true,
        [$($clk:ident)?], [$($cmd_in:ident)?], [$($cmd_out:ident)?],
        [$($data_in:ident),*], [$($data_out:ident),*],
        [$($cd:ident)?], [$($wp:ident)?], [$($card_int:ident)?],
        [$($data_strobe:ident)?], [$($rst:ident)?]
    ) => {};
}

// IO_MUX-routed slots (both slots on ESP32; slot 0 on P4): each bus signal
// lives on a fixed pad selected by an IO_MUX function. The traits are
// implemented only for the mandated `(gpio, af)` pairs, so a wrong pin is a
// compile error. ESP32 names them `HS1_*` (slot 0) / `HS2_*` (slot 1); P4
// names slot 0's pads `SD1_*`.
#[cfg(sdmmc_has_iomux)]
fn configure_iomux_pad(pin: u8, af: crate::gpio::AlternateFunction) {
    crate::gpio::io_mux_reg(pin).modify(|_, w| {
        unsafe { w.mcu_sel().bits(af as u8) };
        w.fun_ie().set_bit();
        w.fun_wpu().set_bit();
        // esp-idf bumps the SD pads to the strongest drive on every chip
        // except ESP32 (where the default of 2 is sufficient). Matches
        // `configure_pin_iomux`.
        #[cfg(not(esp32))]
        unsafe {
            w.fun_drv().bits(3)
        };
        w
    });
}

#[cfg(sdmmc_has_iomux)]
macro_rules! impl_signal_trait {
    ($gpio:ident, $trait:ident, $af:ident, $s:literal) => {
        impl<'d> $trait<'d, $s> for crate::peripherals::$gpio<'d> {
            fn configure(self) {
                configure_iomux_pad(
                    crate::gpio::Pin::number(&self),
                    crate::gpio::AlternateFunction::$af,
                );
            }
        }
    };

    ($gpio:ident, $trait:ident, $af:ident, $s:literal, $l:literal) => {
        impl<'d> $trait<'d, $s, $l> for crate::peripherals::$gpio<'d> {
            fn configure(self) {
                configure_iomux_pad(
                    crate::gpio::Pin::number(&self),
                    crate::gpio::AlternateFunction::$af,
                );
            }
        }
    };
}

// Arms for functions absent on a given chip simply never match (the generated
// macro ignores unmatched functions), so ESP32's `HS*_*` and P4's `SD1_*` can
// coexist here. P4 wires only slot 0 (`SD1_*`) to fixed pads; slot 1 is matrix.
#[cfg(sdmmc_has_iomux)]
for_each_iomux_function! {
    (SD1_CLK, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotClk, $af, 0); };
    (SD1_CMD, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotCmd, $af, 0); };
    (SD1_DATA0, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 0); };
    (SD1_DATA1, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 1); };
    (SD1_DATA2, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 2); };
    (SD1_DATA3, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 3); };

    (SD2_CLK, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotClk, $af, 1); };
    (SD2_CMD, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotCmd, $af, 1); };
    (SD2_DATA0, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 0); };
    (SD2_DATA1, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 1); };
    (SD2_DATA2, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 2); };
    (SD2_DATA3, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 3); };
}

/// A configured card slot, generic over the slot index and driver mode.
pub struct Slot<'d, const S: u8, Dm: DriverMode> {
    id: SlotId,
    config: SlotConfig,
    width: BusWidth,
    data_pins: u8,
    clk_connected: bool,
    cmd_connected: bool,
    _guard: PeripheralGuard,
    power: Option<interconnect::OutputSignal<'d>>,
    cd: Option<interconnect::InputSignal<'d>>,
    wp: Option<interconnect::InputSignal<'d>>,
    _pd: PhantomData<(&'d mut (), Dm)>,
}

impl<'d, const S: u8, Dm: DriverMode> Slot<'d, S, Dm> {
    /// Connects the card clock output.
    pub fn with_clk(mut self, clk: impl SlotClk<'d, S>) -> Self {
        clk.configure();
        self.clk_connected = true;
        self
    }

    /// Connects the bidirectional command line.
    pub fn with_cmd(mut self, cmd: impl SlotCmd<'d, S>) -> Self {
        cmd.configure();
        self.cmd_connected = true;
        self
    }

    /// Connects data line 0 (required for any transfer).
    pub fn with_data0(mut self, d0: impl SlotData<'d, S, 0>) -> Self {
        d0.configure();
        self.note_data_pin(1);
        self
    }

    /// Connects data line 1.
    pub fn with_data1(mut self, d1: impl SlotData<'d, S, 1>) -> Self {
        d1.configure();
        self.note_data_pin(2);
        self
    }

    /// Connects data line 2.
    pub fn with_data2(mut self, d2: impl SlotData<'d, S, 2>) -> Self {
        d2.configure();
        self.note_data_pin(3);
        self
    }

    /// Connects data line 3 (completes the 4-bit bus).
    pub fn with_data3(mut self, d3: impl SlotData<'d, S, 3>) -> Self {
        d3.configure();
        self.note_data_pin(4);
        self
    }

    /// Connects the card-detect input.
    #[cfg(sdmmc_has_gpio_matrix)]
    pub fn with_card_detect(mut self, cd: impl PeripheralInput<'d>) -> Self {
        let pin = cd.into();
        pin.set_input_enable(true);
        slot_info(self.id).cd_in.unwrap().connect_to(&pin);
        self.cd = Some(pin);
        self
    }

    /// Connects the write-protect input.
    #[cfg(sdmmc_has_gpio_matrix)]
    pub fn with_write_protect(mut self, wp: impl PeripheralInput<'d>) -> Self {
        let pin = wp.into();
        pin.set_input_enable(true);
        slot_info(self.id).wp_in.unwrap().connect_to(&pin);
        self.wp = Some(pin);
        self
    }

    /// Connects a GPIO used to switch card power. Driven high here.
    pub fn with_power_enable(mut self, power: impl PeripheralOutput<'d>) -> Self {
        let pin = power.into();
        pin.set_output_high(true);
        pin.apply_output_config(&OutputConfig::default());
        pin.set_output_enable(true);
        self.power = Some(pin);
        self
    }

    /// Returns `true` if a card is detected, or if no card-detect pin is
    /// wired (assume present).
    pub fn is_card_present(&self) -> bool {
        if self.cd.is_none() {
            return true;
        }
        (SDHOST::regs().cdetect().read().card_detect_n().bits() & (1 << self.id.index())) == 0
    }

    /// Returns `true` if the card reports write protection. Returns `false`
    /// if no write-protect pin is wired. Polarity follows
    /// [`SlotConfig::with_wp_active_high`].
    pub fn is_write_protected(&self) -> bool {
        if self.wp.is_none() {
            return false;
        }
        let level =
            (SDHOST::regs().wrtprt().read().write_protect().bits() & (1 << self.id.index())) != 0;
        level == self.config.wp_active_high
    }

    /// Applies bus width + card clock for the slot. Used during card init.
    pub fn set_bus_low_level(&mut self, width: BusWidth, hz: u32) -> Result<(), Error> {
        // The engine-wide module clock is programmed once at construction;
        // here we only derive this slot's card divider from it.
        let (source, div) = STATE
            .settings
            .with(|s| (s.module.clock_source, s.module.module_div));
        let card_div = freq_to_card_div(module_hz(source, div), hz);

        // Immediately apply slot-specific configuration
        STATE.engine.with(|eng| {
            eng.set_card_width(self.id, width);
            eng.set_card_clock(self.id, card_div)?;
            Ok(())
        })?;
        self.width = width;

        // Record this slot's frequency and (re-)apply its slot-specific
        // shared-register tuning now that it is the active slot. Done here
        // (not via `select_slot_sync`) so a frequency change on the already
        // active slot still re-applies the input-delay phase.
        #[cfg(esp32s3)]
        STATE.settings.with(|s| {
            let idx = self.id.index() as usize;
            s.slots[idx].hz = hz;
            s.active_slot = Some(self.id);
            configure_for_slot(s, self.id);
        });

        // Let the freshly (re-)started card clock propagate before the next
        // command, matching esp-idf's `sd_host_set_clk_div`. Without this the
        // first response command after a clock change can be missed.
        crate::rom::ets_delay_us(10);
        Ok(())
    }

    /// Checks that the mandatory pins for a transfer were connected.
    fn validate_pins(&self) -> Result<(), ConfigError> {
        if !self.clk_connected || !self.cmd_connected {
            return Err(ConfigError::MissingClkOrCmd);
        }
        if self.data_pins < 1 {
            return Err(ConfigError::NoData0);
        }
        Ok(())
    }

    /// Issues the 80-clock SD init sequence (no command index).
    pub fn send_init_sequence(&mut self) -> Result<(), Error> {
        select_slot_sync(self.id);
        let r = SDHOST::regs();
        r.rintsts().write(|w| unsafe { w.bits(EVT_CMD_DONE) });
        r.cmdarg().write(|w| unsafe { w.bits(0) });
        r.cmd().write(|w| unsafe {
            w.send_initialization().set_bit();
            w.wait_prvdata_complete().set_bit();
            w.card_number().bits(self.id.index() as u8);
            w.start_cmd().set_bit()
        });
        wait_command_accepted()?;

        // Wait for the 80 init clocks to actually finish before any command.
        for _ in 0..POLL_LIMIT {
            if r.rintsts().read().bits() & EVT_CMD_DONE != 0 {
                r.rintsts().write(|w| unsafe { w.bits(EVT_CMD_DONE) });
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Issues a no-data command and returns its raw response words.
    ///
    /// Low-level bring-up surface; the public API arrives with `MmcBus`.
    #[doc(hidden)]
    pub fn command_blocking(
        &mut self,
        index: u8,
        arg: u32,
        resp_len: ResponseLen,
        check_crc: bool,
        flags: CommandFlags,
    ) -> Result<[u32; 4], Error> {
        if !self.is_card_present() {
            return Err(Error::NoCard);
        }
        select_slot_sync(self.id);
        self.send_command_blocking(index, arg, resp_len, check_crc, flags)
    }

    /// Reads `block_count` blocks into a DMA-capable buffer (CMD17/CMD18).
    ///
    /// Low-level bring-up surface; the public API arrives with `MmcBus`.
    #[doc(hidden)]
    pub fn read_blocks_blocking(
        &mut self,
        cmd_index: u8,
        arg: u32,
        buf: &mut DmaAlignedMut<'_, [u8]>,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], Error> {
        select_slot_sync(self.id);
        let total = buf.len();
        let ptr = dma_ptr(buf.as_ptr() as usize)?;
        let resp =
            self.transfer_blocking(cmd_index, arg, false, ptr, total, block_size, block_count)?;
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        buf.invalidate();
        Ok(resp)
    }

    /// Writes `block_count` blocks from a DMA-capable buffer (CMD24/CMD25).
    ///
    /// Low-level bring-up surface; the public API arrives with `MmcBus`.
    #[doc(hidden)]
    pub fn write_blocks_blocking(
        &mut self,
        cmd_index: u8,
        arg: u32,
        buf: &mut DmaAlignedMut<'_, [u8]>,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], Error> {
        select_slot_sync(self.id);
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        buf.writeback();
        let total = buf.len();
        let ptr = dma_ptr(buf.as_ptr() as usize)?;
        self.transfer_blocking(cmd_index, arg, true, ptr, total, block_size, block_count)
    }

    fn note_data_pin(&mut self, count: u8) {
        self.data_pins = self.data_pins.max(count);
        self.width = if self.data_pins >= 4 {
            BusWidth::Bit4
        } else {
            BusWidth::Bit1
        };
    }

    /// Issues a no-data command and waits for completion in blocking mode.
    fn send_command_blocking(
        &mut self,
        index: u8,
        arg: u32,
        resp_len: ResponseLen,
        check_crc: bool,
        flags: CommandFlags,
    ) -> Result<[u32; 4], Error> {
        let r = SDHOST::regs();
        let consume = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR | EVT_HLE;

        // Clear stale command/response status before issuing.
        r.rintsts().write(|w| unsafe { w.bits(consume) });

        self.issue_command(index, arg, resp_len, check_crc, flags)?;

        // Wait for completion or a command-phase error.
        let done = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR;
        let mut sts = 0;
        let mut completed = false;
        for _ in 0..POLL_LIMIT {
            sts = r.rintsts().read().bits();
            if sts & done != 0 {
                completed = true;
                break;
            }
        }
        if !completed {
            return Err(Error::Timeout);
        }
        map_rintsts(sts)?;

        let resp = read_response(resp_len);
        r.rintsts().write(|w| unsafe { w.bits(consume) });

        // R1b / busy: DesignWare only IRQs busy-clear for writes, so always poll.
        if flags.busy {
            wait_busy_cleared()?;
        }
        Ok(resp)
    }

    /// Single- or multi-block data transfer over the IDMAC, blocking.
    fn transfer_blocking(
        &mut self,
        index: u8,
        arg: u32,
        write: bool,
        buf_ptr: u32,
        total_len: usize,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], Error> {
        let r = SDHOST::regs();

        reset_transfer()?;
        r.blksiz().write(|w| unsafe { w.bits(block_size as u32) });
        r.bytcnt().write(|w| unsafe { w.bits(total_len as u32) });

        // Build the descriptor chain for the whole transfer.
        let ring = ring();
        *ring = [Desc::ZERO; RING_LEN];
        ring[0].flags |= DESC_FIRST;
        let mut t = Transfer {
            ptr: buf_ptr,
            remaining: total_len,
            next_desc: 0,
        };
        fill_descriptors(ring, &mut t, RING_LEN);
        enable_idmac(ring.as_ptr() as u32);
        r.pldmnd().write(|w| unsafe { w.bits(1) });

        self.issue_data_command(index, arg, write, block_count)?;

        let result = run_data_phase(&mut t, ring, write, block_count);

        disable_idmac();
        r.rintsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        r.idsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });

        result?;
        Ok(read_response(ResponseLen::Short))
    }

    /// Programs `cmdarg`/`cmd` for a no-data command and submits it to the CIU.
    fn issue_command(
        &mut self,
        index: u8,
        arg: u32,
        resp_len: ResponseLen,
        check_crc: bool,
        flags: CommandFlags,
    ) -> Result<(), Error> {
        let r = SDHOST::regs();
        r.cmdarg().write(|w| unsafe { w.bits(arg) });
        r.cmd().write(|w| unsafe {
            w.index().bits(index);
            w.response_expect()
                .bit(!matches!(resp_len, ResponseLen::None));
            w.response_length()
                .bit(matches!(resp_len, ResponseLen::Long));
            w.check_response_crc().bit(check_crc);
            w.wait_prvdata_complete().bit(flags.wait_complete);
            w.stop_abort_cmd().bit(flags.stop_abort);
            w.use_hole().set_bit();
            w.card_number().bits(self.id.index());
            w.start_cmd().set_bit()
        });
        wait_command_accepted()
    }

    /// Programs `cmdarg`/`cmd` for a data-transfer command and submits it to the
    /// CIU.
    fn issue_data_command(
        &mut self,
        index: u8,
        arg: u32,
        write: bool,
        block_count: u32,
    ) -> Result<(), Error> {
        let r = SDHOST::regs();
        r.cmdarg().write(|w| unsafe { w.bits(arg) });
        r.cmd().write(|w| unsafe {
            w.index().bits(index);
            w.response_expect().set_bit();
            w.check_response_crc().set_bit();
            w.data_expected().set_bit();
            w.read_write().bit(write);
            w.send_auto_stop().bit(block_count > 1);
            w.wait_prvdata_complete().set_bit();
            w.use_hole().set_bit();
            w.card_number().bits(self.id.index());
            w.start_cmd().set_bit()
        });
        wait_command_accepted()
    }
}

impl<'d, const S: u8> Slot<'d, S, Blocking> {
    /// Reconfigures the slot to operate in [`Async`] mode.
    ///
    /// Binds the `SDHOST` interrupt handler and enables interrupt delivery.
    pub fn into_async(self) -> Slot<'d, S, Async> {
        let r = SDHOST::regs();
        r.rintsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        r.idsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        r.intmask().write(|w| unsafe { w.bits(idle_intmask()) });
        r.idinten().write(|w| unsafe { w.bits(0) });
        r.ctrl().modify(|_, w| w.int_enable().set_bit());
        crate::interrupt::bind_handler(Interrupt::SDIO_HOST, on_interrupt);

        Slot {
            id: self.id,
            config: self.config,
            width: self.width,
            data_pins: self.data_pins,
            clk_connected: self.clk_connected,
            cmd_connected: self.cmd_connected,
            _guard: self._guard,
            power: self.power,
            cd: self.cd,
            wp: self.wp,
            _pd: PhantomData,
        }
    }
}

impl<'d, const S: u8> Slot<'d, S, Async> {
    /// Reconfigures the slot back to [`Blocking`] mode.
    pub fn into_blocking(self) -> Slot<'d, S, Blocking> {
        let r = SDHOST::regs();
        r.ctrl().modify(|_, w| w.int_enable().clear_bit());
        r.intmask().write(|w| unsafe { w.bits(0) });
        r.idinten().write(|w| unsafe { w.bits(0) });
        crate::interrupt::disable(crate::system::Cpu::current(), Interrupt::SDIO_HOST);

        Slot {
            id: self.id,
            config: self.config,
            width: self.width,
            data_pins: self.data_pins,
            clk_connected: self.clk_connected,
            cmd_connected: self.cmd_connected,
            _guard: self._guard,
            power: self.power,
            cd: self.cd,
            wp: self.wp,
            _pd: PhantomData,
        }
    }

    /// Waits for the SDIO card interrupt (card pulls DAT1 low).
    ///
    /// Usage contract: `await` this, service the function via CMD52/53, then
    /// `await` again to re-arm. The interrupt is level-triggered and is
    /// masked on wake to avoid an IRQ storm until the caller re-arms.
    pub async fn wait_for_sdio_interrupt(&mut self) {
        let slot = self.id.index() as usize;
        let bit = slot_info(self.id).io_event;

        SLOT_STATE[slot].io_armed.store(true, Ordering::Release);
        SDHOST::regs()
            .intmask()
            .modify(|rd, w| unsafe { w.bits(rd.bits() | bit) });

        poll_fn(|cx| {
            SLOT_STATE[slot].io_waker.register(cx.waker());
            if SLOT_STATE[slot].io_armed.load(Ordering::Acquire) {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        })
        .await
    }

    /// Issues a control command, mapping protocol types to the engine core.
    async fn cmd_async<'a, C: sdio::ControlCommand + 'a>(
        &mut self,
        cmd: C,
    ) -> Result<C::Resp<'a>, sdio::MmcError> {
        let resp_len = match <C::Resp<'a> as sdio::Response>::LEN {
            sdio::ResponseLen::Zero => ResponseLen::None,
            sdio::ResponseLen::R48 => ResponseLen::Short,
            sdio::ResponseLen::R136 => ResponseLen::Long,
        }
        .for_index(C::INDEX);
        let flags = CommandFlags {
            wait_complete: !is_stop_or_abort(C::INDEX),
            stop_abort: is_stop_or_abort(C::INDEX),
            busy: <C::Resp<'a> as sdio::Response>::BUSY,
        };
        let crc = <C::Resp<'a> as sdio::Response>::CRC;
        let words = self
            .send_command_async(C::INDEX, cmd.arg(), resp_len, crc, flags)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    /// Reads blocks/bytes into the command's buffer via the IDMAC.
    ///
    /// `bounce` is the transaction-scoped scratch owned by `engine_lock`.
    #[cfg_attr(
        not(any(soc_internal_memory_cached, dma_can_access_psram)),
        allow(unused_variables)
    )]
    async fn read_async(
        &mut self,
        index: u8,
        arg: u32,
        buf: &mut [u8],
        block_size: u16,
        block_count: u32,
        bounce: &mut Bounce,
    ) -> Result<[u32; 4], sdio::MmcError> {
        // Small control reads (SCR/SSR/SWITCH) from a cache-unaligned caller
        // buffer can't be DMA'd directly on cached SoCs: invalidation would
        // clobber data sharing the cache line. Bounce them through the
        // aligned scratch buffer and copy the result out.
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        if buf.len() <= BOUNCE_LEN && DmaAlignedMut::new(&mut buf[..]).is_err() {
            let total = buf.len();
            let mut dma = bounce.buf.get_mut();
            let ptr = dma_ptr(dma.as_ptr() as usize)?;
            let resp = self
                .transfer_async(index, arg, false, ptr, total, block_size, block_count)
                .await?;
            dma.invalidate();
            buf.copy_from_slice(&dma[..total]);
            return Ok(resp);
        }

        #[cfg_attr(
            not(any(soc_internal_memory_cached, dma_can_access_psram)),
            allow(unused_mut)
        )]
        let mut dma = DmaAlignedMut::new(buf).map_err(|e| match e {
            crate::dma::DmaBufError::InvalidAlignment(_) => sdio::MmcError::Other,
            _ => sdio::MmcError::Io,
        })?;
        let ptr = dma_ptr(dma.as_ptr() as usize)?;
        let total = dma.len();
        let resp = self
            .transfer_async(index, arg, false, ptr, total, block_size, block_count)
            .await?;
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        dma.invalidate();
        Ok(resp)
    }

    /// Writes blocks/bytes from the command's buffer via the IDMAC.
    ///
    /// `bounce` is the transaction-scoped scratch owned by `engine_lock`.
    #[cfg_attr(
        not(any(soc_internal_memory_cached, dma_can_access_psram)),
        allow(unused_variables)
    )]
    async fn write_async(
        &mut self,
        index: u8,
        arg: u32,
        buf: &[u8],
        block_size: u16,
        block_count: u32,
        bounce: &mut Bounce,
    ) -> Result<[u32; 4], sdio::MmcError> {
        // Cached SoCs: the IDMAC reads the source straight from memory, so
        // the CPU's dirty cache lines must be flushed first. A cache-
        // unaligned caller buffer also can't be flushed in place without
        // touching neighbours, so bounce it through the aligned scratch.
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        if buf.len() <= BOUNCE_LEN {
            let total = buf.len();
            let mut dma = bounce.buf.get_mut();
            dma[..total].copy_from_slice(buf);
            dma.writeback();
            let ptr = dma_ptr(dma.as_ptr() as usize)?;
            return Ok(self
                .transfer_async(index, arg, true, ptr, total, block_size, block_count)
                .await?);
        }

        let ptr = dma_ptr(buf.as_ptr() as usize)?;
        Ok(self
            .transfer_async(index, arg, true, ptr, buf.len(), block_size, block_count)
            .await?)
    }

    /// Issues a no-data command and awaits completion via the interrupt handler.
    async fn send_command_async(
        &mut self,
        index: u8,
        arg: u32,
        resp_len: ResponseLen,
        check_crc: bool,
        flags: CommandFlags,
    ) -> Result<[u32; 4], Error> {
        let r = SDHOST::regs();
        let guard = DropGuard::new((), |()| abort_transfer());

        r.rintsts().write(|w| unsafe { w.bits(INTMASK_CMD) });
        arm_engine(false, false, None);
        r.intmask()
            .write(|w| unsafe { w.bits(idle_intmask() | INTMASK_CMD) });

        self.issue_command(index, arg, resp_len, check_crc, flags)?;
        wait_result().await?;
        let resp = read_response(resp_len);
        if flags.busy {
            wait_busy_poll().await?;
        }
        guard.defuse();
        Ok(resp)
    }

    /// Single- or multi-block data transfer over the IDMAC, interrupt-driven.
    async fn transfer_async(
        &mut self,
        index: u8,
        arg: u32,
        write: bool,
        buf_ptr: u32,
        total_len: usize,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], Error> {
        let guard = DropGuard::new((), |()| abort_transfer());
        let r = SDHOST::regs();

        reset_transfer()?;
        r.blksiz().write(|w| unsafe { w.bits(block_size as u32) });
        r.bytcnt().write(|w| unsafe { w.bits(total_len as u32) });

        // Build the descriptor chain and prime the IDMAC.
        let ring = ring();
        *ring = [Desc::ZERO; RING_LEN];
        ring[0].flags |= DESC_FIRST;
        let mut t = Transfer {
            ptr: buf_ptr,
            remaining: total_len,
            next_desc: 0,
        };
        fill_descriptors(ring, &mut t, RING_LEN);
        enable_idmac(ring.as_ptr() as u32);
        r.pldmnd().write(|w| unsafe { w.bits(1) });

        arm_engine(true, block_count > 1, Some(t));
        r.idinten().write(|w| unsafe { w.bits(IDINTEN_ALL) });
        r.intmask()
            .write(|w| unsafe { w.bits(idle_intmask() | INTMASK_DATA) });

        self.issue_data_command(index, arg, write, block_count)?;
        wait_result().await?;
        if write {
            wait_busy_async().await?;
        }
        disable_idmac();
        let resp = read_response(ResponseLen::Short);
        guard.defuse();
        Ok(resp)
    }
}

impl<'d, const S: u8> sdio::MmcBus for Slot<'d, S, Async> {
    async fn send_command<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, sdio::MmcError>
    where
        C: sdio::ControlCommand + 'a,
    {
        let _lock = STATE.select_slot(slot_id(S)).await;
        self.cmd_async::<C>(cmd).await
    }

    async fn read_blocks<'a, C>(&mut self, mut cmd: C) -> Result<C::Resp<'a>, sdio::MmcError>
    where
        C: sdio::BlockReadCommand + 'a,
    {
        let mut lock = STATE.select_slot(slot_id(S)).await;
        let (bs, bc, arg) = (cmd.block_size().len() as u16, cmd.block_count(), cmd.arg());
        let words = self
            .read_async(C::INDEX, arg, cmd.buf(), bs, bc, &mut lock)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn write_blocks<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, sdio::MmcError>
    where
        C: sdio::BlockWriteCommand + 'a,
    {
        let mut lock = STATE.select_slot(slot_id(S)).await;
        let (bs, bc, arg) = (cmd.block_size().len() as u16, cmd.block_count(), cmd.arg());
        let words = self
            .write_async(C::INDEX, arg, cmd.buf(), bs, bc, &mut lock)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn read_bytes<'a, C>(&mut self, mut cmd: C) -> Result<C::Resp<'a>, sdio::MmcError>
    where
        C: sdio::ByteReadCommand + 'a,
    {
        let mut lock = STATE.select_slot(slot_id(S)).await;
        let (n, arg) = (cmd.byte_count(), cmd.arg());
        let words = self
            .read_async(C::INDEX, arg, cmd.buf(), n as u16, 1, &mut lock)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn write_bytes<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, sdio::MmcError>
    where
        C: sdio::ByteWriteCommand + 'a,
    {
        let mut lock = STATE.select_slot(slot_id(S)).await;
        let (n, arg) = (cmd.byte_count(), cmd.arg());
        let words = self
            .write_async(C::INDEX, arg, cmd.buf(), n as u16, 1, &mut lock)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn init_idle(&mut self, hz: u32) -> Result<(), sdio::MmcError> {
        let _lock = STATE.select_slot(slot_id(S)).await;
        self.validate_pins().map_err(|_| sdio::MmcError::Other)?;
        self.set_bus_low_level(BusWidth::Bit1, hz)?;
        self.send_init_sequence()?;
        Ok(())
    }

    fn set_bus(&mut self, width: sdio::BusWidth, hz: u32) -> Result<(), sdio::MmcError> {
        let w = match width {
            sdio::BusWidth::W1 => BusWidth::Bit1,
            sdio::BusWidth::W4 => BusWidth::Bit4,
            sdio::BusWidth::W8 => return Err(sdio::MmcError::Unsupported),
        };
        if matches!(w, BusWidth::Bit4) && self.data_pins < 4 {
            return Err(sdio::MmcError::Unsupported);
        }
        if hz > 40_000_000 {
            return Err(sdio::MmcError::Unsupported);
        }
        self.set_bus_low_level(w, hz)?;
        Ok(())
    }

    fn supports_mmc(&self) -> bool {
        // Native parallel SD/MMC host (not SPI mode).
        true
    }

    fn supports_bus_width(&self) -> sdio::BusWidth {
        if self.data_pins >= 4 {
            sdio::BusWidth::W4
        } else {
            sdio::BusWidth::W1
        }
    }

    fn supports_frequency(&self) -> u32 {
        40_000_000
    }
}

/// Maps the engine's [`ResponseLen`] from the crate's typed response.
/// CMD12 (and SDIO abort) are stop/abort commands.
fn is_stop_or_abort(index: u8) -> bool {
    index == 12
}

#[cfg(sdmmc_has_gpio_matrix)]
fn connect_bidir(
    pin: interconnect::OutputSignal<'_>,
    input: InputSignal,
    output: OutputSignal,
) -> PinGuard {
    pin.set_output_high(true);
    pin.apply_output_config(&OutputConfig::default().with_pull(Pull::Up));
    pin.set_output_enable(true);
    pin.set_input_enable(true);
    input.connect_to(&pin);
    interconnect::OutputSignal::connect_with_guard(pin, output)
}

/// Spins up to `POLL_LIMIT` times until `ready` holds, else times out.
fn poll_until(mut ready: impl FnMut() -> bool) -> Result<(), Error> {
    for _ in 0..POLL_LIMIT {
        if ready() {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}

/// Programs the shared module clock register (divider, source, phases).
///
/// Field encodings differ per chip (see each chip's `sdmmc_ll`).
#[cfg(esp32s3)]
fn set_module_clock(source: ClockSource, div: u8) {
    // S3: l == period, h == high pulse, n == l; phase dout=90°, din=0°.
    let l = div - 1;
    let h = (div / 2).saturating_sub(1);
    let n = l;
    SDHOST::regs().clk_edge_sel().write(|w| unsafe {
        w.cclkin_edge_drv_sel().bits(1); // output phase 90°
        w.cclkin_edge_sam_sel().bits(0); // input phase 0°
        w.cclkin_edge_slf_sel().bits(0); // core phase 0°
        w.ccllkin_edge_h().bits(h);
        w.ccllkin_edge_l().bits(l);
        w.ccllkin_edge_n().bits(n);
        w.cclk_en().bit(matches!(source, ClockSource::Pll160m))
    });
}

/// Powers the SD card / SD IO domain from on-chip LDO channel 4 at 3.3 V.
///
/// On the ESP32-P4 the dedicated SD IO pins (GPIO39-44) and the card VDD are
/// supplied by internal LDO channel 4 (analog unit `ext_ldo[4]`), which must
/// be brought up before any card communication or every command response
/// times out. esp-idf does this in the application via
/// `sd_pwr_ctrl_new_on_chip_ldo` (`CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID = 4`
/// on the reference board). This is board-specific and belongs in a future
/// regulator API; it lives here for now to bring P4 up.
///
/// Voltage selection (mirrors esp-idf `ldo_ll`): `force_tieh_sel = 1`
/// hands control to software (`tieh_sel`), `tieh_sel = 0` selects the `tieh`
/// bit, and `tieh = 1` ties the output to the 3.3 V rail directly, so the
/// `dref`/`mul` reference is irrelevant. `target0`/`target1` keep their
/// reset (power-on delay) defaults via `modify`.
#[cfg(esp32p4)]
fn enable_sd_io_ldo() {
    let pmu = crate::peripherals::PMU::regs();

    // Limit inrush current while the output cap charges; keep ripple
    // suppression (voltage detector) enabled.
    pmu.ext_ldo_p1_0p2a_ana().modify(|_, w| {
        w.ana_0p2a_en_cur_lim_1().set_bit();
        w.ana_0p2a_en_vdet_1().set_bit()
    });

    // Software-controlled, 3.3 V rail, then enable the regulator.
    pmu.ext_ldo_p1_0p2a().modify(|_, w| {
        w._0p2a_force_tieh_sel_1().set_bit();
        unsafe { w._0p2a_tieh_sel_1().bits(0) };
        w._0p2a_tieh_1().set_bit();
        w._0p2a_xpd_1().set_bit()
    });

    crate::rom::ets_delay_us(500);

    // Drop the inrush current limit once the output has settled.
    pmu.ext_ldo_p1_0p2a_ana()
        .modify(|_, w| w.ana_0p2a_en_cur_lim_1().clear_bit());
    crate::rom::ets_delay_us(100);
}

/// Programs the module clock in `HP_SYS_CLKRST` (P4 keeps the divider/source
/// outside the SDHOST block; see `sdmmc_ll`). Source fixed at PLL160M.
#[cfg(esp32p4)]
fn set_module_clock(_source: ClockSource, div: u8) {
    let c = crate::peripherals::HP_SYS_CLKRST::regs();

    // Enable the PLL_F160M reference clock that feeds the SDMMC LS clock
    // (esp-idf `esp_clk_tree_enable_src`). Without this gate `cclk_in` stays at
    // 0, the controller never clocks, and every command times out.
    c.ref_clk_ctrl2()
        .modify(|_, w| w.ref_160m_clk_en().set_bit());

    c.peri_clk_ctrl01().modify(|_, w| {
        w.sdio_ls_clk_src_sel().bit(false); // PLL_F160M
        w.sdio_ls_clk_en().set_bit()
    });

    // Low-speed clock divider edges (`h`/`l`/`n`); `div == 1` bypasses the
    // divider via `sdio_hs_mode`.
    if div > 1 {
        let h = div / 2 - 1;
        let l = div - 1;
        let n = div - 1;
        c.peri_clk_ctrl02().modify(|_, w| unsafe {
            w.sdio_ls_clk_edge_h().bits(h);
            w.sdio_ls_clk_edge_l().bits(l);
            w.sdio_ls_clk_edge_n().bits(n)
        });
    } else {
        c.peri_clk_ctrl01()
            .modify(|_, w| w.sdio_hs_mode().set_bit());
        c.peri_clk_ctrl02().modify(|_, w| unsafe {
            w.sdio_ls_clk_edge_h().bits(0);
            w.sdio_ls_clk_edge_l().bits(0);
            w.sdio_ls_clk_edge_n().bits(0)
        });
    }

    // Enable the drive (output), sample (input) and self (core) clocks with
    // their default edge phases (drive=1, sample=0, self=0). Without these the
    // controller never drives the card clock out, so commands never complete.
    // Mirrors esp-idf `sdmmc_ll_init_phase_delay`.
    c.peri_clk_ctrl02().modify(|_, w| unsafe {
        w.sdio_ls_drv_clk_en().set_bit();
        w.sdio_ls_sam_clk_en().set_bit();
        w.sdio_ls_slf_clk_en().set_bit();
        w.sdio_ls_drv_clk_edge_sel().bits(1);
        w.sdio_ls_sam_clk_edge_sel().bits(0);
        w.sdio_ls_slf_clk_edge_sel().bits(0)
    });

    // Commit the divider/phase edge configuration.
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().set_bit());
    c.peri_clk_ctrl02()
        .modify(|_, w| w.sdio_ls_clk_edge_cfg_update().clear_bit());
}

/// Programs the shared module clock register (divider, phases).
#[cfg(esp32)]
fn set_module_clock(_source: ClockSource, div: u8) {
    // ESP32: h == period, l == high pulse, n == h; phase dout=din=180°. The
    // module clock source is fixed at PLL160M (no `clk_sel`).
    let h = div - 1;
    let l = (div / 2).saturating_sub(1);
    let n = h;
    SDHOST::regs().clk_edge_sel().write(|w| unsafe {
        w.cclkin_edge_drv_sel().bits(4); // output phase 180°
        w.cclkin_edge_sam_sel().bits(4); // input phase 180°
        w.cclkin_edge_slf_sel().bits(0); // core phase 0°
        w.ccllkin_edge_h().bits(h);
        w.ccllkin_edge_l().bits(l);
        w.ccllkin_edge_n().bits(n)
    });
}

/// Spins until the CIU accepts the command (`start_cmd` self-clears).
fn wait_command_accepted() -> Result<(), Error> {
    let r = SDHOST::regs();
    for _ in 0..POLL_LIMIT {
        if !r.cmd().read().start_cmd().bit_is_set() {
            return Ok(());
        }
        // Hardware-locked error: clear and report.
        if (r.rintsts().read().bits() & (1 << 12)) != 0 {
            r.rintsts().write(|w| unsafe { w.bits(1 << 12) });
            return Err(Error::Timeout);
        }
    }
    Err(Error::Timeout)
}

/// Masks interrupts, tears down the IDMAC and clears engine state.
fn abort_transfer() {
    let r = SDHOST::regs();
    r.intmask().write(|w| unsafe { w.bits(idle_intmask()) });
    r.idinten().write(|w| unsafe { w.bits(0) });
    disable_idmac();
    let _ = reset_transfer();
    STATE.engine.with(|eng| *eng = Engine::IDLE);
}

/// Awaits the terminal result the interrupt handler records.
fn wait_result() -> impl Future<Output = Result<(), Error>> {
    poll_fn(|cx| {
        STATE.engine.with(|eng| match eng.result.take() {
            Some(res) => Poll::Ready(res),
            None => {
                eng.waker.register(cx.waker());
                Poll::Pending
            }
        })
    })
}

/// Waits for the card to release DAT0 after a write via the Busy Clear
/// Interrupt.
///
/// The controller only raises the BCI (which shares the SBE status bit) for
/// data-write commands, and only when generation is enabled in `cardthrctl`;
/// the bit is left enabled solely for the duration of the wait so it cannot
/// be mistaken for a start-bit error during a multi-block transfer.
async fn wait_busy_async() -> Result<(), Error> {
    let r = SDHOST::regs();

    // Fast path: the card may already be ready.
    if !r.status().read().data_busy().bit_is_set() {
        return Ok(());
    }

    // Enable Busy Clear Interrupt generation, then arm and unmask it.
    r.cardthrctl().modify(|_, w| w.cardclrinten().set_bit());
    STATE.engine.with(|eng| {
        *eng = Engine {
            wait_busy: true,
            ..Engine::IDLE
        };
    });
    r.rintsts().write(|w| unsafe { w.bits(EVT_SBE) });
    r.intmask()
        .write(|w| unsafe { w.bits(idle_intmask() | EVT_SBE) });

    // Close the arm race: if busy cleared between the check above and the
    // unmask, the rising edge is already gone, so bail out instead of
    // waiting for an interrupt that will never fire.
    let res = if !r.status().read().data_busy().bit_is_set() {
        STATE.engine.with(|eng| *eng = Engine::IDLE);
        Ok(())
    } else {
        wait_result().await
    };

    // Restore: stop generating the BCI and re-mask the shared SBE bit.
    r.cardthrctl().modify(|_, w| w.cardclrinten().clear_bit());
    r.intmask().write(|w| unsafe { w.bits(idle_intmask()) });
    res
}

/// Polls DAT0 while the card signals busy (R1b).
///
/// Used after no-data commands (e.g. `CMD7`, `MMC_SWITCH`): the controller
/// does not generate a Busy Clear Interrupt for those, so polling is the
/// only option, mirroring ESP-IDF's `wait_for_busy_cleared`.
async fn wait_busy_poll() -> Result<(), Error> {
    for _ in 0..POLL_LIMIT {
        if !SDHOST::regs().status().read().data_busy().bit_is_set() {
            return Ok(());
        }
        yield_now().await;
    }
    Err(Error::Timeout)
}

/// Reads response registers into spec word order (`words[3]` is the MSW).
fn read_response(resp_len: ResponseLen) -> [u32; 4] {
    let r = SDHOST::regs();
    match resp_len {
        ResponseLen::None => [0; 4],
        ResponseLen::Short => [r.resp0().read().bits(), 0, 0, 0],
        ResponseLen::Long => [
            r.resp0().read().bits(),
            r.resp1().read().bits(),
            r.resp2().read().bits(),
            r.resp3().read().bits(),
        ],
    }
}

/// Maps `rintsts` error bits to an [`Error`]; errors take priority.
fn map_rintsts(sts: u32) -> Result<(), Error> {
    if sts & EVT_HLE != 0 {
        return Err(Error::HardwareLocked);
    }
    if sts & EVT_RTO != 0 {
        return Err(Error::ResponseTimeout);
    }
    if sts & EVT_RCRC != 0 {
        return Err(Error::ResponseCrc);
    }
    if sts & EVT_RESP_ERR != 0 {
        return Err(Error::ResponseError);
    }
    if sts & (EVT_DTO | EVT_HTO) != 0 {
        return Err(Error::DataTimeout);
    }
    if sts & (EVT_DCRC | EVT_EBE) != 0 {
        return Err(Error::DataCrc);
    }
    if sts & EVT_SBE != 0 {
        return Err(Error::StartBitError);
    }
    if sts & EVT_FRUN != 0 {
        return Err(Error::FifoOverrun);
    }
    Ok(())
}

/// Polls DAT0 until the card releases the busy signal.
fn wait_busy_cleared() -> Result<(), Error> {
    poll_until(|| !SDHOST::regs().status().read().data_busy().bit_is_set())
}

/// Validates a buffer address for the SDMMC IDMAC and returns its DMA pointer.
///
/// The IDMAC reaches PSRAM only on chips with `sdmmc_psram_dma`.
fn dma_ptr(addr: usize) -> Result<u32, Error> {
    if is_valid_ram_address(addr) {
        return Ok(addr as u32);
    }
    #[cfg(sdmmc_psram_dma)]
    if crate::soc::is_valid_psram_address(addr) {
        return Ok(addr as u32);
    }
    Err(Error::BufferNotDmaCapable)
}

/// Waits for the command response then the data phase, refilling the ring.
fn run_data_phase(
    t: &mut Transfer,
    ring: &mut [Desc; RING_LEN],
    write: bool,
    block_count: u32,
) -> Result<(), Error> {
    let r = SDHOST::regs();
    let consume = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR | EVT_HLE;

    // Command response phase.
    let cmd_done = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR;
    let mut sts = 0;
    let mut got = false;
    for _ in 0..POLL_LIMIT {
        sts = r.rintsts().read().bits();
        if sts & cmd_done != 0 {
            got = true;
            break;
        }
    }
    if !got {
        return Err(Error::Timeout);
    }
    map_rintsts(sts)?;
    r.rintsts().write(|w| unsafe { w.bits(consume) });

    // Data phase: wait for completion, refilling descriptors as the engine
    // frees them. No CPU-side iteration cap: a stalled card or host-side
    // FIFO starvation is terminated by the controller's data timeout
    // (DRTO/HTO), and a DMA fault by IDMAC FBE/DU, so the loop can only spin
    // while the transfer is genuinely making progress.
    let data_err = EVT_DCRC | EVT_DTO | EVT_HTO | EVT_SBE | EVT_EBE | EVT_FRUN;
    loop {
        let sts = r.rintsts().read().bits();
        if sts & data_err != 0 {
            map_rintsts(sts)?;
        }
        let id_sts = r.idsts().read();
        if id_sts.fbe().bit_is_set() || id_sts.du().bit_is_set() {
            return Err(Error::DmaError);
        }
        if t.remaining > 0 {
            let free = free_descriptors(ring, t.next_desc);
            if free > 0 {
                fill_descriptors(ring, t, free);
                r.pldmnd().write(|w| unsafe { w.bits(1) });
            }
        }
        if sts & EVT_DATA_OVER != 0 {
            break;
        }
    }

    // Multi-block transfers append an auto-stop; wait for its completion.
    // This follows DATA_OVER promptly, so a bounded wait is fine, but a
    // missing completion must be reported rather than silently ignored.
    if block_count > 1 {
        poll_until(|| r.rintsts().read().bits() & EVT_ACD != 0)?;
    }
    if write {
        wait_busy_cleared()?;
    }
    Ok(())
}

/// Counts descriptors the engine has released, starting at `next`.
fn free_descriptors(ring: &[Desc; RING_LEN], next: usize) -> usize {
    let mut count = 0;
    for i in 0..RING_LEN {
        let d = &ring[(next + i) % RING_LEN];
        if d.flags & DESC_OWN != 0 {
            break;
        }
        count += 1;
        if d.next == 0 {
            break;
        }
    }
    count
}

/// Fills up to `count` descriptors from the remaining transfer (shared
/// blocking/async refill; mirrors `sd_host_fill_dma_descriptors`).
fn fill_descriptors(ring: &mut [Desc; RING_LEN], t: &mut Transfer, count: usize) {
    for _ in 0..count {
        if t.remaining == 0 {
            break;
        }
        let i = t.next_desc;
        let size = t.remaining.min(DMA_MAX_BUF_LEN);
        let last = size == t.remaining;
        let next_ptr = if last {
            0
        } else {
            &ring[(i + 1) % RING_LEN] as *const Desc as u32
        };
        let first = ring[i].flags & DESC_FIRST;
        let d = &mut ring[i];
        d.flags = DESC_OWN | DESC_CHAINED | first | if last { DESC_LAST } else { 0 };
        d.sizes = ((size + 3) & !3) as u32;
        d.buf1 = t.ptr;
        d.next = next_ptr;
        t.remaining -= size;
        t.ptr += size as u32;
        t.next_desc = (i + 1) % RING_LEN;
    }
    // On write-back-cache SoCs the IDMAC fetches descriptors straight from
    // memory; flush the freshly written ring so it doesn't read stale lines.
    #[cfg(soc_internal_memory_cached)]
    unsafe {
        crate::soc::cache_writeback_addr(ring.as_ptr() as u32, core::mem::size_of_val(ring) as u32);
    }
}

fn reset_transfer() -> Result<(), Error> {
    let r = SDHOST::regs();
    r.ctrl().modify(|_, w| w.fifo_reset().set_bit());
    let res = poll_until(|| !r.ctrl().read().fifo_reset().bit_is_set());

    r.rintsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    r.idsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });

    res
}

/// Enables the IDMAC engine for a transfer and programs the descriptor base.
///
/// `BMOD.SWR` resets the IDMAC's internal registers (including `DBADDR`) on
/// this IP, so the descriptor base must be written *after* the reset,
/// matching the ordering ESP-IDF uses in `sdmmc_host_dma_prepare`.
fn enable_idmac(dbaddr: u32) {
    let r = SDHOST::regs();
    r.ctrl()
        .modify(|rd, w| unsafe { w.bits(rd.bits() | CTRL_DMA_ENABLE | CTRL_USE_INTERNAL_DMA) });
    r.bmod().modify(|_, w| w.swr().set_bit());
    r.idinten().write(|w| unsafe { w.bits(0) });
    r.dbaddr().write(|w| unsafe { w.bits(dbaddr) });
    r.bmod().modify(|_, w| {
        w.de().set_bit();
        w.fb().set_bit()
    });
}

/// Disables the IDMAC engine after a transfer.
fn disable_idmac() {
    let r = SDHOST::regs();
    r.ctrl()
        .modify(|rd, w| unsafe { w.bits(rd.bits() & !CTRL_USE_INTERNAL_DMA) });
    r.bmod().modify(|_, w| {
        w.de().clear_bit();
        w.fb().clear_bit()
    });
    r.ctrl().modify(|_, w| w.dma_reset().set_bit());
}

/// Module clock in Hz for a given source and divider.
fn module_hz(source: ClockSource, div: u8) -> u32 {
    let base = match source {
        ClockSource::Pll160m => 160_000_000,
        //#[cfg(esp32p4)]
        // ClockSource::Apll => unimplemented!(),
        #[cfg(not(esp32p4))]
        ClockSource::Xtal => 40_000_000,
    };
    base / (div as u32)
}

/// Computes the per-card divider for a target frequency.
///
/// `card_clk = module / (2 * div)`; `div == 0` bypasses the divider.
fn freq_to_card_div(module_hz: u32, target_hz: u32) -> u8 {
    if target_hz == 0 || module_hz <= target_hz {
        return 0;
    }
    let div = module_hz.div_ceil(2 * target_hz);
    div.clamp(1, 255) as u8
}

/// Sets the input sampling delay phase used for high-speed clocking.
///
/// The `cclkin_edge_sam_sel` encoding is non-linear; mirror esp-idf's
/// `sdmmc_ll_set_din_delay_phase` (0°, 90°, 180°, 270°).
#[cfg(esp32s3)]
fn set_input_delay_phase(phase: DelayPhase) {
    let v = match phase {
        DelayPhase::_0 => 0u8,
        DelayPhase::_1 => 1,
        DelayPhase::_2 => 4,
        DelayPhase::_3 => 6,
    };
    SDHOST::regs()
        .clk_edge_sel()
        .modify(|_, w| unsafe { w.cclkin_edge_sam_sel().bits(v) });
}
