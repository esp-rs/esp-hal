//! # Secure Digital / MultiMedia Card host (SDMMC / SDIO)
//!
//! ## Overview
//!
//! Driver for the SDMMC/SDIO host controller (`SDHOST`). The controller exposes
//! up to two independent card slots that share a single transfer engine.
//!
//! Blocking engine operations return [`BlockingError`]; [`BlockingError::Busy`]
//! if another slot or an async transfer holds the engine. Async ops await the
//! engine mutex instead. Bus width and card clock are cached and applied on
//! engine acquire (lock order: engine, then settings).

use core::{
    cell::UnsafeCell,
    future::poll_fn,
    marker::PhantomData,
    pin::Pin,
    sync::atomic::Ordering,
    task::{Context, Poll},
};

use embassy_futures::yield_now;
use embassy_sync::{mutex::MutexGuard, waitqueue::WakerRegistration};
use esp_sync::{NonReentrantMutex, RawMutex};
use portable_atomic::AtomicBool;
use procmacros::{BuilderLite, handler};
use sdio::{self as _, MmcError};

#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
use crate::dma::aligned::DmaAlignedRef;
#[cfg(sdmmc_has_gpio_matrix)]
use crate::gpio::{OutputSignal, PinGuard, Pull};
use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    dma::{
        DmaBufError,
        aligned::{DmaAlignedMut, InternalMemory},
    },
    gpio::{
        InputSignal,
        OutputConfig,
        interconnect::{self, PeripheralInput, PeripheralOutput},
    },
    peripherals::{Interrupt, SDHOST},
    private::DropGuard,
    system::{Peripheral, PeripheralGuard},
};

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(esp32p4, path = "esp32p4.rs")]
mod chip_specific;

/// Selects one of the controller's card slots.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlotId {
    /// Slot 0.
    _0,
    /// Slot 1.
    _1,
}

impl SlotId {
    /// Zero-based slot index.
    fn index(self) -> u8 {
        match self {
            SlotId::_0 => 0,
            SlotId::_1 => 1,
        }
    }
}

/// Card clock source feeding the controller's divider.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockSource {
    /// 160 MHz PLL.
    Pll160m,
    /// Crystal oscillator.
    #[cfg(not(esp32p4))]
    Xtal,
}

/// Clock input sampling phase used for high-speed tuning.
#[cfg(sdmmc_delay_phase_num_is_set)]
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BusWidth {
    /// 1-bit bus (DAT0 only).
    #[default]
    Bit1,
    /// 4-bit bus (DAT0–DAT3).
    Bit4,
    /// 8-bit bus (DAT0–DAT7, Card slot 0 only).
    Bit8,
}

/// Controller-wide (engine) configuration.
///
/// These settings drive the shared module clock and therefore apply to the
/// whole controller, not an individual slot.
#[derive(Clone, Copy, Debug, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// The clock source for the module clock.
    clock_source: ClockSource,

    /// The module-clock divider (valid range `2..=16`).
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
    /// Validates field ranges.
    fn validate(&self) -> Result<(), ConfigError> {
        if !(2..=16).contains(&self.module_div) {
            return Err(ConfigError::InvalidModuleDivider);
        }
        Ok(())
    }
}

/// Per-slot configuration.
#[derive(Clone, Copy, Debug, Default, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct SlotConfig {
    /// Input sampling delay phase used for high-speed tuning.
    #[cfg(sdmmc_delay_phase_num_is_set)]
    input_delay_phase: DelayPhase,

    /// Write-protect signal polarity (active-high or active-low).
    wp_active_high: bool,
}

/// Length of the response a command expects.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ResponseLen {
    /// No response.
    None,
    /// Short 48-bit response (`resp[0]`).
    Short,
    /// Long 136-bit response (`resp[0..4]`).
    Long,
}

/// Per-command engine flags.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
#[expect(clippy::enum_variant_names)]
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

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::Timeout => write!(f, "A hardware operation did not complete in time"),
            Error::ResponseCrc => write!(f, "Response CRC check failed"),
            Error::DataCrc => write!(f, "Data CRC or end-bit error"),
            Error::ResponseTimeout => write!(f, "Card did not respond to the command"),
            Error::DataTimeout => write!(f, "Data transfer timed out"),
            Error::FifoOverrun => write!(f, "FIFO under- or overrun during a transfer"),
            Error::StartBitError => write!(f, "Data start-bit error"),
            Error::HardwareLocked => write!(f, "Command could not be loaded (hardware locked)"),
            Error::ResponseError => write!(f, "Controller flagged a response error"),
            Error::DmaError => write!(f, "IDMAC transfer error"),
            Error::NoCard => write!(f, "No card present in the slot"),
            Error::BufferNotDmaCapable => {
                write!(f, "Buffer lies in a region the IDMAC cannot reach")
            }
            Error::Unsupported => write!(f, "Operation not supported"),
        }
    }
}

/// Error returned when applying a [`Config`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::InvalidModuleDivider => {
                write!(f, "Module-clock divider is outside 2..=16")
            }
            ConfigError::SlotInUse => write!(f, "The slot is already in use"),
            ConfigError::MissingClkOrCmd => write!(f, "The clock or command pin was not connected"),
            ConfigError::NoData0 => write!(f, "Data line 0 was not connected"),
        }
    }
}

/// Error from blocking engine operations ([`BlockingError::Busy`] is mutex contention, not CIU
/// HLE).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum BlockingError {
    /// The shared engine is held by another slot or an async transfer.
    Busy,
    /// The operation failed after the engine was acquired.
    Op(Error),
}

impl core::error::Error for BlockingError {}

impl core::fmt::Display for BlockingError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BlockingError::Busy => write!(f, "The shared SDMMC engine is busy"),
            BlockingError::Op(e) => write!(f, "{e}"),
        }
    }
}

impl From<Error> for BlockingError {
    fn from(error: Error) -> Self {
        BlockingError::Op(error)
    }
}

impl From<BlockingError> for MmcError {
    fn from(error: BlockingError) -> Self {
        match error {
            BlockingError::Busy => MmcError::Other,
            BlockingError::Op(e) => e.into(),
        }
    }
}

impl From<Error> for MmcError {
    fn from(error: Error) -> Self {
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

impl From<DmaBufError> for MmcError {
    fn from(error: DmaBufError) -> Self {
        warn!("{:?}", error);
        match error {
            DmaBufError::InvalidAlignment(_) => MmcError::Other,
            _ => MmcError::Io,
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

/// One contiguous DMA segment: `(buffer address, remaining bytes)`.
type Seg = (u32, usize);

/// Tracks progress while filling the descriptor ring.
///
/// A transfer is described as up to three back-to-back segments so that a
/// cache-unaligned caller buffer can still be DMA'd: the aligned middle is
/// transferred in place, while the unaligned head and tail are bounced
/// through an aligned scratch buffer. A fully aligned buffer uses a single
/// segment ([`Transfer::single`]) and the other two are left empty.
#[derive(Clone, Copy)]
struct Transfer {
    segs: [Seg; 3],
    /// Index of the segment currently being linked into descriptors.
    seg: usize,
    next_desc: usize,
}

impl Transfer {
    /// A single, already-DMA-capable contiguous segment.
    fn single(ptr: u32, len: usize) -> Self {
        Transfer {
            segs: [(ptr, len), (0, 0), (0, 0)],
            seg: 0,
            next_desc: 0,
        }
    }

    /// Head / middle / tail segments; any zero-length entry is skipped while
    /// filling descriptors.
    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    fn split(head: Seg, middle: Seg, tail: Seg) -> Self {
        Transfer {
            segs: [head, middle, tail],
            seg: 0,
            next_desc: 0,
        }
    }

    /// Bytes not yet linked into a descriptor. Before any descriptor is
    /// filled this equals the total transfer length.
    fn remaining(&self) -> usize {
        self.segs.iter().map(|(_, len)| *len).sum()
    }
}

/// Driver-owned descriptor ring in internal DMA-capable RAM.
struct DescRing(UnsafeCell<InternalMemory<[Desc; RING_LEN]>>);
// Access only while `ENGINE` is held.
unsafe impl Sync for DescRing {}

static DESC_RING: DescRing = DescRing(UnsafeCell::new(InternalMemory::new([Desc::ZERO; RING_LEN])));

fn ring() -> DmaAlignedMut<'static, [Desc; RING_LEN]> {
    unsafe { &mut *DESC_RING.0.get() }.get_mut()
}

// On SoCs with a data cache the IDMAC buffer must be cache-line aligned in
// both base address and length, otherwise a read invalidate would discard
// neighbouring data and a write would miss CPU-cached bytes. Control
// transfers (SCR = 8 B, SSR / SWITCH = 64 B) we cannot align, so they are
// bounced through an aligned scratch buffer. Two L2 cachelines cover every case.
#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
const BOUNCE_LEN: usize = 256;

// Scratch layout for a bounced, cache-unaligned caller buffer: the unaligned
// head occupies `[0, head_len)` and the unaligned tail
// `[BOUNCE_TAIL_OFF, BOUNCE_TAIL_OFF + tail_len)`. Each edge is shorter than
// the region's DMA alignment (<= 128 B), so the two never overlap and both fit
// within `BOUNCE_LEN`.
#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
const BOUNCE_TAIL_OFF: usize = BOUNCE_LEN / 2;

/// Per-transaction scratch in [`EngineSession`]. On cached SoCs it holds the
/// DMA bounce buffer; the engine mutex provides exclusive access.
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

/// Exclusive engine session: bounce scratch and last HW-programmed slot.
struct EngineSession {
    #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
    bounce: Bounce,
    active_slot: Option<SlotId>,
}

impl EngineSession {
    const INIT: Self = EngineSession {
        bounce: Bounce::INIT,
        active_slot: None,
    };

    fn select_and_apply(&mut self, slot: SlotId) -> Result<(), Error> {
        let wait = SETTINGS.with(|s| {
            let idx = slot.index() as usize;
            let cached = &mut s.slots[idx];
            if self.active_slot == Some(slot) && !cached.dirty {
                return Ok(false);
            }
            self.program_slot_hw(slot, cached, s.module)?;
            cached.dirty = false;
            self.active_slot = Some(slot);
            Ok(true)
        })?;

        if wait {
            crate::rom::ets_delay_us(10);
        }
        Ok(())
    }

    fn program_slot_hw(
        &mut self,
        slot: SlotId,
        cached: &SlotSettings,
        module: Config,
    ) -> Result<(), Error> {
        self.set_card_width(slot, cached.width);
        let card_div =
            freq_to_card_div(module_hz(module.clock_source, module.module_div), cached.hz);
        self.set_card_clock(slot, card_div)?;
        #[cfg(sdmmc_delay_phase_num_is_set)]
        if cached.hz > 25_000_000 {
            chip_specific::set_input_delay_phase(cached.input_delay_phase, cached.hz);
        }
        Ok(())
    }

    fn set_card_width(&mut self, id: SlotId, width: BusWidth) {
        let slot = id.index();
        SDHOST::regs().ctype().modify(|rd, w| unsafe {
            let mut w4 = rd.card_width4().bits() & !(1 << slot);
            let mut w8 = rd.card_width8().bits() & !(1 << slot);
            match width {
                BusWidth::Bit8 => w8 |= 1 << slot,
                BusWidth::Bit4 => w4 |= 1 << slot,
                BusWidth::Bit1 => {}
            }
            w.card_width4().bits(w4);
            w.card_width8().bits(w8)
        });
    }

    fn set_card_clock(&mut self, id: SlotId, card_div: u8) -> Result<(), Error> {
        let slot = id.index();
        let r = SDHOST::regs();

        r.clkena().modify(|rd, w| unsafe {
            w.cclk_enable().bits(rd.cclk_enable().bits() & !(1 << slot))
        });
        self.apply_clock_update(id)?;

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

        r.clkena().modify(|rd, w| unsafe {
            w.cclk_enable().bits(rd.cclk_enable().bits() | (1 << slot));
            w.lp_enable().bits(rd.lp_enable().bits() | (1 << slot))
        });
        self.apply_clock_update(id)
    }

    fn apply_clock_update(&mut self, id: SlotId) -> Result<(), Error> {
        let r = SDHOST::regs();
        r.cmdarg().write(|w| unsafe { w.bits(0) });
        r.cmd().write(|w| unsafe {
            w.update_clock_registers_only().set_bit();
            w.wait_prvdata_complete().set_bit();
            w.card_number().bits(id.index());
            w.start_cmd().set_bit()
        });
        wait_command_accepted()
    }

    fn send_init_sequence(&mut self, slot: SlotId) -> Result<(), Error> {
        let r = SDHOST::regs();
        r.rintsts().write(|w| unsafe { w.bits(EVT_CMD_DONE) });
        r.cmdarg().write(|w| unsafe { w.bits(0) });
        r.cmd().write(|w| unsafe {
            w.send_initialization().set_bit();
            w.wait_prvdata_complete().set_bit();
            w.card_number().bits(slot.index());
            w.start_cmd().set_bit()
        });
        wait_command_accepted()?;

        let result = poll_until(|| r.rintsts().read().bits() & EVT_CMD_DONE != 0);
        if result.is_ok() {
            r.rintsts().write(|w| unsafe { w.bits(EVT_CMD_DONE) });
        }
        result
    }

    fn send_command_blocking(
        &mut self,
        slot: SlotId,
        index: u8,
        arg: u32,
        resp_len: ResponseLen,
        check_crc: bool,
        flags: CommandFlags,
    ) -> Result<[u32; 4], Error> {
        let r = SDHOST::regs();
        let consume = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR | EVT_HLE;

        r.rintsts().write(|w| unsafe { w.bits(consume) });

        self.issue_command(slot, index, arg, resp_len, check_crc, flags)?;

        let done = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR;
        let mut sts = 0;
        poll_until(|| {
            sts = r.rintsts().read().bits();
            sts & done != 0
        })?;
        map_rintsts(sts)?;

        let resp = read_response(resp_len);
        r.rintsts().write(|w| unsafe { w.bits(consume) });

        if flags.busy {
            wait_busy_cleared()?;
        }
        Ok(resp)
    }

    fn transfer_blocking(
        &mut self,
        slot: SlotId,
        index: u8,
        arg: u32,
        write: bool,
        mut t: Transfer,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], Error> {
        let r = SDHOST::regs();

        reset_transfer()?;
        let total_len = t.remaining();
        r.blksiz().write(|w| unsafe { w.bits(block_size as u32) });
        r.bytcnt().write(|w| unsafe { w.bits(total_len as u32) });

        let mut ring = ring();
        *ring.reborrow().into_inner() = [Desc::ZERO; RING_LEN];
        ring[0].flags |= DESC_FIRST;
        fill_descriptors(ring.reborrow(), &mut t, RING_LEN);
        enable_idmac(ring.as_ptr() as u32);
        r.pldmnd().write(|w| unsafe { w.bits(1) });

        self.issue_data_command(slot, index, arg, write, block_count)?;

        let result = run_data_phase(&mut t, ring, write, needs_auto_stop(index, block_count));

        disable_idmac();
        r.rintsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });
        r.idsts().write(|w| unsafe { w.bits(0xFFFF_FFFF) });

        result?;
        Ok(read_response(ResponseLen::Short))
    }

    fn issue_command(
        &mut self,
        slot: SlotId,
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
            w.card_number().bits(slot.index());
            w.start_cmd().set_bit()
        });
        wait_command_accepted()
    }

    fn issue_data_command(
        &mut self,
        slot: SlotId,
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
            w.send_auto_stop().bit(needs_auto_stop(index, block_count));
            w.wait_prvdata_complete().set_bit();
            w.use_hole().set_bit();
            w.card_number().bits(slot.index());
            w.start_cmd().set_bit()
        });
        wait_command_accepted()
    }

    async fn send_command_async(
        &mut self,
        slot: SlotId,
        index: u8,
        arg: u32,
        resp_len: ResponseLen,
        check_crc: bool,
        flags: CommandFlags,
    ) -> Result<[u32; 4], Error> {
        let r = SDHOST::regs();
        let guard = DropGuard::new((), |()| abort_transfer());

        r.rintsts().write(|w| unsafe { w.bits(INTMASK_CMD) });
        arm_transfer(false, false, None);
        r.intmask()
            .write(|w| unsafe { w.bits(idle_intmask() | INTMASK_CMD) });

        self.issue_command(slot, index, arg, resp_len, check_crc, flags)?;
        wait_result().await?;
        let resp = read_response(resp_len);
        if flags.busy {
            wait_busy_poll().await?;
        }
        guard.defuse();
        Ok(resp)
    }

    async fn transfer_async(
        &mut self,
        slot: SlotId,
        index: u8,
        arg: u32,
        write: bool,
        mut t: Transfer,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], Error> {
        let guard = DropGuard::new((), |()| abort_transfer());
        let r = SDHOST::regs();

        reset_transfer()?;
        let total_len = t.remaining();
        r.blksiz().write(|w| unsafe { w.bits(block_size as u32) });
        r.bytcnt().write(|w| unsafe { w.bits(total_len as u32) });

        let mut ring = ring();
        *ring.reborrow().into_inner() = [Desc::ZERO; RING_LEN];
        ring[0].flags |= DESC_FIRST;
        fill_descriptors(ring.reborrow(), &mut t, RING_LEN);
        enable_idmac(ring.as_ptr() as u32);
        r.pldmnd().write(|w| unsafe { w.bits(1) });

        arm_transfer(true, needs_auto_stop(index, block_count), Some(t));
        r.idinten().write(|w| unsafe { w.bits(IDINTEN_ALL) });
        r.intmask()
            .write(|w| unsafe { w.bits(idle_intmask() | INTMASK_DATA) });

        self.issue_data_command(slot, index, arg, write, block_count)?;
        wait_result().await?;
        if write {
            wait_busy_async().await?;
        }
        disable_idmac();
        let resp = read_response(ResponseLen::Short);
        guard.defuse();
        Ok(resp)
    }

    async fn read_async(
        &mut self,
        slot: SlotId,
        index: u8,
        arg: u32,
        buf: &mut [u8],
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], MmcError> {
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        if let Some((head, tail)) = unaligned_split(buf.as_ptr() as usize, buf.len())
            && (head != 0 || tail != 0)
        {
            let len = buf.len();
            let middle_len = len - head - tail;

            let (head_ptr, middle_ptr, tail_ptr) = {
                let scratch = self.bounce.buf.get_mut();
                debug_assert!(head <= BOUNCE_TAIL_OFF && tail <= BOUNCE_LEN - BOUNCE_TAIL_OFF);
                scratch.invalidate();
                let head_ptr = dma_ptr_from_raw(scratch[..].as_ptr())?;
                let tail_ptr = dma_ptr_from_raw(scratch[BOUNCE_TAIL_OFF..].as_ptr())?;
                let middle_ptr = if middle_len != 0 {
                    dma_ptr_from_raw(unsafe { buf.as_ptr().add(head) })?
                } else {
                    0
                };
                (head_ptr, middle_ptr, tail_ptr)
            };

            let resp = self
                .transfer_async(
                    slot,
                    index,
                    arg,
                    false,
                    Transfer::split((head_ptr, head), (middle_ptr, middle_len), (tail_ptr, tail)),
                    block_size,
                    block_count,
                )
                .await?;

            if middle_len != 0 {
                unsafe { DmaAlignedMut::new_unchecked(&mut buf[head..head + middle_len]) }
                    .invalidate();
            }

            let scratch = self.bounce.buf.get_mut();
            scratch.invalidate();
            buf[..head].copy_from_slice(&scratch[..head]);
            buf[len - tail..].copy_from_slice(&scratch[BOUNCE_TAIL_OFF..BOUNCE_TAIL_OFF + tail]);
            return Ok(resp);
        }

        let mut dma = DmaAlignedMut::new(buf)?;
        let ptr = dma_ptr(dma.reborrow())?;
        let total = dma.len();
        let resp = self
            .transfer_async(
                slot,
                index,
                arg,
                false,
                Transfer::single(ptr, total),
                block_size,
                block_count,
            )
            .await?;
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        dma.invalidate();
        Ok(resp)
    }

    async fn write_async(
        &mut self,
        slot: SlotId,
        index: u8,
        arg: u32,
        buf: &[u8],
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], MmcError> {
        #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
        if let Some((head, tail)) = unaligned_split(buf.as_ptr() as usize, buf.len())
            && (head != 0 || tail != 0)
        {
            let len = buf.len();
            let middle_len = len - head - tail;

            let (head_ptr, middle_ptr, tail_ptr) = {
                let mut scratch = self.bounce.buf.get_mut();
                debug_assert!(head <= BOUNCE_TAIL_OFF && tail <= BOUNCE_LEN - BOUNCE_TAIL_OFF);
                scratch[..head].copy_from_slice(&buf[..head]);
                scratch[BOUNCE_TAIL_OFF..BOUNCE_TAIL_OFF + tail]
                    .copy_from_slice(&buf[len - tail..]);
                scratch.writeback();
                let head_ptr = dma_ptr_from_raw(scratch[..].as_ptr())?;
                let tail_ptr = dma_ptr_from_raw(scratch[BOUNCE_TAIL_OFF..].as_ptr())?;
                let middle_ptr = if middle_len != 0 {
                    let middle = &buf[head..head + middle_len];
                    unsafe { DmaAlignedRef::new_unchecked(middle) }.writeback();
                    dma_ptr_from_raw(middle.as_ptr())?
                } else {
                    0
                };
                (head_ptr, middle_ptr, tail_ptr)
            };

            return Ok(self
                .transfer_async(
                    slot,
                    index,
                    arg,
                    true,
                    Transfer::split((head_ptr, head), (middle_ptr, middle_len), (tail_ptr, tail)),
                    block_size,
                    block_count,
                )
                .await?);
        }

        let ptr = dma_ptr_from_raw(buf.as_ptr())?;
        Ok(self
            .transfer_async(
                slot,
                index,
                arg,
                true,
                Transfer::single(ptr, buf.len()),
                block_size,
                block_count,
            )
            .await?)
    }
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

/// ISR/task transfer handshake (distinct from CIU access via `EngineSession`).
struct TransferState {
    transfer: Option<Transfer>,
    result: Option<Result<(), Error>>,
    expect_data: bool,
    multiblock: bool,
    over_seen: bool,
    acd_seen: bool,
    wait_busy: bool,
    waker: WakerRegistration,
}

impl TransferState {
    const IDLE: Self = TransferState {
        transfer: None,
        result: None,
        expect_data: false,
        multiblock: false,
        over_seen: false,
        acd_seen: false,
        wait_busy: false,
        waker: WakerRegistration::new(),
    };
}

/// Persistent controller/slot settings consulted on slot selection.
struct Settings {
    module: Config,
    slots: [SlotSettings; 2],
}

impl Settings {
    const INIT: Self = Settings {
        module: Config {
            clock_source: ClockSource::Pll160m,
            module_div: 2,
        },
        slots: [SlotSettings::INIT; 2],
    };

    fn set_slot_bus(&mut self, slot: SlotId, width: BusWidth, hz: u32) -> Result<(), Error> {
        if hz == 0 || hz > 40_000_000 {
            return Err(Error::Unsupported);
        }
        let idx = slot.index() as usize;
        self.slots[idx].hz = hz;
        self.slots[idx].width = width;
        self.slots[idx].dirty = true;
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct SlotSettings {
    hz: u32,
    width: BusWidth,
    dirty: bool,
    #[cfg(sdmmc_delay_phase_num_is_set)]
    input_delay_phase: DelayPhase,
}

impl SlotSettings {
    const INIT: Self = SlotSettings {
        hz: 25_000_000,
        width: BusWidth::Bit1,
        dirty: true,
        #[cfg(sdmmc_delay_phase_num_is_set)]
        input_delay_phase: DelayPhase::_0,
    };
}

static ENGINE: embassy_sync::mutex::Mutex<RawMutex, EngineSession> =
    embassy_sync::mutex::Mutex::new(EngineSession::INIT);
static SETTINGS: NonReentrantMutex<Settings> = NonReentrantMutex::new(Settings::INIT);
static TRANSFER: NonReentrantMutex<TransferState> = NonReentrantMutex::new(TransferState::IDLE);

fn with_engine_try<R>(
    slot: SlotId,
    f: impl FnOnce(&mut EngineSession) -> Result<R, Error>,
) -> Result<R, BlockingError> {
    let mut session = ENGINE.try_lock().map_err(|_| BlockingError::Busy)?;
    session.select_and_apply(slot).map_err(BlockingError::Op)?;
    f(&mut session).map_err(BlockingError::Op)
}

async fn lock_engine(slot: SlotId) -> Result<MutexGuard<'static, RawMutex, EngineSession>, Error> {
    let mut guard = ENGINE.lock().await;
    guard.select_and_apply(slot)?;
    Ok(guard)
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
    /// Edge latch: set by the ISR when a card interrupt fires, consumed
    /// (swapped to `false`) by the waiter. Decouples the edge from the await.
    io_pending: AtomicBool,
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
            io_pending: AtomicBool::new(false),
            #[cfg(sdmmc_has_gpio_matrix)]
            pins: UnsafeCell::new(SlotPins::new()),
        }
    }
}

const SLOT_COUNT: usize = 2;
static SLOT_STATE: [SlotState; SLOT_COUNT] = [const { SlotState::new() }; SLOT_COUNT];

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
struct SlotInfo {
    io_event: u32,
    #[cfg(sdmmc_has_gpio_matrix)]
    clk_out: Option<OutputSignal>,
    #[cfg(sdmmc_has_gpio_matrix)]
    cmd_in: Option<InputSignal>,
    #[cfg(sdmmc_has_gpio_matrix)]
    cmd_out: Option<OutputSignal>,
    #[cfg(sdmmc_has_gpio_matrix)]
    data_in: &'static [InputSignal],
    #[cfg(sdmmc_has_gpio_matrix)]
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

#[cfg(sdmmc_has_gpio_matrix)]
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

                #[cfg(sdmmc_has_gpio_matrix)]
                clk_out: opt_out!($($clk)?),
                #[cfg(sdmmc_has_gpio_matrix)]
                cmd_in: opt_in!($($cmd_in)?),
                #[cfg(sdmmc_has_gpio_matrix)]
                cmd_out: opt_out!($($cmd_out)?),
                #[cfg(sdmmc_has_gpio_matrix)]
                data_in: &[ $(InputSignal::$data_in),* ],
                #[cfg(sdmmc_has_gpio_matrix)]
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

/// Interrupt mask kept armed while idle: card-detect plus the SDIO card
/// interrupt of any slot that is listening. Preserves listening across
/// transfers (the transfer paths OR their bits onto this base).
fn idle_intmask() -> u32 {
    SDHOST::regs().intmask().read().bits() & (EVT_IO_SLOT0 | EVT_IO_SLOT1) | INTMASK_IDLE
}

/// Arms `TRANSFER` for a new operation and clears stale status.
fn arm_transfer(expect_data: bool, multiblock: bool, transfer: Option<Transfer>) {
    TRANSFER.with(|ts| {
        *ts = TransferState {
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
    let pending = r.mintsts().read().bits();
    let idsts = r.idsts().read().bits();

    TRANSFER.with(|ts| {
        if ts.wait_busy {
            // For an R1b/no-data command the SBE bit doubles as the Busy
            // Clear Interrupt, fired when the card releases DAT0.
            if pending & EVT_SBE != 0 {
                ts.result = Some(Ok(()));
            }
        } else {
            // Refill descriptors the engine has released mid-transfer.
            if let Some(t) = ts.transfer.as_mut()
                && t.remaining() > 0
            {
                let mut ring = ring();
                let free = free_descriptors(ring.reborrow(), t.next_desc);
                if free > 0 {
                    fill_descriptors(ring.reborrow(), t, free);
                    r.pldmnd().write(|w| unsafe { w.bits(1) });
                }
            }

            if ts.result.is_none() {
                let err = map_rintsts(pending)
                    .err()
                    .or(if idsts & (IDSTS_FBE | IDSTS_DU) != 0 {
                        Some(Error::DmaError)
                    } else {
                        None
                    });
                if let Some(e) = err {
                    ts.result = Some(Err(e));
                } else if ts.expect_data {
                    if pending & EVT_DATA_OVER != 0 {
                        ts.over_seen = true;
                    }
                    if pending & EVT_ACD != 0 {
                        ts.acd_seen = true;
                    }
                    if ts.over_seen && (!ts.multiblock || ts.acd_seen) {
                        ts.result = Some(Ok(()));
                    }
                } else if pending & EVT_CMD_DONE != 0 {
                    ts.result = Some(Ok(()));
                }
            }
        }

        if ts.result.is_some() {
            ts.wait_busy = false;
            ts.transfer = None;
            r.intmask().write(|w| unsafe { w.bits(idle_intmask()) });
            r.idinten().write(|w| unsafe { w.bits(0) });
            ts.waker.wake();
        }
    });

    // SDIO card interrupt (edge-triggered): latch it and wake. The write-1-clear
    // of `rintsts` below disarms the edge, so it won't refire until the card
    // signals again; no need to mask. The waiter consumes the latch on wake.
    if pending & EVT_IO_SLOT0 != 0 {
        SLOT_STATE[0].io_pending.store(true, Ordering::Release);
        SLOT_STATE[0].io_waker.wake();
    }
    if pending & EVT_IO_SLOT1 != 0 {
        SLOT_STATE[1].io_pending.store(true, Ordering::Release);
        SLOT_STATE[1].io_waker.wake();
    }

    // Card-detect change: wake both slot listeners.
    if pending & EVT_CD != 0 {
        SLOT_STATE[0].cd_waker.wake();
        SLOT_STATE[1].cd_waker.wake();
    }

    // Write-1-clear the bits handled this pass.
    r.rintsts().write(|w| unsafe { w.bits(pending) });
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
        chip_specific::enable_sd_io_ldo();
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
        chip_specific::set_module_clock(config.clock_source, config.module_div);
        this.reset_engine();

        SETTINGS.with(|s| s.module = config);
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
    /// Both slots can be taken (once each); the shared engine serializes their
    /// transactions. Requesting the same slot twice returns
    /// [`ConfigError::SlotInUse`]. Blocking engine ops return
    /// [`BlockingError::Busy`] on contention.
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
        #[cfg(sdmmc_delay_phase_num_is_set)]
        SETTINGS.with(|s| {
            s.slots[idx].input_delay_phase = config.input_delay_phase;
            s.slots[idx].dirty = true;
        });
        Ok(Slot {
            config,
            data_pins: 0,
            clk_connected: false,
            cmd_connected: false,
            cd_connected: false,
            wp_connected: false,
            _guard: PeripheralGuard::new(Peripheral::SdioHost),
            power: None,
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

// Arms for functions absent on a given chip simply never match (the generated macro ignores
// unmatched functions). P4 wires only slot 0 (`SD1_*`) to fixed pads; slot 1 is matrix.
#[cfg(sdmmc_has_iomux)]
for_each_iomux_function! {
    (SD1_CLK, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotClk, $af, 0); };
    (SD1_CMD, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotCmd, $af, 0); };
    (SD1_DATA0, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 0); };
    (SD1_DATA1, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 1); };
    (SD1_DATA2, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 2); };
    (SD1_DATA3, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 3); };
    (SD1_DATA4, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 4); };
    (SD1_DATA5, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 5); };
    (SD1_DATA6, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 6); };
    (SD1_DATA7, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 0, 7); };

    (SD2_CLK, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotClk, $af, 1); };
    (SD2_CMD, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotCmd, $af, 1); };
    (SD2_DATA0, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 0); };
    (SD2_DATA1, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 1); };
    (SD2_DATA2, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 2); };
    (SD2_DATA3, $gpio:ident, $af:ident) => { impl_signal_trait!($gpio, SlotData, $af, 1, 3); };
}

/// A configured card slot, generic over the slot index and driver mode.
pub struct Slot<'d, const S: u8, Dm: DriverMode> {
    config: SlotConfig,
    data_pins: u8,
    clk_connected: bool,
    cmd_connected: bool,
    cd_connected: bool,
    wp_connected: bool,
    _guard: PeripheralGuard,
    power: Option<interconnect::OutputSignal<'d>>,
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

    /// Connects data line 4.
    pub fn with_data4(mut self, d4: impl SlotData<'d, S, 4>) -> Self {
        d4.configure();
        self.note_data_pin(5);
        self
    }

    /// Connects data line 5.
    pub fn with_data5(mut self, d5: impl SlotData<'d, S, 5>) -> Self {
        d5.configure();
        self.note_data_pin(6);
        self
    }

    /// Connects data line 6.
    pub fn with_data6(mut self, d6: impl SlotData<'d, S, 6>) -> Self {
        d6.configure();
        self.note_data_pin(7);
        self
    }

    /// Connects data line 7 (completes the 8-bit bus).
    pub fn with_data7(mut self, d7: impl SlotData<'d, S, 7>) -> Self {
        d7.configure();
        self.note_data_pin(8);
        self
    }

    /// Connects the card-detect input.
    pub fn with_card_detect(mut self, cd: impl PeripheralInput<'d>) -> Self {
        let pin = cd.into();
        pin.set_input_enable(true);
        slot_info(slot_id(S)).cd_in.unwrap().connect_to(&pin);
        self.cd_connected = true;
        self
    }

    /// Connects the write-protect input.
    pub fn with_write_protect(mut self, wp: impl PeripheralInput<'d>) -> Self {
        let pin = wp.into();
        pin.set_input_enable(true);
        slot_info(slot_id(S)).wp_in.unwrap().connect_to(&pin);
        self.wp_connected = true;
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
        if !self.cd_connected {
            return true;
        }
        (SDHOST::regs().cdetect().read().card_detect_n().bits() & (1 << S)) == 0
    }

    /// Returns `true` if the card reports write protection. Returns `false`
    /// if no write-protect pin is wired. Polarity follows
    /// [`SlotConfig::with_wp_active_high`].
    pub fn is_write_protected(&self) -> bool {
        if !self.wp_connected {
            return false;
        }
        let level = (SDHOST::regs().wrtprt().read().write_protect().bits() & (1 << S)) != 0;
        level == self.config.wp_active_high
    }

    /// Caches bus width and card clock; HW is programmed on the next engine op.
    pub fn set_bus_low_level(&mut self, width: BusWidth, hz: u32) -> Result<(), Error> {
        SETTINGS.with(|s| s.set_slot_bus(slot_id(S), width, hz))
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
    pub fn send_init_sequence(&mut self) -> Result<(), BlockingError> {
        with_engine_try(slot_id(S), |session| session.send_init_sequence(slot_id(S)))
    }

    /// Issues a no-data command and returns its raw response words.
    pub fn command_blocking(
        &mut self,
        index: u8,
        arg: u32,
        resp_len: ResponseLen,
        check_crc: bool,
        flags: CommandFlags,
    ) -> Result<[u32; 4], BlockingError> {
        if !self.is_card_present() {
            return Err(BlockingError::Op(Error::NoCard));
        }
        let slot = slot_id(S);
        with_engine_try(slot, |session| {
            session.send_command_blocking(slot, index, arg, resp_len, check_crc, flags)
        })
    }

    /// Reads `block_count` blocks into a DMA-capable buffer (CMD17/CMD18).
    pub fn read_blocks_blocking(
        &mut self,
        cmd_index: u8,
        arg: u32,
        mut buf: DmaAlignedMut<'_, [u8]>,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], BlockingError> {
        let slot = slot_id(S);
        with_engine_try(slot, |session| {
            let total = buf.len();
            let ptr = dma_ptr(buf.reborrow())?;
            let resp = session.transfer_blocking(
                slot,
                cmd_index,
                arg,
                false,
                Transfer::single(ptr, total),
                block_size,
                block_count,
            )?;
            #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
            buf.invalidate();
            Ok(resp)
        })
    }

    /// Writes `block_count` blocks from a DMA-capable buffer (CMD24/CMD25).
    pub fn write_blocks_blocking(
        &mut self,
        cmd_index: u8,
        arg: u32,
        mut buf: DmaAlignedMut<'_, [u8]>,
        block_size: u16,
        block_count: u32,
    ) -> Result<[u32; 4], BlockingError> {
        let slot = slot_id(S);
        with_engine_try(slot, |session| {
            #[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
            buf.writeback();
            let total = buf.len();
            let ptr = dma_ptr(buf.reborrow())?;
            session.transfer_blocking(
                slot,
                cmd_index,
                arg,
                true,
                Transfer::single(ptr, total),
                block_size,
                block_count,
            )
        })
    }

    fn note_data_pin(&mut self, count: u8) {
        self.data_pins = self.data_pins.max(count);
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
            config: self.config,
            data_pins: self.data_pins,
            clk_connected: self.clk_connected,
            cmd_connected: self.cmd_connected,
            _guard: self._guard,
            power: self.power,
            cd_connected: self.cd_connected,
            wp_connected: self.wp_connected,
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
            config: self.config,
            data_pins: self.data_pins,
            clk_connected: self.clk_connected,
            cmd_connected: self.cmd_connected,
            _guard: self._guard,
            power: self.power,
            cd_connected: self.cd_connected,
            wp_connected: self.wp_connected,
            _pd: PhantomData,
        }
    }

    /// Waits for the SDIO card interrupt (card pulls DAT1 low).
    pub fn wait_for_sdio_interrupt(&mut self) -> impl Future<Output = ()> {
        let slot_id = slot_id(S);
        let slot = slot_id.index() as usize;
        let bit = slot_info(slot_id).io_event;

        WaitForInterruptFuture { slot, bit }
    }

    /// Issues a control command, mapping protocol types to the engine core.
    async fn cmd_async<'a, C: sdio::ControlCommand + 'a>(
        &mut self,
        session: &mut EngineSession,
        cmd: C,
    ) -> Result<C::Resp<'a>, MmcError> {
        let slot = slot_id(S);
        let resp_len = match <C::Resp<'a> as sdio::Response>::LEN {
            sdio::ResponseLen::Zero => ResponseLen::None,
            sdio::ResponseLen::R48 => ResponseLen::Short,
            sdio::ResponseLen::R136 => ResponseLen::Long,
        };
        let flags = CommandFlags {
            wait_complete: !is_stop_or_abort(C::INDEX),
            stop_abort: is_stop_or_abort(C::INDEX),
            busy: <C::Resp<'a> as sdio::Response>::BUSY,
        };
        let crc = <C::Resp<'a> as sdio::Response>::CRC;
        let words = session
            .send_command_async(slot, C::INDEX, cmd.arg(), resp_len, crc, flags)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }
}

impl<'d, const S: u8> sdio::MmcBus for Slot<'d, S, Async> {
    async fn wait_for_event(&mut self) -> Result<(), MmcError> {
        self.wait_for_sdio_interrupt().await;
        Ok(())
    }

    async fn send_command<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: sdio::ControlCommand + 'a,
    {
        let mut session = lock_engine(slot_id(S)).await?;
        self.cmd_async(&mut session, cmd).await
    }

    async fn read_blocks<'a, C>(&mut self, mut cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: sdio::BlockReadCommand + 'a,
    {
        let mut session = lock_engine(slot_id(S)).await?;
        let slot = slot_id(S);
        let (bs, bc, arg) = (cmd.block_size().len() as u16, cmd.block_count(), cmd.arg());
        let words = session
            .read_async(slot, C::INDEX, arg, cmd.buf(), bs, bc)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn write_blocks<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: sdio::BlockWriteCommand + 'a,
    {
        let mut session = lock_engine(slot_id(S)).await?;
        let slot = slot_id(S);
        let (bs, bc, arg) = (cmd.block_size().len() as u16, cmd.block_count(), cmd.arg());
        let words = session
            .write_async(slot, C::INDEX, arg, cmd.buf(), bs, bc)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn read_bytes<'a, C>(&mut self, mut cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: sdio::ByteReadCommand + 'a,
    {
        let mut session = lock_engine(slot_id(S)).await?;
        let slot = slot_id(S);
        let (n, arg) = (cmd.byte_count(), cmd.arg());
        let words = session
            .read_async(slot, C::INDEX, arg, cmd.buf(), n as u16, 1)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn write_bytes<'a, C>(&mut self, cmd: C) -> Result<C::Resp<'a>, MmcError>
    where
        C: sdio::ByteWriteCommand + 'a,
    {
        let mut session = lock_engine(slot_id(S)).await?;
        let slot = slot_id(S);
        let (n, arg) = (cmd.byte_count(), cmd.arg());
        let words = session
            .write_async(slot, C::INDEX, arg, cmd.buf(), n as u16, 1)
            .await?;
        Ok(<C::Resp<'a> as sdio::Response>::from_words(&words))
    }

    async fn init_idle(&mut self, hz: u32) -> Result<(), MmcError> {
        let mut session = lock_engine(slot_id(S)).await?;
        self.validate_pins().map_err(|_| MmcError::Other)?;
        self.set_bus_low_level(BusWidth::Bit1, hz)?;
        session.send_init_sequence(slot_id(S))?;
        Ok(())
    }

    fn set_bus(&mut self, width: sdio::BusWidth, hz: u32) -> Result<(), MmcError> {
        let w = match width {
            sdio::BusWidth::W1 => BusWidth::Bit1,
            sdio::BusWidth::W4 => BusWidth::Bit4,
            sdio::BusWidth::W8 => BusWidth::Bit8,
        };
        if matches!(w, BusWidth::Bit4) && self.data_pins < 4 {
            return Err(MmcError::Unsupported);
        }
        if matches!(w, BusWidth::Bit8) && self.data_pins < 8 {
            return Err(MmcError::Unsupported);
        }
        if hz > 40_000_000 {
            return Err(MmcError::Unsupported);
        }
        self.set_bus_low_level(w, hz)?;
        Ok(())
    }

    fn supports_mmc(&self) -> bool {
        // Native parallel SD/MMC host (not SPI mode).
        true
    }

    fn supports_bus_width(&self) -> sdio::BusWidth {
        if self.data_pins >= 8 {
            sdio::BusWidth::W8
        } else if self.data_pins >= 4 {
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

/// Whether the controller should auto-issue CMD12 (STOP_TRANSMISSION) after the
/// data phase.
///
/// Auto-stop is only valid for the open-ended SD/MMC memory multi-block
/// transfers (CMD18 `READ_MULTIPLE_BLOCK` / CMD25 `WRITE_MULTIPLE_BLOCK`).
/// SDIO CMD53 (`IO_RW_EXTENDED`) carries its own block count in the command and
/// must NOT be followed by CMD12: the card would not respond to it, surfacing as
/// a spurious `ResponseTimeout` at the end of the data phase.
fn needs_auto_stop(index: u8, block_count: u32) -> bool {
    block_count > 1 && matches!(index, 18 | 25)
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

/// Spins until the CIU accepts the command (`start_cmd` self-clears).
fn wait_command_accepted() -> Result<(), Error> {
    let r = SDHOST::regs();
    for _ in 0..POLL_LIMIT {
        if !r.cmd().read().start_cmd().bit_is_set() {
            return Ok(());
        }
        // Hardware-locked error: clear and report.
        const HW_LOCKED: u32 = 1 << 12;
        if (r.rintsts().read().bits() & HW_LOCKED) != 0 {
            r.rintsts().write(|w| unsafe { w.bits(HW_LOCKED) });
            return Err(Error::Timeout);
        }
    }
    Err(Error::Timeout)
}

/// Masks interrupts, tears down the IDMAC and clears transfer state.
fn abort_transfer() {
    let r = SDHOST::regs();
    r.intmask().write(|w| unsafe { w.bits(idle_intmask()) });
    r.idinten().write(|w| unsafe { w.bits(0) });
    disable_idmac();
    let _ = reset_transfer();
    TRANSFER.with(|ts| *ts = TransferState::IDLE);
}

/// Awaits the terminal result the interrupt handler records.
fn wait_result() -> impl Future<Output = Result<(), Error>> {
    poll_fn(|cx| {
        TRANSFER.with(|ts| match ts.result.take() {
            Some(res) => Poll::Ready(res),
            None => {
                ts.waker.register(cx.waker());
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
    TRANSFER.with(|ts| {
        *ts = TransferState {
            wait_busy: true,
            ..TransferState::IDLE
        };
    });
    r.rintsts().write(|w| unsafe { w.bits(EVT_SBE) });
    r.intmask()
        .write(|w| unsafe { w.bits(idle_intmask() | EVT_SBE) });

    // Close the arm race: if busy cleared between the check above and the
    // unmask, the rising edge is already gone, so bail out instead of
    // waiting for an interrupt that will never fire.
    let res = if !r.status().read().data_busy().bit_is_set() {
        TRANSFER.with(|ts| *ts = TransferState::IDLE);
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
fn dma_ptr(buf: DmaAlignedMut<'_, [u8]>) -> Result<u32, Error> {
    dma_ptr_from_raw(buf.as_ptr())
}

fn dma_ptr_from_raw(addr: *const u8) -> Result<u32, Error> {
    // Not in some weird region like flash or RTC memory.
    if crate::soc::is_valid_ram_address(addr as usize) {
        return Ok(addr as u32);
    }
    #[cfg(all(soc_has_psram, not(sdmmc_psram_dma)))]
    if crate::soc::is_valid_psram_address(addr as usize) {
        return Ok(addr as u32);
    }

    Err(Error::BufferNotDmaCapable)
}

/// Splits a caller buffer at `addr` of length `len` into the lengths of its
/// cache-unaligned head and tail. The middle slice `[head, len - tail)` is
/// aligned in both address and size for in-place DMA, while `head` and `tail`
/// (each shorter than the region's DMA alignment) must be bounced through the
/// aligned scratch buffer and copied.
///
/// Returns `None` if the buffer is not in a DMA-capable region; the caller
/// then falls back to the validating direct path which reports the error.
#[cfg(any(soc_internal_memory_cached, dma_can_access_psram))]
fn unaligned_split(addr: usize, len: usize) -> Option<(usize, usize)> {
    let align = crate::dma::aligned::region_dma_alignment(addr)?;
    let head = ((align - addr % align) % align).min(len);
    let tail = (len - head) % align;
    Some((head, tail))
}

/// Waits for the command response then the data phase, refilling the ring.
fn run_data_phase(
    t: &mut Transfer,
    mut ring: DmaAlignedMut<'_, [Desc; RING_LEN]>,
    write: bool,
    auto_stop: bool,
) -> Result<(), Error> {
    let r = SDHOST::regs();
    let consume = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR | EVT_HLE;

    // Command response phase.
    let done = EVT_CMD_DONE | EVT_RTO | EVT_RCRC | EVT_RESP_ERR;
    let mut sts = 0;
    poll_until(|| {
        sts = r.rintsts().read().bits();
        sts & done != 0
    })?;
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
        if t.remaining() > 0 {
            let free = free_descriptors(ring.reborrow(), t.next_desc);
            if free > 0 {
                fill_descriptors(ring.reborrow(), t, free);
                r.pldmnd().write(|w| unsafe { w.bits(1) });
            }
        }
        if sts & EVT_DATA_OVER != 0 {
            break;
        }
    }

    // Open-ended memory multi-block transfers append an auto-stop (CMD12);
    // wait for its completion. This follows DATA_OVER promptly, so a bounded
    // wait is fine, but a missing completion must be reported rather than
    // silently ignored. SDIO CMD53 sends no CMD12, so there is nothing to wait
    // for.
    if auto_stop {
        poll_until(|| r.rintsts().read().bits() & EVT_ACD != 0)?;
    }
    if write {
        wait_busy_cleared()?;
    }
    Ok(())
}

/// Counts descriptors the engine has released, starting at `next`.
fn free_descriptors(ring: DmaAlignedMut<'_, [Desc; RING_LEN]>, next: usize) -> usize {
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
fn fill_descriptors(mut ring: DmaAlignedMut<'_, [Desc; RING_LEN]>, t: &mut Transfer, count: usize) {
    for _ in 0..count {
        // Skip exhausted (or empty) segments to find the next bytes to link.
        while t.seg < t.segs.len() && t.segs[t.seg].1 == 0 {
            t.seg += 1;
        }
        if t.seg >= t.segs.len() {
            break;
        }

        let (ptr, rem) = t.segs[t.seg];
        let i = t.next_desc;
        let size = rem.min(DMA_MAX_BUF_LEN);
        // The transfer ends here only if this chunk drains the current segment
        // and no later segment still has data to link.
        let exhausts_seg = size == rem;
        let later_data = t.segs[t.seg + 1..].iter().any(|(_, len)| *len > 0);
        let last = exhausts_seg && !later_data;
        let next_ptr = if last {
            0
        } else {
            &ring[(i + 1) % RING_LEN] as *const Desc as u32
        };
        let first = ring[i].flags & DESC_FIRST;
        let d = &mut ring[i];
        d.flags = DESC_OWN | DESC_CHAINED | first | if last { DESC_LAST } else { 0 };
        d.sizes = ((size + 3) & !3) as u32;
        d.buf1 = ptr;
        d.next = next_ptr;
        t.segs[t.seg].0 = ptr + size as u32;
        t.segs[t.seg].1 = rem - size;
        t.next_desc = (i + 1) % RING_LEN;
    }

    #[cfg(soc_internal_memory_cached)]
    ring.writeback();
}

fn reset_transfer() -> Result<(), Error> {
    let r = SDHOST::regs();
    r.ctrl().modify(|_, w| w.fifo_reset().set_bit());
    let res = poll_until(|| !r.ctrl().read().fifo_reset().bit_is_set());

    r.rintsts()
        .write(|w| unsafe { w.bits(!(EVT_IO_SLOT0 | EVT_IO_SLOT1)) });
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

struct WaitForInterruptFuture {
    slot: usize,
    bit: u32,
}

impl Future for WaitForInterruptFuture {
    type Output = ();
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Register before consuming so an edge racing the check wakes us.
        SLOT_STATE[self.slot].io_waker.register(cx.waker());

        SDHOST::regs()
            .intmask()
            .modify(|rd, w| unsafe { w.bits(rd.bits() | self.bit) });

        if SLOT_STATE[self.slot]
            .io_pending
            .swap(false, Ordering::AcqRel)
        {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Drop for WaitForInterruptFuture {
    fn drop(&mut self) {
        SDHOST::regs()
            .intmask()
            .modify(|rd, w| unsafe { w.bits(rd.bits() & !self.bit) });
    }
}
