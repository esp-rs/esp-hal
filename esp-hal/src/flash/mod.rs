#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Internal SPI flash (FLASH)
//!
//! ## Overview
//!
//! Blocking access to the chip's attached SPI flash. The driver owns the
//! virtual [`FLASH`] peripheral.
//!
//! Write and erase are NOR-flash operations: bits can only change from 1 to 0
//! without an erase. There is no read-modify-write. [`Flash::read`] and
//! [`Flash::write`] require a 4-byte-aligned flash offset, length, and buffer
//! in internal RAM. [`Flash::erase`] requires 4096-byte alignment.
//!
//! ## Configuration
//!
//! [`Config`] currently only selects the multi-core strategy on dual-core
//! chips (default: automatically park the other core).
//!
//! ## Examples
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::flash::{Config, Flash};
//!
//! #[repr(align(4))]
//! struct Aligned([u8; 32]);
//!
//! let mut flash = Flash::new(peripherals.FLASH, Config::default())?;
//! let mut buf = Aligned([0u8; 32]);
//! flash.read(0x10_020, &mut buf.0)?;
//! # {after_snippet}
//! ```
//!
//! ## Limitations
#![cfg_attr(
    esp32,
    doc = "- On ESP32, a second-stage bootloader must identify the flash chip; the ROM does not.
  [`Flash::new`] fails with [`ConfigError::UnknownFlashChip`] if identification is missing."
)]
//! - [`Flash::write`] and [`Flash::erase`] do not refuse currently mapped flash.
//! - Programming a mapped `.text` or `.rodata` page can destroy the running image or change memory
//!   the compiler treats as immutable.
//! - On dual-core chips, the default strategy stalls the other CPU around every operation,
//!   including reads. The other core may be frozen while holding a lock or inside an interrupt
//!   handler.

use core::marker::PhantomData;

use procmacros::{BuilderLite, ram};
#[cfg(xtensa)]
use xtensa_lx::interrupt::free;

#[cfg(not(xtensa))]
use crate::interrupt::free;
use crate::{Blocking, DriverMode, peripherals::FLASH, soc::is_slice_in_dram};

mod cache;
mod rom;

/// Word size required by the read and write paths, in bytes.
const WORD_SIZE: u32 = 4;
/// Program page size in bytes.
const PAGE_SIZE: u32 = 256;
/// Erase sector size in bytes.
const SECTOR_SIZE: u32 = 4096;
/// Erase block size in bytes.
const BLOCK_SIZE: u32 = 65536;

/// Flash driver configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// How to treat the other CPU during a flash operation.
    ///
    /// Default: [`MultiCoreStrategy::AutoPark`].
    ///
    /// [`MultiCoreStrategy::ignore`] is only safe when the other core cannot
    /// fetch from flash for the duration of the operation.
    #[cfg(multi_core)]
    #[builder_lite(unstable)]
    multi_core_strategy: MultiCoreStrategy,
}

/// Strategy for the other CPU during flash operations.
#[cfg(multi_core)]
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MultiCoreStrategy {
    /// Fail with [`Error::OtherCoreRunning`] if the other core is running.
    Error,
    /// Park the other core for the operation and unpark it afterwards.
    ///
    /// The stall can freeze the other core at any instruction. It may hold a
    /// lock or be inside an interrupt handler. Parking uses the CPU-control
    /// registers even if the application already owns
    /// [`crate::peripherals::CPU_CTRL`].
    #[default]
    AutoPark,
    /// Do not check or park the other core.
    ///
    /// Construct with [`Self::ignore`].
    Ignore(IgnoreMarker),
}

/// Marker so [`MultiCoreStrategy::Ignore`] can only be built via
/// [`MultiCoreStrategy::ignore`].
#[cfg(multi_core)]
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IgnoreMarker {
    _private: (),
}

#[cfg(multi_core)]
impl MultiCoreStrategy {
    /// Do not check or park the other core.
    ///
    /// # Safety
    ///
    /// The other core must not fetch instructions or data from flash for the
    /// duration of every operation, including reads. Flash-backed caches are
    /// unavailable while an operation runs.
    #[instability::unstable]
    pub const unsafe fn ignore() -> Self {
        Self::Ignore(IgnoreMarker { _private: () })
    }
}

/// Error constructing or configuring the flash driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The attached flash chip could not be identified, or its size is not
    /// recognized.
    UnknownFlashChip,
}

impl core::error::Error for ConfigError {}

impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnknownFlashChip => {
                write!(
                    f,
                    "Flash chip could not be identified, or its size is not recognized"
                )
            }
        }
    }
}

/// Flash operation error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "matches ROM result / issue 6203")]
#[non_exhaustive]
pub enum Error {
    /// I/O error.
    IoError,
    /// The operation timed out.
    IoTimeout,
    /// The chip status bits still protect the target range.
    Locked,
    /// Address, length, or buffer pointer is not aligned for this operation.
    ///
    /// [`Flash::read`] and [`Flash::write`] require a 4-byte-aligned flash
    /// offset, length, and buffer pointer. [`Flash::erase`] requires a
    /// 4096-byte range.
    NotAligned,
    /// The range exceeds the detected flash capacity, or `from` > `to`.
    OutOfBounds,
    /// Not supported in the current environment.
    ///
    /// Returned when the buffer is not in internal RAM.
    NotSupported,
    /// The other core is running and the configured strategy is
    /// [`MultiCoreStrategy::Error`].
    #[cfg(all(multi_core, feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    OtherCoreRunning,
    /// Unexpected ROM status code (logged).
    Unknown,
}

impl core::error::Error for Error {}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::IoError => write!(f, "Flash I/O error"),
            Self::IoTimeout => write!(f, "Flash I/O timed out"),
            Self::Locked => write!(f, "Flash is locked for writing"),
            Self::NotAligned => write!(f, "Flash address, length, or buffer is not aligned"),
            Self::OutOfBounds => write!(f, "Flash range is out of bounds"),
            Self::NotSupported => write!(f, "Flash operation is not supported"),
            #[cfg(all(multi_core, feature = "unstable"))]
            Self::OtherCoreRunning => write!(f, "The other CPU core is running"),
            Self::Unknown => write!(f, "Unknown flash error"),
        }
    }
}

/// Identification of the attached flash chip.
///
/// `chip_id` is manufacturer-first (manufacturer in bits 23:16). Geometry is
/// fixed: 256-byte pages, 4 KiB sectors and 64 KiB blocks.
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ChipInfo {
    /// JEDEC id, manufacturer in bits 23:16.
    pub chip_id: u32,
    /// Detected physical capacity in bytes.
    pub capacity: u32,
    /// Erase sector size in bytes.
    pub sector_size: u32,
    /// Erase block size in bytes.
    pub block_size: u32,
    /// Program page size in bytes.
    pub page_size: u32,
}

/// Internal SPI flash driver.
///
/// Only [`Blocking`] mode is constructible.
#[derive(Debug)]
pub struct Flash<'d, Dm: DriverMode> {
    _flash: FLASH<'d>,
    capacity: usize,
    chip_id: u32,
    unlocked: bool,
    #[cfg(multi_core)]
    multi_core_strategy: MultiCoreStrategy,
    _mode: PhantomData<Dm>,
}

impl<'d> Flash<'d, Blocking> {
    /// Creates a new flash driver from the `FLASH` peripheral.
    ///
    /// Capacity is the detected chip size, not the size field in the image
    /// header.
    #[cfg_attr(esp32, doc = "# Limitations")]
    #[cfg_attr(
        esp32,
        doc = "On ESP32, a second-stage bootloader must identify the flash chip; the
ROM does not. Without that, this method returns
[`ConfigError::UnknownFlashChip`]."
    )]
    pub fn new(flash: FLASH<'d>, config: Config) -> Result<Self, ConfigError> {
        let raw_id = rom::cached_device_id();
        let capacity = rom::capacity_from_cached_id(raw_id)?;
        let chip_id = raw_id & 0x00FF_FFFF;
        let mut this = Self {
            _flash: flash,
            capacity,
            chip_id,
            unlocked: false,
            #[cfg(multi_core)]
            multi_core_strategy: config.multi_core_strategy,
            _mode: PhantomData,
        };
        let _ = config;
        this.apply_config(&config)?;
        Ok(this)
    }

    /// Applies a new configuration.
    ///
    /// On dual-core chips this updates the multi-core strategy. On single-core
    /// chips it is a no-op.
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        #[cfg(multi_core)]
        {
            self.multi_core_strategy = config.multi_core_strategy;
        }
        let _ = config;
        Ok(())
    }

    /// Returns the detected physical flash capacity in bytes.
    ///
    /// This is the chip size identified at construction, not the size field in
    /// the image header.
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Returns chip identification and fixed geometry.
    ///
    /// `chip_id` is manufacturer-first. Geometry is always 256-byte pages,
    /// 4096-byte sectors and 64 KiB blocks; it is not probed from the chip.
    #[instability::unstable]
    pub fn chip_info(&self) -> ChipInfo {
        ChipInfo {
            chip_id: self.chip_id,
            capacity: self.capacity as u32,
            sector_size: SECTOR_SIZE,
            block_size: BLOCK_SIZE,
            page_size: PAGE_SIZE,
        }
    }

    /// Reads `data.len()` bytes starting at `offset`.
    ///
    /// `offset` and `data.len()` must be multiples of 4. `data` must be in
    /// internal RAM and word-aligned.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotAligned`] if `offset`, `data.len()`, or the buffer
    /// pointer is not a multiple of 4, [`Error::OutOfBounds`] if the range
    /// exceeds [`Self::capacity`], or [`Error::NotSupported`] if `data` is not
    /// in internal RAM.
    #[ram]
    pub fn read(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Error> {
        if data.is_empty() {
            return self.check_bounds(offset, 0);
        }

        self.check_alignment(WORD_SIZE, offset, data.len())?;
        self.check_bounds(offset, data.len())?;
        check_buffer(data)?;
        self.read_chunked(offset, data)
    }

    /// Writes `data` starting at `offset` using NOR flash semantics.
    ///
    /// The target must already be erased. `offset` and `data.len()` must be
    /// multiples of 4. `data` must be in internal RAM and word-aligned.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotAligned`] if `offset`, `data.len()`, or the buffer
    /// pointer is not a multiple of 4, [`Error::OutOfBounds`] if the range
    /// exceeds [`Self::capacity`], or [`Error::NotSupported`] if `data` is not
    /// in internal RAM.
    ///
    /// # Safety
    ///
    /// The programmed range must not be mapped for instruction fetch or as
    /// immutable data. Writing a mapped `.text` or `.rodata` page overwrites
    /// the running image, or mutates `static` / `.rodata` the compiler treats
    /// as immutable.
    #[ram]
    pub unsafe fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Error> {
        if data.is_empty() {
            return self.check_bounds(offset, 0);
        }

        self.check_alignment(WORD_SIZE, offset, data.len())?;
        self.check_bounds(offset, data.len())?;
        check_buffer(data)?;
        self.ensure_unlocked()?;
        self.write_chunked(offset, data)
    }

    /// Erases flash in `[from, to)`.
    ///
    /// `from` and `to` must be multiples of 4096 bytes ([`ChipInfo::sector_size`]).
    /// `to == capacity` is allowed. There is no byte-granular erase.
    ///
    /// # Errors
    ///
    /// Returns [`Error::NotAligned`] if `from` or `to` is not a multiple of
    /// 4096, [`Error::OutOfBounds`] if `from > to` or the range exceeds
    /// [`Self::capacity`].
    ///
    /// # Safety
    ///
    /// The erased range must not be mapped for instruction fetch or as
    /// immutable data. Erasing a mapped `.text` or `.rodata` page destroys the
    /// running image, or mutates `static` / `.rodata` the compiler treats as
    /// immutable.
    #[ram]
    pub unsafe fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        if from > to {
            return Err(Error::OutOfBounds);
        }
        let len = (to - from) as usize;
        if len == 0 {
            return self.check_bounds(from, 0);
        }

        self.check_alignment(SECTOR_SIZE, from, len)?;
        self.check_bounds(from, len)?;
        self.ensure_unlocked()?;
        self.erase_chunked(from, to)
    }
}

#[ram]
fn check_buffer(buf: &[u8]) -> Result<(), Error> {
    if !is_slice_in_dram(buf) {
        return Err(Error::NotSupported);
    }
    if !(buf.as_ptr() as usize).is_multiple_of(WORD_SIZE as usize) {
        return Err(Error::NotAligned);
    }
    Ok(())
}

impl Flash<'_, Blocking> {
    #[ram]
    fn check_alignment(&self, align: u32, offset: u32, length: usize) -> Result<(), Error> {
        if !offset.is_multiple_of(align) || !length.is_multiple_of(align as usize) {
            return Err(Error::NotAligned);
        }
        Ok(())
    }

    #[ram]
    fn check_bounds(&self, offset: u32, length: usize) -> Result<(), Error> {
        let offset = offset as usize;
        if length > self.capacity || offset > self.capacity - length {
            return Err(Error::OutOfBounds);
        }
        Ok(())
    }

    #[ram]
    fn with_guard<R>(
        &mut self,
        invalidate: Option<(u32, u32)>,
        f: impl FnOnce(&mut Self) -> Result<R, Error>,
    ) -> Result<R, Error> {
        free(|| {
            let _park = ParkGuard::enter(self)?;
            let cache = cache::CacheGuard::suspend();
            let result = f(self);
            if let Some((start, len)) = invalidate {
                cache.invalidate_physical(start, len);
            }
            drop(cache);
            result
        })
    }

    #[ram]
    fn ensure_unlocked(&mut self) -> Result<(), Error> {
        if self.unlocked {
            return Ok(());
        }
        self.with_guard(None, |this| this.unlock_once())
    }

    #[ram]
    fn unlock_once(&mut self) -> Result<(), Error> {
        if self.unlocked {
            return Ok(());
        }
        rom::unlock()?;
        self.unlocked = true;
        Ok(())
    }

    #[ram]
    fn read_chunked(&mut self, mut offset: u32, mut data: &mut [u8]) -> Result<(), Error> {
        while !data.is_empty() {
            let n = data.len().min(rom::ROM_READ_CHUNK);
            let dest = &mut data[..n];
            self.with_guard(None, |_| rom::read(offset, dest))?;
            offset += n as u32;
            data = &mut data[n..];
        }
        Ok(())
    }

    #[ram]
    fn write_chunked(&mut self, mut offset: u32, mut data: &[u8]) -> Result<(), Error> {
        while !data.is_empty() {
            let n = data.len().min(rom::ROM_WRITE_CHUNK);
            let src = &data[..n];
            self.with_guard(Some((offset, n as u32)), |_| rom::write(offset, src))?;
            offset += n as u32;
            data = &data[n..];
        }
        Ok(())
    }

    #[ram]
    fn erase_chunked(&mut self, from: u32, to: u32) -> Result<(), Error> {
        let mut address = from;
        while address < to && !address.is_multiple_of(BLOCK_SIZE) {
            self.with_guard(Some((address, SECTOR_SIZE)), |_| {
                rom::erase_sector(address / SECTOR_SIZE)
            })?;
            address += SECTOR_SIZE;
        }

        while (to - address) >= BLOCK_SIZE {
            self.with_guard(Some((address, BLOCK_SIZE)), |_| {
                rom::erase_block(address / BLOCK_SIZE)
            })?;
            address += BLOCK_SIZE;
        }

        while address < to {
            self.with_guard(Some((address, SECTOR_SIZE)), |_| {
                rom::erase_sector(address / SECTOR_SIZE)
            })?;
            address += SECTOR_SIZE;
        }

        Ok(())
    }
}

/// Parks the other core according to [`MultiCoreStrategy`], unparking on drop.
struct ParkGuard {
    #[cfg(multi_core)]
    parked: Option<crate::system::Cpu>,
}

impl ParkGuard {
    #[ram]
    fn enter(_flash: &Flash<'_, Blocking>) -> Result<Self, Error> {
        cfg_select! {
            multi_core => {
                let parked = match _flash.multi_core_strategy {
                    MultiCoreStrategy::Error => {
                        for other in crate::system::Cpu::other() {
                            if crate::system::is_running(other) {
                                return Err(Error::OtherCoreRunning);
                            }
                        }
                        None
                    }
                    MultiCoreStrategy::AutoPark => {
                        let mut cpu_ctrl = crate::system::CpuControl::new(unsafe {
                            crate::peripherals::CPU_CTRL::steal()
                        });
                        let mut parked = None;
                        for other in crate::system::Cpu::other() {
                            if crate::system::is_running(other) {
                                unsafe { cpu_ctrl.park_core(other) };
                                parked = Some(other);
                            }
                        }
                        parked
                    }
                    MultiCoreStrategy::Ignore(_) => None,
                };
                Ok(Self { parked })
            }
            _ => Ok(Self {}),
        }
    }
}

impl Drop for ParkGuard {
    #[ram]
    fn drop(&mut self) {
        cfg_select! {
            multi_core => {
                if let Some(core) = self.parked {
                    let mut cpu_ctrl = crate::system::CpuControl::new(unsafe {
                        crate::peripherals::CPU_CTRL::steal()
                    });
                    cpu_ctrl.unpark_core(core);
                }
            }
            _ => {}
        }
    }
}
