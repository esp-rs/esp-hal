#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Internal SPI Flash (FLASH)
//!
//! ## Overview
//!
//! Provides blocking access to the attached SPI flash memory. The driver owns
//! the virtual [`FLASH`] peripheral.
//!
//! Write and erase are NOR flash operations: bits change only from 1 to 0
//! without an erase. The driver does not perform read-modify-write.
//! [`Flash::read`] and [`Flash::write`] take word slices (`&[u32]`) and a
//! 4-byte-aligned byte offset. The buffer must reside in DRAM.
//! [`Flash::erase`] operates on 4096-byte sectors.
//!
//! [`Flash::read_encrypted`] and [`Flash::write_encrypted`] provide transparent
//! flash encryption. Encrypted writes require a 16-byte-aligned address and a
//! length that is a multiple of 16 bytes (4 words). The destination must already
//! be erased, and bytes outside the requested range are not programmed.
//! Encrypted reads return plaintext if flash encryption is disabled.
//!
//! For more information, see the
#![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", chip!(), "/api-reference/peripherals/spi_flash/index.html)")]
//! ## Configuration
//!
//! On dual-core chips, [`Config`] selects the multi-core strategy (default:
//! automatically park the other core).
//!
//! ## Usage
//!
//! Construct [`Flash`] from the virtual [`FLASH`] peripheral. `offset` is a
//! flash byte address, not a word index. The driver does not implement traits
//! from `embedded-storage`; higher layers provide partition management and
//! storage trait implementations.
//!
//! ## Examples
//!
//! ### Read a word-aligned range
//!
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::flash::{Config, Flash};
//!
//! let mut flash = Flash::new(peripherals.FLASH, Config::default())?;
//! let mut buf = [0u32; 8];
//! flash.read(0x10_020, &mut buf)?;
//! # {after_snippet}
//! ```
//!
//! ## Implementation State
#![cfg_attr(
    esp32,
    doc = "- On ESP32, a second-stage bootloader must identify the flash chip; the ROM does not.
  [`Flash::new`] fails with [`ConfigError::UnknownFlashChip`] if identification is missing
  (including the ROM placeholder ID `0x001540EF`, a valid W25Q16 ID on other chips)."
)]
//! - Driver bounds checks use the JEDEC density from the ROM-cached device ID. ROM operations also
//!   check the ROM-cached chip size, which a bootloader can set from the application image header.
//!   An operation within [`Flash`] capacity can still fail in the ROM if the ROM-cached size is
//!   smaller.
//! - Operations do not check whether the target flash range is mapped. Writing or erasing a mapped
//!   section (such as `.text` or `.rodata`) can corrupt running code or immutable data.
//! - Each [`Flash::read`] or [`Flash::write`] call executes a single ROM operation. The flash cache
//!   remains disabled (and the other core remains parked) for the entire transfer. Split large
//!   operations in higher-level code if latency is critical.
//! - On dual-core chips, the default strategy stalls the other core around every operation,
//!   including reads. The other core can freeze while holding a lock or executing an interrupt
//!   handler.

use core::marker::PhantomData;

use procmacros::{BuilderLite, ram};
#[cfg(xtensa)]
use xtensa_lx::interrupt::free;

#[cfg(not(xtensa))]
use crate::interrupt::free;
use crate::{Blocking, DriverMode, peripherals::FLASH, soc::is_slice_in_dram};

mod cache;
mod mmu;
mod rom;

/// Word size required by the read and write paths, in bytes.
const WORD_SIZE: u32 = 4;
/// Program page size in bytes.
const PAGE_SIZE: u32 = 256;
/// Erase sector size in bytes.
const SECTOR_SIZE: u32 = 4096;
/// Erase block size in bytes.
const BLOCK_SIZE: u32 = 65536;
/// Flash-encryption AES block size, and the public encrypted-write alignment.
const ENCRYPT_BLOCK_SIZE: u32 = 16;
/// ESP32 ROM encrypted-write row (two AES blocks that share a tweak).
#[cfg(esp32)]
const ESP32_ENCRYPT_ROW: u32 = 32;

/// Flash driver configuration.
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct Config {
    /// Strategy for the other core during flash operations.
    ///
    /// Default: [`MultiCoreStrategy::AutoPark`].
    ///
    /// [`MultiCoreStrategy::ignore`] is safe only when the other core cannot
    /// fetch from flash during the operation.
    #[cfg(multi_core)]
    #[builder_lite(unstable)]
    multi_core_strategy: MultiCoreStrategy,
}

/// Strategy for the other core during flash operations.
#[cfg(multi_core)]
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum MultiCoreStrategy {
    /// Returns [`Error::OtherCoreRunning`] if the other core is running.
    Error,
    /// Parks the other core for the operation and unparks it afterward.
    ///
    /// The stall can freeze the other core at any instruction. The other core can hold a
    /// lock or execute an interrupt handler. Parking uses CPU-control
    /// registers even if the application already owns
    /// [`crate::peripherals::CPU_CTRL`].
    #[default]
    AutoPark,
    /// Does not check or park the other core.
    ///
    /// Construct with [`Self::ignore`].
    Ignore(IgnoreMarker),
}

/// Marker so [`MultiCoreStrategy::Ignore`] can only be built via
/// [`MultiCoreStrategy::ignore`].
#[cfg(multi_core)]
#[doc(hidden)]
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IgnoreMarker {
    _private: (),
}

#[cfg(multi_core)]
impl MultiCoreStrategy {
    /// Creates a strategy that does not check or park the other core.
    ///
    /// # Safety
    ///
    /// The other core must not fetch instructions or data from flash during any
    /// operation, including reads. Flash-backed caches are unavailable while
    /// an operation runs.
    #[instability::unstable]
    pub const unsafe fn ignore() -> Self {
        Self::Ignore(IgnoreMarker { _private: () })
    }
}

/// Error constructing or configuring the flash driver.
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ConfigError {
    /// The attached flash chip cannot be identified, or its size is not
    /// recognized.
    ///
    /// This includes no-response JEDEC sentinels, an unrecognized density
    /// byte, and the ESP32 ROM placeholder ID `0x001540EF` (the ROM does not
    /// execute `RDID`; this value is a valid W25Q16 ID on other chips).
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
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(clippy::enum_variant_names, reason = "matches ROM result / issue 6203")]
#[non_exhaustive]
pub enum Error {
    /// I/O error reported by the ROM.
    ///
    /// Includes a ROM-side size check against its cached chip size when that
    /// is smaller than [`Flash::capacity`].
    IoError,
    /// Operation timed out.
    IoTimeout,
    /// Address or length is not aligned for this operation.
    ///
    /// [`Flash::read`], [`Flash::read_encrypted`], and [`Flash::write`] require a
    /// 4-byte-aligned flash offset. [`Flash::write_encrypted`] requires a
    /// 16-byte address and a word count that is a multiple of 4 (16 bytes).
    /// [`Flash::erase`] requires a 4096-byte range.
    NotAligned,
    /// Address range exceeds [`Flash::capacity`], or `from > to`.
    ///
    /// Capacity is the JEDEC density, not the ROM cached chip size. An address
    /// range within this limit can still fail in the ROM if the cached size is
    /// smaller (often the image-header size on ESP32).
    OutOfBounds,
    /// Not supported in the current environment.
    ///
    /// Returned when the buffer is not in DRAM, when
    /// [`Flash::write_encrypted`] is called while flash encryption is disabled,
    /// or when [`Flash::read_encrypted`] cannot allocate an MMU entry.
    NotSupported,
    /// The other core is running and the configured strategy is
    /// [`MultiCoreStrategy::Error`].
    #[cfg(multi_core)]
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
            Self::NotAligned => write!(f, "Flash address or length is not aligned"),
            Self::OutOfBounds => write!(f, "Flash range is out of bounds"),
            Self::NotSupported => write!(f, "Flash operation is not supported"),
            #[cfg(multi_core)]
            Self::OtherCoreRunning => write!(f, "The other core is running"),
            Self::Unknown => write!(f, "Unknown flash error"),
        }
    }
}

/// Flash chip identification and geometry.
///
/// `chip_id` contains the manufacturer ID in bits 23:16. Geometry is fixed at
/// 256-byte pages, 4096-byte sectors, and 64 KiB blocks; it is not probed from
/// the chip.
#[instability::unstable]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct ChipInfo {
    /// JEDEC ID with manufacturer ID in bits 23:16.
    pub chip_id: u32,
    /// JEDEC-decoded physical capacity in bytes.
    ///
    /// See [`Flash::capacity`].
    pub capacity: usize,
    /// Erase sector size in bytes.
    pub sector_size: u32,
    /// Erase block size in bytes.
    pub block_size: u32,
    /// Program page size in bytes.
    pub page_size: u32,
}

/// Internal SPI flash driver.
///
/// Constructible only in [`Blocking`] mode.
#[instability::unstable]
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
    /// Program page size in bytes.
    #[instability::unstable]
    pub const PAGE_SIZE: u32 = 256;
    /// Erase sector size in bytes.
    #[instability::unstable]
    pub const SECTOR_SIZE: u32 = 4096;
    /// Erase block size in bytes.
    #[instability::unstable]
    pub const BLOCK_SIZE: u32 = 65536;

    /// Creates a new flash driver from the [`FLASH`] peripheral.
    ///
    /// Capacity is determined from the JEDEC density in the ROM-cached device
    /// ID, not the image-header size or the ROM cached chip size used by
    /// read, write, and erase operations.
    ///
    /// # Errors
    ///
    /// - [`ConfigError::UnknownFlashChip`] if the attached flash chip cannot be identified or its
    ///   size is not recognized. On ESP32, this error also occurs when a second-stage bootloader
    ///   has not identified the chip (including the ROM placeholder ID `0x001540EF`).
    #[instability::unstable]
    pub fn new(flash: FLASH<'d>, config: Config) -> Result<Self, ConfigError> {
        let raw_id = rom::cached_device_id();
        let capacity = rom::capacity_from_cached_id(raw_id)?;
        let chip_id = raw_id & 0x00FF_FFFF;
        #[cfg(not(multi_core))]
        let _ = config;

        Ok(Self {
            _flash: flash,
            capacity,
            chip_id,
            unlocked: false,
            #[cfg(multi_core)]
            multi_core_strategy: config.multi_core_strategy,
            _mode: PhantomData,
        })
    }

    /// Applies a new configuration.
    ///
    /// Updates the multi-core strategy on dual-core chips. On single-core
    /// chips, this is a no-op. The return type is reserved for future
    /// configuration options.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        #[cfg(multi_core)]
        {
            self.multi_core_strategy = config.multi_core_strategy;
        }
        let _ = config;
        Ok(())
    }

    /// Returns the JEDEC-decoded flash capacity in bytes.
    ///
    /// Driver bounds checks use this value. ROM read, write, and erase operations
    /// also check the ROM-cached chip size, which a bootloader can set from the
    /// application image header.
    #[instability::unstable]
    pub fn capacity(&self) -> usize {
        self.capacity
    }

    /// Returns chip identification and geometry.
    ///
    /// `chip_id` contains the manufacturer ID in bits 23:16. `capacity` is the
    /// JEDEC density; see [`Self::capacity`]. Geometry is fixed, not probed
    /// from the chip.
    #[instability::unstable]
    pub fn chip_info(&self) -> ChipInfo {
        ChipInfo {
            chip_id: self.chip_id,
            capacity: self.capacity,
            sector_size: SECTOR_SIZE,
            block_size: BLOCK_SIZE,
            page_size: PAGE_SIZE,
        }
    }

    /// Reads words into `data` starting at byte address `offset`.
    ///
    /// `offset` is a flash byte address and must be a multiple of 4. `data`
    /// must reside in DRAM, not in flash, IRAM, RTC memory, or PSRAM. An empty
    /// `data` slice only checks that `offset` is within [`Self::capacity`].
    ///
    /// # Errors
    ///
    /// - [`Error::NotAligned`] if `data` is non-empty and `offset` is not a multiple of 4.
    /// - [`Error::OutOfBounds`] if the range exceeds [`Self::capacity`].
    /// - [`Error::NotSupported`] if `data` is not in DRAM.
    #[cfg_attr(
        multi_core,
        doc = "- [`Error::OtherCoreRunning`] on dual-core chips with [`MultiCoreStrategy::Error`]."
    )]
    /// - [`Error::IoError`], [`Error::IoTimeout`], or [`Error::Unknown`] if the ROM reports an I/O
    ///   error.
    #[instability::unstable]
    #[ram]
    pub fn read(&mut self, offset: u32, data: &mut [u32]) -> Result<(), Error> {
        let Some(len) = byte_len(data) else {
            return Err(Error::OutOfBounds);
        };
        if data.is_empty() {
            return self.check_bounds(offset, 0);
        }

        self.check_word_offset(offset)?;
        self.check_bounds(offset, len)?;
        check_buffer(data)?;
        self.with_guard(None, |_| rom::read(offset, data))
    }

    /// Writes `data` starting at byte address `offset`.
    ///
    /// The target flash range must already be erased. `offset` is a flash byte
    /// address and must be a multiple of 4. `data` must reside in DRAM, not in
    /// flash, IRAM, RTC memory, or PSRAM. An empty `data` slice only checks
    /// that `offset` is within [`Self::capacity`].
    ///
    /// # Errors
    ///
    /// - [`Error::NotAligned`] if `data` is non-empty and `offset` is not a multiple of 4.
    /// - [`Error::OutOfBounds`] if the range exceeds [`Self::capacity`].
    /// - [`Error::NotSupported`] if `data` is not in DRAM.
    #[cfg_attr(
        multi_core,
        doc = "- [`Error::OtherCoreRunning`] on dual-core chips with [`MultiCoreStrategy::Error`]."
    )]
    /// - [`Error::IoError`], [`Error::IoTimeout`], or [`Error::Unknown`] if the ROM reports an I/O
    ///   error.
    ///
    /// # Safety
    ///
    /// The programmed range must not be mapped for instruction fetch or as
    /// immutable data. Writing a mapped `.text` or `.rodata` page overwrites
    /// the running image or mutates data the compiler treats as immutable.
    #[instability::unstable]
    #[ram]
    pub unsafe fn write(&mut self, offset: u32, data: &[u32]) -> Result<(), Error> {
        let Some(len) = byte_len(data) else {
            return Err(Error::OutOfBounds);
        };
        if data.is_empty() {
            return self.check_bounds(offset, 0);
        }

        self.check_word_offset(offset)?;
        self.check_bounds(offset, len)?;
        check_buffer(data)?;
        self.ensure_unlocked()?;
        self.with_guard(Some((offset, len as u32)), |_| rom::write(offset, data))
    }

    /// Erases flash sectors in the range `[from, to)`.
    ///
    /// `from` and `to` must be multiples of [`Self::SECTOR_SIZE`] (4096 bytes);
    /// there is no byte-granular erase. `to == capacity` is allowed. When
    /// `from == to`, this function only checks that `from` is within
    /// [`Self::capacity`].
    ///
    /// # Errors
    ///
    /// - [`Error::NotAligned`] if `from != to` and `from` or `to` is not a multiple of 4096.
    /// - [`Error::OutOfBounds`] if `from > to` or the range exceeds [`Self::capacity`].
    #[cfg_attr(
        multi_core,
        doc = "- [`Error::OtherCoreRunning`] on dual-core chips with [`MultiCoreStrategy::Error`]."
    )]
    /// - [`Error::IoError`], [`Error::IoTimeout`], or [`Error::Unknown`] if the ROM reports an I/O
    ///   error.
    ///
    /// # Safety
    ///
    /// The erased range must not be mapped for instruction fetch or as
    /// immutable data. Erasing a mapped `.text` or `.rodata` page destroys the
    /// running image or mutates data the compiler treats as immutable.
    #[instability::unstable]
    #[ram]
    pub unsafe fn erase(&mut self, from: u32, to: u32) -> Result<(), Error> {
        let Some(len) = to.checked_sub(from) else {
            return Err(Error::OutOfBounds);
        };
        let len = len as usize;
        if len == 0 {
            return self.check_bounds(from, 0);
        }

        self.check_alignment(SECTOR_SIZE, from, len)?;
        self.check_bounds(from, len)?;
        self.ensure_unlocked()?;
        self.erase_range(from, to)
    }

    /// Reads decrypted words into `data` starting at byte address `offset`.
    ///
    /// Uses a temporary MMU mapping to read decrypted data through the cache.
    /// If flash encryption is not enabled, this returns plaintext.
    ///
    /// `offset` is a flash byte address and must be a multiple of 4. `data`
    /// must reside in DRAM, not in flash, IRAM, RTC memory, or PSRAM. An empty
    /// `data` slice only checks that `offset` is within [`Self::capacity`].
    ///
    /// # Errors
    ///
    /// - [`Error::NotAligned`] if `data` is non-empty and `offset` is not a multiple of 4.
    /// - [`Error::OutOfBounds`] if the range exceeds [`Self::capacity`].
    /// - [`Error::NotSupported`] if `data` is not in DRAM or no MMU entry is available.
    #[cfg_attr(
        multi_core,
        doc = "- [`Error::OtherCoreRunning`] on dual-core chips with [`MultiCoreStrategy::Error`]."
    )]
    #[instability::unstable]
    pub fn read_encrypted(&mut self, offset: u32, data: &mut [u32]) -> Result<(), Error> {
        let Some(len) = byte_len(data) else {
            return Err(Error::OutOfBounds);
        };
        if data.is_empty() {
            return self.check_bounds(offset, 0);
        }

        self.check_word_offset(offset)?;
        self.check_bounds(offset, len)?;
        check_buffer(data)?;
        self.with_mmu_guard(|_| mmu::read_flash_encrypted(offset, data))
    }

    /// Writes `data` with transparent flash encryption.
    ///
    /// `offset` must be a multiple of 16, and `data.len()` must be a multiple
    /// of 4 words (16 bytes). The destination must already be erased, and bytes
    /// outside the requested range are not programmed. `data` must reside in
    /// DRAM, not in flash, IRAM, RTC memory, or PSRAM.
    ///
    /// On ESP32, encrypted writes operate in 32-byte rows. If a boundary is not
    /// 32-byte aligned, the driver preserves the adjacent 16-byte block by
    /// re-encrypting its existing plaintext to the same ciphertext. The neighbor
    /// block therefore does not require prior erasing.
    ///
    /// An empty `data` slice only checks that `offset` is within [`Self::capacity`].
    ///
    /// # Errors
    ///
    /// - [`Error::NotSupported`] if flash encryption is disabled. The eFuse gate is checked before
    ///   the arguments are.
    /// - [`Error::NotAligned`] if `data` is non-empty and `offset` is not a multiple of 16, or
    ///   `data.len()` is not a multiple of 4 words (16 bytes).
    /// - [`Error::OutOfBounds`] if the range exceeds [`Self::capacity`].
    /// - [`Error::NotSupported`] if `data` is not in DRAM, or on ESP32 if reading a row neighbor
    ///   fails because no MMU entry is available.
    #[cfg_attr(
        multi_core,
        doc = "- [`Error::OtherCoreRunning`] on dual-core chips with [`MultiCoreStrategy::Error`]."
    )]
    /// - [`Error::IoError`], [`Error::IoTimeout`], or [`Error::Unknown`] if the ROM reports an I/O
    ///   error.
    ///
    /// # Safety
    ///
    /// The programmed range must not be mapped for instruction fetch or as
    /// immutable data. Writing a mapped `.text` or `.rodata` page overwrites
    /// the running image, or mutates `static` or `.rodata` data the compiler
    /// treats as immutable.
    #[instability::unstable]
    #[ram]
    pub unsafe fn write_encrypted(&mut self, offset: u32, data: &[u32]) -> Result<(), Error> {
        #[cfg(not(__test_flash))]
        if !crate::efuse::flash_encryption() {
            return Err(Error::NotSupported);
        }

        let Some(len) = byte_len(data) else {
            return Err(Error::OutOfBounds);
        };
        if data.is_empty() {
            return self.check_bounds(offset, 0);
        }

        self.check_alignment(ENCRYPT_BLOCK_SIZE, offset, len)?;
        self.check_bounds(offset, len)?;
        check_buffer(data)?;
        self.ensure_unlocked()?;

        #[cfg(esp32)]
        let neighbors = self.esp32_row_neighbors(offset, len)?;
        // On ESP32 the programmed range can extend one 16-byte block past each
        // end; `with_guard` flushes the whole cache there, so the range only
        // needs to cover the request.
        self.with_guard(Some((offset, len as u32)), |_| {
            write_encrypted_rows(
                offset,
                words_as_bytes(data),
                #[cfg(esp32)]
                &neighbors,
            )
        })
    }
}

#[inline(always)]
fn byte_len(data: &[u32]) -> Option<usize> {
    data.len().checked_mul(WORD_SIZE as usize)
}

#[inline(always)]
fn words_as_bytes(data: &[u32]) -> &[u8] {
    // SAFETY: inspecting the in-memory bytes of `u32` words.
    unsafe { core::slice::from_raw_parts(data.as_ptr().cast(), size_of_val(data)) }
}

#[inline(always)]
fn check_buffer(buf: &[u32]) -> Result<(), Error> {
    if !is_slice_in_dram(buf) {
        return Err(Error::NotSupported);
    }
    Ok(())
}

/// Word-aligned staging buffer. The ROM encrypts in place.
#[repr(C, align(4))]
struct EncryptRow([u8; XTS_AES_BLOCK_MAX]);

#[ram]
fn write_encrypted_rows(
    offset: u32,
    data: &[u8],
    #[cfg(esp32)] neighbors: &Esp32Neighbors,
) -> Result<(), Error> {
    let mut row = EncryptRow([0; XTS_AES_BLOCK_MAX]);
    let mut i = 0;
    while i < data.len() {
        let row_addr = offset + i as u32;
        let (program_addr, program_len, consumed) =
            cfg_select! {
                esp32 => prepare_esp32_row(&mut row.0, row_addr, &data[i..], neighbors),
                _ => prepare_xts_row(&mut row.0, row_addr, &data[i..]),
            };
        rom::write_encrypted(program_addr, row.0.as_mut_ptr().cast(), program_len)?;
        i += consumed;
    }
    Ok(())
}

#[cfg(esp32)]
struct Esp32Neighbors {
    pre: [u32; 4],
    post: [u32; 4],
}

/// Fill a 32-byte ESP32 ROM row. Returns `(program_addr, program_len, consumed)`.
///
/// Mirrors the ESP32 arm of ESP-IDF `esp_flash_write_encrypted`: a row is two
/// AES blocks sharing an address-derived tweak, so a 16-byte write must carry
/// the decrypted neighbor block along and re-encrypt it to the same ciphertext.
#[cfg(esp32)]
#[ram]
fn prepare_esp32_row(
    buf: &mut [u8; XTS_AES_BLOCK_MAX],
    row_addr: u32,
    remaining: &[u8],
    neighbors: &Esp32Neighbors,
) -> (u32, u32, usize) {
    const BLOCK: usize = ENCRYPT_BLOCK_SIZE as usize;
    const ROW: usize = ESP32_ENCRYPT_ROW as usize;

    if !row_addr.is_multiple_of(ESP32_ENCRYPT_ROW) {
        buf[..BLOCK].copy_from_slice(words_as_bytes(&neighbors.pre));
        buf[BLOCK..ROW].copy_from_slice(&remaining[..BLOCK]);
        (row_addr - ENCRYPT_BLOCK_SIZE, ESP32_ENCRYPT_ROW, BLOCK)
    } else if remaining.len() == BLOCK {
        buf[..BLOCK].copy_from_slice(&remaining[..BLOCK]);
        buf[BLOCK..ROW].copy_from_slice(words_as_bytes(&neighbors.post));
        (row_addr, ESP32_ENCRYPT_ROW, BLOCK)
    } else {
        buf[..ROW].copy_from_slice(&remaining[..ROW]);
        (row_addr, ESP32_ENCRYPT_ROW, ROW)
    }
}

/// Largest row the flash-encryption hardware accepts, matching IDF's
/// `SOC_FLASH_ENCRYPTED_XTS_AES_BLOCK_MAX`.
#[cfg(any(esp32, esp32c2, esp32c3))]
const XTS_AES_BLOCK_MAX: usize = 32;
#[cfg(not(any(esp32, esp32c2, esp32c3)))]
const XTS_AES_BLOCK_MAX: usize = 64;

/// Pick the largest aligned row, as ESP-IDF does for ESP32-S2 and later.
#[cfg(not(esp32))]
#[ram]
fn prepare_xts_row(
    buf: &mut [u8; XTS_AES_BLOCK_MAX],
    row_addr: u32,
    remaining: &[u8],
) -> (u32, u32, usize) {
    let row_size =
        if XTS_AES_BLOCK_MAX >= 64 && row_addr.is_multiple_of(64) && remaining.len() >= 64 {
            64
        } else if row_addr.is_multiple_of(32) && remaining.len() >= 32 {
            32
        } else {
            16
        };
    buf[..row_size].copy_from_slice(&remaining[..row_size]);
    (row_addr, row_size as u32, row_size)
}

impl Flash<'_, Blocking> {
    #[inline(always)]
    fn check_word_offset(&self, offset: u32) -> Result<(), Error> {
        if !offset.is_multiple_of(WORD_SIZE) {
            return Err(Error::NotAligned);
        }
        Ok(())
    }

    #[inline(always)]
    fn check_alignment(&self, align: u32, offset: u32, length: usize) -> Result<(), Error> {
        if !offset.is_multiple_of(align) || !length.is_multiple_of(align as usize) {
            return Err(Error::NotAligned);
        }
        Ok(())
    }

    #[inline(always)]
    fn check_bounds(&self, offset: u32, length: usize) -> Result<(), Error> {
        let offset = offset as usize;
        if length > self.capacity || offset > self.capacity - length {
            return Err(Error::OutOfBounds);
        }
        Ok(())
    }

    /// Park and disable interrupts without suspending the cache.
    ///
    /// Encrypted reads map flash through the MMU and copy with the cache on, so
    /// neither this guard nor anything it calls has to live in RAM.
    fn with_mmu_guard<R>(
        &mut self,
        f: impl FnOnce(&mut Self) -> Result<R, Error>,
    ) -> Result<R, Error> {
        free(|| {
            let _park = ParkGuard::enter(self)?;
            f(self)
        })
    }

    /// Plaintext of row halves the write will touch but not replace.
    ///
    /// Read before the cache-off write path: [`Self::read_encrypted`] needs the
    /// cache on. Re-encrypting this plaintext in [`prepare_esp32_row`] reproduces
    /// the ciphertext already in flash, so those neighbors need no separate erase.
    #[cfg(esp32)]
    fn esp32_row_neighbors(&mut self, offset: u32, len: usize) -> Result<Esp32Neighbors, Error> {
        let mut neighbors = Esp32Neighbors {
            pre: [0; 4],
            post: [0; 4],
        };
        if !offset.is_multiple_of(ESP32_ENCRYPT_ROW) {
            self.read_encrypted(offset - ENCRYPT_BLOCK_SIZE, &mut neighbors.pre)?;
        }
        let end = offset + len as u32;
        if !end.is_multiple_of(ESP32_ENCRYPT_ROW) {
            self.read_encrypted(end, &mut neighbors.post)?;
        }
        Ok(neighbors)
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
            cfg_select! {
                esp32 => {
                    // Address-based invalidate is unavailable; flush while still off.
                    if invalidate.is_some() {
                        cache.flush_while_off();
                    }
                    drop(cache);
                }
                _ => {
                    drop(cache);
                    if let Some((start, len)) = invalidate {
                        mmu::invalidate_mapped(start, len);
                    }
                }
            }
            result
        })
    }

    #[ram]
    fn ensure_unlocked(&mut self) -> Result<(), Error> {
        if self.unlocked {
            return Ok(());
        }
        self.with_guard(None, |this| {
            rom::unlock()?;
            this.unlocked = true;
            Ok(())
        })
    }

    #[ram]
    fn erase_range(&mut self, from: u32, to: u32) -> Result<(), Error> {
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
