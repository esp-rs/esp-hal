# `esp_hal::flash` — Flash Storage Driver

Driver for SPI NOR flash, replacing `esp-storage`. `FlashStorage<'d>` is a non-generic driver that works with both internal boot flash (SPI0/SPI1) and external SPI flash (SPI2/3). The peripheral instance is immediately erased into a private backend enum on construction — no internal dispatch types are public.

`FlashChipDriver` is a data-driven trait — it provides configuration (which commands, what parameters) and `FlashStorage` handles all SPI execution internally.

---

## Module structure

```
esp_hal::flash
├── mod.rs               — public types, re-exports
├── storage.rs           — FlashStorage<'d> (non-generic driver)
└── host.rs              — FlashBackend, FlashCommand, exec logic (ALL PRIVATE)
```

Public items in `mod.rs`:
- `Config`, `ConfigError`
- `FlashStorageError`
- `FlashChipConfig`, `FlashChipInfo`
- `FlashChipDriver`, `GenericDriver` (unstable)
- `InternalFlashExt` (unstable)
- `pub mod ll` (unstable)

### Module documentation

```rust
//! # Flash Storage
//!
//! ## Overview
//! Driver for SPI NOR flash chips. Works with both the internal boot
//! flash (SPI0/SPI1) and external SPI flash chips (SPI2/3).
#![doc = concat!("[ESP-IDF docs](https://docs.espressif.com/projects/esp-idf/en/latest/", crate::soc::chip!(), "/api-reference/peripherals/spi_flash/index.html)")]
//!
//! ## Configuration
//! The default [`Config`] auto-detects flash geometry from ROM.
//! For non-standard chips, use [`FlashChipConfig`] to override
//! geometry, or provide a custom [`FlashChipDriver`] implementation.
//!
//! ## Usage
//! Implements [`embedded_storage::nor_flash::NorFlash`] and
//! [`embedded_storage::nor_flash::ReadNorFlash`] when the
//! `embedded-storage` feature is enabled.
//!
//! ## Examples
//! ### Read internal flash
//! ```rust, no_run
#[doc = crate::before_snippet!()]
//! let mut flash = FlashStorage::new(peripherals.FLASH, Config::default())?;
//! let mut buf = [0u8; 256];
//! flash.read(0x10000, &mut buf)?;
//! # Ok(())
//! # }
//! ```
//!
//! ## Implementation State
//! - Memory-mapped reads — unstable, via [`InternalFlashExt`]
//! - External SPI flash — unstable
//! - Dual/Quad SPI IO modes — not yet supported
```

---

## Public types

### `Config`

```rust
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    #[cfg(multi_core)]
    multi_core_strategy: MultiCoreStrategy,
}
```

### `ConfigError`

```rust
#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConfigError {}

impl core::fmt::Display for ConfigError { ... }
impl core::error::Error for ConfigError {}
```

### `FlashStorageError`

```rust
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashStorageError {
    IoError,
    IoTimeout,
    CantUnlock,
    NotAligned,
    OutOfBounds,
    NotSupported,
    #[cfg(multi_core)]
    OtherCoreRunning,
    Other(i32),
}

impl core::fmt::Display for FlashStorageError { ... }
impl core::error::Error for FlashStorageError {}
```

### `FlashChipConfig`

```rust
#[non_exhaustive]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashChipConfig {
    capacity: u32,
    sector_size: u32,
    block_size: u32,
    page_size: u32,

    #[builder_lite(unstable)]
    read_command: u8,
    #[builder_lite(unstable)]
    page_program_command: u8,
    #[builder_lite(unstable)]
    sector_erase_command: u8,
    #[builder_lite(unstable)]
    block_erase_command: u8,
    #[builder_lite(unstable)]
    chip_erase_command: u8,
    #[builder_lite(unstable)]
    write_enable_command: u8,
    #[builder_lite(unstable)]
    read_status_command: u8,
    #[builder_lite(unstable)]
    status_busy_mask: u8,

    #[builder_lite(unstable)]
    page_program_timeout_us: u32,
    #[builder_lite(unstable)]
    sector_erase_timeout_us: u32,
    #[builder_lite(unstable)]
    block_erase_timeout_us: u32,
    #[builder_lite(unstable)]
    chip_erase_timeout_us: u32,
}
```

### `FlashChipInfo`

```rust
#[non_exhaustive]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashChipInfo {
    pub chip_id: u32,
    pub capacity: u32,
    pub sector_size: u32,
    pub block_size: u32,
    pub page_size: u32,
}
```

---

## Chip driver layer (all unstable)

`FlashChipDriver` is data-driven — it provides the configuration that controls which commands and parameters `FlashStorage` sends. All SPI command execution is internal to `FlashStorage`. `FlashCommand` is a private type in `host.rs`.

### `FlashChipDriver`

NOT sealed — user-implementable for custom chips. Provides config data only.

```rust
#[instability::unstable]
pub trait FlashChipDriver {
    /// Chip parameters — commands, timing, geometry.
    fn config(&self) -> &FlashChipConfig;

    /// Check if this driver supports the given JEDEC chip ID.
    fn probe(&self, chip_id: u32) -> bool;
}
```

`FlashStorage` uses `config()` to determine which command bytes, address widths, and timeouts to use for each operation. For chips that just need different command bytes or timing, implementing `FlashChipDriver` with a custom `FlashChipConfig` is sufficient.

### `GenericDriver`

```rust
#[instability::unstable]
#[non_exhaustive]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GenericDriver {
    config: FlashChipConfig,
}

impl GenericDriver {
    #[instability::unstable]
    pub fn new(config: FlashChipConfig) -> Self { Self { config } }
}

impl FlashChipDriver for GenericDriver {
    fn config(&self) -> &FlashChipConfig { &self.config }
    fn probe(&self, _chip_id: u32) -> bool { true }
}
```

---

## `FlashStorage<'d>`

```rust
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashStorage<'d> {
    backend: FlashBackend,           // private
    guard: PeripheralGuard,
    capacity: u32,
    unlocked: bool,
    _lifetime: PhantomData<&'d ()>,
}

impl<'d> FlashStorage<'d> {
    pub fn new(
        flash: impl Peripheral<P = FLASH> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError>;

    #[instability::unstable]
    pub fn new_spi(
        spi: impl Peripheral<P = impl SpiInstance> + 'd,
        cs: impl Peripheral<P = impl OutputPin> + 'd,
        config: Config,
    ) -> Result<Self, ConfigError>;

    #[instability::unstable]
    pub fn with_chip_driver(
        flash: impl Peripheral<P = FLASH> + 'd,
        config: Config,
        driver: impl FlashChipDriver,
    ) -> Result<Self, ConfigError>;

    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError>;

    pub fn read(&mut self, offset: u32, buffer: &mut [u8]) -> Result<(), FlashStorageError>;
    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), FlashStorageError>;
    pub fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashStorageError>;
    pub fn erase_chip(&mut self) -> Result<(), FlashStorageError>;
    pub fn capacity(&self) -> usize;
    pub fn chip_info(&self) -> FlashChipInfo;

    #[instability::unstable]
    pub fn configure_chip(&mut self, config: &FlashChipConfig);
}

// PeripheralGuard handles Drop

#[cfg(feature = "embedded-storage")]
impl ReadNorFlash for FlashStorage<'_> { ... }
#[cfg(feature = "embedded-storage")]
impl NorFlash for FlashStorage<'_> { ... }
```

### `InternalFlashExt`

```rust
#[instability::unstable]
pub trait InternalFlashExt {
    fn mmap(&mut self, offset: u32, len: u32) -> Result<MappedFlash<'_>, FlashStorageError>;
}

#[instability::unstable]
impl InternalFlashExt for FlashStorage<'_> { ... }
```

---

## Private internals (`host.rs`)

```rust
enum FlashBackend {
    InternalRom,                    // ROM high-level functions (default)
    InternalHal(InternalHalState),  // ROM common_command or Rust registers (chip driver)
    Spi(SpiState),                  // SPI2/3 master driver
}

struct InternalHalState {
    #[cfg(not(any(esp32, esp32s2)))]
    rom_ctx: *mut core::ffi::c_void,
}

struct SpiState { /* SPI master + CS */ }

impl FlashBackend {
    fn exec(&mut self, cmd: FlashCommand<'_>) -> Result<(), FlashStorageError> {
        match self {
            FlashBackend::InternalRom => unreachable!(),
            FlashBackend::InternalHal(state) => {
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32, esp32s2))] {
                        spi1_register_exec(cmd)  // #[ram]
                    } else {
                        rom_hal_exec(state.rom_ctx, cmd)
                    }
                }
            }
            FlashBackend::Spi(state) => spi_master_exec(state, cmd),
        }
    }

    fn is_internal(&self) -> bool {
        !matches!(self, FlashBackend::Spi(_))
    }
}
```

ESP32 vs ESP32-S2 SPI1 register differences handled via `cfg_if!`:
- Address: MSB-aligned (ESP32) vs direct (ESP32-S2)
- Trigger: `cmd.usr` only (ESP32) vs `cmd.val |= 0x40000` (ESP32-S2)

ROM `spi_flash_hal_common_command` binding added to `esp-rom-sys/src/rom/spiflash.rs` for S3/C2/C3/C5/C6/C61/H2. HAL context from `esp_flash_default_chip` ROM symbol.

---

## Stable 1.0 surface

| Item | Stable | Unstable |
|------|--------|----------|
| `Config`, `ConfigError` | yes | |
| `FlashStorage::new(flash, config)` | yes | |
| `FlashStorage::apply_config()` | yes | |
| `FlashStorage::read/write/erase/erase_chip/capacity/chip_info` | yes | |
| `FlashChipConfig` (geometry setters) | yes | |
| `FlashChipConfig` (command/timing setters) | | yes |
| `FlashChipInfo` | yes | |
| `FlashStorageError` | yes | |
| `embedded-storage` trait impls | yes (feature) | |
| `FlashStorage::new_spi()` | | yes |
| `FlashStorage::with_chip_driver()` | | yes |
| `FlashStorage::configure_chip()` | | yes |
| `FlashChipDriver`, `GenericDriver` | | yes |
| `InternalFlashExt` | | yes |
| `ll` module | | yes |

---

## Files to create/modify

| File | Action |
|------|--------|
| `esp-hal/src/flash/mod.rs` | **Create** — public types, re-exports |
| `esp-hal/src/flash/storage.rs` | **Create** — `FlashStorage` |
| `esp-hal/src/flash/host.rs` | **Create** — private backend dispatch |
| `esp-hal/src/lib.rs` | Add `pub mod flash;` |
| `esp-hal/Cargo.toml` | Add optional `embedded-storage` dep |
| `esp-rom-sys/src/rom/spiflash.rs` | Add ROM bindings |
| `esp-bootloader-esp-idf/` | Update deps and imports |
| `examples/peripheral/flash_read_write/` | Update imports |
| `esp-storage/` | Deprecation notice |

---

## Open question

Should `FlashCommand` include an `io_mode` field for dual/quad SPI, or stay single-SPI only initially?

---

## Verification

1. `cargo xtask lint-packages --packages esp-hal --chips esp32c6`
2. `cargo xtask lint-packages --packages esp-hal --chips esp32s3`
3. `cargo xtask lint-packages --packages esp-hal --chips esp32`
4. `cargo xtask fmt-packages`
5. Build flash example and `esp-bootloader-esp-idf`
6. `cargo xtask host-tests`