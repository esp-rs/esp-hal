# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- ESP32-S3: Added SDMMC signals (#2556)
- Added `set_priority` to the `DmaChannel` trait on GDMA devices (#2403, #2526)
- Added `into_async` and `into_blocking` functions for `ParlIoTxOnly`, `ParlIoRxOnly` (#2526)
- ESP32-C6, H2, S3: Added `split` function to the `DmaChannel` trait. (#2526, #2532)
- DMA: `PeripheralDmaChannel` type aliasses and `DmaChannelFor` traits to improve usability. (#2532)
- `dma::{Channel, ChannelRx, ChannelTx}::set_priority` for GDMA devices (#2403)
- `esp_hal::asynch::AtomicWaker` that does not hold a global critical section (#2555)
- `esp_hal::sync::RawMutex` for embassy-sync. (#2555)
- ESP32-C6, H2, S3: Added `split` function to the `DmaChannel` trait. (#2526)
- Added PSRAM configuration to `esp_hal::Config` if `quad-psram` or `octal-psram` is enabled (#2546)
- Added `esp_hal::psram::psram_raw_parts` (#2546)
- The timer drivers `OneShotTimer` & `PeriodicTimer` have `into_async` and `new_typed` methods (#2586)
- `timer::Timer` trait has three new methods, `wait`, `async_interrupt_handler` and `peripheral_interrupt` (#2586)
- Configuration structs in the I2C, SPI, and UART drivers now implement the Builder Lite pattern (#2614)
- Added `I8080::apply_config`, `DPI::apply_config` and `Camera::apply_config` (#2610)
- Introduced the `unstable` feature which will be used to restrict stable APIs to a subset of esp-hal. (#2628)
- HAL configuration structs now implement the Builder Lite pattern (#2645)
- Added `OutputOpenDrain::unlisten` (#2625)
- Added `{Input, Flex}::wait_for` (#2625)
- Peripheral singletons now implement `Debug` and `defmt::Format` (#2682, #2834)
- `BurstConfig`, a device-specific configuration for configuring DMA transfers in burst mode (#2543)
- `{DmaRxBuf, DmaTxBuf, DmaRxTxBuf}::set_burst_config` (#2543)
- Added `SpiDmaBus::split` for moving between manual & automatic DMA buffers (#2824)
- ESP32-S2: DMA support for AES (#2699)
- Added `transfer_in_place_async` and embedded-hal-async implementation to `Spi` (#2691)
- `InterruptHandler` now implements `Hash` and `defmt::Format` (#2830)
- `uart::ConfigError` now implements `Eq` (#2825)
- `i2c::master::Error` now implements `Eq` and `Hash` (#2825)
- `i2c::master::Operation` now implements `Debug`, `PartialEq`, `Eq`, `Hash`, and `Display` (#2825)
- `i2c::master::Config` now implements `PartialEq`, `Eq`, ans `Hash` (#2825)
- `i2c::master::I2c` now implements `Debug`, `PartialEq`, and `Eq` (#2825)
- `i2c::master::Info` now implements `Debug` (#2825)
- `spi::master::Config` now implements `Hash` (#2823)
- `spi::master` drivers now implement `Debug` and `defmt::Format` (#2823)
- `DmaRxBuf`, `DmaTxBuf` and `DmaRxTxBuf` now implement `Debug` and `defmt::Format` (#2823)
- DMA channels (`AnyGdmaChannel`, `SpiDmaChannel`, `I2sDmaChannel`, `CryptoDmaChannel`) and their RX/TX halves now implement `Debug` and `defmt::Format` (#2823)
- `DmaDescriptor` and `DmaDescriptorFlags` now implement `PartialEq` and `Eq` (#2823)
- `gpio::{Event, WakeEvent, GpioRegisterAccess}` now implement `Debug`, `Eq`, `PartialEq` and `Hash` (#2842)
- `gpio::{Level, Pull, AlternateFunction, RtcFunction}` now implement `Hash` (#2842)
- `gpio::{GpioPin, AnyPin, Io, Output, OutputOpenDrain, Input, Flex}` now implement `Debug`, `defmt::Format` (#2842)
- More interrupts are available in `esp_hal::spi::master::SpiInterrupt`, add `enable_listen`,`interrupts` and `clear_interrupts` for ESP32/ESP32-S2 (#2833)

- The `ExtU64` and `RateExtU32` traits have been added to `esp_hal::time` (#2845)

### Changed

- Bump MSRV to 1.83 (#2615)
- In addition to taking by value, peripheral drivers can now mutably borrow DMA channel objects. (#2526)
- DMA channel objects are no longer wrapped in `Channel`. The `Channel` drivers are now managed by DMA enabled peripheral drivers. (#2526)
- The `Dpi` driver and `DpiTransfer` now have a `Mode` type parameter. The driver's asyncness is determined by the asyncness of the `Lcd` used to create it. (#2526)
- `dma::{Channel, ChannelRx, ChannelTx}::set_priority` for GDMA devices (#2403)
- `SystemTimer::set_unit_value` & `SystemTimer::configure_unit` (#2576)
- `SystemTimer` no longer uses peripheral ref (#2576)
- `TIMGX` no longer uses peripheral ref (#2581)
- `SystemTimer::now` has been renamed `SystemTimer::unit_value(Unit)` (#2576)
- `SpiDma` transfers now explicitly take a length along with the DMA buffer object (#2587)
- `dma::{Channel, ChannelRx, ChannelTx}::set_priority` for GDMA devices (#2403)
- `SystemTimer`s `Alarm`s are now type erased (#2576)
- `TimerGroup` `Timer`s are now type erased (#2581)
- PSRAM is now initialized automatically if `quad-psram` or `octal-psram` is enabled (#2546)
- DMA channels are now available via the `Peripherals` struct, and have been renamed accordingly. (#2545)
- Moved interrupt related items from lib.rs, moved to the `interrupt` module (#2613)
- The timer drivers `OneShotTimer` & `PeriodicTimer` now have a `Mode` parameter and type erase the underlying driver by default (#2586)
- `timer::Timer` has new trait requirements of `Into<AnyTimer>`, `'static` and `InterruptConfigurable` (#2586)
- `systimer::etm::Event` no longer borrows the alarm indefinitely (#2586)
- A number of public enums and structs in the I2C, SPI, and UART drivers have been marked with `#[non_exhaustive]` (#2614)
- Interrupt handling related functions are only provided for Blocking UART. (#2610)
- Changed how `Spi`, (split or unsplit) `Uart`, `LpUart`, `I8080`, `Camera`, `DPI` and `I2C` drivers are constructed (#2610)
- I8080, camera, DPI: The various standalone configuration options have been merged into `Config` (#2610)
- Dropped GPIO futures stop listening for interrupts (#2625)
- UART driver's `StopBits` enum variants now correctly use UpperCamelCase (#2669)
- The `PeripheralInput` and `PeripheralOutput` traits are now sealed (#2690)
- `esp_hal::sync::Lock` has been renamed to RawMutex (#2684)
- Updated `esp-pacs` with support for Wi-Fi on the ESP32 and made the peripheral non virtual
- `SpiBitOrder`, `SpiDataMode`, `SpiMode` were renamed to `BitOder`, `DataMode` and `Mode` (#2828)
- `crate::Mode` was renamed to `crate::DriverMode` (#2828)
- Renamed some I2C error variants (#2844)
- I2C: Replaced potential panics with errors. (#2831)
- UART: Make `AtCmdConfig` and `ConfigError` non-exhaustive (#2851)
- UART: Make `AtCmdConfig` use builder-lite pattern (#2851)

### Fixed

- Xtensa devices now correctly enable the `esp-hal-procmacros/rtc-slow` feature (#2594)
- User-bound GPIO interrupt handlers should no longer interfere with async pins. (#2625)
- `spi::master::Spi::{into_async, into_blocking}` are now correctly available on the typed driver, to. (#2674)
- It is no longer possible to safely conjure `GpioPin` instances (#2688)
- UART: Public API follows `C-WORD_ORDER` Rust API standard (`VerbObject` order) (#2851)

### Removed

- Remove more examples. Update doctests. (#2547)
- The `configure` and `configure_for_async` DMA channel functions has been removed (#2403)
- The DMA channel objects no longer have `tx` and `rx` fields. (#2526)
- `SysTimerAlarms` has been removed, alarms are now part of the `SystemTimer` struct (#2576)
- `FrozenUnit`, `AnyUnit`, `SpecificUnit`, `SpecificComparator`, `AnyComparator` have been removed from `systimer` (#2576)
- Remove Dma[Rx|Tx]Buffer::length (#2587)
- `esp_hal::psram::psram_range` (#2546)
- The `Dma` structure has been removed. (#2545)
- Removed `embedded-hal 0.2.x` impls and deps from `esp-hal` (#2593)
- Removed `Camera::set_` functions (#2610)
- `DmaTxBuf::{compute_chunk_size, compute_descriptor_count, new_with_block_size}` (#2543)

- The `prelude` module has been removed (#2845)

## [0.22.0] - 2024-11-20

### Added

- A new config option `PLACE_SWITCH_TABLES_IN_RAM` to improve performance (especially for interrupts) at the cost of slightly more RAM usage (#2331)
- A new config option `PLACE_ANON_IN_RAM` to improve performance (especially for interrupts) at the cost of RAM usage (#2331)
- Add burst transfer support to DMA buffers (#2336)
- `AnyPin` now implements `From<GpioPin<N>>`. (#2326)
- Added `AnySpi` and `AnySpiDmaChannel`. (#2334)
- Added `AnyI2s` and `AnyI2sDmaChannel`. (#2367)
- Added `AnyTwai`. (#2359)
- Added `AnyUart`. (#2381)
- `Pins::steal()` to unsafely obtain GPIO. (#2335)
- `I2c::with_timeout` (#2361)
- `Spi::half_duplex_read` and `Spi::half_duplex_write` (#2373)
- Add RGB/DPI driver (#2415)
- Add `DmaLoopBuf` (#2415)
- `Cpu::COUNT` and `Cpu::current()` (#2411)
- `UartInterrupt` and related functions (#2406)
- I2S Parallel output driver for ESP32. (#2348, #2436, #2472)
- Add an option to configure `WDT` action (#2330)
- `DmaDescriptor` is now `Send` (#2456)
- `into_async` and `into_blocking` functions for most peripherals (#2430, #2461)
- API mode type parameter (currently always `Blocking`) to `master::Spi` and `slave::Spi` (#2430)
- `gpio::{GpioPin, AnyPin, Flex, Output, OutputOpenDrain}::split()` to obtain peripheral interconnect signals. (#2418)
- `gpio::Input::{split(), into_peripheral_output()}` when used with output pins. (#2418)
- `gpio::Output::peripheral_input()` (#2418)
- `{Uart, UartRx, UartTx}::apply_config()` (#2449)
- `{Uart, UartRx, UartTx}` now implement `embassy_embedded_hal::SetConfig` (#2449)
- GPIO ETM tasks and events now accept `InputSignal` and `OutputSignal` (#2427)
- `spi::master::Config` and `{Spi, SpiDma, SpiDmaBus}::apply_config` (#2448)
- `embassy_embedded_hal::SetConfig` is now implemented for `spi::master::{Spi, SpiDma, SpiDmaBus}`, `i2c::master::I2c` (#2448, #2477)
- `slave::Spi::{with_mosi(), with_miso(), with_sclk(), with_cs()}` functions (#2485)
- I8080: Added `set_8bits_order()` to set the byte order in 8-bit mode (#2487)
- `I2c::{apply_config(), with_sda(), with_scl()}` (#2477)
- ESP32-S2: Added missing GPIO alternate functions (#2512)

### Changed

- Peripheral type erasure for SPI (#2334)
- Peripheral type erasure for I2S (#2367)
- Peripheral type erasure for I2C (#2361)
- Peripheral type erasure for TWAI (#2359)
- The SPI driver has been rewritten to allow using half-duplex and full-duplex functionality on the same bus. See the migration guide for details. (#2373)
- Renamed `SpiDma` functions: `dma_transfer` to `transfer`, `dma_write` to `write`, `dma_read` to `read`. (#2373)
- Peripheral type erasure for UART (#2381)
- Changed listening for UART events (#2406)
- Circular DMA transfers now correctly error, `available` returns `Result<usize,DmaError>` now (#2409)
- Interrupt listen/unlisten/clear functions now accept any type that converts into `EnumSet` (i.e. single interrupt flags). (#2442)
- SPI interrupt listening is now only available in Blocking mode. The `set_interrupt_handler` is available via `InterruptConfigurable` (#2442)
- Allow users to create DMA `Preparation`s (#2455)
- The `rmt::asynch::RxChannelAsync` and `rmt::asynch::TxChannelAsync` traits have been moved to `rmt` (#2430)
- Calling `AnyPin::output_signals` on an input-only pin (ESP32 GPIO 34-39) will now result in a panic. (#2418)
- UART configuration types have been moved to `esp_hal::uart` (#2449)
- `spi::master::Spi::new()` no longer takes `frequency` and `mode` as a parameter. (#2448)
- Peripheral interconnections via GPIO pins now use the GPIO matrix. (#2419)
- The I2S driver has been moved to `i2s::master` (#2472)
- `slave::Spi` constructors no longer take pins (#2485)
- The `I2c` master driver has been moved from `esp_hal::i2c` to `esp_hal::i2c::master`. (#2476)
- `I2c` SCL timeout is now defined in bus clock cycles. (#2477)
- Trying to send a single-shot RMT transmission will result in an error now, `RMT` deals with `u32` now, `PulseCode` is a convenience trait now (#2463)
- Removed `get_` prefixes from functions (#2528)
- The `Camera` and `I8080` drivers' constructors now only accepts blocking-mode DMA channels. (#2519)
- Many peripherals are now disabled by default and also get disabled when the driver is dropped (#2544)

### Fixed

- Fix conflict between `RtcClock::get_xtal_freq` and `Rtc::disable_rom_message_printing` (#2360)
- Fixed an issue where interrupts enabled before `esp_hal::init` were disabled. This issue caused the executor created by `#[esp_hal_embassy::main]` to behave incorrectly in multi-core applications. (#2377)
- Fixed `TWAI::transmit_async`: bus-off state is not reached when CANH and CANL are shorted. (#2421)
- ESP32: added UART-specific workaround for https://docs.espressif.com/projects/esp-chip-errata/en/latest/esp32/03-errata-description/esp32/cpu-subsequent-access-halted-when-get-interrupted.html (#2441)
- Fixed some SysTimer race conditions and panics (#2451)
- TWAI: accept all messages by default (#2467)
- I8080: `set_byte_order()` now works correctly in 16-bit mode (#2487)
- ESP32-C6/ESP32-H2: Make higher LEDC frequencies work (#2520)

### Removed

- The `i2s::{I2sWrite, I2sWriteDma, I2sRead, I2sReadDma, I2sWriteDmaAsync, I2sReadDmaAsync}` traits have been removed. (#2316)
- The `ledc::ChannelHW` trait is no longer generic. (#2387)
- The `I2c::new_with_timeout` constructors have been removed (#2361)
- `I2c::new()` no longer takes `frequency` and pins as parameters. (#2477)
- The `spi::master::HalfDuplexReadWrite` trait has been removed. (#2373)
- The `Spi::with_pins` methods have been removed. (#2373)
- The `Spi::new_half_duplex` constructor have been removed. (#2373)
- The `HalfDuplexMode` and `FullDuplexMode` parameters have been removed from `Spi`. (#2373)
- Removed the output pin type parameter from `ledc::{Channel, ChannelIFace}` (#2388)
- Removed the output pin type parameter from `mcpwm::operator::{PwmPin, LinkedPins}` (#2388)
- Removed the output pin type parameter from `parl_io::{ClkOutPin, ClkInPin, RxClkInPin}` (#2388)
- Removed the valid pin type parameter from `parl_io::{TxPinConfigWithValidPin, RxPinConfigWithValidPin}` (#2388)
- Removed the pin type parameters from `parl_io::{TxOneBit, TxTwoBits, TxFourBits, TxEightBits, TxSixteenBits}` (#2388)
- Removed the pin type parameters from `parl_io::{RxOneBit, RxTwoBits, RxFourBits, RxEightBits, RxSixteenBits}` (#2388)
- Removed the pin type parameters from `lcd_cam::lcd::i8080::{TxEightBits, TxSixteenBits}` (#2388)
- Removed the pin type parameters from `lcd_cam::cam::{RxEightBits, RxSixteenBits}` (#2388)
- Most of the async-specific constructors (`new_async`, `new_async_no_transceiver`) have been removed. (#2430)
- The `configure_for_async` DMA functions have been removed (#2430)
- The `Uart::{change_baud, change_stop_bits}` functions have been removed (#2449)
- `gpio::{Input, Output, OutputOpenDrain, Flex, GpioPin}::{peripheral_input, into_peripheral_output}` have been removed. (#2418)
- The `GpioEtm` prefix has been removed from `gpio::etm` types (#2427)
- The `TimerEtm` prefix has been removed from `timer::timg::etm` types (#2427)
- The `SysTimerEtm` prefix has been removed from `timer::systimer::etm` types (#2427)
- The `GpioEtmEventRising`, `GpioEtmEventFalling`, `GpioEtmEventAny` types have been replaced with `Event` (#2427)
- The `TaskSet`, `TaskClear`, `TaskToggle` types have been replaced with `Task` (#2427)
- `{Spi, SpiDma, SpiDmaBus}` configuration methods (#2448)
- `Io::new_with_priority` and `Io::new_no_bind_interrupt`. (#2486)
- `parl_io::{no_clk_pin(), NoClkPin}` (#2531)
- Removed `get_core` function in favour of `Cpu::current` (#2533)

- Removed `uart::Config` setters and `symbol_length`. (#2847)

## [0.21.1]

### Fixed

- Restored blocking `embedded_hal` compatibility for async I2C driver (#2343)
- I2c::transaction is now able to transmit data of arbitrary length (#2481)

## [0.21.0]

### Added

- Introduce traits for the DMA buffer objects (#1976, #2213)
- Implement `embedded-hal` output pin traits for `NoPin` (#2019, #2133)
- Added `esp_hal::init` to simplify HAL initialisation (#1970, #1999)
- Added GpioPin::degrade to create ErasePins easily. Same for AnyPin by accident. (#2075)
- Added missing functions to `Flex`: `unlisten`, `is_interrupt_set`, `wakeup_enable`, `wait_for_high`, `wait_for_low`, `wait_for_rising_edge`, `wait_for_falling_edge`, `wait_for_any_edge`. (#2075)
- `Flex` now implements `Wait`. (#2075)
- Added sleep and wakeup support for esp32c2 (#1922)
- `Input`, `Output`, `OutputOpenDrain` and `Flex` now implement `Peripheral`. (#2094)
- Previously unavailable memory is available via `.dram2_uninit` section (#2079)
- You can now use `Input`, `Output`, `OutputOpenDrain` and `Flex` pins as EXTI and RTCIO wakeup sources (#2095)
- Added `Rtc::set_current_time` to allow setting RTC time, and `Rtc::current_time` to getting RTC time while taking into account boot time (#1883)
- Added APIs to allow connecting signals through the GPIO matrix. (#2128)
- Allow I8080 transfers to be cancelled on the spot (#2191)
- Implement `TryFrom<u32>` for `ledc::timer::config::Duty` (#1984)
- Expose `RtcClock::get_xtal_freq` and `RtcClock::get_slow_freq` publically for all chips (#2183)
- TWAI support for ESP32-H2 (#2199)
- Make `DmaDescriptor` methods public (#2237)
- Added a way to configure watchdogs in `esp_hal::init` (#2180)
- Introduce `DmaRxStreamBuf` (#2242)
- Implement `embedded_hal_async::delay::DelayNs` for `TIMGx` timers (#2084)
- Added `Efuse::read_bit` (#2259)
- Limited SPI slave support for ESP32 (Modes 1 and 3 only) (#2278)
- Added `Rtc::disable_rom_message_printing` (S3 and H2 only) (#2280)
- Added `esp_hal::time::{Duration, Instant}` (#2304)

### Changed

- Make saving and restoring SHA digest state an explicit operation (#2049)
- Reordered RX-TX pairs in all APIs to be consistent (#2074)
- Make saving and restoring SHA digest state an explicit operation (#2049)
- `Delay::new()` is now a `const` function (#1999)
- `Input`, `Output`, `OutputOpenDrain` and `Flex` are now type-erased by default. Use the new `new_typed` constructor to keep using the ZST pin types. (#2075)
- To avoid confusion with the `Rtc::current_time` wall clock time APIs, we've renamed `esp_hal::time::current_time` to `esp_hal::time::now`. (#2091)
- Renamed `touch::Continous` to `touch::Continuous`. (#2094)
- Faster SHA (#2112)
- The (previously undocumented) `ErasedPin` enum has been replaced with the `ErasedPin` struct. (#2094)
- Renamed and merged `Rtc::get_time_us` and `Rtc::get_time_ms` into `Rtc::time_since_boot` (#1883)
- ESP32: Added support for touch sensing on GPIO32 and 33 (#2109)
- Removed gpio pin generics from I8080 driver type. (#2171)
- I8080 driver now decides bus width at transfer time rather than construction time. (#2171)
- Migrate the I8080 driver to a move based API (#2191)
- Replaced `AnyPin` with `InputSignal` and `OutputSignal` and renamed `ErasedPin` to `AnyPin` (#2128)
- Replaced the `ErasedTimer` enum with the `AnyTimer` struct. (#2144)
- `Camera` and `AesDma` now support erasing the DMA channel type (#2258)
- Changed the parameters of `Spi::with_pins` to no longer be optional (#2133)
- Renamed `DummyPin` to `NoPin` and removed all internal logic from it. (#2133)
- The `NO_PIN` constant has been removed. (#2133)
- MSRV bump to 1.79 (#2156)
- Allow handling interrupts while trying to lock critical section on multi-core chips. (#2197)
- Migrate `Camera` to a move based API (#2242).
- Removed the PS-RAM related features, replaced by `quad-psram`/`octal-psram`, `init_psram` takes a configuration parameter, it's now possible to auto-detect PS-RAM size (#2178)
- `EspTwaiFrame` constructors now accept any type that converts into `esp_hal::twai::Id` (#2207)
- Change `DmaTxBuf` to support PSRAM on `esp32s3` (#2161)
- I2c `transaction` is now also available as a inherent function, lift size limit on `write`,`read` and `write_read` (#2262)
- SPI transactions are now cancelled if the transfer object (or async Future) is dropped. (#2216)
- The DMA channel types have been removed from peripherals (#2261)
- `I2C` driver renamed to `I2c` (#2320)
- The GPIO pins are now accessible via `Peripherals` and are no longer part of the `Io` struct (#2508)
- `dma::{ChannelRx, ChannelTx}` now have a `Mode` type parameter (#2519)

### Fixed

- SHA driver can now be safely used in multiple contexts concurrently (#2049)
- Fixed an issue with DMA transfers potentially not waking up the correct async task (#2065)
- Fixed an issue with LCD_CAM i8080 where it would send double the clocks in 16bit mode (#2085)
- Fix i2c embedded-hal transaction (#2028)
- Fix some inconsistencies in DMA interrupt bits (#2169)
- Fix SPI DMA alternating `write` and `read` for ESP32 and ESP32-S2 (#2131)
- Fix I2C ending up in a state when only re-creating the peripheral makes it useable again (#2141)
- Fix `SpiBus::transfer` transferring data twice in some cases (#2159)
- Fixed UART freezing when using `RcFast` clock source on ESP32-C2/C3 (#2170)
- I2S: on ESP32 and ESP32-S2 data is now output to the right (WS=1) channel first. (#2194)
- SPI: Fixed an issue where unexpected data was written outside of the read buffer (#2179)
- SPI: Fixed an issue where `wait` has returned before the DMA has finished writing the memory (#2179)
- SPI: Fixed an issue where repeated calls to `dma_transfer` may end up looping indefinitely (#2179)
- SPI: Fixed an issue that prevented correctly reading the first byte in a transaction (#2179)
- SPI: ESP32: Send address with correct data mode even when no data is sent. (#2231)
- SPI: ESP32: Allow using QSPI mode on SPI3. (#2245)
- PARL_IO: Fixed an issue that caused garbage to be output at the start of some requests (#2211)
- TWAI on ESP32 (#2207)
- TWAI should no longer panic when receiving a non-compliant frame (#2255)
- OneShotTimer: fixed `delay_nanos` behaviour (#2256)
- Fixed unsoundness around `Efuse` (#2259)
- Empty I2C writes to unknown addresses now correctly fail with `AckCheckFailed`. (#2506)

### Removed

- Removed `digest::Digest` implementation from SHA (#2049)
- Removed `NoPinType` in favour of `DummyPin`. (#2068)
- Removed the `async`, `embedded-hal-02`, `embedded-hal`, `embedded-io`, `embedded-io-async`, and `ufmt` features (#2070)
- Removed the `GpioN` type aliasses. Use `GpioPin<N>` instead. (#2073)
- Removed `Peripherals::take`. Use `esp_hal::init` to obtain `Peripherals` (#1999)
- Removed `AnyInputOnlyPin` in favour of `AnyPin`. (#2071)
- Removed the following functions from `GpioPin`: `is_high`, `is_low`, `set_high`, `set_low`, `set_state`, `is_set_high`, `is_set_low`, `toggle`. (#2094)
- Removed `Rtc::get_time_raw` (#1883)
- Removed `_with_default_pins` UART constructors (#2132)
- Removed transfer methods `send`, `send_dma` and `send_dma_async` from `I8080` (#2191)
- Removed `uart::{DefaultRxPin, DefaultTxPin}` (#2132)
- Removed `PcntSource` and `PcntInputConfig`. (#2134)
- Removed the `place-spi-driver-in-ram` feature, this is now enabled via [esp-config](https://docs.rs/esp-config) (#2156)
- Removed `esp_hal::spi::slave::prelude` (#2260)
- Removed `esp_hal::spi::slave::WithDmaSpiN` traits (#2260)
- The `WithDmaAes` trait has been removed (#2261)
- The `I2s::new_i2s1` constructor has been removed (#2261)
- `Peripherals.GPIO` has been removed (#2508)

## [0.20.1] - 2024-08-30

### Fixed

- A build issue when including doc comment prelude (#2040)

## [0.20.0] - 2024-08-29

### Added

- Introduce DMA buffer objects (#1856, #1985)
- Added new `Io::new_no_bind_interrupt` constructor (#1861)
- Added touch pad support for esp32 (#1873, #1956)
- Allow configuration of period updating method for MCPWM timers (#1898)
- Add self-testing mode for TWAI peripheral. (#1929)
- Added a `PeripheralClockControl::reset` to the driver constructors where missing (#1893)
- Added `digest::Digest` implementation to SHA (#1908)
- Added `debugger::debugger_connected`. (#1961)
- DMA: don't require `Sealed` to implement `ReadBuffer` and `WriteBuffer` (#1921)
- Allow DMA to/from psram for esp32s3 (#1827)
- Added missing methods to `SpiDmaBus` (#2016).
- PARL_IO use ReadBuffer and WriteBuffer for Async DMA (#1996)

### Changed

- Peripheral driver constructors don't take `InterruptHandler`s anymore. Use `set_interrupt_handler` to explicitly set the interrupt handler now. (#1819)
- Migrate SPI driver to use DMA buffer objects (#1856, #1985)
- Use the peripheral ref pattern for `OneShotTimer` and `PeriodicTimer` (#1855)
- Improve SYSTIMER API (#1871)
- SHA driver now use specific structs for the hashing algorithm instead of a parameter. (#1908)
- Remove `fn free(self)` in HMAC which goes against esp-hal API guidelines (#1972)
- `AnyPin`, `AnyInputOnyPin` and `DummyPin` are now accessible from `gpio` module (#1918)
- Changed the RSA modular multiplication API to be consistent across devices (#2002)

### Fixed

- Improve error detection in the I2C driver (#1847)
- Fix I2S async-tx (#1833)
- Fix PARL_IO async-rx (#1851)
- SPI: Clear DMA interrupts before (not after) DMA starts (#1859)
- SPI: disable and re-enable MISO and MOSI in `start_transfer_dma`, `start_read_bytes_dma` and `start_write_bytes_dma` accordingly (#1894)
- TWAI: GPIO pins are not configured as input and output (#1906)
- ESP32C6: Make ADC usable after TRNG deinicialization (#1945)
- We should no longer generate 1GB .elf files for ESP32C2 and ESP32C3 (#1962)
- Reset peripherals in driver constructors where missing (#1893, #1961)
- Fixed ESP32-S2 systimer interrupts (#1979)
- Software interrupt 3 is no longer available when it is required by `esp-hal-embassy`. (#2011)
- ESP32: Fixed async RSA (#2002)

### Removed

- This package no longer re-exports the `esp_hal_procmacros::main` macro (#1828)
- The `AesFlavour` trait no longer has the `ENCRYPT_MODE`/`DECRYPT_MODE` associated constants (#1849)
- Removed `FlashSafeDma` (#1856)
- Remove redundant WithDmaSpi traits (#1975)
- `IsFullDuplex` and `IsHalfDuplex` traits (#1985)

## [0.19.0] - 2024-07-15

### Added

- uart: Added `with_cts`/`with_rts`s methods to configure CTS, and RTS pins (#1592)
- uart: Constructors now require TX and RX pins (#1592)
- uart: Added `Uart::new_with_default_pins` constructor (#1592)
- uart: Added `UartTx` and `UartRx` constructors (#1592)
- Add Flex / AnyFlex GPIO pin driver (#1659)
- Add new `DmaError::UnsupportedMemoryRegion` - used memory regions are checked when preparing a transfer now (#1670)
- Add DmaTransactionTxOwned, DmaTransactionRxOwned, DmaTransactionTxRxOwned, functions to do owning transfers added to SPI half-duplex (#1672)
- uart: Implement `embedded_io::ReadReady` for `Uart` and `UartRx` (#1702)
- ESP32-S3: Expose optional HSYNC input in LCD_CAM (#1707)
- ESP32-S3: Add async support to the LCD_CAM I8080 driver (#1834)
- ESP32-C6: Support lp-core as wake-up source (#1723)
- Add support for GPIO wake-up source (#1724)
- gpio: add DummyPin (#1769)
- dma: add Mem2Mem to support memory to memory transfer (#1738)
- Add `uart` wake source (#1727)
- `#[ram(persistent)]` option to replace the unsound `uninitialized` option (#1677)
- uart: Make `rx_timeout` optional in Config struct (#1759)
- Add interrupt related functions to `PeriodicTimer`/`OneShotTimer`, added `ErasedTimer` (#1753)
- Added blocking `read_bytes` method to `Uart` and `UartRx` (#1784)
- Add method to expose `InputPin::is_interrupt_set` in `Input<InputPin>` for use in interrupt handlers (#1829)

### Fixed

- ESP32-S3: Fix DMA waiting check in LCD_CAM (#1707)
- TIMG: Fix interrupt handler setup (#1714)
- Fix `sleep_light` for ESP32-C6 (#1720)
- ROM Functions: Fix address of `ets_update_cpu_frequency_rom` (#1722)
- Fix `regi2c_*` functions for `esp32h2` (#1737)
- Improved `#[ram(zeroed)]` soundness by adding a `bytemuck::Zeroable` type bound (#1677)
- EESP32-S2 / ESP32-S3: Fix UsbDm and UsbDp for Gpio19 and Gpio20
- Fix reading/writing small buffers via SPI master async dma (#1760)
- Remove unnecessary delay in rtc_ctnl (#1794)

### Changed

- Refactor `Dac1`/`Dac2` drivers into a single `Dac` driver (#1661)
- esp-hal-embassy: make executor code optional (but default) again
- Improved interrupt latency on RISC-V based chips (#1679)
- `esp_wifi::initialize` no longer requires running maximum CPU clock, instead check it runs above 80MHz. (#1688)
- Move DMA descriptors from DMA Channel to each individual peripheral driver. (#1719)
- Allow users to easily name DMA channels (#1770)
- Support DMA chunk sizes other than the default 4092 (#1758)
- Improved interrupt latency on Xtensa based chips (#1735)
- Improve PCNT api (#1765)

### Removed

- uart: Removed `configure_pins` methods (#1592)
- Removed `DmaError::Exhausted` error by improving the implementation of the `pop` function (#1664)
- Unsound `#[ram(uninitialized)]` option in favor of the new `persistent` option (#1677)

## [0.18.0] - 2024-06-04

### Added

- i2c: implement `I2C:transaction` for `embedded-hal` and `embedded-hal-async` (#1505)
- spi: implement `with_bit_order` (#1537)
- ESP32-PICO-V3-02: Initial support (#1155)
- `time::current_time` API (#1503)
- ESP32-S3: Add LCD_CAM Camera driver (#1483)
- `embassy-usb` support (#1517)
- SPI Slave support for ESP32-S2 (#1562)
- Add new generic `OneShotTimer` and `PeriodicTimer` drivers, plus new `Timer` trait which is implemented for `TIMGx` and `SYSTIMER` (#1570)
- Feature: correct `TRNG` mechanism #1804

### Fixed

- i2c: i2c1_handler used I2C0 register block by mistake (#1487)
- Removed ESP32 specific code for resolutions > 16 bit in ledc embedded_hal::pwm max_duty_cycle function. (#1441)
- Fixed division by zero in ledc embedded_hal::pwm set_duty_cycle function and converted to set_duty_hw instead of set_duty to eliminate loss of granularity. (#1441)
- Embassy examples now build on stable (#1485)
- Fix delay on esp32h2 (#1535)
- spi: fix dma wrong mode when using eh1 blocking api (#1541)
- uart: make `uart::UartRx::read_byte` public (#1547)
- Fix async serial-usb-jtag (#1561)
- Feeding `RWDT` now actually works (#1645)

### Changed

- Removed unneeded generic parameters on `Usb` (#1469)
- Created virtual peripherals for CPU control and radio clocks, rather than splitting them from `SYSTEM` (#1428)
- `IO`, `ADC`, `DAC`, `RTC*`, `LEDC`, `PWM` and `PCNT` drivers have been converted to camel case format (#1473)
- RNG is no longer TRNG, the `CryptoRng` implementation has been removed. To track this being re-added see #1499 (#1498)
- Make software interrupts shareable (#1500)
- The `SystemParts` struct has been renamed to `SystemControl`, and now has a constructor which takes the `SYSTEM` peripheral (#1495)
- Timer abstraction: refactor `systimer` and `timer` modules into a common `timer` module (#1527)
- Removed the `embassy-executor-thread` and `embassy-executor-interrupt` features, they are now enabled by default when `embassy` is enabled. (#1485)
- Software interrupt 3 is now used instead of software interrupt 0 on the thread aware executor on multicore systems (#1485)
- Timer abstraction: refactor `systimer` and `timer` modules into a common `timer` module (#1527)
- Refactoring of GPIO module, have drivers for Input,Output,OutputOpenDrain, all drivers setup their GPIOs correctly (#1542)
- DMA transactions are now found in the `dma` module (#1550)
- Remove unnecessary generics from PARL_IO driver (#1545)
- Use `Level enum` in GPIO constructors instead of plain bools (#1574)
- rmt: make ChannelCreator public (#1597)

### Removed

- Removed the `SystemExt` trait (#1495)
- Removed the `GpioExt` trait (#1496)
- Embassy support (and all related features) has been removed, now available in the `esp-hal-embassy` package instead (#1595)

## [0.17.0] - 2024-04-18

### Added

- Add `ADC::read_blocking` to xtensa chips (#1293)
- ESP32-C6 / ESP32-H2: Implement `ETM` for general purpose timers (#1274)
- `interrupt::enable` now has a direct CPU enable counter part, `interrupt::enable_direct` (#1310)
- `Delay::delay(time: fugit::MicrosDurationU64)`
- Added async support for TWAI (#1320)
- Add TWAI support for ESP32-C6 (#1323)
- `GpioPin::steal` unsafe API (#1363)
- Inherent implementions of GPIO pin `set_low`, `is_low`, etc.
- Warn users when attempting to build using the `dev` profile (#1420)
- Async uart now reports interrupt errors(overflow, glitch, frame error, parity) back to user of read/write. uart clock decimal part configured for c2,c3,s3 (#1168, #1445)
- Add mechanism to configure UART source clock (#1416)
- `GpioPin` got a function `set_state(bool)` (#1462)
- Add definitions of external USB PHY peripheral I/O signals
- Expose e-hal ErrorKind::NoAcknowledge in I2C driver (#1454)
- Add remaining peripheral signals for LCD_CAM (#1466)

### Fixed

- Reserve `esp32` ROM stacks to prevent the trashing of dram2 section (#1289)
- Fixing `esp-wifi` + `TRNG` issue on `ESP32-S2` (#1272)
- Fixed core1 startup using the wrong stack on the esp32 and esp32s3 (#1286).
- ESP32: Apply fix for Errata 3.6 in all the places necessary. (#1315)
- ESP32 & ESP32-S2: Fix I²C frequency (#1306)
- UART's TX/RX FIFOs are now cleared during initialization (#1344)
- Fixed `LCD_CAM I8080` driver potentially sending garbage to display (#1301)
- The TWAI driver can now be used without requiring the `embedded-hal` traits (#1355)
- USB pullup/pulldown now gets properly cleared and does not interfere anymore on esp32c3 and esp32s3 (#1244)
- Fixed GPIO counts so that using async code with the higher GPIO number should no longer panic (#1361, #1362)
- ESP32/ESP32-S2: Wait for I2S getting out of TX_IDLE when starting a transfer (#1375)
- Fixed writes to SPI not flushing before attempting to write, causing corrupted writes (#1381)
- fix AdcConfig::adc_calibrate for xtensa targets (#1379)
- Fixed a divide by zero panic when setting the LEDC duty cycle to 0 with `SetDutyCycle::set_duty_cycle` (#1403)
- Support 192 and 256-bit keys for AES (#1316)
- Fixed MCPWM DeadTimeCfg bit values (#1378)
- ESP32 LEDC `set_duty_cycle` used HighSpeedChannel for LowSpeedChannel (#1457)

### Changed

- TIMG: Allow use without the embedded-hal-02 traits in scope (#1367)
- DMA: use channel clusters
- Remove `Ext32` and `RateExtU64` from prelude
- Prefer mutable references over moving for DMA transactions (#1238)
- Support runtime interrupt binding, adapt GPIO driver (#1231)
- Renamed `eh1` feature to `embedded-hal`, feature-gated `embedded-hal@0.2.x` trait implementations (#1273)
- Enable `embedded-hal` feature by default, instead of the `embedded-hal-02` feature (#1313)
- `Uart` structs now take a `Mode` parameter which defines how the driver is initialized (#1294)
- `Rmt` can be created in async or blocking mode. The blocking constructor takes an optional interrupt handler argument. (#1341)
- All `Instance` traits are now sealed, and can no longer be implemented for arbitrary types (#1346)
- DMA channels can/have to be explicitly created for async or blocking drivers, added `set_interrupt_handler` to DMA channels, SPI, I2S, PARL_IO, don't enable interrupts on startup for DMA, I2S, PARL_IO, GPIO (#1300)
- UART: Rework `change_baud` so it is possible to set baud rate even after instantiation (#1350)
- Runtime ISR binding for SHA,ECC and RSA (#1354)
- Runtime ISR binding for I2C (#1376)
- `UsbSerialJtag` can be created in async or blocking mode. The blocking constructor takes an optional interrupt handler argument (#1377)
- SYSTIMER and TIMG instances can now be created in async or blocking mode (#1348)
- Runtime ISR binding for TWAI (#1384)
- ESP32-C6: The `gpio::lp_gpio` module has been renamed to `gpio::lp_io` to match the peripheral name (#1397)
- Runtime ISR binding for assist_debug (#1395)
- Runtime ISR binding for software interrupts, software interrupts are split now, interrupt-executor takes the software interrupt to use, interrupt-executor is easier to use (#1398)
- PCNT: Runtime ISR binding (#1396)
- Runtime ISR binding for RTC (#1405)
- Improve MCPWM DeadTimeCfg API (#1378)
- `SystemTimer`'s `Alarm` methods now require `&mut self` (#1455)

### Removed

- Remove package-level type exports (#1275)
- Removed `direct-vectoring` & `interrupt-preemption` features, as they are now enabled by default (#1310)
- Removed the `rt` and `vectored` features (#1380)
- Remove partial support for the ESP32-P4 (#1461)

## [0.16.1] - 2024-03-12

- Resolved an issue with the `defmt` dependency/feature (#1264)

### Changed

- Use ROM `memcpy` over compiler builtins (#1255)
- Do not ensure randomness or implement the `CryptoRng` trait for ESP32-P4/S2 (#1267)

## [0.16.0] - 2024-03-08

### Added

- Add initial support for the ESP32-P4 (#1101)
- Implement `embedded_hal::pwm::SetDutyCycle` trait for `ledc::channel::Channel` (#1097)
- ESP32-P4: Add initial GPIO support (#1109)
- ESP32-P4: Add initial support for interrupts (#1112)
- ESP32-P4: Add efuse reading support (#1114)
- ESP32-S3: Added LCD_CAM I8080 driver (#1086)
- Allow for splitting of the USB Serial JTAG peripheral into tx/rx components (#1024)
- `RngCore` trait is implemented (#1122)
- Support Rust's `stack-protector` feature (#1135)
- Adding clock support for `ESP32-P4` (#1145)
- Implementation OutputPin and InputPin for AnyPin (#1067)
- Implement `estimate_xtal_frequency` for ESP32-C6 / ESP32-H2 (#1174)
- A way to push into I2S DMA buffer via a closure (#1189)
- Added basic `LP-I2C` driver for C6 (#1185)
- Ensuring that the random number generator is TRNG. (#1200)
- ESP32-C6: Add timer wakeup source for deepsleep (#1201)
- Introduce `InterruptExecutor::spawner()` (#1211)
- Add `InterruptHandler` struct, which couples interrupt handlers and their priority together (#1299)

### Fixed

- Fix embassy-time tick rate not set when using systick as the embassy timebase (#1124)
- Fix `get_raw_core` on Xtensa (#1126)
- Fix docs.rs documentation builds (#1129)
- Fix circular DMA (#1144)
- Fix `hello_rgb` example for ESP32 (#1173)
- Fixed the multicore critical section on Xtensa (#1175)
- Fix timer `now` for esp32c3 and esp32c6 (#1178)
- Wait for registers to get synced before reading the timer count for all chips (#1183)
- Fix I2C error handling (#1184)
- Fix circular DMA (#1189)
- Fix esp32c3 uart initialization (#1156)
- Fix ESP32-S2 I2C read (#1214)
- Reset/init UART if it's not the console UART (#1213)

### Changed

- DmaDescriptor struct to better model the hardware (#1054)
- DMA descriptor count no longer needs to be multiplied by 3 (#1054)
- RMT channels no longer take the channel number as a generic param (#959)
- The `esp-hal-common` package is now called `esp-hal` (#1131)
- Refactor the `Trace` driver to be generic around its peripheral (#1140)
- Auto detect crystal frequency based on `RtcClock::estimate_xtal_frequency()` (#1165)
- ESP32-S3: Configure 32k ICACHE (#1169)
- Lift the minimal buffer size requirement for I2S (#1189)
- Replaced `SystemTimer::TICKS_PER_SEC` with `SystemTimer::ticks_per_sec()` (#1981)

### Removed

- Remove `xtal-26mhz` and `xtal-40mhz` features (#1165)
- All chip-specific HAL packages have been removed (#1196)

### Breaking

- `ADC` and `DAC` drivers now take virtual peripherals in their constructors, instead of splitting `APB_SARADC`/`SENS` (#1100)
- The `DAC` driver's constructor is now `new` instead of `dac`, to be more consistent with other APIs (#1100)
- The DMA peripheral is now called `Dma` for devices with both PDMA and GDMA controllers (#1125)
- The `ADC` driver's constructor is now `new` instead of `adc`, to be more consistent with other APIs (#1133)
- `embassy-executor`'s `integrated-timers` is no longer enabled by default.
- Renamed `embassy-time-systick` to `embassy-time-systick-16mhz` for use with all chips with a systimer, except `esp32s2`. Added `embassy-time-systick-80mhz` specifically for the `esp32s2`. (#1247)

## [0.15.0] - 2024-01-19

### Added

- ESP32-C6: Properly initialize PMU (#974)
- Implement overriding base mac address (#1044)
- Add `rt-riscv` and `rt-xtensa` features to enable/disable runtime support (#1057)
- ESP32-C6: Implement deep sleep (#918)
- Add `embedded-io` feature to each chip-specific HAL (#1072)
- Add `embassy-time-driver` to `esp-hal-common` due to updating `embassy-time` to `v0.3.0` (#1075)
- ESP32-S3: Added support for 80Mhz PSRAM (#1069)
- ESP32-C3/S3: Add workaround for USB pin exchange on usb-serial-jtag (#1104).
- ESP32C6: Added LP_UART initialization (#1113)
- Add `place-spi-driver-in-ram` feature to `esp-hal-common` (#1096)

### Changed

- Set up interrupts for the DMA and async enabled peripherals only when `async` feature is provided (#1042)
- Update to `1.0.0` releases of the `embedded-hal-*` packages (#1068)
- Update `embassy-time` to `0.3.0` and embassy-executor to `0.5.0` release due to the release of the `embedded-hal-*` packages (#1075)
- No longer depend on `embassy-time` (#1092)
- Update to latest `smart-leds-trait` and `smart-leds` packages (#1094)

### Fixed

- ESP32: correct gpio 32/33 in errata36() (#1053)
- ESP32: make gpio 4 usable as analog pin (#1078)
- Fix double &mut for the `SetDutyCycle` impl on `PwmPin` (#1033)
- ESP32/ESP32-S3: Fix stack-top calculation for app-core (#1081)
- ESP32/ESP32-S2/ESP32-S3: Fix embassy-time-timg0 driver (#1091)
- ESP32: ADC readings are no longer inverted (#1093)

### Removed

### Breaking

- Unify the low-power peripheral names (`RTC_CNTL` and `LP_CLKRST` to `LPWR`) (#1064)

## [0.14.1] - 2023-12-13

### Fixed

- Fix SHA for all targets (#1021)

## [0.14.0] - 2023-12-12

### Added

- ESP32-C6: LP core clock is configurable (#907)
- Derive `Clone` and `Copy` for `EspTwaiFrame` (#914)
- A way to configure inverted pins (#912)
- Added API to check a GPIO-pin's interrupt status bit (#929)
- A `embedded_io_async::Read` implementation for `UsbSerialJtag` (#889)
- `RtcClock::get_xtal_freq`, `RtcClock::get_slow_freq` (#957)
- Added Rx Timeout functionality to async Uart (#911)
- RISC-V: Thread-mode and interrupt-mode executors, `#[main]` macro (#947)
- A macro to make it easier to create DMA buffers and descriptors (#935)
- I2C timeout is configurable (#1011)
- ESP32-C6/ESP32-H2: `flip-link` feature gives zero-cost stack overflow protection (#1008)

### Changed

- Improve DMA documentation & clean up module (#915)
- Only allow a single version of `esp-hal-common` to be present in an application (#934)
- ESP32-C3/C6 and ESP32-H2 can now use the `zero-rtc-bss` feature to enable `esp-hal-common/rv-zero-rtc-bss` (#867)
- Reuse `ieee802154_clock_enable/disable()` functions for BLE and rename `ble_ieee802154_clock_enable()` (#953)
- The `embedded-io` trait implementations are now gated behind the `embedded-io` feature (#964)
- Simplifed RMT channels and channel creators (#958)
- Reworked construction of I2S driver instances (#983)
- ESP32-S2/S3: Don't require GPIO 18 to create a USB peripheral driver instance (#990)
- Updated to latest release candidate (`1.0.0-rc.2`) for `embedded-hal{-async,-nb}` (#994)
- Explicit panic when hitting the `DefaultHandler` (#1005)
- Relevant interrupts are now auto enabled in `embassy::init` (#1014).

### Fixed

- ESP32-C2/C3 examples: fix build error (#899)
- ESP32-S3: Fix GPIO interrupt handler crashing when using GPIO48. (#898)
- Fixed short wait times in embassy causing hangs (#906)
- Make sure to clear LP/RTC RAM before loading code (#916)
- Async RMT channels can be used concurrently (#925)
- Xtensa: Allow using `embassy-executor`'s thread-mode executor if neither `embassy-executor-thread`, nor `embassy-executor-interrupt` is enabled. (#937)
- Uart Async: Improve interrupt handling and irq <--> future communication (#977)
- RISC-V: Fix stack allocation (#988)
- ESP32-C6: Fix used RAM (#997)
- ESP32-H2: Fix used RAM (#1003)
- Fix SPI slave DMA dma_read and dma_write (#1013)
- ESP32-C6/H2: Fix disabling of interrupts (#1040)

### Removed

- Direct boot support has been removed (#903).
- Removed the `mcu-boot` feature from `esp32c3-hal` (#938)
- Removed SpiBusController and SpiBusDevice in favour of embedded-hal-bus and embassy-embedded-hal implementataions. (#978)

### Breaking

- `Spi::new`/`Spi::new_half_duplex` takes no gpio pin now, instead you need to call `with_pins` to setup those (#901).
- ESP32-C2, ESP32-C3, ESP32-S2: atomic emulation trap has been removed. (#904) (#985)
  - When upgrading you must either remove [these lines](https://github.com/esp-rs/riscv-atomic-emulation-trap#usage) from your `.cargo/config.toml`.
  - Usage of `core::sync::atomic::*` in dependent crates should be replaced with [portable-atomic](https://github.com/taiki-e/portable-atomic).
- RSA driver now takes `u32` words instead of `u8` bytes. The expected slice length is now 4 times shorter. (#981)

## [0.13.1] - 2023-11-02

### Fixed

- ESP32-C3: Make sure BLE and WiFi are not powered down when esp-wifi needs them (#891)
- ESP32-C6/H2: Fix setting UART baud rate (#893)

## [0.13.0] - 2023-10-31

### Added

- Implement SetFrequencyCycle and PwmPin from embedded_hal for PwmPin of MCPWM. (#880)
- Added `embassy-time-systick` to ESP32-S2 (#827)
- Implement enabling/disabling BLE clock on ESP32-C6 (#784)
- Async support for RMT (#787)
- Implement `defmt::Format` for more types (#786)
- Add new_no_miso to Spi FullDuplexMode (#794)
- Add UART support for splitting into TX and RX (#754)
- Async support for I2S (#801)
- Async support for PARL_IO (#807)
- ETM driver, GPIO ETM (#819)
- (G)DMA AES support (#821)
- SYSTIMER ETM functionality (#828)
- Adding async support for RSA peripheral(doesn't work properly for `esp32` chip - issue will be created)(#790)
- Added sleep support for ESP32-C3 with timer and GPIO wakeups (#795)
- Support for ULP-RISCV including Delay and GPIO (#840, #845)
- Add bare-bones SPI slave support, DMA only (#580, #843)
- Embassy `#[main]` convenience macro (#841)
- Add a `defmt` feature to the `esp-hal-smartled` package (#846)
- Support 16MB octal PS-RAM for ESP32-S3 (#858)
- RISCV TRACE Encoder driver for ESP32-C6 / ESP32-H2 (#864)
- `embedded_hal` 1 `InputPin` and `embedded_hal_async` `Wait` impls for open drain outputs (#905)

### Changed

- Bumped MSRV to 1.67 (#798)
- Optimised multi-core critical section implementation (#797)
- Changed linear- and curve-calibrated ADC to provide readings in mV (#836)

### Fixed

- S3: Allow powering down RC_FAST_CLK (#796)
- UART/ESP32: fix calculating FIFO counter with `get_rx_fifo_count()` (#804)
- Xtensa targets: Use ESP32Reset - not Reset (#823)
- Examples should now work with the `defmt` feature (#810)
- Fixed a race condition causing SpiDma to stop working unexpectedly (#869)
- Fixed async uart serial, and updated the embassy_serial examples (#871).
- Fix ESP32-S3 direct-boot (#873)
- Fix ESP32-C6 ADC (#876)
- Fix ADC Calibration not being used on ESP32-S2 and ESP32-S3 (#1000)

### Removed

- `Pin::is_pcore_interrupt_set` (#793)
- `Pin::is_pcore_non_maskable_interrupt_set` (#793)
- `Pin::is_acore_interrupt_set` (#793)
- `Pin::is_acore_non_maskable_interrupt_set` (#793)
- `Pin::enable_hold` (#793)
- Removed the generic return type for ADC reads (#792)

### Breaking

- `Uart::new` now takes the `&Clocks` struct to ensure baudrate is correct for CPU/APB speed. (#808)
- `Uart::new_with_config` takes an `Config` instead of `Option<Config>`. (#808)
- `Alarm::set_period` takes a period (duration) instead of a frequency (#812)
- `Alarm::interrupt_clear` is now `Alarm::clear_interrupt` to be consistent (#812)
- The `PeripheralClockControl` struct is no longer public, drivers no longer take this as a parameter (#817)
- Unify the system peripheral, `SYSTEM`, `DPORT` and `PCR` are now all exposed as `SYSTEM` (#832).
- Unified the ESP32's and ESP32-C2's xtal frequency features (#831)
- Replace any underscores in feature names with dashes (#833)
- The `spi` and `spi_slave` modules have been refactored into the `spi`, `spi::master`, and `spi::slave` modules (#843)
- The `WithDmaSpi2`/`WithDmaSpi3` structs are no longer generic around the inner peripheral type (#853)
- The `SarAdcExt`/`SensExt` traits are now collectively named `AnalogExt` instead (#857)
- Replace the `radio` module with peripheral singleton structs (#852)
- The SPI traits are no longer re-exported in the main prelude, but from preludes in `spi::master`/`spi::slave` instead (#860)
- The `embedded-hal-1` and `embedded-hal-async` traits are no longer re-exported in the prelude (#860)

## [0.12.0] - 2023-09-05

### Added

- Implement RTCIO pullup, pulldown and hold control for Xtensa MCUs (#684)
- S3: Implement RTCIO wakeup source (#690)
- Add PARL_IO driver for ESP32-C6 / ESP32-H2 (#733, #760)
- Implement `ufmt_write::uWrite` trait for USB Serial JTAG (#751)
- Add HMAC peripheral support (#755)
- Add multicore-aware embassy executor for Xtensa MCUs (#723, #756).
- Add interrupt-executor for Xtensa MCUs (#723, #756).
- Add missing `Into<Gpio<Analog, GPIONUN>>` conversion (#764)
- Updated `clock` module documentation (#774)
- Add `log` feature to enable log output (#773)
- Add `defmt` feature to enable log output (#773)
- A new macro to load LP core code on ESP32-C6 (#779)
- Add `ECC`` peripheral driver (#785)
- Initial LLD support for Xtensa chips (#861).

### Changed

- Update the `embedded-hal-*` packages to `1.0.0-rc.1` and implement traits from `embedded-io` and `embedded-io-async` (#747)
- Moved AlignmentHelper to its own module (#753)
- Disable all watchdog timers by default at startup (#763)
- `log` crate is now opt-in (#773)

### Fixed

- Fix `psram` availability lookup in `esp-hal-common` build script (#718)
- Fix wrong `dram_seg` length in `esp32s2-hal` linker script (#732)
- Fix setting alarm when a timer group is used as the alarm source. (#730)
- Fix `Instant::now()` not counting in some cases when using TIMG0 as the timebase (#737)
- Fix number of ADC attenuations for ESP32-C6 (#771)
- Fix SHA registers access (#805)

### Breaking

- `CpuControl::start_app_core()` now takes an `FnOnce` closure (#739)

## [0.11.0] - 2023-08-10

### Added

- Add initial LP-IO support for ESP32-C6 (#639)
- Implement sleep with some wakeup methods for `esp32` (#574)
- Add a new RMT driver (#653, #667, #695)
- Implemented calibrated ADC API for ESP32-S3 (#641)
- Add MCPWM DeadTime configuration (#406)
- Implement sleep with some wakeup methods for `esp32-s3` (#660, #689, #696)
- Add feature enabling directly hooking the interrupt vector table (#621)
- Add `ClockControl::max` helper for all chips (#701)
- Added module-level documentation for all peripherals (#680)
- Implement sleep with some wakeup methods for `esp32-s3` (#660)
- Add `FlashSafeDma` wrapper for eh traits which ensure correct DMA transfer from source data in flash (ROM) (#678)

### Changed

- Update `embedded-hal-*` alpha packages to their latest versions (#640)
- Implement the `Clone` and `Copy` traits for the `Rng` driver (#650)
- Use all remaining memory as core-0's stack (#716)

### Fixed

- Fixed Async Uart `read` when `set_at_cmd` is not used (#652)
- USB device support is working again (#656)
- Add missing interrupt status read for esp32s3, which fixes USB-SERIAL-JTAG interrupts (#664)
- GPIO interrupt status bits are now properly cleared (#670)
- Increase frequency resolution in `set_periodic` (#686)
- Fixed ESP32-S2, ESP32-S3, ESP32-C2, ESP32-C3 radio clock gating (#679, #681)
- Partially fix ESP32 radio clocks (#709)
- Fixed "ESP32/ESP32-S2 RMT transmission with with data.len() > RMT_CHANNEL_RAM_SIZE results in TransmissionError" #707 (#710)

### Removed

- Remove the `allow-opt-level-z` feature from `esp32c3-hal` (#654)
- Remove the old `pulse_control` driver (#694)

### Breaking

- `DmaTransfer::wait` and `I2sReadDmaTransfer::wait_receive` now return `Result` (#665)
- `gpio::Pin` is now object-safe (#687)

## [0.10.0] - 2023-06-04

### Added

- Add `WithDmaSpi3` to prelude for ESP32S3 (#623)
- Add bare-bones PSRAM support for ESP32 (#506)
- Add initial support for the ESP32-H2 (#513, #526, #527, #528, #530, #538, #544, #548, #551, #556, #560, #566, #549, #564, #569, #576, #577, #589, #591, #597)
- Add bare-bones PSRAM support for ESP32-S3 (#517)
- Add async support to the I2C driver (#519)
- Implement Copy and Eq for EspTwaiError (#540)
- Add LEDC hardware fade support (#475)
- Added support for multicore async GPIO (#542)
- Add a fn to poll DMA transfers (#559)
- Add unified field-based efuse access (#567)
- Move `esp-riscv-rt` into esp-hal (#578)
- Add CRC functions from ESP ROM (#587)
- Add a `debug` feature to enable the PACs' `impl-register-debug` feature (#596)
- Add initial support for `I2S` in ESP32-H2 (#597)
- Add octal PSRAM support for ESP32-S3 (#610)
- Add MD5 functions from ESP ROM (#618)
- Add embassy async `read` support for `uart` (#620)
- Add bare-bones support to run code on ULP-RISCV / LP core (#631)
- Add ADC calibration implementation for a riscv chips (#555)
- Add `async` implementation for `USB Serial/JTAG`(#632)

### Changed

- Simplify the `Delay` driver, derive `Clone` and `Copy` (#568)
- DMA types can no longer be constructed by the user (#625)
- Move core interrupt handling from Flash to RAM for RISC-V chips (ESP32-H2, ESP32-C2, ESP32-C3, ESP32-C6) (#541)
- Change LED pin to GPIO2 in ESP32 blinky example (#581)
- Update ESP32-H2 and ESP32-C6 clocks and remove `i2c_clock` for all chips but ESP32 (#592)
- Use both timers in `TIMG0` for embassy time driver when able (#609)
- Re-work `RadioExt` implementations, add support for ESP32-H2 (#627)
- Improve examples documentation (#533)
- esp32h2-hal: added README (#585)
- Update `esp-hal-procmacros` package dependencies and features (#628)

### Fixed

- Corrected the expected DMA descriptor counts (#622, #625)
- DMA is supported for SPI3 on ESP32-S3 (#507)
- `change_bus_frequency` is now available on `SpiDma` (#529)
- Fixed a bug where a GPIO interrupt could erroneously fire again causing the next `await` on that pin to instantly return `Poll::Ok` (#537)
- Set `vecbase` on core 1 (ESP32, ESP32-S3) (#536)
- ESP32-S3: Move PSRAM related function to RAM (#546)
- ADC driver will now apply attenuation values to the correct ADC's channels. (#554)
- Sometimes half-duplex non-DMA SPI reads were reading garbage in non-release mode (#552)
- ESP32-C3: Fix GPIO5 ADC channel id (#562)
- ESP32-H2: Fix direct-boot feature (#570)
- Fix Async GPIO not disabling interupts on chips with multiple banks (#572)
- ESP32-C6: Support FOSC CLK calibration for ECO1+ chip revisions (#593)
- Fixed CI by pinning the log crate to 0.4.18 (#600)
- ESP32-S3: Fix calculation of PSRAM start address (#601)
- Fixed wrong variable access (FOSC CLK calibration for ESP32-C6 #593)
- Fixed [trap location in ram](https://github.com/esp-rs/esp-hal/pull/605#issuecomment-1604039683) (#605)
- Fix rom::crc docs (#611)
- Fixed a possible overlap of `.data` and `.rwtext` (#616)
- Avoid SDA/SCL being low while configuring pins for I2C (#619)

### Breaking

- Simplified user-facing SpiDma and I2s types (#626)
- Significantly simplified user-facing GPIO pin types. (#553)
- No longer re-export the `soc` module and the contents of the `interrupt` module at the package level (#607)

## [0.9.0] - 2023-05-02

### Added

- Add bare-bones PSRAM support for ESP32-S2 (#493)
- Add `DEBUG_ASSIST` functionality (#484)
- Add RSA peripheral support (#467)
- Add PeripheralClockControl argument to `timg`, `wdt`, `sha`, `usb-serial-jtag` and `uart` constructors (#463)
- Added API to raise and reset software interrupts (#426)
- Implement `embedded_hal_nb::serial::*` traits for `UsbSerialJtag` (#498)

### Fixed

- Fix `get_wakeup_cause` comparison error (#472)
- Use 192 as mclk_multiple for 24-bit I2S (#471)
- Fix `CpuControl::start_app_core` signature (#466)
- Move `rwtext` after other RAM data sections (#464)
- ESP32-C3: Disable `usb_pad_enable` when setting GPIO18/19 to input/output (#461)
- Fix 802.15.4 clock enabling (ESP32-C6) (#458)
- ESP32-S3: Disable usb_pad_enable when setting GPIO19/20 to input/output (#645)

### Changed

- Update `embedded-hal-async` and `embassy-*` dependencies (#488)
- Update to `embedded-hal@1.0.0-alpha.10` and `embedded-hal-nb@1.0.0-alpha.2` (#487)
- Let users configure the LEDC output pin as open-drain (#474)
- Use bitflags to decode wakeup cause (#473)
- Minor linker script additions (#470)
- Minor documentation improvements (#460)

### Removed

- Remove unnecessary generic from `UsbSerialJtag` driver (#492)
- Remove `#[doc(inline)]` from esp-hal-common re-exports (#490)

## [0.8.0] - 2023-03-27

## [0.7.1] - 2023-02-22

## [0.7.0] - 2023-02-21

## [0.5.0] - 2023-01-26

## [0.4.0] - 2022-12-12

## [0.3.0] - 2022-11-17

## [0.2.0] - 2022-09-13

## [0.1.0] - 2022-08-05

[Unreleased]: https://github.com/esp-rs/esp-hal/compare/v0.22.0...HEAD
[0.22.0]: https://github.com/esp-rs/esp-hal/compare/v0.21.1...v0.22.0
[0.21.1]: https://github.com/esp-rs/esp-hal/compare/v0.21.0...v0.21.1
[0.21.0]: https://github.com/esp-rs/esp-hal/compare/v0.20.1...v0.21.0
[0.20.1]: https://github.com/esp-rs/esp-hal/compare/v0.20.0...v0.20.1
[0.20.0]: https://github.com/esp-rs/esp-hal/compare/v0.19.0...v0.20.0
[0.19.0]: https://github.com/esp-rs/esp-hal/compare/v0.18.0...v0.19.0
[0.18.0]: https://github.com/esp-rs/esp-hal/compare/v0.17.0...v0.18.0
[0.17.0]: https://github.com/esp-rs/esp-hal/compare/v0.16.1...v0.17.0
[0.16.1]: https://github.com/esp-rs/esp-hal/compare/v0.16.0...v0.16.1
[0.16.0]: https://github.com/esp-rs/esp-hal/compare/v0.15.0...v0.16.0
[0.15.0]: https://github.com/esp-rs/esp-hal/compare/v0.14.1...v0.15.0
[0.14.1]: https://github.com/esp-rs/esp-hal/compare/v0.14.0...v0.14.1
[0.14.0]: https://github.com/esp-rs/esp-hal/compare/v0.13.1...v0.14.0
[0.13.1]: https://github.com/esp-rs/esp-hal/compare/v0.13.0...v0.13.1
[0.13.0]: https://github.com/esp-rs/esp-hal/compare/v0.12.0...v0.13.0
[0.12.0]: https://github.com/esp-rs/esp-hal/compare/v0.11.0...v0.12.0
[0.11.0]: https://github.com/esp-rs/esp-hal/compare/v0.10.0...v0.11.0
[0.10.0]: https://github.com/esp-rs/esp-hal/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/esp-rs/esp-hal/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/esp-rs/esp-hal/compare/v0.7.1...v0.8.0
[0.7.1]: https://github.com/esp-rs/esp-hal/compare/v0.7.0...v0.7.1
[0.7.0]: https://github.com/esp-rs/esp-hal/compare/v0.5.0...v0.7.0
[0.5.0]: https://github.com/esp-rs/esp-hal/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/esp-rs/esp-hal/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/esp-rs/esp-hal/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/esp-rs/esp-hal/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/esp-rs/esp-hal/releases/tag/v0.1.0
