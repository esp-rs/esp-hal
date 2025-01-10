# `esp-rs` API Guidelines

## About

This is a living document - make sure to check the latest version of this document.

> [!NOTE]
> Not all of the currently existing code follows this guideline, yet.

In general, the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines) apply to all projects in the ESP-RS GitHub organization where possible.
  - Especially for public API but if possible also for internal APIs.

## Amendments to the Rust API Guidelines

- `C-RW-VALUE` and `C-SERDE` do not apply.
- `C-QUESTION-MARK`: `?` is not applicable in the context of the `main` function in our driver, documentation example makes no sense.
- `C-COMMON-TRAITS`:
  The set of traits to implement depend on the type and use case. In esp-hal, we can highlight a few such use cases and provide recommendations what should be implemented. If nothing here applies, use your best judgement.
  - Driver structures: `Debug`
  - Driver configuration: `Default`, `Debug`, `PartialEq/Eq`, `Clone/Copy`, `Hash`
    - `Clone/Copy` depends on the size and contents of the structure. They should generally be implemented, unless there is a good reason not to.
    - The `Default` configuration needs to make sense for a particular driver, and applying the default configuration must not fail.
  - Error types: `Debug`, `PartialEq/Eq`, `Clone/Copy`, `Hash`, `Error`, `Display`

## Construction and Destruction of Drivers

- Drivers must take peripherals via the `PeripheralRef` pattern - they don't consume peripherals directly.
- If a driver requires pins, those pins should be configured using `fn with_signal_name(self, pin: impl Peripheral<P = impl PeripheralInput> + 'd) -> Self` or `fn with_signal_name(self, pin: impl Peripheral<P = impl PeripheralOutput> + 'd) -> Self`
- If a driver supports multiple peripheral instances (for example, I2C0 is one such instance):
  - The driver should not be generic over the peripheral instance.
  - The author must to use `crate::any_peripheral` to define the "any" peripheral instance type.
  - The driver must implement a `new` constructor that automatically converts the peripheral instance into the any type.
- If a driver is configurable, configuration options should be implemented as a `Config` struct in the same module where the driver is located.
  - The driver's constructor should take the config struct by value, and it should return `Result<Self, ConfigError>`.
  - The `ConfigError` enum should be separate from other `Error` enums used by the driver.
  - The driver should implement `fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError>`.
  - In case the driver's configuration is infallible (all possible combinations of options are supported by the hardware), the `ConfigError` should be implemented as an empty `enum`.
  - Configuration structs should derive `procmacros::BuilderLite` in order to automatically implement the Builder Lite pattern for them.
- If a driver implements both blocking and async operations, or only implements blocking operations, but may support asynchronous ones in the future, the driver's type signature must include a `crate::Mode` type parameter.
- By default, constructors must configure the driver for blocking mode. The driver must implement `into_async` (and a matching `into_blocking`) function that reconfigures the driver.
  - `into_async` must configure the driver and/or the associated DMA channels. This most often means enabling an interrupt handler.
  - `into_blocking` must undo the configuration done by `into_async`.
- The asynchronous driver implementation must also expose the blocking methods (except for interrupt related functions).
- Drivers must have a `Drop` implementation resetting the peripheral to idle state. There are some exceptions to this:
  - GPIO where common usage is to "set and drop" so they can't be changed
  - Where we don't want to disable the peripheral as it's used internally, for example SYSTIMER is used by `time::now()` API. See `KEEP_ENABLED` in src/system.rs
  - A driver doesn't need to do anything special for deinitialization and has a `PeripheralGuard` field which implements the disabling and resetting of the peripheral.
- Consider using a builder-like pattern for driver construction.

## Interoperability

- Don't use `log::XXX!` macros directly - use the wrappers in `fmt.rs` (e.g. just `info!` instead of `log::info!` or importing `log::*`)!
- Consider implementing common ecosystem traits, like the ones in `embedded-hal` or `embassy-embedded-hal`.
  - Where the guidelines suggest implementing `Debug`, `defmt::Format` should also be implemented.
    - The `defmt::Format` implementation needs to be gated behind the `defmt` feature.
    - see [this example](https://github.com/esp-rs/esp-hal/blob/df2b7bd8472cc1d18db0d9441156575570f59bb3/esp-hal/src/spi/mod.rs#L15)
    - e.g. `#[cfg_attr(feature = "defmt", derive(defmt::Format))]`
  - Implementations of common, but unstable traits (e.g. `embassy_embedded_hal::SetConfig`) need to be gated with the `unstable` feature.

## API Surface

- API documentation must be provided for every new driver and API.
- Private details should not leak into the public API, and should be made private where technically possible.
  - Implementation details that _need_ to be public should be marked with `#[doc(hidden)]` and a comment as to why it needs to be public.
    - For the time being, this includes any `Instance` traits, and `State` or `Info` structs as well.
  - Functions which technically need to be public but shouldn't be callable by the user need to be sealed.
    - see [this example in Rust's core library](https://github.com/rust-lang/rust/blob/044a28a4091f2e1a5883f7fa990223f8b200a2cd/library/core/src/error.rs#L89-L100)
- Any public traits, that **must not** be implemented downstream need to be `Sealed`.
- Prefer compile-time checks over runtime checks where possible, prefer a fallible API over panics.
- Follow naming conventions in order to be consistent across drivers - take inspiration from existing drivers.
- Design APIs in a way that they are easy to use.
- Driver API decisions should be assessed individually, don't _not_ just follow embedded-hal or other ecosystem trait crates. Expose the capabilities of the hardware. (Ecosystem traits are implemented on top of the inherent API)
- Avoid type states and extraneous generics whenever possible
  - These often lead to usability problems, and tend to just complicate things needlessly - sometimes it can be a good tradeoff to make a type not ZST
  - Common cases of useless type info is storing pin information - this is usually not required after configuring the pins and will bloat the complexity of the type massively. When following the `PeripheralRef` pattern it's not needed in order to keep users from re-using the pin while in use
- Avoiding `&mut self` when `&self` is safe to use. `&self` is generally easier to use as an API. Typical applications of this are where the methods just do writes to registers which don't have side effects.
- Maintain order consistency in the API, such as in the case of pairs like RX/TX.
- If your driver provides a way to listen for interrupts, the interrupts should be listed in a `derive(EnumSetType)` enum as opposed to one function per interrupt flag.
- If a driver only implements a subset of a peripheral's capabilities, it should be placed in the `peripheral::subcategory` module.
  - For example, if a driver implements the slave-mode I2C driver, it should be placed into `i2c::slave`.
  - This helps us reducing the need of introducing breaking changes if we implement additional functionalities.
- Avoid abbreviations and contractions in the API, where possible.
  - Saving a few characters may introduce ambiguity, e.g `SpiTransDone`, is it `Transmit` or `Transfer`?
  - Common abbreviations, that are well understood such as `Dma` are perfectly fine.

## Maintainability

- Avoid excessive use of macros unless there is no other option; modification of the PAC crates should be considered before resorting to macros.
- Every line of code is a liability. Take some time to see if your implementation can be simplified before opening a PR.
- If you are porting code from ESP-IDF (or anything else), please include a link WITH the commit hash in it, and please highlight the relevant line(s) of code
- If necessary provide further context as comments (consider linking to code, PRs, TRM - make sure to use permanent links, e.g. include the hash when linking to a Git repository, include the revision, page number etc. when linking to TRMs)
- Prefer line comments (//) to block comments (/* ... */)
- Generally, follow common "good practices" and idiomatic Rust style
- All `Future` objects (public or private) must be marked with ``#[must_use = "futures do nothing unless you `.await` or poll them"]``.
- Prefer `cfg_if!` (or, if the branches just pick between separate values of the same variable, `cfg!()`) over multiple exclusive `#[cfg]` attributes. `cfg_if!`/`cfg!()` visually divide the options, often results in simpler conditions and simplifies adding new branches in the future.

## Driver implementation

- If a common `Instance` trait is used for multiple peripherals, those traits should not have any logic implemented in them.
- The `Instance` traits should only be used to access information about a peripheral instance.
- The internal implementation of the driver should be non-generic over the peripheral instance. This helps the compiler produce smaller code.
- The author is encouraged to return a static shared reference to an `Info` and a `State` structure from the `Instance` trait.
  - The `Info` struct should describe the peripheral. Do not use any interior mutability.
  - The `State` struct should contain counters, wakers and other, mutable state. As this is accessed via a shared reference, interior mutability and atomic variables are preferred.

## Modules Documentation

Modules should have the following documentation format:
```rust
//! # Peripheral Name (Peripheral Acronym)
//!
//! ## Overview
//! Small description of the peripheral, see ESP-IDF docs or TRM
//!
//! ## Configuration
//! Explain how can the peripheral be configured, and which parameters can be configured
//!
//! ## Usage
//! Explain if we implement any external traits
//!
//! ## Examples
//!
//! ### Name of the Example
//! Small description of the example if needed
//! ```rust, no_run
//! ...
//! ```
//!
//! ## Implementation State
//! List unsupported features
```
- If any of the headers is empty, remove it
- When possible, use ESP-IDF docs and TRM as references and include links if possible.
  - In case of referencing an ESP-IDF link make it chip-specific, for example:
    ```
    #![doc = concat!("[ESP-IDF documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", crate::soc::chip!(), "/api-reference/peripherals/etm.html)")]
    ```
  - In case of referencing a TRM chapter, use the `crate::trm_markdown_link!()` macro. If you are referring to a particular chapter, you may use `crate::trm_markdown_link!("#chapter_anchor")`.
- Documentation examples must be short
  - But must also provide value beyond what the rustdoc generated docs show
    - Showing a snippet of a slightly more complex interaction, for example inverting the signals for a driver
    - Showing construction if it is more complex, or requires some non-obvious precursor steps. Think about this for drivers that take a generic instance to construct, rustdoc doesn't do a good job of showing what concrete things can be passed into a constructor.
  - For more complex scenarios, create an example.
- Use rustdoc syntax for linking to other documentation items instead of markdown links where possible
  - https://doc.rust-lang.org/rustdoc/write-documentation/linking-to-items-by-name.html
