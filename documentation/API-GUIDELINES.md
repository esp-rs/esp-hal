# `esp-rs` API Guidelines

## About

This is a living document - make sure to check the latest version of this document.

> [!NOTE]
> Not all of the currently existing code follows this guideline, yet.

In general, the [Rust API Guidelines](https://rust-lang.github.io/api-guidelines) apply to all projects in the ESP-RS GitHub organization where possible. (`C-RW-VALUE` and `C-SERDE` do not apply)

Especially for public API but if possible also for internal APIs.

The following paragraphs contain additional recommendations.

## Construction and Destruction of Drivers

- Drivers should take peripherals via the `PeripheralRef` pattern - they don't consume peripherals directly.
- If a driver requires pins, those pins should be configured using `fn with_pin_name(self, pin: impl Peripheral<P = InputConnection> + 'd) -> Self)` or `fn with_pin_name(self, pin: impl Peripheral<P = OutputConnection> + 'd) -> Self)`
- If a driver supports multiple peripheral instances:
  - The peripheral instance type should be positioned as the last type parameter of the driver type.
  - The peripheral instance type should default to a type that supports any of the peripheral instances.
  - The author is encouraged to use `crate::any_peripheral` to define the "any" peripheral instance type.
  - The driver should implement a `new` constructor that automatically converts the peripheral instance into the any type, and a `new_typed` that preserves the peripheral type.
- If a driver implements both blocking and async operations, or only implements blocking operations, but may support asynchronous ones in the future, the driver's type signature should include a `crate::Mode` type parameter.
- By default, constructors should configure the driver for blocking mode. The driver should implement `into_async` (and a matching `into_blocking`) function that reconfigures the driver.
  - `into_async` should configure the driver and/or the associated DMA channels. This most often means enabling an interrupt handler.
  - `into_blocking` should undo the configuration done by `into_async`.
- The asynchronous driver implemntation should also expose the blocking methods (except for interrupt related functions).
- Consider adding a `Drop` implementation resetting the peripheral to idle state.
- Consider using a builder-like pattern for driver construction.

## Interoperability

- `cfg` gated `defmt` derives and impls are added to new structs and enums.
    - see [this example](https://github.com/esp-rs/esp-hal/blob/df2b7bd8472cc1d18db0d9441156575570f59bb3/esp-hal/src/spi/mod.rs#L15)
    - e.g. `#[cfg_attr(feature = "defmt", derive(defmt::Format))]`
- Don't use `log::XXX!` macros directly - use the wrappers in `fmt.rs` (e.g. just `info!` instead of `log::info!` or importing `log::*`)!
- Prefer implementing common ecosystem traits, like the ones in `embedded-hal` or `embassy-embedded-hal`.

## API Surface

- API documentation shouldn't be an afterthought.
- Private details shouldn't leak into the public API, and should be made private where technically possible.
  - Implementation details that _need_ to be public should be marked with `#[doc(hidden)]` and a comment as to why it needs to be public.
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
    - For example starting a timer is fine for `&self`, worst case a timer will be started twice if two parts of the program call it. You can see a real example of this [here](https://github.com/esp-rs/esp-hal/pull/1500#pullrequestreview-2015911974)
- Maintain order consistency in the API, such as in the case of pairs like RX/TX.
- If your driver provides a way to listen for interrupts, the interrupts should be listed in a `derive(EnumSetType)` enum as opposed to one function per interrupt flag.
- If a driver only implements a subset of a peripheral's capabilities, it should be placed in the `peripheral::subcategory` module.
  - For example, if a driver implements the slave-mode I2C driver, it should be placed into `i2c::slave`.
  - This helps us reducing the need of introducing breaking changes if we implement additional functionalities.

## Maintainability

- Avoid excessive use of macros unless there is no other option; modification of the PAC crates should be considered before resorting to macros.
- Every line of code is a liability. Take some time to see if your implementation can be simplified before opening a PR.
- If you are porting code from ESP-IDF (or anything else), please include a link WITH the commit hash in it, and please highlight the relevant line(s) of code
- If necessary provide further context as comments (consider linking to code, PRs, TRM - make sure to use permanent links, e.g. include the hash when linking to a Git repository, include the revision, page number etc. when linking to TRMs)
- Generally, follow common "good practices" and idiomatic Rust style
- All `Future` objects (public or private) must be marked with ``#[must_use = "futures do nothing unless you `.await` or poll them"]``.
- Prefer `cfg_if!` over multiple exclusive `#[cfg]` attributes. `cfg_if!` visually divides the options, often results in simpler conditions and simplifies adding new branches in the future.

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
- Documentation examples should be short and basic. For more complex scenarios, create an example.
