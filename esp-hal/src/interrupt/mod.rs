//! # Interrupt support
//!
//! Usually peripheral drivers offer a mechanism to register your interrupt
//! handler. e.g. the systimer offers `set_interrupt_handler` to register a
//! handler for a specific alarm. Other drivers might take an interrupt handler
//! as an optional parameter to their constructor.
//!
//! This is the preferred way to register handlers.
//!
//! ## Example using the peripheral driver to register an interrupt handler
//!
//! ```no_run
//! #[entry]
//! fn main() -> ! {
//! ...
//!     let mut sw_int = system.software_interrupt_control;
//!     critical_section::with(|cs| {
//!         sw_int
//!            .software_interrupt0
//!            .set_interrupt_handler(swint0_handler);
//!         SWINT0
//!            .borrow_ref_mut(cs)
//!            .replace(sw_int.software_interrupt0);
//!     });
//!
//!     // trigger the interrupt
//!     critical_section::with(|cs| {
//!         SWINT0.borrow_ref_mut(cs).as_mut().unwrap().raise();
//!     });
//! ...
//! }
//!
//! // use the `handler` macro to define a handler, optionally you can set a priority
//! #[handler(priority = esp_hal::interrupt::Priority::Priority2)]
//! fn swint0_handler() {
//!     esp_println::println!("SW interrupt0");
//!     critical_section::with(|cs| {
//!         SWINT0.borrow_ref_mut(cs).as_mut().unwrap().reset();
//!     });
//! }
//! ```
//!
//! There are additional ways to register interrupt handlers which are generally
//! only meant to be used in very special situations (mostly internal to the HAL
//! or the supporting libraries). Those are outside the scope of this
//! documentation.
//!
//! It is even possible, but not recommended, to bind an interrupt directly to a
//! CPU interrupt. This can offer lower latency, at the cost of more complexity
//! in the interrupt handler. See the `direct_vectoring.rs` example
//!
//! We reserve a number of CPU interrupts, which cannot be used; see
//! [`RESERVED_INTERRUPTS`].
#![warn(missing_docs)]

#[cfg(riscv)]
pub use self::riscv::*;
#[cfg(xtensa)]
pub use self::xtensa::*;

#[cfg(riscv)]
mod riscv;
#[cfg(xtensa)]
mod xtensa;

/// An interrupt handler
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct InterruptHandler {
    f: extern "C" fn(),
    prio: Priority,
}

impl InterruptHandler {
    /// Creates a new [InterruptHandler] which will call the given function at
    /// the given priority.
    pub const fn new(f: extern "C" fn(), prio: Priority) -> Self {
        Self { f, prio }
    }

    /// The function to be called
    #[inline]
    pub fn handler(&self) -> extern "C" fn() {
        self.f
    }

    /// Priority to be used when registering the interrupt
    #[inline]
    pub fn priority(&self) -> Priority {
        self.prio
    }

    /// Call the function
    #[inline]
    pub(crate) extern "C" fn call(&self) {
        (self.f)()
    }
}
