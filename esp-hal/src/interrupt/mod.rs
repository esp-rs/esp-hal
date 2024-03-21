//! # Interrupt support
//!
//! Interrupt support functionality depends heavily on the features enabled.
//!
//! When the `vectored` feature is enabled, the
//! [`enable`] method will map interrupt to a CPU
//! interrupt, and handle the `vector`ing to the peripheral interrupt, for
//! example `UART0`.
//!
//! It is also possible, but not recommended, to bind an interrupt directly to a
//! CPU interrupt. This can offer lower latency, at the cost of more complexity
//! in the interrupt handler.
//!
//! The `vectored` reserves a number of CPU interrupts, which cannot be used see
//! [`RESERVED_INTERRUPTS`].
//!
//! ## Example
//! ```no_run
//! #[entry]
//! fn main() -> ! {
//!     ...
//!     critical_section::with(|cs| SWINT.borrow_ref_mut(cs).replace(sw_int));
//!
//!     // enable the interrupt
//!     interrupt::enable(
//!         peripherals::Interrupt::FROM_CPU_INTR0,
//!         interrupt::Priority::Priority1,
//!     )
//!     .unwrap();
//!
//!     // trigger the interrupt
//!     SWINT
//!        .borrow_ref_mut(cs)
//!        .as_mut()
//!        .unwrap()
//!        .raise(SoftwareInterrupt::SoftwareInterrupt0);
//!
//!     loop {}
//! }
//!
//! #[interrupt]
//! fn FROM_CPU_INTR0() {
//!     esp_println::println!("SW interrupt0");
//!     critical_section::with(|cs| {
//!         SWINT
//!             .borrow_ref_mut(cs)
//!             .as_mut()
//!             .unwrap()
//!             .reset(SoftwareInterrupt::SoftwareInterrupt0);
//!     });
//! }
//! ```

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
    pub const fn new(f: extern "C" fn(), prio: Priority) -> Self {
        Self { f, prio }
    }

    #[inline]
    pub fn handler(&self) -> extern "C" fn() {
        self.f
    }

    #[inline]
    pub fn priority(&self) -> Priority {
        self.prio
    }

    #[inline]
    pub(crate) extern "C" fn call(&self) {
        (self.f)()
    }
}
