//! Interrupt support
//!
//! ## Overview
//! The `interrupt` driver is a crucial module for ESP chips. Its primary
//! purpose is to manage and handle interrupts, which are asynchronous events
//! requiring immediate attention from the CPU. Interrupts are essential in
//! various applications, such as real-time tasks, I/O communications, and
//! handling external events like hardware signals.
//!
//! The core functionality of the `interrupt` driver revolves around the
//! management of interrupts. When an interrupt occurs, it temporarily stops the
//! ongoing CPU operations, saves its current state, and starts executing the
//! corresponding interrupt service routine (ISR). The interrupt service routine
//! is a user-defined function that performs the necessary actions to handle the
//! specific interrupt. Once the ISR is executed, the driver restores the saved
//! CPU state and resumes normal program execution.
//!
//! In scenarios where multiple interrupts may occur simultaneously, the
//! interrupt driver determines the `priority` of each interrupt. This
//! prioritization ensures that critical or high-priority tasks are handled
//! first. It helps prevent delays in time-sensitive applications and allows the
//! system to allocate resources efficiently. This functionality is provided and
//! implemented by the `priority` enum.
//!
//!
//! ## Example
//! ```no_run
//! #[entry]
//! fn main() -> ! {
//!     ...
//!     critical_section::with(|cs| SWINT.borrow_ref_mut(cs).replace(sw_int));
//!
//!     interrupt::enable(
//!         peripherals::Interrupt::FROM_CPU_INTR0,
//!         interrupt::Priority::Priority1,
//!     )
//!     .unwrap();
//!     interrupt::enable(
//!         peripherals::Interrupt::FROM_CPU_INTR1,
//!         interrupt::Priority::Priority2,
//!     )
//!     .unwrap();
//!     interrupt::enable(
//!         peripherals::Interrupt::FROM_CPU_INTR2,
//!         interrupt::Priority::Priority2,
//!     )
//!     .unwrap();
//!     interrupt::enable(
//!         peripherals::Interrupt::FROM_CPU_INTR3,
//!         interrupt::Priority::Priority15,
//!     )
//!     .unwrap();
//!     unsafe { riscv::interrupt::enable() }
//!
//!     // raise mid priority interrupt.
//!     // The handler raises one interrupt at lower priority, one at same and one at
//!     // higher. We expect to see the higher priority served immeiately before
//!     // exiting the handler Once the handler is exited we expect to see same
//!     // priority and low priority interrupts served in that order
//!     critical_section::with(|cs| {
//!         SWINT
//!             .borrow_ref_mut(cs)
//!             .as_mut()
//!             .unwrap()
//!             .raise(SoftwareInterrupt::SoftwareInterrupt1);
//!     });
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
//! #[interrupt]
//! fn FROM_CPU_INTR1() {
//!     esp_println::println!("SW interrupt1 entry");
//!     critical_section::with(|cs| {
//!         SWINT
//!         .borrow_ref_mut(cs)
//!         .as_mut()
//!         .unwrap()
//!         .reset(SoftwareInterrupt::SoftwareInterrupt1);
//!     SWINT
//!         .borrow_ref_mut(cs)
//!         .as_mut()
//!         .unwrap()
//!         .raise(SoftwareInterrupt::SoftwareInterrupt2); // raise interrupt at same priority
//!     SWINT
//!         .borrow_ref_mut(cs)
//!         .as_mut()
//!         .unwrap()
//!         .raise(SoftwareInterrupt::SoftwareInterrupt3); // raise interrupt at higher priority
//!     SWINT
//!         .borrow_ref_mut(cs)
//!         .as_mut()
//!         .unwrap()
//!         .raise(SoftwareInterrupt::SoftwareInterrupt0); // raise interrupt at
//!                                                        // lower priority
//! });
//! esp_println::println!("SW interrupt1 exit");
//! }
//! #[interrupt]
//! fn FROM_CPU_INTR2() {
//!     esp_println::println!("SW interrupt2");
//!     critical_section::with(|cs| {
//!     SWINT
//!         .borrow_ref_mut(cs)
//!         .as_mut()
//!         .unwrap()
//!         .reset(SoftwareInterrupt::SoftwareInterrupt2);
//!     });
//! }
//! #[interrupt]
//! fn FROM_CPU_INTR3() {
//!     esp_println::println!("SW interrupt3");
//!     critical_section::with(|cs| {
//!     SWINT
//!         .borrow_ref_mut(cs)
//!         .as_mut()
//!         .unwrap()
//!         .reset(SoftwareInterrupt::SoftwareInterrupt3);
//!     });
//! }
//! ```

#[cfg(riscv)]
pub use riscv::*;
#[cfg(xtensa)]
pub use xtensa::*;

#[cfg(riscv)]
mod riscv;
#[cfg(xtensa)]
mod xtensa;
