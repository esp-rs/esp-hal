#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Software Interrupts
//!
//! The [`SoftwareInterruptControl`] struct gives access to the available
//! software interrupts.
//!
//! The [`SoftwareInterrupt`] struct allows raising or resetting software
//! interrupts using the [`raise()`][SoftwareInterrupt::raise] and
//! [`reset()`][SoftwareInterrupt::reset] methods.
//!
//! ## Examples
//!
//! ```rust, no_run
//! # {before_snippet}
//! let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
//!
//! // Take the interrupt you want to use.
//! let mut int0 = sw_ints.software_interrupt0;
//!
//! // Set up the interrupt handler. Do this in a critical section so the global
//! // contains the interrupt object before the interrupt is triggered.
//! critical_section::with(|cs| {
//!     int0.set_interrupt_handler(swint0_handler);
//!     SWINT0.borrow_ref_mut(cs).replace(int0);
//! });
//! # {after_snippet}
//!
//! # use core::cell::RefCell;
//! # use critical_section::Mutex;
//! # use esp_hal::interrupt::software::{SoftwareInterrupt, SoftwareInterruptControl};
//! // ... somewhere outside of your main function
//!
//! // Define a shared handle to the software interrupt.
//! static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> = Mutex::new(RefCell::new(None));
//!
//! #[handler]
//! fn swint0_handler() {
//!     println!("SW interrupt0 handled");
//!
//!     // Clear the interrupt request.
//!     critical_section::with(|cs| {
//!         if let Some(swint) = SWINT0.borrow_ref(cs).as_ref() {
//!             swint.reset();
//!         }
//!     });
//! }
//! ```

use core::marker::PhantomData;

use crate::{
    interrupt::{self, InterruptConfigurable, InterruptHandler},
    peripherals::Interrupt,
    system::Cpu,
};

/// A software interrupt can be triggered by software.
#[non_exhaustive]
pub struct SoftwareInterrupt<'d, const NUM: u8> {
    _lifetime: PhantomData<&'d mut ()>,
}

impl<const NUM: u8> SoftwareInterrupt<'_, NUM> {
    /// Unsafely create an instance of this peripheral out of thin air.
    ///
    /// # Safety
    ///
    /// You must ensure that you're only using one instance of this type at a
    /// time.
    #[inline]
    pub unsafe fn steal() -> Self {
        Self {
            _lifetime: PhantomData,
        }
    }

    /// Creates a new peripheral reference with a shorter lifetime.
    ///
    /// Use this method if you would like to keep working with the peripheral
    /// after you dropped the driver that consumes this.
    ///
    /// See [Peripheral singleton] section for more information.
    ///
    /// [Peripheral singleton]: crate#peripheral-singletons
    pub fn reborrow(&mut self) -> SoftwareInterrupt<'_, NUM> {
        unsafe { SoftwareInterrupt::steal() }
    }

    /// Sets the interrupt handler for this software-interrupt
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        let interrupt;

        for_each_sw_interrupt! {
            (all $( ($n:literal, $interrupt_name:ident, $f:ident) ),*) => {
                interrupt = match NUM {
                    $($n => Interrupt::$interrupt_name,)*
                    _ => unreachable!(),
                };
            };
        }

        for core in Cpu::other() {
            interrupt::disable(core, interrupt);
        }
        unsafe { interrupt::bind_interrupt(interrupt, handler.handler()) };
        unwrap!(interrupt::enable(interrupt, handler.priority()));
    }

    /// Trigger this software-interrupt
    pub fn raise(&self) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let regs = crate::peripherals::INTPRI::regs();
            } else {
                let regs = crate::peripherals::SYSTEM::regs();
            }
        }
        let reg;

        for_each_sw_interrupt! {
            (all $( ($n:literal, $i:ident, $f:ident) ),*) => {
                reg = match NUM {
                    $($n => regs.cpu_intr_from_cpu($n),)*
                    _ => unreachable!(),
                };
            };
        }

        reg.write(|w| w.cpu_intr().set_bit());
        _ = reg.read(); // Read back to ensure the write is completed.

        // Insert enough delay to ensure the interrupt is fired before returning.
        sw_interrupt_delay!();
    }

    /// Resets this software-interrupt
    pub fn reset(&self) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let regs = crate::peripherals::INTPRI::regs();
            } else {
                let regs = crate::peripherals::SYSTEM::regs();
            }
        }
        let reg;

        for_each_sw_interrupt! {
            (all $( ($n:literal, $i:ident, $f:ident) ),*) => {
                reg = match NUM {
                    $($n => regs.cpu_intr_from_cpu($n),)*
                    _ => unreachable!(),
                };
            };
        }

        reg.write(|w| w.cpu_intr().clear_bit());
    }
}

impl<const NUM: u8> crate::private::Sealed for SoftwareInterrupt<'_, NUM> {}

impl<const NUM: u8> InterruptConfigurable for SoftwareInterrupt<'_, NUM> {
    fn set_interrupt_handler(&mut self, handler: interrupt::InterruptHandler) {
        SoftwareInterrupt::set_interrupt_handler(self, handler);
    }
}

for_each_sw_interrupt! {
    (all $( ($n:literal, $i:ident, $field:ident) ),*) => {
        /// This gives access to the available software interrupts.
        ///
        /// This struct contains several instances of software interrupts that can be
        /// used for signaling between different parts of a program or system.
        #[non_exhaustive]
        pub struct SoftwareInterruptControl<'d> {
            $(
                #[doc = concat!("Software interrupt ", stringify!($n), ".")]
                pub $field: SoftwareInterrupt<'d, $n>,
            )*
        }

        impl<'d> SoftwareInterruptControl<'d> {
            /// Create a new instance of the software interrupt control.
            pub fn new(_peripheral: crate::peripherals::SW_INTERRUPT<'d>) -> Self {
                SoftwareInterruptControl {
                    $(
                        $field: SoftwareInterrupt {
                            _lifetime: PhantomData,
                        },
                    )*
                }
            }
        }
    };
}
