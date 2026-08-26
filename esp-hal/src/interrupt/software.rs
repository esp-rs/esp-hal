#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Software Interrupts
//!
//! The [`SoftwareInterrupt`] struct allows raising or resetting software
//! interrupts using the [`raise()`][SoftwareInterrupt::raise] and
//! [`reset()`][SoftwareInterrupt::reset] methods.
//!
//! ## Examples
//!
//! ```rust, no_run
//! # {before_snippet}
//! // Take the interrupt you want to use.
//! let mut int0 = SoftwareInterrupt::new(peripherals.FROM_CPU_INTR2);
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
//! # use esp_hal::interrupt::software::SoftwareInterrupt;
//! // ... somewhere outside of your main function
//!
//! // Define a shared handle to the software interrupt.
//! static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<2>>>> = Mutex::new(RefCell::new(None));
//!
//! #[esp_hal::handler]
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
    private::Sealed,
    system::Cpu,
};

/// A software interrupt can be triggered by software.
#[non_exhaustive]
pub struct SoftwareInterrupt<'d, const NUM: u8> {
    _lifetime: PhantomData<&'d mut ()>,
}

impl<'d, const NUM: u8> SoftwareInterrupt<'d, NUM> {
    /// Creates a new software interrupt driver.
    #[inline]
    pub const fn new(instance: impl Instance<NUM> + 'd) -> Self {
        core::mem::forget(instance); // needed to make `new` const
        Self {
            _lifetime: PhantomData,
        }
    }

    /// Sets the interrupt handler for this software-interrupt.
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
        interrupt::bind_handler(interrupt, handler);
    }

    /// Triggers this software-interrupt.
    #[crate::ram]
    pub fn raise(&self) {
        let regs = cfg_select! {
            soc_has_intpri => crate::peripherals::INTPRI::regs(),
            _ => crate::peripherals::SYSTEM::regs(),
        };
        let reg = regs.cpu_intr_from_cpu(NUM as usize);

        cfg_select! {
            xtensa => {
                reg.write(|w| w.cpu_intr().set_bit());
                // Read back to ensure the write is completed.
                _ = reg.read();
            }
            _ => {
                crate::interrupt::free(|| {
                    reg.write(|w| w.cpu_intr().set_bit());
                    // Wait for the interrupt to actually take effect.
                    while !self.is_pending() {}
                });
            }
        }
    }

    #[cfg(riscv)]
    fn is_pending(&self) -> bool {
        let interrupt;
        for_each_sw_interrupt! {
            (all $( ($n:literal, $interrupt_name:ident, $f:ident) ),*) => {
                interrupt = match NUM {
                    $($n => Interrupt::$interrupt_name,)*
                    _ => unreachable!(),
                };
            };
        }
        crate::interrupt::InterruptStatus::is_pending(interrupt)
    }

    /// Resets this software-interrupt.
    pub fn reset(&self) {
        cfg_select! {
            soc_has_intpri => {
                let regs = crate::peripherals::INTPRI::regs();
            }
            _ => {
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

/// A software interrupt instance.
pub trait Instance<const NUM: u8>: Sealed {}

for_each_sw_interrupt! {
    ($n:literal, $i:ident, $field:ident) => {
        impl Instance<$n> for crate::peripherals::$i<'_> {}
    };
}
