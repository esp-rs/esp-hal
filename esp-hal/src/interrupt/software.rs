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
#![doc = crate::before_snippet!()]
//! let sw_ints =
//!     SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
//!
//! // Take the interrupt you want to use.
//! let mut int0 = sw_ints.software_interrupt0;
//! let mut int1 = sw_ints.software_interrupt1;
//! let mut int2 = sw_ints.software_interrupt2;
//! let mut int3 = sw_ints.software_interrupt3;
//!
//! // Set up the interrupt handler. Do this in a critical section so the global
//! // contains the interrupt object before the interrupt is triggered.
//! critical_section::with(|cs| {
//!     int0.set_interrupt_handler(swint0_handler);
//!     SWINT0.borrow_ref_mut(cs).replace(int0);
//!
//!     int1.set_interrupt_handler(swint1_handler);
//!     SWINT1.borrow_ref_mut(cs).replace(int1);
//!
//!     int2.set_interrupt_handler(swint2_handler);
//!     SWINT2.borrow_ref_mut(cs).replace(int2);
//!
//!     int3.set_interrupt_handler(swint3_handler);
//!     SWINT3.borrow_ref_mut(cs).replace(int3);
//! });
//! # }
//!
//! # use core::cell::RefCell;
//! # use critical_section::Mutex;
//! # use esp_hal::interrupt::software::{SoftwareInterrupt, SoftwareInterruptControl};
//! // ... somewhere outside of your main function
//!
//! // Define a shared handle to the software interrupt.
//! static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> =
//!     Mutex::new(RefCell::new(None));
//!
//! static SWINT1: Mutex<RefCell<Option<SoftwareInterrupt<1>>>> =
//!     Mutex::new(RefCell::new(None));
//!
//! static SWINT2: Mutex<RefCell<Option<SoftwareInterrupt<2>>>> =
//!     Mutex::new(RefCell::new(None));
//!  
//! static SWINT3: Mutex<RefCell<Option<SoftwareInterrupt<3>>>> =
//!     Mutex::new(RefCell::new(None));
//!
//! #[handler]
//! fn swint0_handler() {
//!     // esp_println::println!("SW interrupt0 handled");
//!
//!     // Clear the interrupt request.
//!     critical_section::with(|cs| {
//!         SWINT0.borrow_ref(cs).as_ref().unwrap().reset();
//!     });
//! }
//!
//! #[handler]
//! fn swint1_handler() {
//!     // esp_println::println!("SW interrupt1 handled");
//!
//!     // Clear the interrupt request.
//!     critical_section::with(|cs| {
//!         SWINT1.borrow_ref(cs).as_ref().unwrap().reset();
//!     });
//! }
//!
//! #[handler]
//! fn swint2_handler() {
//!     // esp_println::println!("SW interrupt2 handled");
//!
//!     // Clear the interrupt request.
//!     critical_section::with(|cs| {
//!         SWINT2.borrow_ref(cs).as_ref().unwrap().reset();
//!     });
//! }
//!
//! #[handler]
//! fn swint3_handler() {
//!     // esp_println::println!("SW interrupt3 handled");
//!
//!     // Clear the interrupt request.
//!     critical_section::with(|cs| {
//!         SWINT3.borrow_ref(cs).as_ref().unwrap().reset();
//!     });
//! }
//! ```

use crate::{interrupt::InterruptHandler, InterruptConfigurable};

/// A software interrupt can be triggered by software.
#[non_exhaustive]
pub struct SoftwareInterrupt<const NUM: u8>;

impl<const NUM: u8> SoftwareInterrupt<NUM> {
    /// Sets the interrupt handler for this software-interrupt
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        let interrupt = match NUM {
            0 => crate::peripherals::Interrupt::FROM_CPU_INTR0,
            1 => crate::peripherals::Interrupt::FROM_CPU_INTR1,
            2 => crate::peripherals::Interrupt::FROM_CPU_INTR2,
            3 => crate::peripherals::Interrupt::FROM_CPU_INTR3,
            _ => unreachable!(),
        };

        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, interrupt);
        }
        unsafe { crate::interrupt::bind_interrupt(interrupt, handler.handler()) };
        unwrap!(crate::interrupt::enable(interrupt, handler.priority()));
    }

    /// Trigger this software-interrupt
    pub fn raise(&self) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let system = unsafe { &*crate::peripherals::INTPRI::PTR };
            } else {
                let system = unsafe { &*crate::peripherals::SYSTEM::PTR };
            }
        }

        match NUM {
            0 => system
                .cpu_intr_from_cpu_0()
                .write(|w| w.cpu_intr_from_cpu_0().set_bit()),
            1 => system
                .cpu_intr_from_cpu_1()
                .write(|w| w.cpu_intr_from_cpu_1().set_bit()),
            2 => system
                .cpu_intr_from_cpu_2()
                .write(|w| w.cpu_intr_from_cpu_2().set_bit()),
            3 => system
                .cpu_intr_from_cpu_3()
                .write(|w| w.cpu_intr_from_cpu_3().set_bit()),
            _ => unreachable!(),
        };
    }

    /// Resets this software-interrupt
    pub fn reset(&self) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32c6, esp32h2))] {
                let system = unsafe { &*crate::peripherals::INTPRI::PTR };
            } else {
                let system = unsafe { &*crate::peripherals::SYSTEM::PTR };
            }
        }

        match NUM {
            0 => system
                .cpu_intr_from_cpu_0()
                .write(|w| w.cpu_intr_from_cpu_0().clear_bit()),
            1 => system
                .cpu_intr_from_cpu_1()
                .write(|w| w.cpu_intr_from_cpu_1().clear_bit()),
            2 => system
                .cpu_intr_from_cpu_2()
                .write(|w| w.cpu_intr_from_cpu_2().clear_bit()),
            3 => system
                .cpu_intr_from_cpu_3()
                .write(|w| w.cpu_intr_from_cpu_3().clear_bit()),
            _ => unreachable!(),
        };
    }

    /// Unsafely create an instance of this peripheral out of thin air.
    ///
    /// # Safety
    ///
    /// You must ensure that you're only using one instance of this type at a
    /// time.
    #[inline]
    pub unsafe fn steal() -> Self {
        Self
    }
}

impl<const NUM: u8> crate::peripheral::Peripheral for SoftwareInterrupt<NUM> {
    type P = SoftwareInterrupt<NUM>;

    #[inline]
    unsafe fn clone_unchecked(&self) -> Self::P {
        Self::steal()
    }
}

impl<const NUM: u8> crate::private::Sealed for SoftwareInterrupt<NUM> {}

impl<const NUM: u8> InterruptConfigurable for SoftwareInterrupt<NUM> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        SoftwareInterrupt::set_interrupt_handler(self, handler);
    }
}

/// This gives access to the available software interrupts.
///
/// This struct contains several instances of software interrupts that can be
/// used for signaling between different parts of a program or system. Each
/// interrupt is identified by an index (0 to 3).
#[cfg_attr(
    multi_core,
    doc = r#"

Please note: Software interrupt 3 is reserved
for inter-processor communication when using
`esp-hal-embassy`."#
)]
#[non_exhaustive]
pub struct SoftwareInterruptControl {
    /// Software interrupt 0.
    pub software_interrupt0: SoftwareInterrupt<0>,
    /// Software interrupt 1.
    pub software_interrupt1: SoftwareInterrupt<1>,
    /// Software interrupt 2.
    pub software_interrupt2: SoftwareInterrupt<2>,
    #[cfg(not(all(feature = "__esp_hal_embassy", multi_core)))]
    /// Software interrupt 3. Only available when not using `esp-hal-embassy`,
    /// or on single-core systems.
    pub software_interrupt3: SoftwareInterrupt<3>,
}

impl SoftwareInterruptControl {
    /// Create a new instance of the software interrupt control.
    pub fn new(_peripheral: crate::peripherals::SW_INTERRUPT) -> Self {
        SoftwareInterruptControl {
            software_interrupt0: SoftwareInterrupt {},
            software_interrupt1: SoftwareInterrupt {},
            software_interrupt2: SoftwareInterrupt {},
            #[cfg(not(all(feature = "__esp_hal_embassy", multi_core)))]
            software_interrupt3: SoftwareInterrupt {},
        }
    }
}
