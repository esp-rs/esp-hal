#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Interrupt support
//!
//! ## Overview
//! This module routes one or more peripheral interrupt sources to any one
//! of the CPU’s peripheral interrupts.
//!
//! ## Configuration
//! Usually peripheral drivers offer a mechanism to register your interrupt
//! handler. e.g. the systimer offers `set_interrupt_handler` to register a
//! handler for a specific alarm. Other drivers might take an interrupt handler
//! as an optional parameter to their constructor.
//!
//! This is the preferred way to register handlers.
//!
//! There are additional ways to register interrupt handlers which are generally
//! only meant to be used in very special situations (mostly internal to the HAL
//! or the supporting libraries). Those are outside the scope of this
//! documentation.
#![cfg_attr(
    riscv,
    doc = r#"
It is even possible, but not recommended, to bind an interrupt directly to a
CPU interrupt. This can offer lower latency, at the cost of more complexity
in the interrupt handler. See the `direct_vectoring.rs` example

We reserve a number of CPU interrupts, which cannot be used; see
[`RESERVED_INTERRUPTS`].
"#
)]
//! ## Examples
//!
//! ### Using the peripheral driver to register an interrupt handler
//!
//! ```rust, no_run
//! # {before_snippet}
//! let mut sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
//! critical_section::with(|cs| {
//!     sw_int
//!         .software_interrupt0
//!         .set_interrupt_handler(swint0_handler);
//!     SWINT0
//!         .borrow_ref_mut(cs)
//!         .replace(sw_int.software_interrupt0);
//! });
//!
//! critical_section::with(|cs| {
//!     if let Some(swint) = SWINT0.borrow_ref(cs).as_ref() {
//!         swint.raise();
//!     }
//! });
//! #
//! # loop {}
//! # }
//!
//! # use core::cell::RefCell;
//! #
//! # use critical_section::Mutex;
//! # use esp_hal::interrupt::software::{SoftwareInterrupt, SoftwareInterruptControl};
//! # use esp_hal::interrupt::Priority;
//! # use esp_hal::interrupt::InterruptHandler;
//! #
//! static SWINT0: Mutex<RefCell<Option<SoftwareInterrupt<0>>>> = Mutex::new(RefCell::new(None));
//!
//! #[handler(priority = Priority::Priority1)]
//! fn swint0_handler() {
//!     println!("SW interrupt0");
//!     critical_section::with(|cs| {
//!         if let Some(swint) = SWINT0.borrow_ref(cs).as_ref() {
//!             swint.reset();
//!         }
//!     });
//! }
//! ```

#[cfg(riscv)]
pub use self::riscv::*;
#[cfg(xtensa)]
pub use self::xtensa::*;
use crate::system::Cpu;

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        use crate::peripherals::DPORT as INTERRUPT_CORE0;
        use crate::peripherals::DPORT as INTERRUPT_CORE1;
    } else {
        use crate::peripherals::INTERRUPT_CORE0;
        #[cfg(esp32s3)]
        use crate::peripherals::INTERRUPT_CORE1;
    }
}

#[cfg(riscv)]
mod riscv;
#[cfg(xtensa)]
mod xtensa;

pub mod software;

#[cfg(feature = "rt")]
#[unsafe(no_mangle)]
extern "C" fn EspDefaultHandler() {
    panic!("Unhandled interrupt on {:?}", crate::system::Cpu::current());
}

/// Default (unhandled) interrupt handler
pub const DEFAULT_INTERRUPT_HANDLER: InterruptHandler = InterruptHandler::new(
    {
        unsafe extern "C" {
            fn EspDefaultHandler();
        }

        unsafe {
            core::mem::transmute::<unsafe extern "C" fn(), extern "C" fn()>(EspDefaultHandler)
        }
    },
    Priority::min(),
);

/// Trait implemented by drivers which allow the user to set an
/// [InterruptHandler]
pub trait InterruptConfigurable: crate::private::Sealed {
    #[cfg_attr(
        not(multi_core),
        doc = "Registers an interrupt handler for the peripheral."
    )]
    #[cfg_attr(
        multi_core,
        doc = "Registers an interrupt handler for the peripheral on the current core."
    )]
    #[doc = ""]
    /// Note that this will replace any previously registered interrupt
    /// handlers. Some peripherals offer a shared interrupt handler for
    /// multiple purposes. It's the users duty to honor this.
    ///
    /// You can restore the default/unhandled interrupt handler by using
    /// [DEFAULT_INTERRUPT_HANDLER]
    fn set_interrupt_handler(&mut self, handler: InterruptHandler);
}

/// Represents an ISR callback function
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IsrCallback {
    f: extern "C" fn(),
}

impl IsrCallback {
    /// Construct a new callback from the callback function.
    pub fn new(f: extern "C" fn()) -> Self {
        // a valid fn pointer is non zero
        Self { f }
    }

    /// Returns the address of the callback.
    pub fn address(self) -> usize {
        self.f as usize
    }

    /// The callback function.
    ///
    /// This is aligned and can be called.
    pub fn callback(self) -> extern "C" fn() {
        self.f
    }
}

impl PartialEq for IsrCallback {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::fn_addr_eq(self.callback(), other.callback())
    }
}

/// An interrupt handler
#[cfg_attr(
    multi_core,
    doc = "**Note**: Interrupts are handled on the core they were setup on, if a driver is initialized on core 0, and moved to core 1, core 0 will still handle the interrupt."
)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

    /// The Isr callback.
    #[inline]
    pub fn handler(&self) -> IsrCallback {
        IsrCallback::new(self.f)
    }

    /// Priority to be used when registering the interrupt
    #[inline]
    pub fn priority(&self) -> Priority {
        self.prio
    }
}

const STATUS_WORDS: usize = property!("interrupts.status_registers");

/// Representation of peripheral-interrupt status bits.
#[derive(Clone, Copy, Default, Debug)]
pub struct InterruptStatus {
    status: [u32; STATUS_WORDS],
}

impl InterruptStatus {
    /// Get status of peripheral interrupts
    pub fn current() -> InterruptStatus {
        match Cpu::current() {
            Cpu::ProCpu => InterruptStatus {
                status: core::array::from_fn(|idx| {
                    INTERRUPT_CORE0::regs()
                        .core_0_intr_status(idx)
                        .read()
                        .bits()
                }),
            },
            #[cfg(multi_core)]
            Cpu::AppCpu => InterruptStatus {
                status: core::array::from_fn(|idx| {
                    INTERRUPT_CORE1::regs()
                        .core_1_intr_status(idx)
                        .read()
                        .bits()
                }),
            },
        }
    }

    const fn empty() -> Self {
        InterruptStatus {
            status: [0u32; STATUS_WORDS],
        }
    }

    /// Is the given interrupt bit set
    pub fn is_set(&self, interrupt: u8) -> bool {
        (self.status[interrupt as usize / 32] & (1 << (interrupt % 32))) != 0
    }

    /// Set the given interrupt status bit
    pub fn set(&mut self, interrupt: u8) {
        self.status[interrupt as usize / 32] |= 1 << (interrupt % 32);
    }

    /// Return an iterator over the set interrupt status bits
    pub fn iterator(&self) -> InterruptStatusIterator {
        InterruptStatusIterator {
            status: *self,
            idx: 0,
        }
    }
}

/// Iterator over set interrupt status bits
pub struct InterruptStatusIterator {
    status: InterruptStatus,
    idx: usize,
}

impl Iterator for InterruptStatusIterator {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        for i in self.idx..STATUS_WORDS {
            if self.status.status[i] != 0 {
                let bit = self.status.status[i].trailing_zeros();
                self.idx = i;
                self.status.status[i] &= !1 << bit;
                return Some((bit + 32 * i as u32) as u8);
            }
        }
        self.idx = usize::MAX;
        None
    }
}

// Peripheral interrupt API.

use crate::{peripherals::Interrupt, system::Cpu};

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        use crate::peripherals::DPORT as INTERRUPT_CORE0;
        use crate::peripherals::DPORT as INTERRUPT_CORE1;
    } else {
        use crate::peripherals::INTERRUPT_CORE0;
        #[cfg(multi_core)]
        use crate::peripherals::INTERRUPT_CORE1;
    }
}

pub(crate) fn setup_interrupts() {
    // disable all known peripheral interrupts
    for peripheral_interrupt in 0..255 {
        crate::peripherals::Interrupt::try_from(peripheral_interrupt)
            .map(|intr| {
                #[cfg(multi_core)]
                disable(Cpu::AppCpu, intr);
                disable(Cpu::ProCpu, intr);
            })
            .ok();
    }
}

/// Assign a peripheral interrupt to a CPU interrupt.
pub(crate) fn map(core: Cpu, interrupt: Interrupt, cpu_interrupt: CpuInterrupt) {
    map_raw(core, interrupt, cpu_interrupt as u32)
}

/// Disable the given peripheral interrupt.
#[inline]
pub fn disable(core: Cpu, interrupt: Interrupt) {
    map_raw(core, interrupt, DISABLED_CPU_INTERRUPT)
}

pub(super) fn map_raw(core: Cpu, interrupt: Interrupt, cpu_interrupt: u32) {
    match core {
        Cpu::ProCpu => {
            INTERRUPT_CORE0::regs()
                .core_0_intr_map(interrupt as usize)
                .write(|w| unsafe { w.bits(cpu_interrupt) });
        }
        #[cfg(multi_core)]
        Cpu::AppCpu => {
            INTERRUPT_CORE1::regs()
                .core_1_intr_map(interrupt as usize)
                .write(|w| unsafe { w.bits(cpu_interrupt) });
        }
    }
}

/// Get cpu interrupt assigned to peripheral interrupt
pub(crate) fn mapped_to(cpu: Cpu, interrupt: Interrupt) -> Option<CpuInterrupt> {
    mapped_to_raw(cpu, interrupt as u32)
}

pub(crate) fn mapped_to_raw(cpu: Cpu, interrupt: u32) -> Option<CpuInterrupt> {
    let cpu_intr = match cpu {
        Cpu::ProCpu => INTERRUPT_CORE0::regs()
            .core_0_intr_map(interrupt as usize)
            .read()
            .bits(),
        #[cfg(multi_core)]
        Cpu::AppCpu => INTERRUPT_CORE1::regs()
            .core_1_intr_map(interrupt as usize)
            .read()
            .bits(),
    };
    let cpu_intr = CpuInterrupt::from_u32(cpu_intr)?;

    if cpu_intr.is_mappable() {
        Some(cpu_intr)
    } else {
        None
    }
}

/// Get the vectored peripheral interrupts configured for the core at the given priority
/// matching the given status.
#[inline]
pub(crate) fn configured_interrupts(status: InterruptStatus, level: u32) -> InterruptStatus {
    let core = Cpu::current();
    let mut res = InterruptStatus::empty();

    for interrupt_nr in status.iterator() {
        if let Some(cpu_interrupt) = crate::interrupt::mapped_to_raw(core, interrupt_nr as u32)
            && cpu_interrupt.is_vectored()
            && cpu_interrupt.level() as u32 == level
        {
            res.set(interrupt_nr);
        }
    }
    res
}

/// Get status of peripheral interrupts
#[inline]
pub fn status(core: Cpu) -> InterruptStatus {
    match core {
        Cpu::ProCpu => InterruptStatus::from(
            INTERRUPT_CORE0::regs().core_0_intr_status(0).read().bits(),
            INTERRUPT_CORE0::regs().core_0_intr_status(1).read().bits(),
            #[cfg(any(interrupts_status_registers = "3", interrupts_status_registers = "4"))]
            INTERRUPT_CORE0::regs().core_0_intr_status(2).read().bits(),
            #[cfg(interrupts_status_registers = "4")]
            INTERRUPT_CORE0::regs().core_0_intr_status(3).read().bits(),
        ),
        #[cfg(multi_core)]
        Cpu::AppCpu => InterruptStatus::from(
            INTERRUPT_CORE1::regs().core_1_intr_status(0).read().bits(),
            INTERRUPT_CORE1::regs().core_1_intr_status(1).read().bits(),
            #[cfg(any(interrupts_status_registers = "3", interrupts_status_registers = "4"))]
            INTERRUPT_CORE1::regs().core_1_intr_status(2).read().bits(),
            #[cfg(interrupts_status_registers = "4")]
            INTERRUPT_CORE1::regs().core_1_intr_status(3).read().bits(),
        ),
    }
}
