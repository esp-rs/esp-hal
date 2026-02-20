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
in the interrupt handler.
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
use crate::{peripherals::Interrupt, system::Cpu};

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

use crate::pac;

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
#[derive(Debug, Clone)]
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
                self.status.status[i] ^= 1 << bit;
                self.idx = i;
                return Some((bit + 32 * i as u32) as u8);
            }
        }
        self.idx = usize::MAX;
        None
    }
}

// Peripheral interrupt API.

fn vector_entry(interrupt: Interrupt) -> &'static pac::Vector {
    cfg_if::cfg_if! {
        if #[cfg(xtensa)] {
            &pac::__INTERRUPTS[interrupt as usize]
        } else {
            &pac::__EXTERNAL_INTERRUPTS[interrupt as usize]
        }
    }
}

/// Returns the currently bound interrupt handler.
pub fn bound_handler(interrupt: Interrupt) -> Option<IsrCallback> {
    unsafe {
        let vector = vector_entry(interrupt);

        let addr = vector._handler;
        if addr as usize == 0 {
            return None;
        }

        Some(IsrCallback::new(core::mem::transmute::<
            unsafe extern "C" fn(),
            extern "C" fn(),
        >(addr)))
    }
}

/// Binds the given handler to a peripheral interrupt.
///
/// The interrupt handler will be enabled at the specified priority level.
///
/// The interrupt handler will be called on the core where it is registered.
/// Only one interrupt handler can be bound to a peripheral interrupt.
pub fn bind_handler(interrupt: Interrupt, handler: InterruptHandler) {
    unsafe {
        let vector = vector_entry(interrupt);

        let ptr = (&raw const vector._handler).cast::<usize>().cast_mut();

        // On RISC-V MCUs we may be protecting the trap section using a watchpoint.
        // If we do, we need to temporarily disable this protection.
        #[cfg(all(riscv, write_vec_table_monitoring))]
        if crate::soc::trap_section_protected() {
            crate::debugger::DEBUGGER_LOCK.lock(|| {
                let wp = crate::debugger::clear_watchpoint(1);
                ptr.write_volatile(handler.handler().address());
                crate::debugger::restore_watchpoint(1, wp);
            });
            enable(interrupt, handler.priority());
            return;
        }

        ptr.write_volatile(handler.handler().address());
    }
    enable(interrupt, handler.priority());
}

/// Enables a peripheral interrupt at a given priority, using vectored CPU interrupts.
///
/// Note that interrupts still need to be enabled globally for interrupts
/// to be serviced.
///
/// Internally, this function maps the interrupt to the appropriate CPU interrupt
/// for the specified priority level.
#[inline]
pub fn enable(interrupt: Interrupt, level: Priority) {
    enable_on_cpu(Cpu::current(), interrupt, level);
}

pub(crate) fn enable_on_cpu(cpu: Cpu, interrupt: Interrupt, level: Priority) {
    let cpu_interrupt = priority_to_cpu_interrupt(interrupt, level);
    map_raw(cpu, interrupt, cpu_interrupt as u32);
}

/// Disable the given peripheral interrupt.
///
/// Internally, this function maps the interrupt to a disabled CPU interrupt.
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
    CpuInterrupt::from_u32(cpu_intr)
}

/// Represents the priority level of running code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum RunLevel {
    /// The base level
    ThreadMode,

    /// An elevated level, usually an interrupt handler's.
    Interrupt(Priority),
}

impl RunLevel {
    /// Get the current run level (the level below which interrupts are masked).
    #[inline]
    pub fn current() -> Self {
        current_runlevel()
    }

    /// Changes the current run level to the specified level and returns the previous level.
    ///
    /// # Safety
    ///
    /// This function must only be used to raise the runlevel and to restore it
    /// to a previous value. It must not be used to arbitrarily lower the
    /// runlevel.
    #[inline]
    pub unsafe fn change(to: Self) -> Self {
        unsafe { change_current_runlevel(to) }
    }

    /// Checks if the run level indicates thread mode.
    #[inline]
    pub fn is_thread(&self) -> bool {
        matches!(self, RunLevel::ThreadMode)
    }
}

impl PartialEq<Priority> for RunLevel {
    fn eq(&self, other: &Priority) -> bool {
        *self == RunLevel::Interrupt(*other)
    }
}

impl From<RunLevel> for u32 {
    fn from(level: RunLevel) -> Self {
        match level {
            RunLevel::ThreadMode => 0,
            RunLevel::Interrupt(priority) => priority as u32,
        }
    }
}

/// Priority Level Error
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PriorityError {
    /// The priority is not valid
    InvalidInterruptPriority,
}

impl TryFrom<u32> for RunLevel {
    type Error = PriorityError;

    fn try_from(priority: u32) -> Result<Self, Self::Error> {
        if priority == 0 {
            Ok(RunLevel::ThreadMode)
        } else {
            Priority::try_from(priority).map(RunLevel::Interrupt)
        }
    }
}

#[cfg(feature = "rt")]
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

    unsafe { crate::interrupt::init_vectoring() };
}

#[inline(always)]
fn should_handle(core: Cpu, interrupt_nr: u32, level: u32) -> bool {
    if let Some(cpu_interrupt) = crate::interrupt::mapped_to_raw(core, interrupt_nr)
        && cpu_interrupt.is_vectored()
        && cpu_interrupt.level() == level
    {
        true
    } else {
        false
    }
}
