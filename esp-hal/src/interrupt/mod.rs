#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # Interrupt support
//!
//! This module contains code to configure and handle peripheral interrupts.
//!
//! ## Overview
//!
//! Peripheral interrupt requests are not handled by the CPU directly. Instead, they are routed
//! through the interrupt matrix, which maps peripheral interrupts to CPU interrupts. There are
//! more peripheral interrupt signals than CPU interrupts, and multiple peripheral interrupts can
//! be routed to the same CPU interrupt. A set of CPU interrupts are configured to run a default
//! handler routine, which polls the interrupt controller and calls the appropriate handlers for the
//! pending peripheral interrupts.
//!
//! This default handler implements interrupt nesting - meaning a higher [`Priority`] interrupt can
//! preempt a lower priority interrupt handler. The number of priorities is a chip-specific detail.
//!
//! ## Usage
//!
//! Peripheral drivers manage interrupts for you. Where appropriate, a `set_interrupt_handler`
//! function is provided, which allows you to register a function to handle interrupts at a priority
//! level of your choosing. Interrupt handler functions need to be marked by the [`#[handler]`]
//! attribute. These drivers also provide `listen` and `unlisten` functions that control whether an
//! interrupt will be generated for the matching event or not. For more information and examples,
//! consult the documentation of the specific peripheral drivers.
//!
//! If you are writing your own peripheral driver, you will need to first register interrupt
//! handlers using the [peripheral singletons'] `bind_X_interrupt` functions. You can use the
//! matching `enable` and `disable` functions to control the peripheral interrupt in the interrupt
//! matrix, or you can, depending on the peripheral, set or clear the appropriate enable bits in the
//! `int_ena` register.
//!
//! [`#[handler]`]: crate::handler
//! [peripheral singletons']: crate::peripherals::I2C0
//!
//! ## Software interrupts
//!
//! The [`software`] module implements software interrupts using peripheral interrupt signals.
#![cfg_attr(
    multi_core,
    doc = "This mechanism can be used to implement efficient cross-core communication."
)]

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

unstable_driver! {
    pub mod software;
}

#[cfg(feature = "rt")]
#[unsafe(no_mangle)]
extern "C" fn EspDefaultHandler() {
    panic!("Unhandled interrupt on {:?}", crate::system::Cpu::current());
}

/// Default (unhandled) interrupt handler
#[instability::unstable]
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
#[instability::unstable]
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
#[instability::unstable]
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
    doc = r"

**Note**: Interrupts are handled on the core they were setup on. If a driver is initialized
on core 0, and moved to core 1, core 0 will still handle the interrupt."
)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
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
#[instability::unstable]
pub struct InterruptStatus {
    status: [u32; STATUS_WORDS],
}

impl InterruptStatus {
    /// Get status of peripheral interrupts
    #[instability::unstable]
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
    #[instability::unstable]
    pub fn is_set(&self, interrupt: u8) -> bool {
        (self.status[interrupt as usize / 32] & (1 << (interrupt % 32))) != 0
    }

    /// Set the given interrupt status bit
    #[instability::unstable]
    pub fn set(&mut self, interrupt: u8) {
        self.status[interrupt as usize / 32] |= 1 << (interrupt % 32);
    }

    /// Return an iterator over the set interrupt status bits
    #[instability::unstable]
    pub fn iterator(&self) -> InterruptStatusIterator {
        InterruptStatusIterator {
            status: *self,
            idx: 0,
        }
    }
}

/// Iterator over set interrupt status bits
#[derive(Debug, Clone)]
#[instability::unstable]
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
#[instability::unstable]
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
#[instability::unstable]
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
#[instability::unstable]
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
#[instability::unstable]
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
#[instability::unstable]
pub enum RunLevel {
    /// The base level
    ThreadMode,

    /// An elevated level, usually an interrupt handler's.
    Interrupt(Priority),
}

impl RunLevel {
    /// Get the current run level (the level below which interrupts are masked).
    #[inline]
    #[instability::unstable]
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
    #[instability::unstable]
    pub unsafe fn change(to: Self) -> Self {
        unsafe { change_current_runlevel(to) }
    }

    /// Checks if the run level indicates thread mode.
    #[inline]
    #[instability::unstable]
    pub fn is_thread(&self) -> bool {
        matches!(self, RunLevel::ThreadMode)
    }

    pub(crate) fn try_from_u32(priority: u32) -> Result<Self, PriorityError> {
        if priority == 0 {
            Ok(RunLevel::ThreadMode)
        } else {
            Priority::try_from_u32(priority).map(RunLevel::Interrupt)
        }
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
#[instability::unstable]
pub enum PriorityError {
    /// The priority is not valid
    InvalidInterruptPriority,
}

#[instability::unstable]
impl TryFrom<u32> for RunLevel {
    type Error = PriorityError;

    fn try_from(priority: u32) -> Result<Self, Self::Error> {
        Self::try_from_u32(priority)
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
#[cfg(feature = "rt")]
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
