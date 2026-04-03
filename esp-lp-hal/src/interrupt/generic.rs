use portable_atomic::{AtomicPtr, Ordering};

use super::{current_interrupts, enable};
pub use crate::pac::Interrupt;
use crate::pac::{__EXTERNAL_INTERRUPTS, Vector};

/// Hard-coded limit of a single u32 interrupt status bank,
/// for all ULP / LP chips.
const MAX_IRQ_STATUS_WORDS: usize = 1;

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

fn vector_entry(interrupt: Interrupt) -> &'static Vector {
    &__EXTERNAL_INTERRUPTS[interrupt as usize]
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
        ptr.write_volatile(handler.handler().address());
    }
    enable(interrupt, handler.priority());
}

/// Trait implemented by drivers which allow the user to set an
/// [InterruptHandler]
pub trait InterruptConfigurable {
    #[doc = "Registers an interrupt handler for the peripheral."]
    #[doc = ""]
    /// Note that this will replace any previously registered interrupt
    /// handlers. Some peripherals offer a shared interrupt handler for
    /// multiple purposes. It's the users duty to honor this.
    fn set_interrupt_handler(&mut self, handler: InterruptHandler);
}

/// Represents an ISR callback function
#[derive(Copy, Clone)]
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

/// Interrupt priority levels.
/// For esp-lp-hal there is only one priority level, and this is only included
/// so that the API is compatible with esp-hal handler macro.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
#[non_exhaustive]
pub enum Priority {
    /// Priority level 1.
    Priority1 = 1,
}

impl Priority {
    /// Maximum interrupt priority
    pub const fn max() -> Priority {
        Priority::Priority1
    }

    /// Minimum interrupt priority
    pub const fn min() -> Priority {
        Priority::Priority1
    }
}

/// An interrupt handler
#[derive(Copy, Clone)]
pub struct InterruptHandler {
    f: extern "C" fn(),
    prio: Priority,
}

impl InterruptHandler {
    /// Creates a new [InterruptHandler] which will call the given function
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

/// Iterator over set interrupt status bits
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct InterruptStatusIterator {
    status: InterruptStatus,
    idx: usize,
}

impl InterruptStatusIterator {
    /// Create a new InterruptStatusIterator
    pub fn new(status: InterruptStatus) -> Self {
        Self { status, idx: 0 }
    }
}

impl Iterator for InterruptStatusIterator {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        for i in self.idx..MAX_IRQ_STATUS_WORDS {
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

/// Ergonomic C function pointer wrapper, with null-pointer checking
pub struct CFnPtr(AtomicPtr<()>);

#[allow(clippy::new_without_default)]
impl CFnPtr {
    /// Create a new null pointer
    pub const fn new() -> Self {
        Self(AtomicPtr::new(core::ptr::null_mut()))
    }

    /// Store pointer to a C function
    pub fn store(&self, f: extern "C" fn()) {
        self.0.store(f as *mut (), Ordering::Relaxed);
    }

    /// Call the stored C function, if set
    pub fn call(&self) {
        let ptr = self.0.load(Ordering::Relaxed);
        if !ptr.is_null() {
            unsafe { (core::mem::transmute::<*mut (), extern "C" fn()>(ptr))() };
        }
    }
}

/// Representation of peripheral-interrupt status bits.
#[doc(hidden)]
#[derive(Clone, Copy, Default, Debug)]
pub struct InterruptStatus {
    status: [u32; MAX_IRQ_STATUS_WORDS],
}

impl InterruptStatus {
    /// Get status of interrupts
    /// This bitmask should match the Interrupt enum.
    pub fn current() -> Self {
        Self {
            status: [current_interrupts()],
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
        InterruptStatusIterator::new(*self)
    }
}

impl From<u32> for InterruptStatus {
    fn from(value: u32) -> Self {
        Self { status: [value] }
    }
}
