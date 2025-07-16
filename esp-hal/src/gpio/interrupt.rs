//! GPIO interrupt handling
//!
//! ## Requirements
//!
//! - On devices other than the P4, there is a single interrupt handler. GPIO interrupt handling
//!   must not interfere with the async API in this single handler.
//! - Async operations take pins by `&mut self`, so they can only be accessed after the operation is
//!   complete, or cancelled. They may be defined to overwrite the configuration of the manual
//!   interrupt API, but not affect the interrupt handler.
//! - Manual `listen` operations don't need to be prepared for async operations, but async
//!   operations need to be prepared to handle cases where the pin was configured to listen for an
//!   event - or even that the user unlistened the pin but left the interrupt status set.
//!
//! The user should be careful when using the async API and the manual interrupt
//! API together. For performance reasons, we will not prevent the user handler
//! from running in response to an async event.
//!
//! ## Single-shot interaction with user interrupt handlers
//!
//! The async API disables the pin's interrupt when triggered. This makes async
//! operations single-shot. If there is no user handler, the other GPIO
//! interrupts are also single-shot. This is because the user has no way to
//! handle multiple events in this case, the API only allows querying whether
//! the interrupt has fired or not. Disabling the interrupt also means that the
//! interrupt status bits are not cleared, so the `is_interrupt_set` works by
//! default as expected.
//!
//! When the user sets a custom interrupt handler, the built-in interrupt
//! handler will only disable the async interrupts. The user handler is
//! responsible for clearing the interrupt status bits or disabling the
//! interrupts, based on their needs. This is communicated to the user in the
//! documentation of the `Io::set_interrupt_handler` function.
//!
//! ## Critical sections
//!
//! The interrupt handler runs in a GPIO-specific critical section. The critical
//! section is required because interrupts are disabled by modifying the pin
//! register, which is not an atomic operation. The critical section also
//! ensures that a higher priority task waken by an async pin event (or the user
//! handler) will only run after the interrupt handler has finished.
//!
//! ## Signaling async completion
//!
//! The completion is signalled by clearing a flag in an AtomicU32. This flag is
//! set at the start of the async operation, and cleared when the interrupt
//! handler is called. The flag is not accessible by the user, so they can't
//! force-complete an async operation accidentally from the interrupt handler.
//!
//! We could technically use the interrupt status on single-core chips, but it
//! would be slightly more complicated to prevent the user from breaking things.
//! (If the user were to clear the interrupt status, we would need to re-enable
//! it, for PinFuture to detect the completion).
//!
//! TODO: currently, direct-binding a GPIO interrupt handler will completely
//! break the async API. We will need to expose a way to handle async events.

use portable_atomic::{AtomicPtr, Ordering};
use procmacros::ram;
use strum::EnumCount;

#[cfg(feature = "rt")]
use crate::interrupt::{self, DEFAULT_INTERRUPT_HANDLER};
use crate::{
    gpio::{AnyPin, GPIO_LOCK, GpioBank, InputPin, set_int_enable},
    interrupt::Priority,
    peripherals::{GPIO, Interrupt},
};

/// Convenience constant for `Option::None` pin
pub(super) static USER_INTERRUPT_HANDLER: CFnPtr = CFnPtr::new();

pub(super) struct CFnPtr(AtomicPtr<()>);
impl CFnPtr {
    pub const fn new() -> Self {
        Self(AtomicPtr::new(core::ptr::null_mut()))
    }

    pub fn store(&self, f: extern "C" fn()) {
        self.0.store(f as *mut (), Ordering::Relaxed);
    }

    pub fn call(&self) {
        let ptr = self.0.load(Ordering::Relaxed);
        if !ptr.is_null() {
            unsafe { (core::mem::transmute::<*mut (), extern "C" fn()>(ptr))() };
        }
    }
}

#[cfg(feature = "rt")]
pub(crate) fn bind_default_interrupt_handler() {
    // We first check if a handler is set in the vector table.
    if let Some(handler) = interrupt::bound_handler(Interrupt::GPIO) {
        // We only allow binding the default handler if nothing else is bound.
        // This prevents silently overwriting RTIC's interrupt handler, if using GPIO.
        if !core::ptr::fn_addr_eq(handler, DEFAULT_INTERRUPT_HANDLER.handler()) {
            // The user has configured an interrupt handler they wish to use.
            info!("Not using default GPIO interrupt handler: already bound in vector table");
            return;
        }
    }

    // The vector table doesn't contain a custom entry. Still, the
    // peripheral interrupt may already be bound to something else.
    for cpu in cores() {
        if interrupt::bound_cpu_interrupt_for(cpu, Interrupt::GPIO).is_some() {
            info!("Not using default GPIO interrupt handler: peripheral interrupt already in use");
            return;
        }
    }

    unsafe { interrupt::bind_interrupt(Interrupt::GPIO, default_gpio_interrupt_handler) };

    // By default, we use lowest priority
    set_interrupt_priority(Interrupt::GPIO, Priority::min());
}

cfg_if::cfg_if! {
    if #[cfg(esp32)] {
        // On ESP32, the interrupt fires on the core that started listening for a pin event.
        fn cores() -> impl Iterator<Item = crate::system::Cpu> {
            crate::system::Cpu::all()
        }
    } else {
        fn cores() -> [crate::system::Cpu; 1] {
            [crate::system::Cpu::current()]
        }
    }
}

pub(super) fn set_interrupt_priority(interrupt: Interrupt, priority: Priority) {
    for cpu in cores() {
        unwrap!(crate::interrupt::enable_on_cpu(cpu, interrupt, priority));
    }
}

/// The default GPIO interrupt handler, when the user has not set one.
///
/// This handler will disable all pending interrupts and leave the interrupt
/// status bits unchanged. This enables functions like `is_interrupt_set` to
/// work correctly.
#[ram]
#[cfg(feature = "rt")]
extern "C" fn default_gpio_interrupt_handler() {
    GPIO_LOCK.lock(|| {
        let banks = interrupt_status();

        // Handle the async interrupts
        for (bank, intrs) in banks {
            // Get the mask of active async pins and clear the relevant bits to signal
            // completion. This way the user may clear the interrupt status
            // without worrying about the async bit being cleared.
            let async_pins = bank.async_operations().load(Ordering::Relaxed);

            // Wake up the tasks
            handle_async_pins(bank, async_pins, intrs);

            // Disable the remaining interrupts.
            let mut intrs = intrs & !async_pins;
            while intrs != 0 {
                let pin_pos = intrs.trailing_zeros();
                intrs -= 1 << pin_pos;

                let pin_nr = pin_pos as u8 + bank.offset();

                // The remaining interrupts are not async, we treat them as single-shot.
                set_int_enable(pin_nr, Some(0), 0, false);
            }
        }
    });
}

/// The user GPIO interrupt handler, when the user has set one.
///
/// This handler only disables interrupts associated with async pins. The user
/// handler is responsible for clearing the interrupt status bits or disabling
/// the interrupts.
#[ram]
pub(super) extern "C" fn user_gpio_interrupt_handler() {
    GPIO_LOCK.lock(|| {
        // Read interrupt status before the user has a chance to modify them.
        let banks = interrupt_status();

        // Call the user handler before clearing interrupts. The user can use the enable
        // bits to determine which interrupts they are interested in. Clearing the
        // interupt status or enable bits have no effect on the rest of the
        // interrupt handler.
        USER_INTERRUPT_HANDLER.call();

        // Handle the async interrupts
        for (bank, intrs) in banks {
            // Get the mask of active async pins and clear the relevant bits to signal
            // completion. This way the user may clear the interrupt status
            // without worrying about the async bit being cleared.
            let async_pins = bank.async_operations().load(Ordering::Relaxed);

            // Wake up the tasks
            handle_async_pins(bank, async_pins, intrs);
        }
    });
}

#[derive(Clone, Copy)]
pub(crate) enum InterruptStatusRegisterAccess {
    Bank0,
    #[cfg(gpio_has_bank_1)]
    Bank1,
}

impl InterruptStatusRegisterAccess {
    pub(crate) fn interrupt_status_read(self) -> u32 {
        cfg_if::cfg_if! {
            if #[cfg(esp32)] {
                match self {
                    Self::Bank0 => GPIO::regs().status().read().bits(),
                    Self::Bank1 => GPIO::regs().status1().read().bits(),
                }
            } else if #[cfg(any(esp32c2, esp32c3, esp32c6, esp32h2))] {
                GPIO::regs().pcpu_int().read().bits()
            } else if #[cfg(any(esp32s2, esp32s3))] {
                // Whilst the S3 is a dual core chip, it shares the enable registers between
                // cores so treat it as a single core device
                match self {
                    Self::Bank0 => GPIO::regs().pcpu_int().read().bits(),
                    Self::Bank1 => GPIO::regs().pcpu_int1().read().bits(),
                }
            }
        }
    }
}

fn interrupt_status() -> [(GpioBank, u32); GpioBank::COUNT] {
    let intrs_bank0 = InterruptStatusRegisterAccess::Bank0.interrupt_status_read();

    #[cfg(gpio_has_bank_1)]
    let intrs_bank1 = InterruptStatusRegisterAccess::Bank1.interrupt_status_read();

    [
        (GpioBank::_0, intrs_bank0),
        #[cfg(gpio_has_bank_1)]
        (GpioBank::_1, intrs_bank1),
    ]
}

// We have separate variants for single-core and multi-core async pin handling.
// Single core can be much simpler because no code is running in parallel, so we
// don't have to be so careful with the order of operations. On multi-core,
// however, the order can actually break things, regardless of the critical
// section (because tasks on the other core may be waken in inappropriate
// times).

#[cfg(single_core)]
fn handle_async_pins(bank: GpioBank, async_pins: u32, intrs: u32) {
    let mut async_intrs = async_pins & intrs;
    while async_intrs != 0 {
        let pin_pos = async_intrs.trailing_zeros();
        async_intrs -= 1 << pin_pos;

        let pin_nr = pin_pos as u8 + bank.offset();

        // Disable the interrupt for this pin.
        set_int_enable(pin_nr, Some(0), 0, false);

        unsafe { AnyPin::steal(pin_nr) }.waker().wake();
    }

    // This is an optimization (in case multiple pin interrupts are handled at once)
    // so that PinFuture doesn't have to clear interrupt status bits one by one
    // for each pin. We need to clear after disabling the interrupt, so that a pin
    // event can't re-set the bit.
    bank.write_interrupt_status_clear(async_pins & intrs);

    // On a single-core chip, the lock around `handle_async_pins` ensures
    // that the interrupt handler will not be interrupted by other code,
    // so we can safely write back without an atomic CAS.
    bank.async_operations()
        .store(async_pins & !intrs, Ordering::Relaxed);
}

#[cfg(multi_core)]
fn handle_async_pins(bank: GpioBank, async_pins: u32, intrs: u32) {
    // First, disable pin interrupts. If we were to do this after clearing the async
    // flags, the PinFuture destructor may try to take a critical section which
    // isn't necessary.
    let mut async_intrs = async_pins & intrs;
    while async_intrs != 0 {
        let pin_pos = async_intrs.trailing_zeros();
        async_intrs -= 1 << pin_pos;

        let pin_nr = pin_pos as u8 + bank.offset();

        // Disable the interrupt for this pin.
        set_int_enable(pin_nr, Some(0), 0, false);
    }

    // This is an optimization (in case multiple pin interrupts are handled at once)
    // so that PinFuture doesn't have to clear interrupt status bits one by one
    // for each pin. We need to clear after disabling the interrupt, so that a pin
    // event can't re-set the bit.
    bank.write_interrupt_status_clear(async_pins & intrs);

    // Clearing the async bit needs to be the last state change, as this signals
    // completion.
    // On multi-core chips, we need to use a CAS to ensure that only
    // the handled async bits are cleared.
    bank.async_operations().fetch_and(!intrs, Ordering::Relaxed);

    // Now we can wake the tasks. This needs to happen after completion - waking an
    // already running task isn't a big issue, but not waking a waiting one is.
    // Doing it sooner would mean that on a multi-core chip a task could be
    // waken, but the Future wouldn't actually be resolved, and so it might
    // never wake again.
    let mut async_intrs = async_pins & intrs;
    while async_intrs != 0 {
        let pin_pos = async_intrs.trailing_zeros();
        async_intrs -= 1 << pin_pos;

        let pin_nr = pin_pos as u8 + bank.offset();

        unsafe { AnyPin::steal(pin_nr) }.waker().wake();
    }
}
