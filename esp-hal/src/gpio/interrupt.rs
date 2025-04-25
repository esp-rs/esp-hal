//! GPIO interrupt handling
//!
//! GPIO interrupt handling must work at the same time as the async API. Async
//! operations take pins by `&mut self`, so they can only be accessed after the
//! operation is complete, or cancelled. This means that manual `listen`
//! operations don't need to be prepared for async operations, but
//! async operations need to be prepared to handle cases where the pin was
//! configured to listen for an event.
//!
//! The async API communicated completion with an atomic bitfield. This
//! bitfield is cleared when the interrupt handler is called. This is done
//! because the user may register a custom interrupt handler which either
//! disables the interrupts or clears the interrupt status bits. The user
//! handler must not interfere with the async API, so we need some signaling
//! mechanism private to the async API.
//!
//! The async API disables the pin's interrupt when triggered. This makes async
//! operations single-shot. If there is no user handler, the other GPIO
//! interrupts are also single-shot. This is because the user has no way to
//! handle multiple events, the API only allows querying whether the interrupt
//! has fired or not. Disabling the interrupt also means that the interrupt
//! status bits are not cleared, so the `is_interrupt_set` works by default as
//! expected.
//!
//! Disabling the interrupt also allows the async Future to complete cheaply by
//! forgetting itself. Otherwise, to prevent a race condition, the Future's Drop
//! implementation needs to take the critical section to disable the interrupt.
//!
//! When the user sets a custom interrupt handler, the built-in interrupt
//! handler will only disable the async interrupts. The user handler is
//! responsible for clearing the interrupt status bits or disabling the
//! interrupts, based on their needs. This is communicated to the user in the
//! documentation of the `Io::set_interrupt_handler` function.
//!
//! TODO: currently, direct-binding a GPIO interrupt handler will completely
//! break the async API. We will need to expose a way to handle async events.
//!
//! The interrupt handler runs in a GPIO-specific critical section. The critical
//! section is required because interrupts are disabled by modifying the pin
//! register, which is not an atomic operation. The critical section also
//! ensures that a higher priority task waken by an async pin event (or the user
//! handler) will only run after the interrupt handler has finished.

use portable_atomic::{AtomicPtr, Ordering};
use procmacros::ram;
use strum::EnumCount;

use crate::{
    gpio::{GpioBank, InterruptStatusRegisterAccess, asynch, set_int_enable},
    interrupt::{self, DEFAULT_INTERRUPT_HANDLER, Priority},
    peripherals::Interrupt,
    sync::RawMutex,
};

/// Convenience constant for `Option::None` pin
pub(super) static USER_INTERRUPT_HANDLER: CFnPtr = CFnPtr::new();

pub(super) static GPIO_LOCK: RawMutex = RawMutex::new();

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

pub(crate) fn bind_default_interrupt_handler() {
    // We first check if a handler is set in the vector table.
    if let Some(handler) = interrupt::bound_handler(Interrupt::GPIO) {
        let handler = handler as *const unsafe extern "C" fn();

        // We only allow binding the default handler if nothing else is bound.
        // This prevents silently overwriting RTIC's interrupt handler, if using GPIO.
        if !core::ptr::eq(handler, DEFAULT_INTERRUPT_HANDLER.handler() as _) {
            // The user has configured an interrupt handler they wish to use.
            info!("Not using default GPIO interrupt handler: already bound in vector table");
            return;
        }
    }
    // The vector table doesn't contain a custom entry.Still, the
    // peripheral interrupt may already be bound to something else.
    if interrupt::bound_cpu_interrupt_for(crate::system::Cpu::current(), Interrupt::GPIO).is_some()
    {
        info!("Not using default GPIO interrupt handler: peripheral interrupt already in use");
        return;
    }

    unsafe { interrupt::bind_interrupt(Interrupt::GPIO, default_gpio_interrupt_handler) };
    // By default, we use lowest priority
    unwrap!(interrupt::enable(Interrupt::GPIO, Priority::min()));
}

/// The default GPIO interrupt handler, when the user has not set one.
///
/// This handler will disable all pending interrupts and leave the interrupt
/// status bits unchanged. This enables functions like `is_interrupt_set` to
/// work correctly.
#[ram]
extern "C" fn default_gpio_interrupt_handler() {
    GPIO_LOCK.lock(|| {
        let banks = interrupt_status();

        // Handle the async interrupts
        for (bank, intrs) in banks {
            // Get the mask of active async pins and clear the relevant bits to signal
            // completion. This way the user may clear the interrupt status
            // without worrying about the async bit being cleared.
            let async_pins = bank.async_operations().fetch_and(!intrs, Ordering::Relaxed);

            // Wake up the tasks
            handle_async_pins(bank, intrs & async_pins);

            // Disable the remaining interrupts.
            let mut intrs = intrs & !async_pins;
            while intrs != 0 {
                let pin_pos = intrs.trailing_zeros();
                intrs -= 1 << pin_pos;

                let pin_nr = pin_pos as u8 + bank.offset();

                // Disable the interrupt for this pin. This must be done as the last operation
                // on the pin, so that `async fn wait_for` will not
                // race with the interrupt handler.
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
            let async_pins = bank.async_operations().fetch_and(!intrs, Ordering::Relaxed);

            // Wake up the tasks
            handle_async_pins(bank, intrs & async_pins);
        }
    });
}

fn interrupt_status() -> [(GpioBank, u32); GpioBank::COUNT] {
    let intrs_bank0 = InterruptStatusRegisterAccess::Bank0.interrupt_status_read();

    #[cfg(gpio_bank_1)]
    let intrs_bank1 = InterruptStatusRegisterAccess::Bank1.interrupt_status_read();

    [
        (GpioBank::_0, intrs_bank0),
        #[cfg(gpio_bank_1)]
        (GpioBank::_1, intrs_bank1),
    ]
}

fn handle_async_pins(bank: GpioBank, mut async_intrs: u32) {
    while async_intrs != 0 {
        let pin_pos = async_intrs.trailing_zeros();
        async_intrs -= 1 << pin_pos;

        let pin_nr = pin_pos as u8 + bank.offset();
        // The pin is async, we need to wake up the task.
        asynch::PIN_WAKERS[pin_nr as usize].wake();

        // Disable the interrupt for this pin. This must be done as the last operation
        // on the pin, so that `async fn wait_for` will not
        // race with the interrupt handler.
        set_int_enable(pin_nr, Some(0), 0, false);
    }
}
