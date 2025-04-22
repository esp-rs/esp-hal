use portable_atomic::{AtomicPtr, Ordering};
use procmacros::ram;

use crate::{
    gpio::{GpioBank, InterruptStatusRegisterAccess, asynch, set_int_enable},
    interrupt::{self, DEFAULT_INTERRUPT_HANDLER, Priority},
    peripherals::Interrupt,
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

#[ram]
pub(super) extern "C" fn user_gpio_interrupt_handler() {
    handle_pin_interrupts(|| USER_INTERRUPT_HANDLER.call());
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

#[ram]
extern "C" fn default_gpio_interrupt_handler() {
    handle_pin_interrupts(|| ());
}

#[ram]
fn handle_pin_interrupts(user_handler: fn()) {
    let intrs_bank0 = InterruptStatusRegisterAccess::Bank0.interrupt_status_read();

    #[cfg(gpio_bank_1)]
    let intrs_bank1 = InterruptStatusRegisterAccess::Bank1.interrupt_status_read();

    user_handler();

    let banks = [
        (GpioBank::_0, intrs_bank0),
        #[cfg(gpio_bank_1)]
        (GpioBank::_1, intrs_bank1),
    ];

    for (bank, intrs) in banks {
        // Get the mask of active async pins and also unmark them in the same go.
        let async_pins = bank.async_operations().fetch_and(!intrs, Ordering::Relaxed);

        // Wake up the tasks
        let mut intr_bits = intrs & async_pins;
        while intr_bits != 0 {
            let pin_pos = intr_bits.trailing_zeros();
            intr_bits -= 1 << pin_pos;

            let pin_nr = pin_pos as u8 + bank.offset();

            crate::interrupt::free(|| {
                asynch::PIN_WAKERS[pin_nr as usize].wake();
                set_int_enable(pin_nr, Some(0), 0, false);
            });
        }

        bank.write_interrupt_status_clear(intrs);
    }
}
