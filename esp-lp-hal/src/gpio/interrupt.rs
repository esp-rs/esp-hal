pub use interrupt::{Interrupt, InterruptHandler, InterruptStatus};
pub use procmacros::handler;

use super::{Io, LpIo};
use crate::interrupt;

/// Convenience constant for `Option::None` pin
pub static USER_INTERRUPT_HANDLER: interrupt::CFnPtr = interrupt::CFnPtr::new();

impl crate::interrupt::InterruptConfigurable for Io {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

/// The user GPIO interrupt handler, when the user has set one.
///
/// The user handler is responsible for clearing the interrupt status bits or disabling
/// the interrupts.
#[handler]
pub fn user_gpio_interrupt_handler() {
    // Call the user handler before clearing interrupts. The user can use the enable
    // bits to determine which interrupts they are interested in. Clearing the
    // interrupt status or enable bits have no effect on the rest of the
    // interrupt handler.
    USER_INTERRUPT_HANDLER.call();
}

/// Event type used to trigger interrupts.
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
pub enum Event {
    /// Interrupts trigger on rising pin edge.
    RisingEdge  = 1,
    /// Interrupts trigger on falling pin edge.
    FallingEdge = 2,
    /// Interrupts trigger on either rising or falling pin edges.
    AnyEdge     = 3,
    /// Interrupts trigger on low level
    LowLevel    = 4,
    /// Interrupts trigger on high level
    HighLevel   = 5,
}

/// Read the interrupt status of all pins
/// Bit 0 == GPIO0, in the returned value
#[inline]
pub fn gpio_interrupt_status() -> u32 {
    #[cfg(any(esp32s2, esp32s3))]
    {
        unsafe { &*LpIo::PTR }.status().read().bits() >> 10
    }

    #[cfg(esp32c6)]
    {
        todo!()
    }
}

/// Clear the interrupt status for a bit mask of pins
/// Expects pinmask bit 0 == GPIO0
#[inline]
pub fn gpio_interrupt_clear(pinmask: u32) {
    #[cfg(any(esp32s2, esp32s3))]
    unsafe { &*LpIo::PTR }
        .status_w1tc()
        .write(|w| unsafe { w.bits(pinmask << 10) });
    #[cfg(esp32c6)]
    todo!()
}

/// Set GPIO event listening.
///
/// - `N`: the pin to configure
/// - `int_type`: interrupt type code, value from [Event], 0 to disable interrupts. If None, will
///   leave the int_type setting as-is.
/// - `wake_up`: whether to wake up from light sleep.
pub fn enable_pin_interrupt<const N: u8>(int_type: u8) {
    #[cfg(any(esp32s2, esp32s3))]
    {
        let gpio_pin = unsafe { &*LpIo::PTR }.pin(N as usize);
        gpio_pin.write(|w| unsafe { w.int_type().bits(int_type) });
    }
    #[cfg(esp32c6)]
    todo!()
}

/// Clear pin interrupt
pub fn clear_pin_interrupt<const N: u8>() {
    gpio_interrupt_clear(1 << N);
}

/// Read pin interrupt status
pub fn is_interrupt_set<const N: u8>() -> bool {
    let stat = gpio_interrupt_status();
    (stat & (1 << N)) != 0
}

/// Enable / disable pin wakeup
pub fn pin_wakeup_enable<const N: u8>(en: bool) {
    #[cfg(any(esp32s2, esp32s3))]
    {
        let gpio_pin = unsafe { &*LpIo::PTR }.pin(N as usize);
        gpio_pin.write(|w| w.wakeup_enable().bit(en));
    }
    #[cfg(esp32c6)]
    todo!()
}
