pub use interrupt::{Interrupt, InterruptHandler, InterruptStatus};
pub use procmacros::handler;

use super::{Io, LpIo};
use crate::interrupt;

/// Convenience constant for `Option::None` pin
pub(super) static USER_INTERRUPT_HANDLER: interrupt::CFnPtr = interrupt::CFnPtr::new();

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

/// The default GPIO interrupt handler, when the user has not set one.
///
/// This handler will disable all pending interrupts and leave the interrupt
/// status bits unchanged. This enables functions like `is_interrupt_set` to
/// work correctly.
#[handler]
fn default_gpio_interrupt_handler() {
    let status = gpio_interrupt_status();
    gpio_interrupt_disable(status);
}

#[doc(hidden)]
pub fn bind_default_interrupt_handler() {
    interrupt::bind_handler(Interrupt::GPIO_INT, default_gpio_interrupt_handler);
}

/// Read the interrupt status of all pins
/// Bit 0 == GPIO0, in the returned value
#[inline]
pub fn gpio_interrupt_status() -> u32 {
    unsafe { &*LpIo::PTR }.status().read().bits() >> 10
}

/// Clear the interrupt status for a bit mask of pins
/// Expects pinmask bit 0 == GPIO0
#[inline]
pub fn gpio_interrupt_clear(pinmask: u32) {
    unsafe { &*LpIo::PTR }
        .status_w1tc()
        .write(|w| unsafe { w.bits(pinmask << 10) });
}

/// Disable interrupts for a bit mask of pins
fn gpio_interrupt_disable(pin_mask: u32) {
    // expects pinmask bit 0 == GPIO0
    let pin_iter = InterruptStatus::from(pin_mask).iterator();
    for n in pin_iter {
        unsafe { &*LpIo::PTR }
            .pin(n as usize)
            .write(|w| unsafe { w.int_type().bits(0) });
    }
}

/// Set GPIO event listening.
///
/// - `N`: the pin to configure
/// - `int_type`: interrupt type code, value from [Event], 0 to disable interrupts. If None, will
///   leave the int_type setting as-is.
/// - `wake_up`: whether to wake up from light sleep.
pub fn enable_pin_interrupt<const N: u8>(int_type: u8) {
    let gpio_pin = unsafe { &*LpIo::PTR }.pin(N as usize);

    // - 0: GPIO interrupt disable
    // - 1: rising edge trigger
    // - 2: falling edge trigger
    // - 3: any edge trigger
    // - 4: low level trigger
    // - 5: high level trigger
    gpio_pin.write(|w| unsafe { w.int_type().bits(int_type) });
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

/// Global GPIO wakeup enable/disable
pub fn gpio_wakeup_enable(enable: bool) {
    #[cfg(esp32s2)]
    unsafe { &*crate::pac::RTC_CNTL::PTR }
        .ulp_cp_timer()
        .write(|w| w.ulp_cp_gpio_wakeup_ena().bit(enable));
    #[cfg(esp32s3)]
    unsafe { &*crate::pac::RTC_CNTL::PTR }
        .rtc_ulp_cp_timer()
        .write(|w| w.ulp_cp_gpio_wakeup_ena().bit(enable));
}

/// Clear GPIO wakeup status
pub fn gpio_wakeup_clear() {
    #[cfg(esp32s2)]
    unsafe { &*crate::pac::RTC_CNTL::PTR }
        .ulp_cp_timer()
        .write(|w| w.ulp_cp_gpio_wakeup_clr().set_bit());
    #[cfg(esp32s3)]
    unsafe { &*crate::pac::RTC_CNTL::PTR }
        .rtc_ulp_cp_timer()
        .write(|w| w.ulp_cp_gpio_wakeup_clr().set_bit());
}
