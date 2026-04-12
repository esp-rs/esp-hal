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
    let gpio_bank_reg = LpIo::ptr() as usize;
    let int_type_mask: u32 = 0xFFFFFFFF ^ (0b111 << 7);

    for n in pin_iter {
        let gpio_base = gpio_bank_reg + 0x28 + ((n as usize) * 4);
        let gpio_pin = gpio_base as *mut u32;
        // Read settings, clear int_type flags, write settings
        let mut pin_setting = unsafe { gpio_pin.read_volatile() };
        pin_setting &= int_type_mask;
        unsafe { gpio_pin.write_volatile(pin_setting) }
    }
}

/// Set GPIO event listening.
///
/// - `N`: the pin to configure
/// - `int_type`: interrupt type code, value from [Event], 0 to disable interrupts. If None, will
///   leave the int_type setting as-is.
/// - `wake_up`: whether to wake up from light sleep.
pub fn enable_pin_interrupt<const N: u8>(int_type: Option<u8>, wakeup_enable: bool) {
    // let GPIO_BASE = unsafe { &*LpIo::PTR }.pin0().as_ptr().addr();
    // TODO: Add LpIo::regs().pins(number : usize) to esp-pacs
    let gpio_bank_reg = LpIo::ptr() as usize;
    let gpio_base = gpio_bank_reg + 0x28 + ((N as usize) * 4);
    let gpio_pin = gpio_base as *mut u32;

    // Read the current setting
    let mut pin_setting = unsafe { gpio_pin.read_volatile() };

    // Write int_type if specified
    if let Some(int_type) = int_type {
        // Bits 7:9
        // - 0: GPIO interrupt disable
        // - 1: rising edge trigger
        // - 2: falling edge trigger
        // - 3: any edge trigger
        // - 4: low level trigger
        // - 5: high level trigger
        let int_type_mask: u32 = 0xFFFFFFFF ^ (0b111 << 7);
        pin_setting &= int_type_mask;
        pin_setting |= (int_type as u32) << 7;
    }

    // Bit 10, wakeup enable.
    // Used with UlpCoreWakeupSource::Gpio.
    let wakeup_en_mask: u32 = 0xFFFFFFFF ^ (0b1 << 10);
    pin_setting &= wakeup_en_mask;
    if wakeup_enable {
        pin_setting |= 0b1 << 10;
    }

    unsafe { gpio_pin.write_volatile(pin_setting) }
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
    unsafe { &*crate::pac::RTC_CNTL::PTR }
        .rtc_ulp_cp_timer()
        .write(|w| w.ulp_cp_gpio_wakeup_ena().bit(enable));
}

/// Clear GPIO wakeup status
pub fn gpio_wakeup_clear() {
    unsafe { &*crate::pac::RTC_CNTL::PTR }
        .rtc_ulp_cp_timer()
        .write(|w| w.ulp_cp_gpio_wakeup_clr().set_bit());
}
