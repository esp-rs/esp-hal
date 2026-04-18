//! # General Purpose Input/Output
//!
//! ## Overview
//!
//! It's assumed that GPIOs are already configured correctly by the HP core.
//!
//! This driver supports various operations on GPIO pins, primarily manipulating
//! the pin state (setting high/low, toggling).
//!
//! This module also implements a number of traits from `embedded-hal` to
//! provide a common interface for GPIO pins.
//!
//! ## Examples
//!
//! ```rust,no_run
//! fn main(gpio0: Input<0>, gpio1: Output<1>) -> ! {
//!     loop {
//!         let input_state: bool = gpio0.input_state();
//!         gpio.set_output(input_state);
//!
//!         esp_lp_hal::delay::Delay.delay_millis(50);
//!     }
//! }
//! ```

/// Used by the `entry` procmacro
pub mod conjure;
pub use conjure::*;

#[cfg(feature = "embedded-hal")]
/// Provides embedded-hal trait impls
pub mod ehal;

/// Provides GPIO interrupt support
// TODO: Implement GPIO interrupts for ESP32C6 LP core
#[cfg(feature = "interrupts")]
pub mod interrupt;
#[cfg(feature = "interrupts")]
pub use interrupt::{
    Event,
    Interrupt,
    InterruptHandler,
    USER_INTERRUPT_HANDLER,
    gpio_interrupt_clear,
    gpio_interrupt_status,
    user_gpio_interrupt_handler,
};

cfg_if::cfg_if! {
    if #[cfg(esp32c6)] {
        type LpIo = crate::pac::LP_IO;
        /// Maximum GPIO pin number.
        pub const MAX_GPIO_PIN: u8 = 7;
    } else {
        type LpIo = crate::pac::RTC_IO;
        /// Maximum GPIO pin number.
        pub const MAX_GPIO_PIN: u8 = 21;
    }
}

/// General Purpose Input/Output driver
pub struct Io {
    _io_peripheral: LpIo,
}

impl Io {
    /// Initialize the I/O driver.
    pub fn new(_io_peripheral: LpIo) -> Self {
        Io { _io_peripheral }
    }

    #[cfg(feature = "interrupts")]
    #[doc = "Registers an interrupt handler for all GPIO pins."]
    /// Note that when using interrupt handlers registered by this function, or
    /// by defining a `#[no_mangle] unsafe extern "C" fn GPIO()` function, we do
    /// **not** clear the interrupt status register or the interrupt enable
    /// setting for you. Based on your use case, you need to do one of this
    /// yourself:
    ///
    /// - Disabling the interrupt enable setting for the GPIO pin allows you to handle an event once
    ///   per call to [`listen()`]. Using this method, the [`is_interrupt_set()`] method will return
    ///   `true` if the interrupt is set even after your handler has finished running.
    /// - Clearing the interrupt status register allows you to handle an event repeatedly after
    ///   [`listen()`] is called. Using this method, [`is_interrupt_set()`] will return `false`
    ///   after your handler has finished running.
    ///
    /// [`listen()`]: Input::listen
    /// [`is_interrupt_set()`]: Input::is_interrupt_set
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        USER_INTERRUPT_HANDLER.store(handler.handler().callback());
        #[cfg(any(esp32s2, esp32s3))]
        {
            crate::interrupt::bind_handler(Interrupt::GPIO_INT, user_gpio_interrupt_handler);
        }

        #[cfg(esp32c6)]
        {
            todo!()
        }
    }
}

/// Digital input or output level.
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
pub enum Level {
    /// Low
    Low,
    /// High
    High,
}

impl core::ops::Not for Level {
    type Output = Self;

    fn not(self) -> Self {
        match self {
            Self::Low => Self::High,
            Self::High => Self::Low,
        }
    }
}

impl Level {
    /// Create a [`Level`] from [`bool`].
    ///
    /// Like `<Level as From<bool>>::from(val)`, but `const`.
    pub(crate) const fn const_from(val: bool) -> Self {
        match val {
            true => Self::High,
            false => Self::Low,
        }
    }

    /// Convert a [`Level`] to [`bool`].
    ///
    /// Like `<bool as From<Level>>::from(self)`, but `const`.
    pub(crate) const fn const_into(self) -> bool {
        match self {
            Level::Low => false,
            Level::High => true,
        }
    }
}

impl From<bool> for Level {
    fn from(val: bool) -> Self {
        Self::const_from(val)
    }
}

impl From<Level> for bool {
    fn from(level: Level) -> bool {
        level.const_into()
    }
}

/// Flexible pin driver.
/// Provides a common implementation for input and output pins.
/// Currently hidden, as it is not intended to be used directly,
/// because it does not handle changing the pin modes.
#[doc(hidden)]
pub struct Flex<const PIN: u8> {}

#[allow(clippy::new_without_default)]
impl<const PIN: u8> Flex<PIN> {
    /// Create flexible pin driver for a [Pin].
    /// No mode change happens.
    pub const fn new() -> Self {
        Self {}
    }

    // Input functions

    /// Get the current pin input level.
    pub fn level(&self) -> Level {
        cfg_if::cfg_if! {
            if #[cfg(esp32c6)] {
                ((unsafe { &*LpIo::PTR }.in_().read().bits() >> PIN) & 0x1 != 0).into()
            } else if #[cfg(esp32s2)] {
                ((unsafe { &*LpIo::PTR }.in_().read().gpio_in_next().bits() >> PIN) & 0x1 != 0).into()
            } else if #[cfg(esp32s3)] {
                ((unsafe { &*LpIo::PTR }.in_().read().next().bits() >> PIN) & 0x1 != 0).into()
            }
        }
    }

    // Output functions

    /// Set the output level.
    pub fn set_level(&mut self, level: Level) {
        if level == Level::High {
            unsafe { &*LpIo::PTR }
                .out_w1ts()
                .write(|w| unsafe { w.out_data_w1ts().bits(1 << PIN) });
        } else {
            unsafe { &*LpIo::PTR }
                .out_w1tc()
                .write(|w| unsafe { w.out_data_w1tc().bits(1 << PIN) });
        }
    }

    /// What level output is set to
    pub fn output_level(&self) -> Level {
        cfg_if::cfg_if! {
            if #[cfg(esp32c6)] {
                ((unsafe { &*LpIo::PTR }.out().read().bits() >> PIN) & 0x1 != 0).into()
            } else if #[cfg(esp32s2)] {
                ((unsafe { &*LpIo::PTR }.out().read().gpio_out_data().bits() >> PIN) & 0x1 != 0).into()
            } else if #[cfg(esp32s3)] {
                ((unsafe { &*LpIo::PTR }.out().read().data().bits() >> PIN) & 0x1 != 0).into()
            }
        }
    }

    /// Toggle pin output
    pub fn toggle(&mut self) {
        let level = self.output_level();
        self.set_level(!level);
    }

    // Interrupt-related APIs
    cfg_if::cfg_if! {
        if #[cfg(feature="interrupts")]
        {
            /// Listen for interrupts.
            pub fn listen(&mut self, event : Event, wakeup_enable : bool) {
                // Validate the inputs
                if wakeup_enable {
                    match event {
                        Event::LowLevel | Event::HighLevel => {},
                        _ => {
                            panic!("GPIO wakeup is only supported with LowLevel or HighLevel event types.");
                        }
                    }
                }

                self.clear_interrupt();
                interrupt::enable_pin_interrupt::<PIN>(event as u8);
                interrupt::pin_wakeup_enable::<PIN>(wakeup_enable);
            }

            /// Un-listen for interrupts.
            pub fn unlisten(&mut self) {
                interrupt::enable_pin_interrupt::<PIN>(0);
                interrupt::pin_wakeup_enable::<PIN>(false);
                self.clear_interrupt();
            }

            /// Clear pin interrupts
            pub fn clear_interrupt(&mut self) {
                interrupt::clear_pin_interrupt::<PIN>();
            }

            /// Read pin interrupt
            pub fn is_interrupt_set(&mut self) -> bool {
                interrupt::is_interrupt_set::<PIN>()
            }
        }
    }
}

/// Digital input.
pub struct Input<const PIN: u8> {
    pin: Flex<PIN>,
}

#[allow(clippy::new_without_default)]
impl<const PIN: u8> Input<PIN> {
    /// Creates a new GPIO input.
    pub const fn new() -> Self {
        let pin = Flex::<PIN>::new();
        Self { pin }
    }
    /// Get the current pin input level.
    pub fn level(&self) -> Level {
        self.pin.level()
    }

    // Interrupt-related APIs
    cfg_if::cfg_if! {
        if #[cfg(feature="interrupts")]
        {
            /// Listen for interrupts.
            pub fn listen(&mut self, event : Event, wakeup_enable : bool) {
                self.pin.listen(event, wakeup_enable);
            }
            /// Un-listen for interrupts.
            pub fn unlisten(&mut self) {
                self.pin.unlisten();
            }
            /// Clear pin interrupts
            pub fn clear_interrupt(&mut self) {
                interrupt::clear_pin_interrupt::<PIN>();
            }
            /// Read pin interrupt
            pub fn is_interrupt_set(&mut self) -> bool {
                interrupt::is_interrupt_set::<PIN>()
            }
        }
    }
}

/// Digital output.
pub struct Output<const PIN: u8> {
    pin: Flex<PIN>,
}

#[allow(clippy::new_without_default)]
impl<const PIN: u8> Output<PIN> {
    /// Creates a new GPIO output.
    pub const fn new() -> Self {
        let pin = Flex::<PIN>::new();
        Self { pin }
    }
    /// Set the output level.
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level);
    }
    /// Returns which level the pin is set to.
    pub fn output_level(&self) -> Level {
        self.pin.output_level()
    }
    /// Toggles the pin output.
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }
}

/// Digital output, open drain mode
/// Can be read as an input pin.
pub struct OutputOpenDrain<const PIN: u8> {
    pin: Flex<PIN>,
}

#[allow(clippy::new_without_default)]
impl<const PIN: u8> OutputOpenDrain<PIN> {
    /// Creates a new GPIO output.
    pub const fn new() -> Self {
        let pin = Flex::<PIN>::new();
        Self { pin }
    }
    /// Get the current pin input level.
    pub fn level(&self) -> Level {
        self.pin.level()
    }
    /// Set the output level.
    pub fn set_level(&mut self, level: Level) {
        self.pin.set_level(level);
    }
    /// Returns which level the pin is set to.
    pub fn output_level(&self) -> Level {
        self.pin.output_level()
    }
    /// Toggles the pin output.
    pub fn toggle(&mut self) {
        self.pin.toggle()
    }

    // Interrupt-related APIs
    cfg_if::cfg_if! {
        if #[cfg(feature="interrupts")]
        {
            /// Listen for interrupts.
            pub fn listen(&mut self, event : Event, wakeup_enable : bool) {
                self.pin.listen(event, wakeup_enable);
            }
            /// Un-listen for interrupts.
            pub fn unlisten(&mut self) {
                self.pin.unlisten();
            }
            /// Clear pin interrupts
            pub fn clear_interrupt(&mut self) {
                interrupt::clear_pin_interrupt::<PIN>();
            }
            /// Read pin interrupt
            pub fn is_interrupt_set(&mut self) -> bool {
                interrupt::is_interrupt_set::<PIN>()
            }
        }
    }
}
