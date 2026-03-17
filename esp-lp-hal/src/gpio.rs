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

cfg_if::cfg_if! {
    if #[cfg(esp32c6)] {
        type LpIo = crate::pac::LP_IO;
        const MAX_GPIO_PIN: u8 = 7;
    } else {
        type LpIo = crate::pac::RTC_IO;
        const MAX_GPIO_PIN: u8 = 21;
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
pub struct Flex<const PIN: u8> {}

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
}

/// Digital input.
pub struct Input<const PIN: u8> {
    pin: Flex<PIN>,
}

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
}

/// Digital output.
pub struct Output<const PIN: u8> {
    pin: Flex<PIN>,
}

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
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_output<const PIN: u8>() -> Option<Output<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(Output::<PIN>::new())
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_output_open_drain<const PIN: u8>() -> Option<OutputOpenDrain<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(OutputOpenDrain::<PIN>::new())
    }
}

// Used by the `entry` procmacro:
#[doc(hidden)]
pub unsafe fn conjure_input<const PIN: u8>() -> Option<Input<PIN>> {
    if PIN > MAX_GPIO_PIN {
        None
    } else {
        Some(Input::<PIN>::new())
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::ErrorType for Input<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::ErrorType for Output<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::ErrorType for OutputOpenDrain<PIN> {
    type Error = core::convert::Infallible;
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::InputPin for Input<PIN> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::High)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::Low)
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::OutputPin for Output<PIN> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::StatefulOutputPin for Output<PIN> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::High)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::Low)
    }
}

// OutputOpenDrain
#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::InputPin for OutputOpenDrain<PIN> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::High)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::Low)
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::OutputPin for OutputOpenDrain<PIN> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
impl<const PIN: u8> embedded_hal::digital::StatefulOutputPin for OutputOpenDrain<PIN> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::High)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::Low)
    }
}
