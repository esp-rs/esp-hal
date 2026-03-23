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
        /// Maximum GPIO pin number.
        pub const MAX_GPIO_PIN: u8 = 7;
    } else {
        type LpIo = crate::pac::RTC_IO;
        /// Maximum GPIO pin number.
        pub const MAX_GPIO_PIN: u8 = 21;
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

/// Event used to trigger interrupts.
#[cfg(any(esp32s2, esp32s3))]
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

#[cfg(any(esp32s2, esp32s3))]
impl From<WakeEvent> for Event {
    fn from(value: WakeEvent) -> Self {
        match value {
            WakeEvent::LowLevel => Event::LowLevel,
            WakeEvent::HighLevel => Event::HighLevel,
        }
    }
}

/// Event used to wake up from light sleep.
#[cfg(any(esp32s2, esp32s3))]
#[derive(Debug, Eq, PartialEq, Copy, Clone, Hash)]
pub enum WakeEvent {
    /// Wake on low level
    LowLevel  = 4,
    /// Wake on high level
    HighLevel = 5,
}

/// Set GPIO event listening.
///
/// - `N`: the pin to configure
/// - `int_type`: interrupt type code, value from [Event], 0 to disable interrupts. If None, will
///   leave the int_type setting as-is.
/// - `wake_up`: whether to wake up from light sleep.
#[cfg(any(esp32s2, esp32s3))]
fn enable_pin_interrupt<const N: u8>(int_type: Option<u8>, wakeup_enable: bool) {
    // let GPIO_BASE = unsafe { &*LpIo::PTR }.pin0().as_ptr().addr();
    // TODO: Add LpIo::regs().pins(number : usize) to esp-pacs
    let gpio_base = 0xa400 + 0x28 + ((N as usize) * 4);
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

#[cfg(any(esp32s2, esp32s3))]
fn clear_pin_interrupt<const N: u8>() {
    unsafe { &*LpIo::PTR }
        .status_w1tc()
        .write(|w| unsafe { w.bits(1 << N) });
}

/// Flexible pin driver.
/// Provides a common implementation for input and output pins.
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

    /// Listen for interrupts.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn listen(&mut self, event: Event) {
        self.clear_interrupts();
        enable_pin_interrupt::<PIN>(Some(event as u8), false);
    }

    /// Un-listen for interrupts.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn unlisten(&mut self) {
        enable_pin_interrupt::<PIN>(Some(0), false);
        self.clear_interrupts();
    }

    /// Clear pin interrupts
    #[cfg(any(esp32s2, esp32s3))]
    pub fn clear_interrupts(&mut self) {
        clear_pin_interrupt::<PIN>();
    }

    /// Enable pin as a wake-up source.
    /// This will change the interrupt type, when enabling.
    /// Interrupt type will not be changed, when disabling.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) {
        self.clear_interrupts();
        if enable {
            let int_evt = Event::from(event);
            enable_pin_interrupt::<PIN>(Some(int_evt as u8), true);
        } else {
            enable_pin_interrupt::<PIN>(None, true);
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
    /// Listen for interrupts.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event);
    }
    /// Un-listen for interrupts.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn unlisten(&mut self) {
        self.pin.unlisten();
    }
    /// Enable pin as a wake-up source.
    /// This will change the interrupt type, when enabling.
    /// Interrupt type will not be changed, when disabling.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) {
        self.pin.wakeup_enable(enable, event);
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
    /// Listen for interrupts.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn listen(&mut self, event: Event) {
        self.pin.listen(event);
    }
    /// Un-listen for interrupts.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn unlisten(&mut self) {
        self.pin.unlisten();
    }
    /// Enable pin as a wake-up source.
    /// This will change the interrupt type, when enabling.
    /// Interrupt type will not be changed, when disabling.
    #[cfg(any(esp32s2, esp32s3))]
    pub fn wakeup_enable(&mut self, enable: bool, event: WakeEvent) {
        self.pin.wakeup_enable(enable, event);
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
