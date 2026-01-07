//! Dedicated GPIO access.
//!
//! This module implements fast GPIO access using special-purpose CPU features.
//! Dedicated GPIO access works by connecting peripheral signals between the CPU
//! and the GPIO pins, instead of using the GPIO registers via the peripheral bus.
//!
//! To use dedicated GPIOs, first you must obtain the channel objects. The channels
//! are created by calling the [`DedicatedGpio::new`] function. The fields of the
//! [`DedicatedGpio`] struct represent the channels. The channels are [`DedicatedGpioChannel`]
//! objects, which by default represent both input and output channels. The channels can be split
//! into [`DedicatedGpioInputChannel`] and [`DedicatedGpioOutputChannel`] halves, or may be
//! used unsplit.
#![doc = concat!(r#"

 > "#, chip!() , r#" specific: There are "#, property!("dedicated_gpio.channel_count", str) , r#" dedicated GPIO channels available.

 "#)]
//! Next, configure the pins you want to use as normal GPIO pin drivers. Then you can wrap them
//! into the dedicated GPIO drivers:
//! - [`DedicatedGpioInput`]
//! - [`DedicatedGpioOutput`]
//! - [`DedicatedGpioFlex`]
//!
//! The drivers can take channels and pins by value or by reference. Pass these objects by reference
//! if you plan on reusing them again, after dropping the dedicated driver.
//!
//! Due to how the hardware works, [`DedicatedGpioOutput`] can drive any number of GPIO pins.
#![cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Dedicated GPIOs are tied to CPU cores and a driver must only be used on the core that has created it.
Do not send the drivers to another core, either directly, or indirectly via a thread that is not pinned to a core.
</section>
"#
)]

use core::{convert::Infallible, marker::PhantomData};

use procmacros::doc_replace;
use strum::EnumCount as _;

use crate::{
    gpio::{
        AnyPin,
        GpioBank,
        InputSignal,
        Level,
        OutputSignal,
        interconnect::{PeripheralOutput, PeripheralSignal},
    },
    peripherals::GPIO_DEDICATED,
    private::Sealed,
    system::Cpu,
};

for_each_dedicated_gpio!(
    (channels $(($ch:literal)),*) => {
        paste::paste!(
            /// Dedicated GPIO channels.
            pub struct DedicatedGpio<'lt> {
                $(
                    #[doc = concat!("Channel ", stringify!($ch))]
                    pub [< channel $ch >]: DedicatedGpioChannel<'lt, $ch>,
                )*
            }
        );

        impl<'lt> DedicatedGpio<'lt> {
            /// Initializes the dedicated GPIO features and returns the channels.
            pub fn new(_: GPIO_DEDICATED<'lt>) -> Self {
                #[cfg(dedicated_gpio_needs_initialization)]
                Self::initialize();

                paste::paste!(
                    Self {
                        $([<channel $ch>]: DedicatedGpioChannel {
                            input: DedicatedGpioInputChannel { _marker: PhantomData },
                            output: DedicatedGpioOutputChannel { _marker: PhantomData },
                        },)*
                    }
                )
            }
        }
    };
);

impl DedicatedGpio<'_> {
    #[cfg(dedicated_gpio_needs_initialization)]
    fn initialize() {
        crate::system::PeripheralClockControl::enable(crate::system::Peripheral::DedicatedGpio);

        #[cfg(esp32s2)]
        {
            // Allow instruction access, which has better performance than register access.
            let regs = unsafe { esp32s2::DEDICATED_GPIO::steal() };
            regs.out_cpu()
                .write(|w| unsafe { w.bits((1 << property!("dedicated_gpio.channel_count")) - 1) });
        }
    }
}

// Channel traits allow us to implement DedicatedInputOutput, and users can choose to work with 8 IO
// channels for inputs/outputs instead of 8+8.

/// Marker trait for dedicated GPIO input channels.
pub trait InputChannel: Sealed {
    #[doc(hidden)]
    const CH: u8;

    #[doc(hidden)]
    fn mask(&self) -> u32 {
        1 << Self::CH
    }

    #[doc(hidden)]
    fn input_signal(&self) -> InputSignal {
        for_each_dedicated_gpio!(
            (signals $( ($cpu:literal, $signal_idx:literal, $signal_name:ident) ),*) => {
                const CHANNEL_COUNT: usize = property!("dedicated_gpio.channel_count");
                const SIGNALS: [[InputSignal; CHANNEL_COUNT]; Cpu::COUNT] = const {
                    let mut signals = [[core::mem::MaybeUninit::uninit(); CHANNEL_COUNT]; Cpu::COUNT];

                    $(
                        signals[$cpu][$signal_idx].write(InputSignal::$signal_name);
                    )*

                    unsafe { core::mem::transmute(signals) }
                };
            };
        );
        SIGNALS[Cpu::current() as usize][Self::CH as usize]
    }
}

/// Marker trait for dedicated GPIO output channels.
pub trait OutputChannel: Sealed {
    #[doc(hidden)]
    const CH: u8;

    #[doc(hidden)]
    fn mask(&self) -> u32 {
        1 << Self::CH
    }

    #[doc(hidden)]
    fn output_signal(&self) -> OutputSignal {
        for_each_dedicated_gpio!(
            (signals $( ($cpu:literal, $signal_idx:literal, $signal_name:ident) ),* ) => {
                const CHANNEL_COUNT: usize = property!("dedicated_gpio.channel_count");
                const SIGNALS: [[OutputSignal; CHANNEL_COUNT]; Cpu::COUNT] = const {
                    let mut signals = [[core::mem::MaybeUninit::uninit(); CHANNEL_COUNT]; Cpu::COUNT];

                    $(
                        signals[$cpu][$signal_idx].write(OutputSignal::$signal_name);
                    )*

                    unsafe { core::mem::transmute(signals) }
                };
            };
        );
        SIGNALS[Cpu::current() as usize][Self::CH as usize]
    }
}

// Driver traits, to allow working with Flex. These are implemented for both owned types and
// references, so that the user is not forced to give up the pin driver, and that we're not forced
// to figure out Drop semantics with `reborrow`, and the pins aren't remembered by the dedicated
// drivers anyway.
// We can't use the interconnect traits, because we want them to be implemented for references, too.

/// Marker trait for GPIO input drivers.
pub trait InputDriver: Sealed {
    #[doc(hidden)]
    fn pin(&self) -> u8;

    /// Connects the pin's peripheral input to an input signal source.
    #[doc(hidden)]
    fn set_input_connection(&self, signal: InputSignal);
}

#[instability::unstable]
impl InputDriver for super::Input<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin.pin
    }

    fn set_input_connection(&self, signal: InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}

#[instability::unstable]
impl InputDriver for &mut super::Input<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin.pin
    }

    fn set_input_connection(&self, signal: InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}

#[instability::unstable]
impl InputDriver for super::Flex<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin
    }

    fn set_input_connection(&self, signal: InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}

#[instability::unstable]
impl InputDriver for &mut super::Flex<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin
    }

    fn set_input_connection(&self, signal: InputSignal) {
        self.connect_input_to_peripheral(signal);
    }
}

/// Marker trait for GPIO output drivers.
pub trait OutputDriver: Sealed {
    #[doc(hidden)]
    fn pin(&self) -> u8;

    #[doc(hidden)]
    fn set_output_connection(&self, signal: OutputSignal);
}

#[instability::unstable]
impl OutputDriver for super::Output<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin.pin
    }

    fn set_output_connection(&self, signal: OutputSignal) {
        self.pin.pin.connect_peripheral_to_output(signal);
    }
}

#[instability::unstable]
impl OutputDriver for &mut super::Output<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin.pin
    }

    fn set_output_connection(&self, signal: OutputSignal) {
        self.pin.pin.connect_peripheral_to_output(signal);
    }
}

#[instability::unstable]
impl OutputDriver for super::Flex<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin
    }

    fn set_output_connection(&self, signal: OutputSignal) {
        self.pin.connect_peripheral_to_output(signal);
    }
}

#[instability::unstable]
impl OutputDriver for &mut super::Flex<'_> {
    fn pin(&self) -> u8 {
        self.pin.pin
    }

    fn set_output_connection(&self, signal: OutputSignal) {
        self.pin.connect_peripheral_to_output(signal);
    }
}

/// A single dedicated GPIO channel, both input and output.
///
/// You can split the channel by moving its fields out into separate input/output channel variables.
pub struct DedicatedGpioChannel<'lt, const CH: u8> {
    /// Channel input
    pub input: DedicatedGpioInputChannel<'lt, CH>,

    /// Channel output
    pub output: DedicatedGpioOutputChannel<'lt, CH>,
}

impl<const CH: u8> Sealed for DedicatedGpioChannel<'_, CH> {}
impl<const CH: u8> Sealed for &mut DedicatedGpioChannel<'_, CH> {}

impl<const CH: u8> DedicatedGpioChannel<'_, CH> {
    /// Conjures a new dedicated GPIO channel out of thin air.
    ///
    /// # Safety
    ///
    /// The [`DedicatedGpio`] struct must be initialized before this function is called. There
    /// should only be one reference to the channel in use at one time.
    pub unsafe fn steal() -> Self {
        unsafe {
            Self {
                input: DedicatedGpioInputChannel::steal(),
                output: DedicatedGpioOutputChannel::steal(),
            }
        }
    }
}

impl<const CH: u8> InputChannel for DedicatedGpioChannel<'_, CH> {
    const CH: u8 = CH;
}

impl<const CH: u8> OutputChannel for DedicatedGpioChannel<'_, CH> {
    const CH: u8 = CH;
}

impl<const CH: u8> InputChannel for &mut DedicatedGpioChannel<'_, CH> {
    const CH: u8 = CH;
}

impl<const CH: u8> OutputChannel for &mut DedicatedGpioChannel<'_, CH> {
    const CH: u8 = CH;
}

/// A single dedicated GPIO input channel.
pub struct DedicatedGpioInputChannel<'lt, const CH: u8> {
    _marker: PhantomData<&'lt mut ()>,
}

impl<const CH: u8> Sealed for DedicatedGpioInputChannel<'_, CH> {}
impl<const CH: u8> Sealed for &mut DedicatedGpioInputChannel<'_, CH> {}

impl<const CH: u8> DedicatedGpioInputChannel<'_, CH> {
    /// Conjures a new dedicated GPIO input channel out of thin air.
    ///
    /// # Safety
    ///
    /// The [`DedicatedGpio`] struct must be initialized before this function is called. There
    /// should only be one reference to the channel in use at one time.
    pub unsafe fn steal() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<const CH: u8> InputChannel for DedicatedGpioInputChannel<'_, CH> {
    const CH: u8 = CH;
}

impl<const CH: u8> InputChannel for &mut DedicatedGpioInputChannel<'_, CH> {
    const CH: u8 = CH;
}

/// A single dedicated GPIO output channel.
pub struct DedicatedGpioOutputChannel<'lt, const CH: u8> {
    _marker: PhantomData<&'lt mut ()>,
}

impl<const CH: u8> Sealed for DedicatedGpioOutputChannel<'_, CH> {}
impl<const CH: u8> Sealed for &mut DedicatedGpioOutputChannel<'_, CH> {}

impl<const CH: u8> DedicatedGpioOutputChannel<'_, CH> {
    /// Conjures a new dedicated GPIO output channel out of thin air.
    ///
    /// # Safety
    ///
    /// The [`DedicatedGpio`] struct must be initialized before this function is called. There
    /// should only be one reference to the channel in use at one time.
    pub unsafe fn steal() -> Self {
        Self {
            _marker: PhantomData,
        }
    }
}

impl<const CH: u8> OutputChannel for DedicatedGpioOutputChannel<'_, CH> {
    const CH: u8 = CH;
}

impl<const CH: u8> OutputChannel for &mut DedicatedGpioOutputChannel<'_, CH> {
    const CH: u8 = CH;
}

#[doc_replace]
/// A dedicated GPIO input driver.
#[cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Note that the driver must only be used on the core that has created it. Do not send the driver to
another core, either directly, or indirectly via a thread that is not pinned to a core.
</section>
"#
)]
/// ## Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// #
/// use esp_hal::gpio::{
///     Input,
///     InputConfig,
///     Level,
///     dedicated::{DedicatedGpio, DedicatedGpioInput},
/// };
///
/// // Create an input pin driver:
/// let input = Input::new(peripherals.GPIO0, InputConfig::default());
///
/// // Create a dedicated GPIO driver:
/// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
/// let mut dedicated_input = DedicatedGpioInput::new(channels.channel0, input);
///
/// // Now you can use the pin:
/// let level = dedicated_input.level();
/// #
/// # {after_snippet}
/// ```
pub struct DedicatedGpioInput<'lt> {
    _marker: PhantomData<&'lt mut ()>,
    mask: u32,
    #[cfg(all(debug_assertions, multi_core))]
    core: Cpu,
}

impl<'lt> DedicatedGpioInput<'lt> {
    /// Creates a new dedicated GPIO input driver.
    pub fn new<CH, P>(channel: CH, pin: P) -> Self
    where
        CH: InputChannel + 'lt,
        P: InputDriver + 'lt,
    {
        pin.set_input_connection(channel.input_signal());
        Self {
            mask: channel.mask(),
            _marker: PhantomData,
            #[cfg(all(debug_assertions, multi_core))]
            core: Cpu::current(),
        }
    }

    /// Read the current state of the GPIO pins.
    #[inline(always)]
    pub fn level(&mut self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        let bits = ll::read_in();
        Level::from(bits & self.mask != 0)
    }
}

impl embedded_hal::digital::ErrorType for DedicatedGpioInput<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for DedicatedGpioInput<'_> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::High)
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::Low)
    }
}

#[doc_replace]
/// A dedicated GPIO output driver.
///
/// Due to how the hardware works, [`DedicatedGpioOutput`] can drive any number of GPIO pins. To
/// create a driver, you can use the [`DedicatedGpioOutput::new`] method, then
/// [`DedicatedGpioOutput::with_pin`] to add output drivers.
#[cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Note that the driver must only be used on the core that has created it. Do not send the driver to
another core, either directly, or indirectly via a thread that is not pinned to a core.
</section>
"#
)]
/// ## Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// #
/// use esp_hal::gpio::{
///     Level,
///     Output,
///     OutputConfig,
///     dedicated::{DedicatedGpio, DedicatedGpioOutput},
/// };
///
/// // Create a dedicated GPIO driver:
/// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
/// let dedicated_output = DedicatedGpioOutput::new(channels.channel0);
///
/// // Add any number of output pin drivers:
/// let output = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
/// let mut dedicated_output = dedicated_output.with_pin(output);
///
/// // Now you can use the pin.
/// dedicated_output.set_level(Level::High);
/// #
/// # {after_snippet}
/// ```
pub struct DedicatedGpioOutput<'lt> {
    _marker: PhantomData<&'lt mut ()>,
    mask: u32,
    signal: OutputSignal,
    contained_gpios: [u32; GpioBank::COUNT],
    #[cfg(all(debug_assertions, multi_core))]
    core: Cpu,
}

impl<'lt> DedicatedGpioOutput<'lt> {
    /// Creates a new dedicated GPIO output driver.
    ///
    /// This function returns an empty driver. You will need to add output drivers to it using the
    /// [`Self::with_pin`] method.
    pub fn new<CH>(channel: CH) -> Self
    where
        CH: OutputChannel + 'lt,
    {
        ll::set_output_enabled(channel.mask(), true);
        Self {
            mask: channel.mask(),
            signal: channel.output_signal(),
            contained_gpios: [0; GpioBank::COUNT],
            _marker: PhantomData,
            #[cfg(all(debug_assertions, multi_core))]
            core: Cpu::current(),
        }
    }

    /// Adds a new output driver to the GPIO pins.
    ///
    /// A dedicated GPIO output driver can control any number of GPIO pins. The pins will be
    /// released when the driver is dropped. This function does not change the state of the newly
    /// added GPIO pin.
    pub fn with_pin(mut self, pin: impl OutputDriver + 'lt) -> Self {
        pin.set_output_connection(self.signal);

        let bank = pin.pin() / 32;
        let pin_in_bank = pin.pin() % 32;
        self.contained_gpios[bank as usize] |= 1 << pin_in_bank;

        self
    }

    /// Changes the current state of the GPIO pins.
    #[inline(always)]
    pub fn set_level(&mut self, level: Level) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        if level == Level::High {
            ll::write(self.mask, self.mask);
        } else {
            ll::write(self.mask, 0);
        }
    }

    /// Returns the current output state of the GPIO pins.
    ///
    /// Returns [`Level::High`] if any of the GPIO pins are set high, otherwise [`Level::Low`].
    #[inline(always)]
    pub fn output_level(&mut self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        Level::from(ll::read_out() & self.mask != 0)
    }
}

impl Drop for DedicatedGpioOutput<'_> {
    fn drop(&mut self) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        // Set the contained GPIOs back to GPIO mode.
        for (bank, mut pins) in self.contained_gpios.into_iter().enumerate() {
            let bank_offset = bank as u8 * 32;
            while pins != 0 {
                let pin = pins.trailing_zeros() as u8;
                pins &= !(1 << pin);

                let gpio = unsafe { AnyPin::steal(bank_offset + pin) };
                gpio.disconnect_from_peripheral_output();
                gpio.connect_peripheral_to_output(OutputSignal::GPIO);
            }
        }
    }
}

impl embedded_hal::digital::ErrorType for DedicatedGpioOutput<'_> {
    type Error = Infallible;
}

impl embedded_hal::digital::OutputPin for DedicatedGpioOutput<'_> {
    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low);
        Ok(())
    }

    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High);
        Ok(())
    }
}

#[doc_replace]
/// A dedicated GPIO input and output driver.
#[cfg_attr(
    xtensa,
    doc = r#"

On ESP32-S2 and ESP32-S3, the GPIO's output is always enabled.
"#
)]
#[cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Note that the driver must only be used on the core that has created it. Do not send the driver to
another core, either directly, or indirectly via a thread that is not pinned to a core.
</section>
"#
)]
/// ## Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// #
/// use esp_hal::gpio::{
///     Flex,
///     Input,
///     Level,
///     dedicated::{DedicatedGpio, DedicatedGpioFlex},
/// };
///
/// // Create a pin driver:
/// let flex = Flex::new(peripherals.GPIO0);
///
/// // Create a dedicated GPIO driver:
/// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
/// let mut dedicated_io = DedicatedGpioFlex::new(channels.channel0, flex);
///
/// // Now you can use the pin:
/// let level = dedicated_io.level();
/// #
/// # {after_snippet}
/// ```
pub struct DedicatedGpioFlex<'lt> {
    _marker: PhantomData<&'lt mut ()>,
    mask: u32,
    pin: u8,
    #[cfg(all(debug_assertions, multi_core))]
    core: Cpu,
}

impl<'lt> DedicatedGpioFlex<'lt> {
    /// Creates a new dedicated GPIO input/output driver.
    pub fn new<CH, P>(channel: CH, pin: P) -> Self
    where
        CH: InputChannel + OutputChannel + 'lt,
        P: InputDriver + OutputDriver + 'lt,
    {
        pin.set_input_connection(channel.input_signal());
        pin.set_output_connection(channel.output_signal());
        Self {
            mask: <CH as OutputChannel>::mask(&channel),
            pin: <P as OutputDriver>::pin(&pin),
            _marker: PhantomData,
            #[cfg(all(debug_assertions, multi_core))]
            core: Cpu::current(),
        }
    }

    /// Enables or disables the output buffer of the GPIO pin.
    #[cfg(riscv)] // Xtensas always have the output enabled.
    pub fn set_output_enabled(&mut self, enabled: bool) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        ll::set_output_enabled(self.mask, enabled);
    }

    /// Change the current state of the GPIO pin.
    #[inline(always)]
    pub fn set_level(&mut self, level: Level) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        if level == Level::High {
            ll::write(self.mask, self.mask);
        } else {
            ll::write(self.mask, 0);
        }
    }

    /// Returns the current output state of the GPIO pin.
    #[inline(always)]
    pub fn output_level(&mut self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        Level::from(ll::read_out() & self.mask != 0)
    }

    /// Read the current state of the GPIO pins.
    #[inline(always)]
    pub fn level(&mut self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());
        let bits = ll::read_in();
        Level::from(bits & self.mask != 0)
    }
}

impl Drop for DedicatedGpioFlex<'_> {
    fn drop(&mut self) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(self.core, Cpu::current());

        let gpio = unsafe { AnyPin::steal(self.pin) };
        gpio.disconnect_from_peripheral_output();
        gpio.connect_peripheral_to_output(OutputSignal::GPIO);
    }
}

#[doc_replace]
/// Low-level function to write any number of dedicated output channels at once.
///
/// The `mask` selects which channels to write to. Channel positions are in
/// order: bit 0 marks channel 0, bit 1 marks channel 1, etc.
///
/// `value` is the value to write to the selected channels. A 0 bit sets the
/// channel's output to low, a 1 bit sets it to high.
///
/// # Example
///
/// ```rust,no_run
/// # {before_snippet}
/// #
/// use esp_hal::gpio::{
///     Level,
///     Output,
///     OutputConfig,
///     dedicated::{DedicatedGpio, DedicatedGpioOutput, write_ll},
/// };
///
/// // Create dedicated GPIO drivers:
/// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
/// let dedicated_output0 = DedicatedGpioOutput::new(channels.channel0);
/// let dedicated_output1 = DedicatedGpioOutput::new(channels.channel1);
/// let dedicated_output2 = DedicatedGpioOutput::new(channels.channel2);
///
/// // Configure output drivers:
/// let output0 = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
/// let output1 = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());
/// let output2 = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
/// let output3 = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
///
/// // Configure dedicated output drivers:
/// let dedicated_output0 = dedicated_output0.with_pin(output0).with_pin(output1);
/// let dedicated_output1 = dedicated_output1.with_pin(output2);
/// let dedicated_output2 = dedicated_output2.with_pin(output3);
///
/// // Write all pins at once:
/// const MASK: u32 = 0b00000111;
/// const VALUE: u32 = 0b00000101; // Set GPIOs 0, 1, and 3 to high, 2 to low.
/// write_ll(MASK, VALUE);
/// #
/// # {after_snippet}
/// ```
#[cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Only channels configured on the current CPU core can be used.
</section>
"#
)]
#[inline(always)]
pub fn write_ll(mask: u32, value: u32) {
    ll::write(mask, value);
}

/// Low-level function to read the current state of all dedicated input channels.
///
/// The returned value is a bitmask where each bit represents the state of a
/// channel. Bit 0 represents channel 0, bit 1 represents channel 1, etc.
#[inline(always)]
pub fn read_all_ll() -> u32 {
    ll::read_in()
}

// TODO: bundle drivers
// - Bundles should support short-lived operations as well as longer bundling
//   - A pin may belong to multiple bundles at the same time
//   - A bundle should be easy to create from drivers of different lifetimes
//     - A bundle should have an empty constructor and `.with_` functions to attach drivers
//       - `.with_` functions shall take drivers by shared? reference.
//         - data races are not a worry
//     - It should be possible to attach a Flex driver to input and output bundles
//   - A bundle needs to precompute its mask

#[cfg(esp32s2)]
mod ll {
    #[inline(always)]
    pub(super) fn set_output_enabled(_mask: u32, _en: bool) {
        // nothing to do
    }

    #[inline(always)]
    pub(super) fn read_in() -> u32 {
        let val;
        unsafe { core::arch::asm!("get_gpio_in {0}", out(reg) val) };
        val
    }

    #[inline(always)]
    pub(super) fn read_out() -> u32 {
        let val;
        unsafe { core::arch::asm!("rur.gpio_out {0}", out(reg) val) };
        val
    }

    #[inline(always)]
    pub(super) fn write(mask: u32, value: u32) {
        unsafe { core::arch::asm!("wr_mask_gpio_out {0}, {1}", in(reg) mask, in(reg) value) }
    }
}

#[cfg(esp32s3)]
mod ll {
    #[inline(always)]
    pub(super) fn set_output_enabled(_mask: u32, _en: bool) {
        // nothing to do
    }

    #[inline(always)]
    pub(super) fn read_in() -> u32 {
        let val;
        unsafe { core::arch::asm!("ee.get_gpio_in {0}", out(reg) val) };
        val
    }

    #[inline(always)]
    pub(super) fn read_out() -> u32 {
        let val;
        unsafe { core::arch::asm!("rur.gpio_out {0}", out(reg) val) };
        val
    }

    #[inline(always)]
    pub(super) fn write(mask: u32, value: u32) {
        unsafe { core::arch::asm!("ee.wr_mask_gpio_out {0}, {1}", in(reg) mask, in(reg) value) }
    }
}

#[cfg(riscv)]
mod ll {
    // CSR_GPIO_OEN_USER   0x803
    // CSR_GPIO_IN_USER    0x804
    // CSR_GPIO_OUT_USER   0x805

    #[inline(always)]
    pub(super) fn set_output_enabled(mask: u32, en: bool) {
        riscv::read_csr!(0x803);
        riscv::write_csr!(0x803);

        unsafe {
            let bits = _read();
            if en {
                _write(bits | mask as usize)
            } else {
                _write(bits & !mask as usize)
            }
        }
    }

    #[inline(always)]
    pub(super) fn read_in() -> u32 {
        riscv::read_csr!(0x804);
        unsafe { _read() as u32 }
    }

    #[inline(always)]
    pub(super) fn read_out() -> u32 {
        riscv::read_csr!(0x805);
        unsafe { _read() as u32 }
    }

    #[inline(always)]
    pub(super) fn write(mask: u32, value: u32) {
        riscv::set!(0x805);
        riscv::clear!(0x805);
        unsafe {
            _set((mask & value) as usize);
            _clear((mask & !value) as usize);
        }
    }
}
