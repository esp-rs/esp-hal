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
//!
//! ## Bundles
//!
//! If you need to read or update multiple channels together, you can use the bundle helpers:
//! - [`DedicatedGpioInputBundle`]
//! - [`DedicatedGpioOutputBundle`]
//! - [`DedicatedGpioFlexBundle`]
//!
//! Bundles are lightweight objects that precompute a channel mask from one or more drivers,
//! allowing multi-channel operations with a single low-level read/write.
//!
//! ## Low-level functions
//!
//! This module also exposes low-level helpers for direct, channel-bitmask-based access:
//! - [`write_ll`]: write output levels for a selected set of channels in one operation
//! - [`read_all_ll`]: read the current input levels of all channels
#![cfg_attr(
    not(esp32s3),
    doc = r#"- [`output_levels_ll`]: read the current output levels of all channels"#
)]
#![cfg_attr(
    esp32s3,
    doc = r#"- `output_levels_ll`: read the current output levels of all channels (not available on ESP32-S3 due to an LLVM bug, see <https://github.com/espressif/llvm-project/issues/120>)"#
)]
//! These functions operate purely on channel bitmasks (bit 0 -> channel 0, bit 1 -> channel 1, ...)
//! and do not track pin configuration. Prefer the higher-level drivers and bundles unless you
//! specifically need the lowest overhead.

#![cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Dedicated GPIOs are tied to CPU cores and a driver must only be used on the core that has created it.
Do not send the drivers to another core, either directly, or indirectly via a thread that is not pinned to a core.
</section>
"#
)]
//! ## Examples
//! ### sharing drivers across multiple bundles
//!
//! The same driver can be borrowed by multiple bundles at the same time**.
//!
//! In this example, we build two *input* bundles:
//! - `bundle_a` reads channels 0, 1, 2
//! - `bundle_b` reads channels 0, 2, 4
//!
//! Both bundles share the same `in0` and `in2` drivers, and we use [`DedicatedGpioInputBundle::all_high`]
//! to check if all channels in each bundle are currently high.
//!
//! ```rust, no_run
//!
//! # {before_snippet}
//! use esp_hal::gpio::{
//!     Input,
//!     InputConfig,
//!     dedicated::{DedicatedGpio, DedicatedGpioInput, DedicatedGpioInputBundle},
//! };
//!
//! // Create channels.
//! let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
//!
//! // Configure GPIO inputs
//! let p0 = Input::new(peripherals.GPIO0, InputConfig::default());
//! let p1 = Input::new(peripherals.GPIO1, InputConfig::default());
//! let p2 = Input::new(peripherals.GPIO2, InputConfig::default());
//! let p4 = Input::new(peripherals.GPIO4, InputConfig::default());
//!
//! let in0 = DedicatedGpioInput::new(channels.channel0.input, p0);
//! let in1 = DedicatedGpioInput::new(channels.channel1.input, p1);
//! let in2 = DedicatedGpioInput::new(channels.channel2.input, p2);
//! let in4 = DedicatedGpioInput::new(channels.channel4.input, p4);
//!
//! // Bundle A reads channels 0, 1, 2.
//! let mut bundle_a = DedicatedGpioInputBundle::new();
//! bundle_a
//!     .enable_input(&in0)
//!     .enable_input(&in1)
//!     .enable_input(&in2);
//!
//! // Bundle B reads channels 0, 2, 4.
//! // Note: `in0` and `in2` are *shared* with bundle A.
//! let mut bundle_b = DedicatedGpioInputBundle::new();
//! bundle_b
//!     .enable_input(&in0)
//!     .enable_input(&in2)
//!     .enable_input(&in4);
//!
//! // Check whether all channels in each bundle are currently high.
//! let a_all_high = bundle_a.all_high(); // true if ch0, ch1, ch2 are all high
//! let b_all_high = bundle_b.all_high(); // true if ch0, ch2, ch4 are all high
//! # {after_snippet}
//! ```
//! 
//! This pattern is useful when different subsystems want different views of the same set of
//! input channels without duplicating driver setup.
//! 
//! 

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
#[doc = ""]
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
    pub fn level(&self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

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
    esp32s3,
    doc = r#"

<section class="warning">
The method <code>output_level</code> is currently not available on ESP32-S3 due to
an LLVM bug. See <a href=https://github.com/espressif/llvm-project/issues/120>https://github.com/espressif/llvm-project/issues/120</a> for details.
</section>
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
#[doc = ""]
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
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        if level == Level::High {
            ll::write(self.mask, self.mask);
        } else {
            ll::write(self.mask, 0);
        }
    }

    /// Returns the current output state of the GPIO pins.
    ///
    /// Returns [`Level::High`] if any of the GPIO pins are set high, otherwise [`Level::Low`].
    #[cfg(not(esp32s3))]
    #[inline(always)]
    pub fn output_level(&mut self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        Level::from(ll::read_out() & self.mask != 0)
    }
}

impl Drop for DedicatedGpioOutput<'_> {
    fn drop(&mut self) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

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
#[doc = ""]
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
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        ll::set_output_enabled(self.mask, enabled);
    }

    /// Change the current state of the GPIO pin.
    #[inline(always)]
    pub fn set_level(&mut self, level: Level) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        if level == Level::High {
            ll::write(self.mask, self.mask);
        } else {
            ll::write(self.mask, 0);
        }
    }

    /// Returns the current output state of the GPIO pin.
    #[cfg(not(esp32s3))]
    #[inline(always)]
    pub fn output_level(&mut self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        Level::from(ll::read_out() & self.mask != 0)
    }

    /// Read the current state of the GPIO pins.
    #[inline(always)]
    pub fn level(&self) -> Level {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        let bits = ll::read_in();
        Level::from(bits & self.mask != 0)
    }
}

impl Drop for DedicatedGpioFlex<'_> {
    fn drop(&mut self) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

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

/// Low-level function to read the current output levels of all dedicated GPIO channels.
///
/// The returned value is a bitmask where each bit represents the output level of a channel:
/// bit 0 -> channel 0, bit 1 -> channel 1, etc. A bit value of 1 means the channel output is high.
#[cfg(not(esp32s3))]
#[inline(always)]
pub fn output_levels_ll() -> u32 {
    ll::read_out()
}

#[doc_replace]
/// A bundle of dedicated GPIO output drivers.
///
/// An output bundle precomputes a channel mask from one or more
/// [`DedicatedGpioOutput`] drivers. This lets you update multiple dedicated output
/// channels with a single low-level write.
///
/// Attaching a driver does **not** change any output state. Dropping the output
/// drivers still controls when the underlying GPIO pins are released back to
/// normal GPIO mode.
///
/// ## Relationship to pins
///
/// Dedicated output channels can be connected to multiple GPIO pins via
/// [`DedicatedGpioOutput::with_pin`]. The bundle operates on *channels*, not
/// individual pins: writing channel bits affects every pin connected to that
/// channel.
#[cfg_attr(
    esp32s3,
    doc = r#"

<section class="warning">
The method <code>output_levels</code> is currently not available on ESP32-S3 due to
an LLVM bug. See <a href=https://github.com/espressif/llvm-project/issues/120>https://github.com/espressif/llvm-project/issues/120</a> for details.
</section>
"#
)]
#[cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Dedicated GPIO channels are tied to CPU cores. All output drivers attached to a bundle must be
configured on the same core, and the bundle must only be used on the core that created it.
</section>
"#
)]
#[doc = ""]
/// ## Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// #
/// use esp_hal::gpio::{
///     Level,
///     Output,
///     OutputConfig,
///     dedicated::{DedicatedGpio, DedicatedGpioOutput, DedicatedGpioOutputBundle},
/// };
///
/// // Create channels:
/// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
///
/// // Create output drivers for three channels:
/// let out0 = DedicatedGpioOutput::new(channels.channel0);
/// let out1 = DedicatedGpioOutput::new(channels.channel1);
/// let out2 = DedicatedGpioOutput::new(channels.channel2);
///
/// // Attach GPIO pins to the dedicated outputs (any number of pins per channel):
/// let p0 = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());
/// let p1 = Output::new(peripherals.GPIO1, Level::Low, OutputConfig::default());
/// let p2 = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
///
/// let out0 = out0.with_pin(p0);
/// let out1 = out1.with_pin(p1);
/// let out2 = out2.with_pin(p2);
///
/// // Build a bundle that controls channels 0..=2:
/// let mut bundle = DedicatedGpioOutputBundle::new();
/// bundle
///     .enable_output(&out0)
///     .enable_output(&out1)
///     .enable_output(&out2);
///
/// // Set channel 0 and 2 high, keep channel 1 unchanged:
/// bundle.set_high(0b101);
///
/// // Now drive all channels in the bundle at once:
/// // - ch0 = 0
/// // - ch1 = 1
/// // - ch2 = 0
/// bundle.write_bits(0b010);
/// #
/// # {after_snippet}
/// ```
pub struct DedicatedGpioOutputBundle<'lt> {
    _marker: PhantomData<&'lt ()>,
    mask: u32,
    #[cfg(all(debug_assertions, multi_core))]
    core: Cpu,
}

impl<'lt> DedicatedGpioOutputBundle<'lt> {
    /// Creates a new, empty dedicated GPIO output bundle.
    ///
    /// A bundle is a *logical* grouping of one or more [`DedicatedGpioOutput`] drivers.
    /// Internally, it stores a precomputed channel mask (see [`Self::mask`]) which allows
    /// writing multiple dedicated GPIO channels efficiently.
    ///
    /// The returned bundle initially contains no channels. Add outputs using
    /// [`Self::enable_output`].
    ///
    /// ## Notes
    ///
    /// - Creating a bundle does **not** configure any hardware by itself.
    pub fn new() -> Self {
        Self {
            _marker: PhantomData,
            mask: 0,
            #[cfg(all(debug_assertions, multi_core))]
            core: Cpu::current(),
        }
    }

    #[doc_replace]
    /// Returns the channel mask of this bundle.
    ///
    /// Each bit corresponds to one dedicated GPIO channel:
    /// - bit 0 -> channel 0
    /// - bit 1 -> channel 1
    /// - ...
    ///
    /// A bit is set to 1 if that channel is currently included in the bundle.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    ///
    /// use esp_hal::gpio::{
    ///     Level,
    ///     Output,
    ///     OutputConfig,
    ///     dedicated::{DedicatedGpio, DedicatedGpioOutput, DedicatedGpioOutputBundle},
    /// };
    ///
    /// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
    /// let out0 = DedicatedGpioOutput::new(channels.channel0)
    ///      .with_pin(Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default()));
    /// let out2 = DedicatedGpioOutput::new(channels.channel2)
    ///      .with_pin(Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default()));
    /// let mut bundle = DedicatedGpioOutputBundle::new();
    /// bundle.enable_output(&out0).enable_output(&out2);
    ///
    /// let mask = bundle.mask(); // bit 0 -> ch0, bit 2 -> ch2, ...
    /// // For this bundle: mask == 0b0000_0101 (channels 0 and 2).
    ///
    /// # {after_snippet}
    /// ```
    #[inline(always)]
    pub fn mask(&self) -> u32 {
        self.mask
    }

    /// Attaches (enables) a dedicated output driver in this bundle. After enabling, you will 
    /// be able to control the output channels of `out` via this bundle.
    ///
    /// This method logically borrows the provided [`DedicatedGpioOutput`] for the lifetime `'lt`.
    /// Multiple bundles may borrow the same output driver at the same time. Check examples in the module-level 
    /// documentation for more.
    ///
    /// ## Notes
    ///
    /// - This function does not change any GPIO output state.
    #[cfg_attr(
        multi_core,
        doc = r#"
<section class="warning">
All dedicated GPIO drivers in a bundle must be configured on the same core as the bundle itself
</section>
        "#
    )]
    pub fn enable_output<'d>(&mut self, out: &'lt DedicatedGpioOutput<'d>) -> &mut Self {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            out.core, self.core,
            "Trying to enable a dedicated GPIO driver configured on a different core from the bundle."
        );
        self.mask |= out.mask;
        self
    }
    
    /// Disables a dedicated output driver in this bundle.
    ///
    /// This updates the internal mask by clearing the channel bit(s) of `out`. After disabling,
    /// future *bundle* operations will no longer touch those channels.
    ///
    /// ## Notes
    ///
    /// - This does **not** affect `out` itself. It only changes which channels future *bundle*
    ///   operations will touch.
    /// - This does **not** end the lifetime-based borrow of `out`. Even after disabling, `out`
    ///   still cannot be moved or dropped while this bundle exists.
    /// - You can re-enable it later via [`Self::enable_output`].
    #[cfg_attr(
        multi_core,
        doc = r#"
<section class="warning">
You should only disable dedicated GPIO drivers that were configured on the same core as the bundle itself.
</section>
"#
    )]
    pub fn disable_output<'d>(&mut self, out: &'lt DedicatedGpioOutput<'d>) -> &mut Self {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            out.core, self.core, 
            "Trying to disable a dedicated GPIO driver configured on a different core from the bundle."
        );
        self.mask &= !out.mask;
        self
    }

    /// Sets selected channels **high**.
    ///
    /// For every bit set to 1 in `bits`, the corresponding dedicated output channel is driven
    /// high. Bits set to 0 are left unchanged.
    ///
    /// ## Example
    ///
    /// `bundle.set_high(0b1011_0001)` sets channels 0, 4, 5, and 7 high.
    ///
    /// <section class="warning">
    ///
    /// The caller must ensure that `bits` only contains channels included in this bundle,
    /// i.e. `bits & !self.mask() == 0`.
    ///
    /// For example, if the bundle mask is `0b0000_1011` (channels 0, 1, and 3), then `bits`
    /// must not set bit 2 (e.g. `0b0000_0100`), or you would modify channel 2 outside the bundle.
    ///
    /// When compiled with `debug-assertions`, this condition is checked at runtime.
    /// </section>
    #[inline(always)]
    pub fn set_high(&mut self, bits: u32) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        debug_assert!(
            (bits & !self.mask) == 0,
            "Trying to set bits outside of the bundle mask"
        );
        ll::write(bits, bits); // or ll::write(self.mask, bits);
    }

    /// Sets selected channels **low**.
    ///
    /// For every bit set to 1 in `bits`, the corresponding dedicated output channel is driven
    /// low. Bits set to 0 are left unchanged.
    ///
    /// ## Example
    ///
    /// `bundle.set_low(0b1011_0001)` sets channels 0, 4, 5, and 7 low.
    ///
    /// <section class="warning">
    ///
    /// The caller must ensure that `bits` only contains channels included in this bundle,
    /// i.e. `bits & !self.mask() == 0`.
    ///
    /// For example, if the bundle mask is `0b0000_1011` (channels 0, 1, and 3), then `bits`
    /// must not set bit 7 (e.g. `0b1000_0000`), or you would clear channel 7 outside the bundle.
    ///
    /// When compiled with `debug-assertions`, this condition is checked at runtime.
    /// </section>
    #[inline(always)]
    pub fn set_low(&mut self, bits: u32) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        debug_assert!(
            (bits & !self.mask) == 0,
            "Trying to clear bits outside of the bundle mask"
        );
        ll::write(bits, 0);
        // or ll::write(bits & self.mask, 0);
        // the latter is safer (preventing writing to channels outside of the bundle) but slower
    }

    /// Writes output levels for **all channels included by this bundle**.
    ///
    /// This method updates the output state of every channel whose bit is set in
    /// [`Self::mask`]. Channels not included in the bundle are not modified.
    ///
    /// `bits` provides the level for each channel in the bundle:
    /// - bit = 0 -> low
    /// - bit = 1 -> high
    ///
    /// ## Example
    ///
    /// If `self.mask()` is `0b1111_0000` (bundle contains channels 4..=7), then
    /// `bundle.write_bits(0b0001_0000)` sets channel 4 high and channels 5..=7 low,
    /// while leaving channels 0..=3 unchanged.
    ///
    /// <section class="warning">
    ///
    /// This function only writes channels selected by [`Self::mask`]. It will **not**
    /// change channels outside the mask, even if `bits` contains 1s there.
    ///
    /// Make sure this "masked write" behavior matches what you intend.
    ///
    /// </section>
    #[inline(always)]
    pub fn write_bits(&mut self, bits: u32) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        ll::write(self.mask, bits);
    }

    /// Returns the current output levels of the channels included by this bundle.
    ///
    /// For channels outside the bundle mask, the corresponding bits are always 0.
    ///
    /// ## Example
    ///
    /// If the bundle mask is `0b0000_1011` (channels 0, 1, and 3), then
    /// `output_levels()` will only contain bits 0, 1, and 3, regardless of the output
    /// state of other channels.
    #[cfg(not(esp32s3))]
    #[inline(always)]
    pub fn output_levels(&self) -> u32 {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );
        ll::read_out() & self.mask
    }
}

#[doc_replace]
/// A bundle of dedicated GPIO input drivers.
///
/// An input bundle precomputes a channel mask from one or more [`DedicatedGpioInput`]
/// drivers. This lets you read multiple dedicated input channels with a single
/// low-level read (see [`DedicatedGpioInputBundle::levels`]).
///
/// Attaching a driver does **not** change any pin state. The bundle only stores a
/// channel mask; it does not own pins or remember which GPIOs were connected.
///
/// ## Relationship to pins
///
/// Dedicated input channels are fed by GPIO pins that were connected when creating
/// [`DedicatedGpioInput`]. The bundle operates on *channels*, not individual pins:
/// each bit in the returned mask corresponds to a dedicated input channel.
///
/// If you later reconfigure a GPIO pin (e.g. connect it to a different peripheral
/// input), that changes what the channel reads. The bundle does not prevent such
/// reconfiguration.
#[cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Dedicated GPIO channels are tied to CPU cores. All input drivers attached to a bundle must be
configured on the same core, and the bundle must only be used on the core that created it.
</section>
"#
)]
#[doc = ""]
/// ## Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// #
/// use esp_hal::gpio::{
///     Input,
///     InputConfig,
///     dedicated::{DedicatedGpio, DedicatedGpioInput, DedicatedGpioInputBundle},
/// };
///
/// // Create channels:
/// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
///
/// // Create input drivers for two channels:
/// let p0 = Input::new(peripherals.GPIO0, InputConfig::default());
/// let p1 = Input::new(peripherals.GPIO1, InputConfig::default());
/// let in0 = DedicatedGpioInput::new(channels.channel0, p0);
/// let in1 = DedicatedGpioInput::new(channels.channel1, p1);
///
/// // Build a bundle that reads channels 0 and 1:
/// let mut bundle = DedicatedGpioInputBundle::new();
/// bundle.enable_input(&in0).enable_input(&in1);
///
/// // Read both channels at once:
/// let bits = bundle.levels();
/// // bit 0 -> channel 0, bit 1 -> channel 1, ...
/// #
/// # {after_snippet}
/// ```
pub struct DedicatedGpioInputBundle<'lt> {
    _marker: PhantomData<&'lt ()>,
    mask: u32,
    #[cfg(all(debug_assertions, multi_core))]
    core: Cpu,
}

impl<'lt> DedicatedGpioInputBundle<'lt> {
    /// Creates a new, empty dedicated GPIO input bundle.
    ///
    /// A bundle is a *logical* grouping of one or more [`DedicatedGpioInput`] drivers.
    /// Internally, it stores a precomputed channel mask which allows reading multiple
    /// dedicated input channels efficiently.
    ///
    /// The returned bundle initially contains no channels. Add inputs using
    /// [`Self::enable_input`].
    ///
    /// ## Notes
    ///
    /// - Creating a bundle does **not** configure any hardware by itself.
    pub fn new() -> Self {
        Self {
            _marker: PhantomData,
            mask: 0,
            #[cfg(all(debug_assertions, multi_core))]
            core: Cpu::current(),
        }
    }

    #[doc_replace]
    /// Returns the channel mask of this bundle.
    ///
    /// Each bit corresponds to one dedicated GPIO channel:
    /// - bit 0 -> channel 0
    /// - bit 1 -> channel 1
    /// - ...
    ///
    /// A bit is set to 1 if that channel is currently included in the bundle.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    ///
    /// use esp_hal::gpio::{
    ///     Input,
    ///     InputConfig,
    ///     dedicated::{DedicatedGpio, DedicatedGpioInput, DedicatedGpioInputBundle},
    /// };
    ///
    /// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
    /// let in0 = DedicatedGpioInput::new(
    ///     channels.channel0,
    ///     Input::new(peripherals.GPIO0, InputConfig::default()),
    /// );
    /// let in2 = DedicatedGpioInput::new(
    ///     channels.channel2,
    ///     Input::new(peripherals.GPIO2, InputConfig::default()),
    /// );
    ///
    /// let mut bundle = DedicatedGpioInputBundle::new();
    /// bundle.enable_input(&in0).enable_input(&in2);
    ///
    /// let mask = bundle.mask(); // bit 0 -> ch0, bit 2 -> ch2, ...
    /// // For this bundle: mask == 0b0000_0101 (channels 0 and 2).
    ///
    /// # {after_snippet}
    /// ```
    #[inline(always)]
    pub fn mask(&self) -> u32 {
        self.mask
    }

    /// Attaches (enables) an already-configured dedicated input driver to this bundle. After enabling, you
    /// will be able to read the input channels of `inp` via this bundle.
    ///
    /// This method logically borrows the provided [`DedicatedGpioInput`] for the lifetime `'lt`.
    /// Multiple bundles may borrow the same input driver at the same time. Check examples in the module-level 
    /// documentation for more.
    ///
    /// ## Notes
    ///
    /// - This function does not change any input state.
    #[cfg_attr(
        multi_core,
        doc = r#"
<section class="warning">
All dedicated GPIO drivers in a bundle must be configured on the same core as the bundle itself.
</section>
"#
    )]
    pub fn enable_input<'d>(&mut self, inp: &'lt DedicatedGpioInput<'d>) -> &mut Self {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            inp.core, self.core,
            "Trying to enable a dedicated GPIO driver configured on a different core from the bundle."
        );
        self.mask |= inp.mask;
        self
    }
    
    /// Disables a dedicated input driver in this bundle.
    ///
    /// This updates the internal mask by clearing the channel bit(s) of `inp`. After disabling,
    /// future *bundle* operations will no longer touch those channels.
    ///
    /// ## Notes
    ///
    /// - This does **not** affect `inp` itself. It only changes which channels future *bundle*
    ///   operations will touch.
    /// - This does **not** end the lifetime-based borrow of `inp`. Even after disabling, `inp`
    ///   still cannot be moved or dropped while this bundle exists.
    /// - You can re-enable it later via [`Self::enable_input`].
    #[cfg_attr(
        multi_core,
        doc = r#"
<section class="warning">
You should only disable dedicated GPIO drivers that were configured on the same core as the bundle itself.
</section>
"#
    )]
    pub fn disable_input<'d>(&mut self, inp: &'lt DedicatedGpioInput<'d>) -> &mut Self {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            inp.core, self.core, 
            "Trying to disable a dedicated GPIO driver configured on a different core from the bundle."
        );
        self.mask &= !inp.mask;
        self
    }

    /// Reads the current state of the channels included by this bundle.
    ///
    /// For channels outside the bundle mask, the corresponding bits are always 0.
    ///
    /// ## Example
    ///
    /// If the bundle mask is `0b0000_1011` (channels 0, 1, and 3), then
    /// `levels()` will only contain bits 0, 1, and 3, regardless of the state
    /// of other channels.
    #[inline(always)]
    pub fn levels(&self) -> u32 {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        ll::read_in() & self.mask
    }

    /// Returns `true` if all channels in this bundle are currently high.
    #[inline(always)]
    pub fn all_high(&self) -> bool {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        (ll::read_in() & self.mask) == self.mask
    }

    /// Returns `true` if all channels in this bundle are currently low.
    #[inline(always)]
    pub fn all_low(&self) -> bool {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        (ll::read_in() & self.mask) == 0
    }
}

#[doc_replace]
/// A bundle of dedicated GPIO flex drivers (input + output).
///
/// A flex bundle precomputes a channel mask from one or more [`DedicatedGpioFlex`]
/// drivers. This lets you:
/// - update multiple dedicated output channels with a single low-level write, and
/// - read multiple dedicated input channels with a single low-level read.
///
/// Attaching a driver does **not** change any pin state. The bundle only stores a
/// channel mask; it does not own pins or remember which GPIOs were connected.
///
/// ## Relationship to pins
///
/// A [`DedicatedGpioFlex`] connects one dedicated channel to one GPIO pin for both
/// input and output. The bundle operates on *channels*, not pins: writing channel bits
/// affects the pins currently connected to those channels.
#[cfg_attr(
    esp32s3,
    doc = r#"

<section class="warning">
The method <code>output_levels</code> is currently not available on ESP32-S3 due to
an LLVM bug. See <a href=https://github.com/espressif/llvm-project/issues/120>https://github.com/espressif/llvm-project/issues/120</a> for details.
</section>
"#
)]
#[cfg_attr(
    multi_core,
    doc = r#"

<section class="warning">
Dedicated GPIO channels are tied to CPU cores. All flex drivers attached to a bundle must be
configured on the current and same core, and the bundle must only be used on the core that created it.

For example, do not create the bundle on core 0 and then call <code>write_bits</code> from a task
running on core 1.
</section>
"#
)]
#[doc = ""]
/// ## Examples
///
/// ```rust, no_run
/// # {before_snippet}
/// #
/// use esp_hal::gpio::{
///     Flex,
///     Level,
///     dedicated::{DedicatedGpio, DedicatedGpioFlex, DedicatedGpioFlexBundle},
/// };
///
/// // Create channels:
/// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
///
/// // Create flex drivers for two channels:
/// let p0 = Flex::new(peripherals.GPIO0);
/// let p1 = Flex::new(peripherals.GPIO1);
/// let f0 = DedicatedGpioFlex::new(channels.channel0, p0);
/// let f1 = DedicatedGpioFlex::new(channels.channel1, p1);
///
/// // Build a bundle that controls channels 0 and 1:
/// let mut bundle = DedicatedGpioFlexBundle::new();
/// bundle.enable_flex(&f0).enable_flex(&f1);
///
/// // Set channel 0 high:
/// bundle.set_high(0b0000_0001);
///
/// // Drive both channels at once:
/// bundle.write_bits(0b0000_0010); // ch0 low, ch1 high
///
/// // Read both channels at once:
/// let in_bits = bundle.levels();
/// #
/// # {after_snippet}
/// ```
/// ...
pub struct DedicatedGpioFlexBundle<'lt> {
    _marker: PhantomData<&'lt ()>,
    mask: u32,
    #[cfg(all(debug_assertions, multi_core))]
    core: Cpu,
}

impl<'lt> DedicatedGpioFlexBundle<'lt> {
    /// Creates a new, empty dedicated GPIO flex bundle.
    ///
    /// A bundle is a *logical* grouping of one or more [`DedicatedGpioFlex`] drivers.
    /// Internally, it stores a precomputed channel mask (see [`Self::mask`]) which allows
    /// writing multiple dedicated GPIO channels efficiently.
    ///
    /// The returned bundle initially contains no channels. Add flex drivers using
    /// [`Self::enable_flex`].
    ///
    /// ## Notes
    ///
    /// - Creating a bundle does **not** configure any hardware by itself.
    pub fn new() -> Self {
        Self {
            _marker: PhantomData,
            mask: 0,
            #[cfg(all(debug_assertions, multi_core))]
            core: Cpu::current(),
        }
    }

    /// Returns the channel mask of this bundle.
    ///
    /// Each bit corresponds to one dedicated GPIO channel:
    /// - bit 0 -> channel 0
    /// - bit 1 -> channel 1
    /// - ...
    ///
    /// A bit is set to 1 if that channel is currently included in the bundle.
    ///
    /// ## Example
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    ///
    /// use esp_hal::gpio::{
    ///     Flex,
    ///     dedicated::{DedicatedGpio, DedicatedGpioFlex, DedicatedGpioFlexBundle},
    /// };
    ///
    /// let channels = DedicatedGpio::new(peripherals.GPIO_DEDICATED);
    /// let f0 = DedicatedGpioFlex::new(channels.channel0, Flex::new(peripherals.GPIO0));
    /// let f2 = DedicatedGpioFlex::new(channels.channel2, Flex::new(peripherals.GPIO2));
    ///
    /// let mut bundle = DedicatedGpioFlexBundle::new();
    /// bundle.enable_flex(&f0).enable_flex(&f2);
    ///
    /// let mask = bundle.mask(); // bit 0 -> ch0, bit 2 -> ch2, ...
    /// // For this bundle: mask == 0b0000_0101 (channels 0 and 2).
    ///
    /// # {after_snippet}
    /// ```

    #[inline(always)]
    pub fn mask(&self) -> u32 {
        self.mask
    }

    /// Disables a dedicated flex driver in this bundle.
    ///
    /// This updates the internal mask by clearing the channel bit(s) of `flex`. After disabling,
    /// future *bundle* operations will no longer touch those channels.
    ///
    /// ## Notes
    ///
    /// - This does **not** affect `flex` itself. It only changes which channels future *bundle*
    ///   operations will touch.
    /// - This does **not** end the lifetime-based borrow of `flex`. Even after disabling, `flex`
    ///   still cannot be moved or dropped while this bundle exists.
    /// - You can re-enable it later via [`Self::enable_flex`].
    #[cfg_attr(
        multi_core,
        doc = r#"
<section class="warning">
You should only disable dedicated GPIO drivers that were configured on the same core as the bundle itself.
</section>
"#
    )]
    pub fn disable_flex<'d>(&mut self, flex: &'lt DedicatedGpioFlex<'d>) -> &mut Self {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            flex.core, self.core,
            "Trying to disable a dedicated GPIO driver configured on a different core from the bundle."
        );
        self.mask &= !flex.mask;
        self
    }

    /// Attaches (enables) an already-configured dedicated flex driver to this bundle. After enabling, you
    /// will be able to control the input/output channels of `flex` via this bundle.
    ///
    /// This method logically borrows the provided [`DedicatedGpioFlex`] for the lifetime `'lt`.
    /// Multiple bundles may borrow the same flex driver at the same time. Check examples in the module-level 
    /// documentation for more.
    ///
    /// ## Notes
    ///
    /// - This function does not change any input/output state.
    #[cfg_attr(
        multi_core,
        doc = r#"
<section class="warning">
All dedicated GPIO drivers in a bundle must be configured on the same core as the bundle itself.
</section>
"#
    )]
    pub fn enable_flex<'d>(&mut self, flex: &'lt DedicatedGpioFlex<'d>) -> &mut Self {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            flex.core, self.core,
            "Trying to enable a dedicated GPIO driver configured on a different core from the bundle."
        );
        self.mask |= flex.mask;
        self
    }

    /// Sets selected channels **high**.
    ///
    /// For every bit set to 1 in `bits`, the corresponding channel is driven high.
    /// Bits set to 0 are left unchanged.
    ///
    /// ## Example
    ///
    /// If the bundle contains channels 0 and 2, then `bundle.set_high(0b0000_0101)` sets
    /// channel 0 and 2 high.
    ///
    /// <section class="warning">
    ///
    /// The caller must ensure that `bits` only contains channels included in this bundle,
    /// i.e. `bits & !self.mask() == 0`.
    ///
    /// For example, if the bundle mask is `0b0000_0011` (channels 0 and 1), then `bits` must not
    /// set bit 2 (e.g. `0b0000_0100`), or you would modify channel 2 outside the bundle.
    ///
    /// When compiled with `debug-assertions`, this condition is checked at runtime.
    /// </section>
    #[inline(always)]
    pub fn set_high(&mut self, bits: u32) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        debug_assert!(
            (bits & !self.mask) == 0,
            "Trying to set bits outside of the bundle mask"
        );
        ll::write(bits, bits); // or ll::write(self.mask, bits);
    }

    /// Sets selected channels **low**.
    ///
    /// For every bit set to 1 in `bits`, the corresponding channel is driven low.
    /// Bits set to 0 are left unchanged.
    ///
    /// ## Example
    ///
    /// If the bundle contains channels 0 and 2, then `bundle.set_low(0b0000_0101)` sets
    /// channel 0 and 2 low.
    ///
    /// <section class="warning">
    ///
    /// The caller must ensure that `bits` only contains channels included in this bundle,
    /// i.e. `bits & !self.mask() == 0`.
    ///
    /// For example, if the bundle mask is `0b0000_0011` (channels 0 and 1), then `bits` must not
    /// set bit 7 (e.g. `0b1000_0000`), or you would modify channel 7 outside the bundle.
    ///
    /// When compiled with `debug-assertions`, this condition is checked at runtime.
    /// </section>
    #[inline(always)]
    pub fn set_low(&mut self, bits: u32) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        debug_assert!(
            (bits & !self.mask) == 0,
            "Trying to clear bits outside of the bundle mask"
        );
        ll::write(bits, 0); // or ll::write(bits & self.mask, 0); the latter is safer but slower
    }

    /// Writes output levels for **all channels included by this bundle**.
    ///
    /// This method updates the output state of every channel whose bit is set in
    /// [`Self::mask`]. Channels not included in the bundle are not modified.
    ///
    /// `bits` provides the level for each channel in the bundle:
    /// - bit = 0 -> low
    /// - bit = 1 -> high
    ///
    /// ## Example
    ///
    /// If `self.mask()` is `0b1111_0000` (bundle contains channels 4..=7), then
    /// `bundle.write_bits(0b0001_0000)` sets channel 4 high and channels 5..=7 low,
    /// while leaving channels 0..=3 unchanged.
    ///
    /// <section class="warning">
    ///
    /// This function only writes channels selected by [`Self::mask`]. It will **not**
    /// change channels outside the mask, even if `bits` contains 1s there.
    ///
    /// Make sure this "masked write" behavior matches what you intend.
    ///
    /// </section>
    #[inline(always)]
    pub fn write_bits(&mut self, bits: u32) {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );
        ll::write(self.mask, bits);
    }

    /// Returns the current output levels of the channels included by this bundle.
    ///
    /// For channels outside the bundle mask, the corresponding bits are always 0.
    ///
    /// ## Example
    ///
    /// If the bundle mask is `0b0000_1011` (channels 0, 1, and 3), then
    /// `output_levels()` will only contain bits 0, 1, and 3, regardless of the output
    /// state of other channels.
    #[cfg(not(esp32s3))]
    #[inline(always)]
    pub fn output_levels(&self) -> u32 {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );
        ll::read_out() & self.mask
    }

    /// Reads the current state of the channels included by this bundle.
    ///
    /// For channels outside the bundle mask, the corresponding bits are always 0.
    ///
    /// ## Example
    ///
    /// If the bundle mask is `0b0000_1011` (channels 0, 1, and 3), then
    /// `levels()` will only contain bits 0, 1, and 3, regardless of the state
    /// of other channels.
    #[inline(always)]
    pub fn levels(&self) -> u32 {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        ll::read_in() & self.mask
    }

    /// Returns `true` if all channels in this bundle are currently high.
    #[inline(always)]
    pub fn all_high(&self) -> bool {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        (ll::read_in() & self.mask) == self.mask
    }
    
    /// Returns `true` if all channels in this bundle are currently low.
    #[inline(always)]
    pub fn all_low(&self) -> bool {
        #[cfg(all(debug_assertions, multi_core))]
        debug_assert_eq!(
            self.core,
            Cpu::current(),
            "Dedicated GPIO used on a different CPU core than it was created on"
        );

        (ll::read_in() & self.mask) == 0
    }
}

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

    #[cfg(not(esp32s3))]
    #[inline(always)]
    pub(super) fn read_out() -> u32 {
        // currently unavailable due to an LLVM bug, see https://github.com/espressif/llvm-project/issues/120
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
