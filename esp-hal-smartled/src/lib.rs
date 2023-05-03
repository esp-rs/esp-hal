//! This adapter allows for the use of an RMT output channel to easily interact
//! with RGB LEDs and use the convenience functions of the
//! [`smart-leds`](https://crates.io/crates/smart-leds) crate.
//!
//! _This is a simple implementation where every LED is adressed in an
//! individual RMT operation. This is working perfectly fine in blocking mode,
//! but in case this is used in combination with interrupts that might disturb
//! the sequential sending, an alternative implementation (addressing the LEDs
//! in a sequence in a single RMT send operation) might be required!_
//!
//! ## Example
//!
//! ```rust,ignore
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let pulse = PulseControl::new(
//!     peripherals.RMT,
//!     &mut system.peripheral_clock_control,
//!     ClockSource::APB,
//!     0,
//!     0,
//!     0,
//! )
//! .unwrap();
//!
//! let led = <smartLedAdapter!(1)>::new(pulse.channel0, io.pins.gpio0);
//! ```

#![no_std]
#![deny(missing_docs)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

use core::slice::IterMut;

#[cfg(any(feature = "esp32", feature = "esp32s2"))]
use esp_hal_common::pulse_control::ClockSource;
use esp_hal_common::{
    gpio::OutputPin,
    peripheral::Peripheral,
    pulse_control::{ConfiguredChannel, OutputChannel, PulseCode, RepeatMode, TransmissionError},
};
use fugit::NanosDuration;
use smart_leds_trait::{SmartLedsWrite, RGB8};

// Specifies what clock frequency we're using for the RMT peripheral (if
// properly configured)
//
// TODO: Factor in clock configuration, this needs to be revisited once #24 and
// #44 have been addressed.
#[cfg(feature = "esp32")]
const SOURCE_CLK_FREQ: u32 = 40_000_000;
#[cfg(feature = "esp32c3")]
const SOURCE_CLK_FREQ: u32 = 40_000_000;
#[cfg(feature = "esp32c6")]
const SOURCE_CLK_FREQ: u32 = 40_000_000;
#[cfg(feature = "esp32s2")]
const SOURCE_CLK_FREQ: u32 = 40_000_000;
#[cfg(feature = "esp32s3")]
const SOURCE_CLK_FREQ: u32 = 40_000_000;

const SK68XX_CODE_PERIOD: u32 = 1200;
const SK68XX_T0H_NS: u32 = 320;
const SK68XX_T0L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T0H_NS;
const SK68XX_T1H_NS: u32 = 640;
const SK68XX_T1L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T1H_NS;

const SK68XX_T0H_CYCLES: NanosDuration<u32> =
    NanosDuration::<u32>::from_ticks((SK68XX_T0H_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500);
const SK68XX_T0L_CYCLES: NanosDuration<u32> =
    NanosDuration::<u32>::from_ticks((SK68XX_T0L_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500);
const SK68XX_T1H_CYCLES: NanosDuration<u32> =
    NanosDuration::<u32>::from_ticks((SK68XX_T1H_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500);
const SK68XX_T1L_CYCLES: NanosDuration<u32> =
    NanosDuration::<u32>::from_ticks((SK68XX_T1L_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500);

/// All types of errors that can happen during the conversion and transmission
/// of LED commands
#[derive(Debug)]
pub enum LedAdapterError {
    /// Raised in the event that the provided data container is not large enough
    BufferSizeExceeded,
    /// Raised if something goes wrong in the transmission,
    TransmissionError(TransmissionError),
}

/// Macro to generate adapters with an arbitrary buffer size fitting for a
/// specific number of `$buffer_size` LEDs to be addressed.
///
/// Attempting to use more LEDs that the buffer is configured for will result in
/// an `LedAdapterError:BufferSizeExceeded` error.
#[macro_export]
macro_rules! smartLedAdapter {
    ( $buffer_size: literal ) => {
        // The size we're assigning here is calculated as following
        //  (
        //   Nr. of LEDs
        //   * channels (r,g,b -> 3)
        //   * pulses per channel 8)
        //  ) + 1 additional pulse for the end delimiter
        SmartLedsAdapter::<_, { $buffer_size * 24 + 1 }>
    };
}

/// Adapter taking an RMT channel and a specific pin and providing RGB LED
/// interaction functionality using the `smart-leds` crate
pub struct SmartLedsAdapter<CHANNEL, const BUFFER_SIZE: usize> {
    channel: CHANNEL,
    rmt_buffer: [u32; BUFFER_SIZE],
}

impl<'d, CHANNEL, const BUFFER_SIZE: usize> SmartLedsAdapter<CHANNEL, BUFFER_SIZE>
where
    CHANNEL: ConfiguredChannel,
{
    /// Create a new adapter object that drives the pin using the RMT channel.
    pub fn new<UnconfiguredChannel, O: OutputPin + 'd>(
        mut channel: UnconfiguredChannel,
        pin: impl Peripheral<P = O> + 'd,
    ) -> SmartLedsAdapter<CHANNEL, BUFFER_SIZE>
    where
        UnconfiguredChannel: OutputChannel<ConfiguredChannel<'d, O> = CHANNEL>,
    {
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        channel
            .set_idle_output_level(false)
            .set_carrier_modulation(false)
            .set_channel_divider(1)
            .set_idle_output(true);

        #[cfg(any(feature = "esp32", feature = "esp32s2"))]
        channel
            .set_idle_output_level(false)
            .set_carrier_modulation(false)
            .set_channel_divider(1)
            .set_idle_output(true)
            .set_clock_source(ClockSource::APB);

        let channel = channel.assign_pin(pin);

        Self {
            channel,
            rmt_buffer: [0; BUFFER_SIZE],
        }
    }

    fn convert_rgb_to_pulse(
        value: RGB8,
        mut_iter: &mut IterMut<u32>,
    ) -> Result<(), LedAdapterError> {
        SmartLedsAdapter::<CHANNEL, BUFFER_SIZE>::convert_rgb_channel_to_pulses(value.g, mut_iter)?;
        SmartLedsAdapter::<CHANNEL, BUFFER_SIZE>::convert_rgb_channel_to_pulses(value.r, mut_iter)?;
        SmartLedsAdapter::<CHANNEL, BUFFER_SIZE>::convert_rgb_channel_to_pulses(value.b, mut_iter)?;

        Ok(())
    }

    fn convert_rgb_channel_to_pulses(
        channel_value: u8,
        mut_iter: &mut IterMut<u32>,
    ) -> Result<(), LedAdapterError> {
        for position in [128, 64, 32, 16, 8, 4, 2, 1] {
            *mut_iter.next().ok_or(LedAdapterError::BufferSizeExceeded)? =
                match channel_value & position {
                    0 => PulseCode {
                        level1: true,
                        length1: SK68XX_T0H_CYCLES,
                        level2: false,
                        length2: SK68XX_T0L_CYCLES,
                    }
                    .into(),
                    _ => PulseCode {
                        level1: true,
                        length1: SK68XX_T1H_CYCLES,
                        level2: false,
                        length2: SK68XX_T1L_CYCLES,
                    }
                    .into(),
                }
        }

        Ok(())
    }
}

impl<CHANNEL, const BUFFER_SIZE: usize> SmartLedsWrite for SmartLedsAdapter<CHANNEL, BUFFER_SIZE>
where
    CHANNEL: ConfiguredChannel,
{
    type Error = LedAdapterError;
    type Color = RGB8;

    /// Convert all RGB8 items of the iterator to the RMT format and
    /// add them to internal buffer, then start a singular RMT operation
    /// based on that buffer.
    fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: Iterator<Item = I>,
        I: Into<Self::Color>,
    {
        // We always start from the beginning of the buffer
        let mut seq_iter = self.rmt_buffer.iter_mut();

        // Add all converted iterator items to the buffer.
        // This will result in an `BufferSizeExceeded` error in case
        // the iterator provides more elements than the buffer can take.
        for item in iterator {
            SmartLedsAdapter::<CHANNEL, BUFFER_SIZE>::convert_rgb_to_pulse(
                item.into(),
                &mut seq_iter,
            )?;
        }

        // Finally, add an end element.
        *seq_iter.next().ok_or(LedAdapterError::BufferSizeExceeded)? = 0;

        // Perform the actual RMT operation. We use the u32 values here right away.
        match self
            .channel
            .send_pulse_sequence_raw(RepeatMode::SingleShot, &self.rmt_buffer)
        {
            Ok(_) => Ok(()),
            Err(x) => Err(LedAdapterError::TransmissionError(x)),
        }
    }
}
