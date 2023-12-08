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
//! let rmt = Rmt::new(peripherals.RMT, 80u32.MHz(), &clocks).unwrap();
//!
//! let rmt_buffer = smartLedBuffer!(1);
//! let mut led = SmartLedsAdapter::new(rmt.channel0, io.pins.gpio2, rmt_buffer);
//! ```

#![no_std]
#![deny(missing_docs)]
#![doc(html_logo_url = "https://avatars.githubusercontent.com/u/46717278")]

use core::{fmt::Debug, slice::IterMut};

use esp_hal_common::{
    gpio::OutputPin,
    peripheral::Peripheral,
    rmt::{Error as RmtError, PulseCode, TxChannel, TxChannelConfig, TxChannelCreator},
};
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
#[cfg(feature = "esp32h2")]
const SOURCE_CLK_FREQ: u32 = 16_000_000;
#[cfg(feature = "esp32s2")]
const SOURCE_CLK_FREQ: u32 = 40_000_000;
#[cfg(feature = "esp32s3")]
const SOURCE_CLK_FREQ: u32 = 40_000_000;

const SK68XX_CODE_PERIOD: u32 = 1200;
const SK68XX_T0H_NS: u32 = 320;
const SK68XX_T0L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T0H_NS;
const SK68XX_T1H_NS: u32 = 640;
const SK68XX_T1L_NS: u32 = SK68XX_CODE_PERIOD - SK68XX_T1H_NS;

const SK68XX_T0H_CYCLES: u16 = ((SK68XX_T0H_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500) as u16;
const SK68XX_T0L_CYCLES: u16 = ((SK68XX_T0L_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500) as u16;
const SK68XX_T1H_CYCLES: u16 = ((SK68XX_T1H_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500) as u16;
const SK68XX_T1L_CYCLES: u16 = ((SK68XX_T1L_NS * (SOURCE_CLK_FREQ / 1_000_000)) / 500) as u16;

/// All types of errors that can happen during the conversion and transmission
/// of LED commands
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LedAdapterError {
    /// Raised in the event that the provided data container is not large enough
    BufferSizeExceeded,
    /// Raised if something goes wrong in the transmission,
    TransmissionError(RmtError),
}

/// Macro to allocate a buffer sized for a specific number of LEDs to be
/// addressed.
///
/// Attempting to use more LEDs that the buffer is configured for will result in
/// an `LedAdapterError:BufferSizeExceeded` error.
#[macro_export]
macro_rules! smartLedBuffer {
    ( $buffer_size: literal ) => {
        // The size we're assigning here is calculated as following
        //  (
        //   Nr. of LEDs
        //   * channels (r,g,b -> 3)
        //   * pulses per channel 8)
        //  ) + 1 additional pulse for the end delimiter
        [0u32; $buffer_size * 24 + 1]
    };
}

/// Adapter taking an RMT channel and a specific pin and providing RGB LED
/// interaction functionality using the `smart-leds` crate
pub struct SmartLedsAdapter<TX, const CHANNEL: u8, const BUFFER_SIZE: usize>
where
    TX: TxChannel<CHANNEL>,
{
    channel: Option<TX>,
    rmt_buffer: [u32; BUFFER_SIZE],
}

impl<'d, TX, const CHANNEL: u8, const BUFFER_SIZE: usize> SmartLedsAdapter<TX, CHANNEL, BUFFER_SIZE>
where
    TX: TxChannel<CHANNEL>,
{
    /// Create a new adapter object that drives the pin using the RMT channel.
    pub fn new<C, O>(
        channel: C,
        pin: impl Peripheral<P = O> + 'd,
        rmt_buffer: [u32; BUFFER_SIZE],
    ) -> SmartLedsAdapter<TX, CHANNEL, BUFFER_SIZE>
    where
        O: OutputPin + 'd,
        C: TxChannelCreator<'d, TX, O, CHANNEL>,
    {
        let config = TxChannelConfig {
            clk_divider: 1,
            idle_output_level: false,
            carrier_modulation: false,
            idle_output: true,

            ..TxChannelConfig::default()
        };

        let channel = channel.configure(pin, config).unwrap();

        Self {
            channel: Some(channel),
            rmt_buffer,
        }
    }

    fn convert_rgb_to_pulse(
        value: RGB8,
        mut_iter: &mut IterMut<u32>,
    ) -> Result<(), LedAdapterError> {
        Self::convert_rgb_channel_to_pulses(value.g, mut_iter)?;
        Self::convert_rgb_channel_to_pulses(value.r, mut_iter)?;
        Self::convert_rgb_channel_to_pulses(value.b, mut_iter)?;

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

impl<TX, const CHANNEL: u8, const BUFFER_SIZE: usize> SmartLedsWrite
    for SmartLedsAdapter<TX, CHANNEL, BUFFER_SIZE>
where
    TX: TxChannel<CHANNEL>,
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
            Self::convert_rgb_to_pulse(item.into(), &mut seq_iter)?;
        }

        // Finally, add an end element.
        *seq_iter.next().ok_or(LedAdapterError::BufferSizeExceeded)? = 0;

        // Perform the actual RMT operation. We use the u32 values here right away.
        let channel = self.channel.take().unwrap();
        match channel.transmit(&self.rmt_buffer).wait() {
            Ok(chan) => {
                self.channel = Some(chan);
                Ok(())
            }
            Err((e, chan)) => {
                self.channel = Some(chan);
                Err(LedAdapterError::TransmissionError(e))
            }
        }
    }
}
