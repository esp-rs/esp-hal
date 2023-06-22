//! # Remote Control Peripheral (RMT)
//!
//! ### Summary
//! The ESP32 variants include a remote control peripheral (RMT) that
//! is designed to handle infrared remote control signals. For that
//! purpose, it can convert bitstreams of data (from the RAM) into
//! pulse codes and even modulate those codes into a carrier wave.
//!
//! It can also convert received pulse codes (again, with carrier
//! wave support) into data bits.
//!
//! A secondary use case for this peripheral is to drive RGB(W) LEDs
//! that bear an internal IC and use a pulse code protocol.
//!
//! ### Channels
//! The RMT peripheral has the following channels available
//! on individual chips:
//!
//! * The **ESP32** has 8 channels, each of them can be either receiver or
//!   transmitter
//! * The **ESP32-C3** has 4 channels, `Channel0` and `Channel1` hardcoded for
//!   transmitting signals and `Channel2` and `Channel3` hardcoded for receiving
//!   signals.
//! * The **ESP32-C6** has 4 channels, `Channel0` and `Channel1` hardcoded for
//!   transmitting signals and `Channel2` and `Channel3` hardcoded for receiving
//!   signals.
//! * The **ESP32-H2** has 4 channels, `Channel0` and `Channel1` hardcoded for
//!   transmitting signals and `Channel2` and `Channel3` hardcoded for receiving
//!   signals.
//! * The **ESP32-S2** has 4 channels, each of them can be either receiver or
//!   transmitter.
//! * The **ESP32-S3** has 8 channels, `Channel0`-`Channel3` hardcdoded for
//!   transmitting signals and `Channel4`-`Channel7` hardcoded for receiving
//!   signals.
//!
//! ### Implementation State
//! * FIFO mode is not supported (there appear to be some issues with FIFO mode
//!   in some variants and for consistency all variants therefore we use
//!   NON-FIFO mode everywhere)
//! * Non-blocking mode is currently not supported!
//! * Input channels are currently not supported!
//!
//! ### Example (for ESP32-C3)
//! ```
//! let mut peripherals = peripherals::Peripherals::take();
//!
//! // Configure RMT peripheral globally
//! let pulse = PulseControl::new(
//!     peripherals.RMT,
//!     &mut peripherals.SYSTEM,
//!     ClockSource::APB,
//!     0, // Integer part of the RMT-wide clock divider
//!     0, // Numerator part of the RMT-wide clock divider
//!     0, // Denominator part of the RMT-wide clock divider
//! )
//! .unwrap();
//!
//! // Get reference to channel
//! let mut rmt_channel0 = pulse.channel0;
//!
//! // Set up channel
//! rmt_channel0
//!     .set_idle_output_level(false)
//!     .set_carrier_modulation(false)
//!     .set_channel_divider(1)
//!     .set_idle_output(true);
//!
//! // Assign GPIO pin where pulses should be sent to
//! let mut rmt_channel0 = rmt_channel0.assign_pin(io.pins.gpio8);
//!
//! // Create pulse sequence
//! let mut seq = [PulseCode {
//!     level1: true,
//!     length1: 10u32.nanos(),
//!     level2: false,
//!     length2: 90u32.nanos(),
//! }; 288];
//!
//! // Send sequence
//! rmt_channel0
//!     .send_pulse_sequence(RepeatMode::SingleShot, &seq)
//!     .unwrap();
//! ```

#![deny(missing_docs)]

use core::slice::Iter;

use fugit::NanosDurationU32;
pub use paste::paste;

#[cfg(any(esp32c6, esp32h2))]
use crate::peripherals::PCR;
use crate::{
    gpio::{OutputPin, OutputSignal},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::RMT,
    system::PeripheralClockControl,
};

/// Errors that can occur when the peripheral is configured
#[derive(Debug)]
pub enum SetupError {
    /// The global configuration for the RMT peripheral is invalid
    /// (e.g. the fractional parameters are outOfBound)
    InvalidGlobalConfig,
}

/// Errors that can occur during a transmission attempt
#[derive(Debug)]
pub enum TransmissionError {
    /// Generic Transmission Error
    Failure(bool, bool, bool, bool),
    /// The maximum number of transmissions (`=(2^10)-1`) was exceeded
    RepetitionOverflow,
    /// The `RepeatNtimes` and `Forever` modesl are only feasible if the
    /// sequence fits into the RAM in one go. If the sequence has > 48
    /// elements, the `RepeatNtimes` and `Forever` modes cannot be used.
    IncompatibleRepeatMode,
}

/// Specifies the mode with which pulses are sent out in transmitter channels
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RepeatMode {
    /// Send sequence once
    SingleShot,
    /// Send sequence N times (`N < (2^10)`)
    #[cfg(not(esp32))]
    RepeatNtimes(u16),
    /// Repeat sequence until stopped by additional function call
    Forever,
}

/// Specify the clock source for the RMT peripheral
#[cfg(any(esp32c3, esp32c6, esp32s3))]
#[derive(Debug, Copy, Clone)]
pub enum ClockSource {
    /// Application-level clock
    APB    = 1,
    /// 20 MHz internal oscillator
    RTC20M = 2,
    /// External clock source
    XTAL   = 3,
}

/// Specify the clock source for the RMT peripheral on the ESP32-H2
#[cfg(esp32h2)]
#[derive(Debug, Copy, Clone)]
pub enum ClockSource {
    /// External clock source
    XTAL   = 0,
    /// 20 MHz internal oscillator
    RTC20M = 1,
}

// Clock source to bool
#[cfg(esp32h2)]
impl Into<bool> for ClockSource {
    fn into(self) -> bool {
        match self {
            ClockSource::XTAL => false,
            ClockSource::RTC20M => true,
        }
    }
}

/// Specify the clock source for the RMT peripheral on the ESP32 and ESP32-S3
/// variants
#[cfg(any(esp32s2, esp32))]
#[derive(Debug, Copy, Clone)]
pub enum ClockSource {
    /// Reference Tick (usually configured to 1 us)
    RefTick = 0,
    /// Application-level clock
    APB     = 1,
}

// Specifies how many entries we can store in the RAM section that is allocated
// to the RMT channel
#[cfg(any(esp32s2, esp32))]
const CHANNEL_RAM_SIZE: u8 = 64;
#[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
const CHANNEL_RAM_SIZE: u8 = 48;

// Specifies where the RMT RAM section starts for the particular ESP32 variant
#[cfg(esp32s2)]
const RMT_RAM_START: usize = 0x3f416400;
#[cfg(esp32c3)]
const RMT_RAM_START: usize = 0x60016400;
#[cfg(esp32c6)]
const RMT_RAM_START: usize = 0x60006400;
#[cfg(esp32h2)]
const RMT_RAM_START: usize = 0x60007400;
#[cfg(esp32)]
const RMT_RAM_START: usize = 0x3ff56800;
#[cfg(esp32s3)]
const RMT_RAM_START: usize = 0x60016800;

/// Object representing the state of one pulse code per ESP32-C3 TRM
///
/// Allows for the assignment of two levels and their lenghts
#[derive(Clone, Copy, Debug)]
pub struct PulseCode {
    /// Logical output level in the first pulse code interval
    pub level1: bool,
    /// Length of the first pulse code interval (in clock cycles)
    pub length1: NanosDurationU32,
    /// Logical output level in the second pulse code interval
    pub level2: bool,
    /// Length of the second pulse code interval (in clock cycles)
    pub length2: NanosDurationU32,
}

/// Convert a pulse code structure into a u32 value that can be written
/// into the data registers
impl From<PulseCode> for u32 {
    #[inline(always)]
    fn from(p: PulseCode) -> u32 {
        // The Pulse Code format in the RAM appears to be
        // little-endian

        // The length1 value resides in bits [14:0]
        let mut entry: u32 = p.length1.ticks() as u32;

        // If level1 is high, set bit 15, otherwise clear it
        if p.level1 {
            entry |= 1 << 15;
        } else {
            entry &= !(1 << 15);
        }

        // If level2 is high, set bit 31, otherwise clear it
        if p.level2 {
            entry |= 1 << 31;
        } else {
            entry &= !(1 << 31);
        }

        // The length2 value resides in bits [30:16]
        entry |= (p.length2.ticks() as u32) << 16;

        entry
    }
}

/// Functionality that every OutputChannel must support
pub trait OutputChannel {
    /// Output channel type
    type ConfiguredChannel<'d, P>
    where
        P: OutputPin + 'd;

    /// Set the logical level that the connected pin is pulled to
    /// while the channel is idle
    fn set_idle_output_level(&mut self, level: bool) -> &mut Self;

    /// Enable/Disable the output while the channel is idle
    fn set_idle_output(&mut self, state: bool) -> &mut Self;

    /// Set channel clock divider value
    fn set_channel_divider(&mut self, divider: u8) -> &mut Self;

    /// Enable/Disable carrier modulation
    fn set_carrier_modulation(&mut self, state: bool) -> &mut Self;

    /// Set the clock source (for the ESP32-S2 abd ESP32 this can be done on a
    /// channel level)
    #[cfg(any(esp32s2, esp32))]
    fn set_clock_source(&mut self, source: ClockSource) -> &mut Self;

    /// Assign a pin that should be driven by this channel
    fn assign_pin<'d, P: OutputPin>(
        self,
        pin: impl Peripheral<P = P> + 'd,
    ) -> Self::ConfiguredChannel<'d, P>;
}

/// Functionality that is allowed only on `ConfiguredChannel`
pub trait ConfiguredChannel {
    /// Send a pulse sequence in a blocking fashion
    fn send_pulse_sequence<const N: usize>(
        &mut self,
        repeat_mode: RepeatMode,
        sequence: &[PulseCode; N],
    ) -> Result<(), TransmissionError>;

    /// Send a raw pulse sequence in a blocking fashion
    ///
    /// In this function we expect the `sequence` elements to be already
    /// in the correct u32 format that is understood by the RMT.
    /// Please refer to the reference manual or use the variant which
    /// accepts `PulseCode` objects instead.
    fn send_pulse_sequence_raw<const N: usize>(
        &mut self,
        repeat_mode: RepeatMode,
        sequence: &[u32; N],
    ) -> Result<(), TransmissionError>;

    /// Stop any ongoing (repetitive) transmission
    ///
    /// This function needs to be called to stop sending when
    /// previously a sequence was sent with `RepeatMode::Forever`.
    fn stop_transmission(&self);
}

macro_rules! channel_instance {
    ($num:literal, $cxi:ident, $output_signal:path
        ) => {
        /// RX/TX Input/Output Channel
        pub struct $cxi {
            mem_offset: usize,
        }
        impl $cxi {
            /// Create a new channel instance
            pub fn new() -> Self {
                let mut channel = $cxi { mem_offset: 0 };

                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))] {
                        // Apply default configuration
                        unsafe { &*RMT::PTR }.ch_tx_conf0[$num].modify(|_, w| unsafe {
                            // Configure memory block size
                            w.mem_size()
                                .bits(1)
                        });
                    }
                    else {
                        conf0!($num).modify(|_, w| unsafe {
                            // Configure memory block size
                            w.mem_size()
                                .bits(1)
                        });
                        conf1!($num).modify(|_, w|
                            // Configure memory block size
                            w.mem_owner()
                            .clear_bit()
                        );

                    }
                };

                #[cfg(esp32)]
                conf0!($num).modify(|_, w|
                    // Enable clock
                    w.clk_en()
                        .set_bit()
                        // Disable forced power down of the peripheral (just to be sure)
                        .mem_pd()
                        .clear_bit()
                );

                channel.set_carrier_modulation(false);
                channel.set_idle_output_level(false);
                channel.set_idle_output(false);
                channel.set_channel_divider(1);

                channel
            }

            /// Write a sequence of pulse codes into the RMT fifo buffer
            #[inline(always)]
            fn write_sequence(
                &mut self,
                seq_iter: &mut Iter<u32>,
                max_inserted_elements: u8,
            ){
                for _ in 0..max_inserted_elements {
                    match seq_iter.next() {
                        None => {
                            break;
                        }
                        Some(pulse) => self.load_fifo(*pulse),
                    }
                }
            }

            #[inline(always)]
            fn load_fifo(&mut self, value: u32) {
                let base_ptr: usize = RMT_RAM_START + ($num * CHANNEL_RAM_SIZE as usize * 4);
                let ram_ptr = (base_ptr + self.mem_offset) as *mut u32;
                unsafe {
                    ram_ptr.write_volatile(value);
                }

                self.mem_offset += 4;
                if self.mem_offset >= CHANNEL_RAM_SIZE as usize * 4 {
                    self.mem_offset = 0;
                }
            }

            #[inline(always)]
            fn reset_fifo(&mut self) {
                self.mem_offset = 0;
            }
        }

        paste!(
            #[doc = "Wrapper for`" $cxi "` object."]
            pub struct [<Configured $cxi>]<'d, P> {
                channel: $cxi,
                _pin: PeripheralRef<'d, P>
            }

            impl<'d, P: OutputPin> ConfiguredChannel for [<Configured $cxi>]<'d, P> {
                /// Send a pulse sequence in a blocking fashion
                fn send_pulse_sequence<const N: usize>(
                    &mut self,
                    repeat_mode: RepeatMode,
                    sequence: &[PulseCode; N],
                ) -> Result<(), TransmissionError> {
                    let precomputed_sequence = sequence.map(|x| u32::from(x));

                    self.send_pulse_sequence_raw(repeat_mode, &precomputed_sequence)
                }

            /// Send a raw pulse sequence in a blocking fashion
            ///
            /// In this function we expect the `sequence` elements to be already
            /// in the correct u32 format that is understood by the RMT.
            /// Please refer to the reference manual or use the variant which
            /// accepts `PulseCode` objects instead.
            ///
            /// We expect that the end marker is already part of the provided
            /// sequence and to be provided in all modes!
            fn send_pulse_sequence_raw<const N: usize>(
                &mut self,
                repeat_mode: RepeatMode,
                sequence: &[u32; N],
            ) -> Result<(), TransmissionError> {
                // Check for any configuration error states
                match repeat_mode {
                    #[cfg(not(esp32))]
                    RepeatMode::RepeatNtimes(val) => {
                        if val >= 1024 {
                            return Err(TransmissionError::RepetitionOverflow);
                        }
                        if sequence.len() > CHANNEL_RAM_SIZE as usize {
                            return Err(TransmissionError::IncompatibleRepeatMode);
                        }
                    }
                    RepeatMode::Forever => {
                        if sequence.len() > CHANNEL_RAM_SIZE as usize {
                            return Err(TransmissionError::IncompatibleRepeatMode);
                        }
                    }
                    _ => (),
                };

                // Depending on the variant, other registers have to be used here
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32, esp32s2))] {
                        let conf_reg = & conf1!($num);
                    } else {
                        let conf_reg = & unsafe{ &*RMT::PTR }.ch_tx_conf0[$num];
                    }
                }

                // The ESP32 does not support loop/count modes, as such we have to
                // only configure a subset of registers
                cfg_if::cfg_if! {
                    if #[cfg(esp32)] {
                        // Configure counting mode and repetitions
                        unsafe { &*RMT::PTR }.ch_tx_lim[$num].modify(|_, w| unsafe {
                            // Set the interrupt threshold for sent pulse codes to
                            // half the size of the RAM in case we use wrap mode
                            w.tx_lim()
                                .bits(CHANNEL_RAM_SIZE as u16 /2)
                        });
                    } else {
                        // Extract repetition value
                        let mut reps = 0;
                        if let RepeatMode::RepeatNtimes(val) = repeat_mode {
                            reps = val;
                        }

                        // Configure counting mode and repetitions
                        unsafe { &*RMT::PTR }.ch_tx_lim[$num].modify(|_, w| unsafe {
                            // Set number of repetitions
                            w.tx_loop_num()
                                .bits(reps)
                                // Enable loop counting
                                .tx_loop_cnt_en()
                                .bit(reps != 0)
                                // Reset any pre-existing counting value
                                .loop_count_reset()
                                .set_bit()
                                // Set the interrupt threshold for sent pulse codes to 24
                                // (= half the size of the RAM) in case we use wrap mode
                                .tx_lim()
                                .bits(CHANNEL_RAM_SIZE as u16/2)
                        });
                    }
                }

                #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
                conf_reg.modify(|_, w| {
                    // Set config update bit
                    w.conf_update().set_bit()
                });

                // Setup configuration
                conf_reg.modify(|_, w| {
                    // Set configure continuous
                    // (also reset FIFO buffer pointers)
                    w.tx_conti_mode()
                        .bit(repeat_mode != RepeatMode::SingleShot)
                        .mem_rd_rst()
                        .set_bit()
                        .apb_mem_rst()
                        .set_bit()
                });

                self.channel.reset_fifo();
                let mut sequence_iter = sequence.iter();

                // We have to differentiate here if we can fit the whole sequence
                // in the RAM in one go or if we have to use the wrap mode to split
                // the sequence into chuncks.
                if sequence.len() >= CHANNEL_RAM_SIZE as usize {
                    // Write the first 48 entries
                    self.channel.write_sequence(&mut sequence_iter, CHANNEL_RAM_SIZE);
                } else {
                    // Write whole sequence to FIFO RAM
                    self.channel.write_sequence(&mut sequence_iter, CHANNEL_RAM_SIZE);
                }

                // Clear the relevant interrupts
                //
                // (since this is a write-through register, we can do this
                // safely for multiple separate channel instances without
                // having concurrency issues)
                // Depending on the variant, other registers have to be used here
                cfg_if::cfg_if! {
                    if #[cfg(esp32)] {
                        unsafe { &*RMT::PTR }.int_clr.write(|w| {
                            // The ESP32 variant does not have the loop functionality
                            paste!(
                                w.[<ch $num _tx_end_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _err_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _tx_thr_event_int_clr>]()
                                    .set_bit()
                            )
                        });
                    } else if #[cfg(esp32s2)] {
                        unsafe { &*RMT::PTR }.int_clr.write(|w| {
                            paste!(
                                w.[<ch $num _tx_end_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _tx_loop_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _err_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _tx_thr_event_int_clr>]()
                                    .set_bit()
                            )
                        });
                    } else {
                        unsafe { &*RMT::PTR }.int_clr.write(|w| {
                            paste!(
                                w.[<ch $num _tx_end_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _tx_loop_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _tx_err_int_clr>]()
                                    .set_bit()
                                    .[<ch $num _tx_thr_event_int_clr>]()
                                    .set_bit()
                            )
                        });
                    }
                }

                // always enable tx wrap
                #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
                unsafe { &*RMT::PTR }.ch_tx_conf0[$num].modify(|_, w| {
                    w.mem_tx_wrap_en()
                        .set_bit()
                });

                // apply configuration updates
                #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
                unsafe { &*RMT::PTR }.ch_tx_conf0[$num].modify(|_, w| {
                    w.conf_update()
                        .set_bit()
                });

                // Depending on the variant, other registers have to be used here
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32, esp32s2))] {
                        conf1!($num).modify(|_, w| w.tx_start().set_bit());
                    } else {
                        unsafe{ &*RMT::PTR }.ch_tx_conf0[$num].modify(|_, w| w.tx_start().set_bit());
                    }
                }

                // If we're in forever mode, we return right away, otherwise we wait
                // for completion
                if repeat_mode != RepeatMode::Forever {
                    // Wait for interrupt being raised, either completion or error
                    loop {
                        let interrupts = unsafe { &*RMT::PTR }.int_raw.read();

                        match (
                            #[cfg(not(esp32h2))]
                            unsafe { interrupts.ch_tx_end_int_raw($num).bit() },
                            #[cfg(esp32h2)]
                            interrupts.[<ch $num _tx_end_int_raw>]().bit(),
                            // The ESP32 variant does not support the loop functionality
                            #[cfg(esp32)]
                            false,
                            #[cfg(esp32h2)]
                            interrupts.[<ch $num _tx_loop_int_raw>]().bit(),
                            #[cfg(not(any(esp32, esp32h2)))]
                            unsafe {interrupts.ch_tx_loop_int_raw($num).bit()},
                            // The C3/S3 have a slightly different interrupt naming scheme
                            #[cfg(any(esp32, esp32s2))]
                            unsafe { interrupts.ch_err_int_raw($num).bit() },
                            #[cfg(any(esp32c3, esp32c6, esp32s3))]
                            unsafe { interrupts.ch_tx_err_int_raw($num).bit() },
                            #[cfg(not(esp32h2))]
                            unsafe { interrupts.ch_tx_thr_event_int_raw($num).bit() },
                            #[cfg(esp32h2)]
                            interrupts.[<ch $num _tx_err_int_raw>]().bit(),
                            #[cfg(esp32h2)]
                            interrupts.[<ch $num _tx_thr_event_int_raw>]().bit(),
                        ) {
                            // SingleShot completed and no error -> success
                            (true, false, false, _) => break,
                            // Sequence completed and no error -> success
                            (false, true, false, _) => {
                                // Stop transmitting (only necessary in sequence case)
                                self.stop_transmission();
                                break;
                            }
                            // Refill the buffer
                            (false, false, false, true) => {
                                self.channel.write_sequence(&mut sequence_iter, CHANNEL_RAM_SIZE / 2);

                                // Clear the threshold interrupt (write-through)
                                unsafe { &*RMT::PTR }.int_clr.write(|w| {
                                    paste!(w.[<ch $num _tx_thr_event_int_clr>]().set_bit())
                                });
                        }
                            // Neither completed nor error -> continue busy waiting
                            (false, false, false, false) => (),
                            // Anything else constitutes an error state
                            _ => {
                                return Err(TransmissionError::Failure(
                                    #[cfg(not(esp32h2))]
                                    unsafe { interrupts.ch_tx_end_int_raw($num).bit() },
                                     #[cfg(esp32h2)]
                                    interrupts.[<ch $num _tx_end_int_raw>]().bit(),
                                    // The ESP32 variant does not support the loop functionality
                                    #[cfg(esp32)]
                                    false,
                                    #[cfg(esp32h2)]
                                    interrupts.[<ch $num _tx_loop_int_raw>]().bit(),
                                    #[cfg(not(any(esp32, esp32h2)))]
                                    unsafe {interrupts.ch_tx_loop_int_raw($num).bit()},
                                    // The C3/S3 have a slightly different interrupt naming scheme
                                    #[cfg(any(esp32, esp32s2))]
                                    unsafe { interrupts.ch_err_int_raw($num).bit() },
                                    #[cfg(any(esp32c3, esp32c6, esp32s3))]
                                    unsafe { interrupts.ch_tx_err_int_raw($num).bit() },
                                    #[cfg(not(esp32h2))]
                                    unsafe { interrupts.ch_tx_thr_event_int_raw($num).bit() },
                                    #[cfg(esp32h2)]
                                    interrupts.[<ch $num _tx_err_int_raw>]().bit(),
                                    #[cfg(esp32h2)]
                                    interrupts.[<ch $num _tx_thr_event_int_raw>]().bit(),
                                ))
                            }
                        }
                    }
                }

                Ok(())
            }

            /// Stop any ongoing (repetitive) transmission
            ///
            /// This function needs to be called to stop sending when
            /// previously a sequence was sent with `RepeatMode::Forever`.
            fn stop_transmission(&self) {
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))] {
                        unsafe { &*RMT::PTR }
                            .ch_tx_conf0[$num]
                            .modify(|_, w| w.tx_stop().set_bit());
                    }
                    else if #[cfg(esp32s2)] {
                        conf1!($num)
                            .modify(|_, w| w.tx_stop().set_bit());
                    }
                    // The ESP32 variant does not have any way to stop a
                    // transmission once it has been started!
                };
            }
            }

        );
    };
}

macro_rules! output_channel {
    ($num:literal, $cxi:ident, $output_signal:path
        ) => {
            paste!(

        impl OutputChannel for $cxi {

            type ConfiguredChannel<'d, P> = [<Configured $cxi>]<'d, P>
                where P: OutputPin + 'd;

            /// Set the logical level that the connected pin is pulled to
            /// while the channel is idle
            #[inline(always)]
            fn set_idle_output_level(&mut self, level: bool) -> &mut Self {
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))] {
                        unsafe { &*RMT::PTR }
                            .ch_tx_conf0[$num]
                            .modify(|_, w| w.idle_out_lv().bit(level));
                    }
                    else {
                        conf1!($num)
                            .modify(|_, w| w.idle_out_lv().bit(level));
                    }
                };
                self
            }

            /// Enable/Disable the output while the channel is idle
            #[inline(always)]
            fn set_idle_output(&mut self, state: bool) -> &mut Self {
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))] {
                        unsafe { &*RMT::PTR }
                            .ch_tx_conf0[$num]
                            .modify(|_, w| w.idle_out_en().bit(state));
                    }
                    else {
                        conf1!($num)
                            .modify(|_, w| w.idle_out_en().bit(state));
                    }
                };
                self
            }

            /// Set channel clock divider value
            #[inline(always)]
            fn set_channel_divider(&mut self, divider: u8) -> &mut Self {
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))] {
                        unsafe { &*RMT::PTR }
                            .ch_tx_conf0[$num]
                            .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                    }
                    else {
                        conf0!($num)
                            .modify(|_, w| unsafe { w.div_cnt().bits(divider) });
                    }
                };
                self
            }

            /// Enable/Disable carrier modulation
            #[inline(always)]
            fn set_carrier_modulation(&mut self, state: bool) -> &mut Self {
                cfg_if::cfg_if! {
                    if #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))] {
                        unsafe { &*RMT::PTR }
                            .ch_tx_conf0[$num]
                            .modify(|_, w| w.carrier_en().bit(state));
                    }
                    else {
                        conf0!($num)
                            .modify(|_, w| w.carrier_en().bit(state));
                    }
                };
                self
            }

            /// Set the clock source (for the ESP32-S2 and ESP32 this can be done on a
            /// channel level)
            #[cfg(any(esp32s2, esp32))]
            #[inline(always)]
            fn set_clock_source(&mut self, source: ClockSource) -> &mut Self {
                let bit_value = match source {
                    ClockSource::RefTick => false,
                    ClockSource::APB => true,
                };

                conf1!($num)
                    .modify(|_, w| w.ref_always_on().bit(bit_value));
                self
            }

            /// Assign a pin that should be driven by this channel
            fn assign_pin<'d, RmtPin: OutputPin >(
                self,
                pin: impl Peripheral<P = RmtPin> + 'd
            ) -> [<Configured $cxi>]<'d, RmtPin> {
                crate::into_ref!(pin);
                // Configure Pin as output anc connect to signal
                pin.set_to_push_pull_output()
                    .connect_peripheral_to_output($output_signal);

                [<Configured $cxi>] {
                    channel: self,
                    _pin: pin
                }
            }
        }
    );
    };
}

#[cfg(esp32)]
macro_rules! conf0 {
    ($channel: literal) => {
        match $channel {
            0 => &unsafe { &*RMT::PTR }.ch0conf0,
            1 => &unsafe { &*RMT::PTR }.ch1conf0,
            2 => &unsafe { &*RMT::PTR }.ch2conf0,
            3 => &unsafe { &*RMT::PTR }.ch3conf0,
            4 => &unsafe { &*RMT::PTR }.ch4conf0,
            5 => &unsafe { &*RMT::PTR }.ch5conf0,
            6 => &unsafe { &*RMT::PTR }.ch6conf0,
            7 => &unsafe { &*RMT::PTR }.ch7conf0,
            _ => panic!("Attempted access to non-existing channel!"),
        }
    };
}

#[cfg(esp32)]
macro_rules! conf1 {
    ($channel: literal) => {
        match $channel {
            0 => &unsafe { &*RMT::PTR }.ch0conf1,
            1 => &unsafe { &*RMT::PTR }.ch1conf1,
            2 => &unsafe { &*RMT::PTR }.ch2conf1,
            3 => &unsafe { &*RMT::PTR }.ch3conf1,
            4 => &unsafe { &*RMT::PTR }.ch4conf1,
            5 => &unsafe { &*RMT::PTR }.ch5conf1,
            6 => &unsafe { &*RMT::PTR }.ch6conf1,
            7 => &unsafe { &*RMT::PTR }.ch7conf1,
            _ => panic!("Attempted access to non-existing channel!"),
        }
    };
}

#[cfg(esp32s2)]
macro_rules! conf0 {
    ($channel: literal) => {
        match $channel {
            0 => &unsafe { &*RMT::PTR }.ch0conf0,
            1 => &unsafe { &*RMT::PTR }.ch1conf0,
            2 => &unsafe { &*RMT::PTR }.ch2conf0,
            3 => &unsafe { &*RMT::PTR }.ch3conf0,
            _ => panic!("Attempted access to non-existing channel!"),
        }
    };
}

#[cfg(esp32s2)]
macro_rules! conf1 {
    ($channel: literal) => {
        match $channel {
            0 => &unsafe { &*RMT::PTR }.ch0conf1,
            1 => &unsafe { &*RMT::PTR }.ch1conf1,
            2 => &unsafe { &*RMT::PTR }.ch2conf1,
            3 => &unsafe { &*RMT::PTR }.ch3conf1,
            _ => panic!("Attempted access to non-existing channel!"),
        }
    };
}

macro_rules! rmt {
    (
        $global_conf_reg:ident,
        $(
            ($num:literal, $cxi:ident, $obj_name:ident, $output_signal:path),
        )+
    )
 => {
    /// Remote Control (RMT) peripheral driver
    pub struct PulseControl<'d> {
        /// The underlying register block
        reg: PeripheralRef<'d, RMT>,
        $(
            /// RMT channel $cxi
            pub $obj_name: $cxi,
        )+
    }

    impl<'d> PulseControl<'d> {
        /// Create a new pulse controller instance
        #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
        pub fn new(
            instance: impl Peripheral<P = RMT> + 'd,
            peripheral_clock_control: &mut PeripheralClockControl,
            clk_source: ClockSource,
            div_abs: u8,
            div_frac_a: u8,
            div_frac_b: u8,
        ) -> Result<Self, SetupError> {

            crate::into_ref!(instance);
            let pc = PulseControl {
                reg: instance,
                $(
                    $obj_name: $cxi::new(),
                )+
            };

            pc.enable_peripheral(peripheral_clock_control);
            pc.config_global(clk_source, div_abs, div_frac_a, div_frac_b)?;

            Ok(pc)
        }

        /// Create a new pulse controller instance
        #[cfg(any(esp32, esp32s2))]
        pub fn new(
            instance: impl Peripheral<P = RMT> + 'd,
            peripheral_clock_control: &mut PeripheralClockControl,
        ) -> Result<Self, SetupError> {

            crate::into_ref!(instance);
            let pc = PulseControl {
                reg: instance,
                $(
                    $obj_name: $cxi::new(),
                )+
            };

            pc.enable_peripheral(peripheral_clock_control);
            pc.config_global()?;

            Ok(pc)
        }

        // Enable the RMT peripherals clock in the system peripheral
        fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
            peripheral_clock_control.enable(crate::system::Peripheral::Rmt);
        }

        /// Assign the global (peripheral-wide) configuration. This
        /// is mostly the divider setup and the clock source selection
        ///
        /// The dividing factor for the source
        /// clock is calculated as follows:
        ///
        /// divider = absolute_part + 1 + (fractional_part_a / fractional_part_b)
        #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
        fn config_global(
            &self,
            clk_source: ClockSource,
            div_abs: u8,
            div_frac_a: u8,
            div_frac_b: u8,
        ) -> Result<(), SetupError> {
            // Before assigning, confirm that the fractional parameters for
            // the divider are within bounds
            if div_frac_a > 64 || div_frac_b > 64 {
                return Err(SetupError::InvalidGlobalConfig);
            }

            // TODO: Confirm that the selected clock source is enabled in the
            // system / rtc_cntl peripheral? Particularly relevant for clock sources
            // other than APB_CLK, this needs to be revisited once #24 and $44 have been
            // addressed!

            // Configure peripheral

            #[cfg(any(esp32c6, esp32h2))]
            let pcr = unsafe { &*PCR::ptr() };

            #[cfg(any(esp32c6, esp32h2))]
            pcr.rmt_sclk_conf.write(|w| w.sclk_en().set_bit());


            self.reg.sys_conf.modify(|_, w|
                // Enable clock
                w.clk_en()
                    .set_bit()
                    // Force Clock on
                    .mem_clk_force_on()
                    .set_bit()
                    // Enable Source clock
                    .sclk_active()
                    .set_bit()
                    // Disable forced power down of the peripheral (just to be sure)
                    .mem_force_pd()
                    .clear_bit()
                    // Disable FIFO mode
                    .apb_fifo_mask()
                    .set_bit());
                    // Select clock source
                #[cfg(not(any(esp32c6, esp32h2)))]
                self.reg.sys_conf.modify(|_, w| unsafe {
                    w.sclk_sel()
                    .bits(clk_source as u8)
                    // Set absolute part of divider
                    .sclk_div_num()
                    .bits(div_abs)
                    // Set fractional parts of divider to 0
                    .sclk_div_a()
                    .bits(div_frac_a)
                    .sclk_div_b()
                    .bits(div_frac_b) });
                #[cfg(esp32c6)]
                pcr.rmt_sclk_conf.modify(|_,w| unsafe {
                    w.sclk_sel()
                    .bits(clk_source as u8)
                    // Set absolute part of divider
                    .sclk_div_num()
                    .bits(div_abs)
                    // Set fractional parts of divider to 0
                    .sclk_div_a()
                    .bits(div_frac_a)
                    .sclk_div_b()
                    .bits(div_frac_b)
                    });
                #[cfg(esp32h2)]
                pcr.rmt_sclk_conf.modify(|_,w| unsafe {
                    w.sclk_sel()
                    .bit(clk_source.into())
                    // Set absolute part of divider
                    .sclk_div_num()
                    .bits(div_abs)
                    // Set fractional parts of divider to 0
                    .sclk_div_a()
                    .bits(div_frac_a)
                    .sclk_div_b()
                    .bits(div_frac_b)
                });

            // Disable all interrupts
            self.reg.int_ena.write(|w| unsafe { w.bits(0) });

            // Clear all interrupts
            self.reg.int_clr.write(|w| unsafe { w.bits(0xffffffff) });

            Ok(())
        }

        /// Assign the global (peripheral-wide) configuration.
        #[cfg(any(esp32s2, esp32))]
        fn config_global(&self) -> Result<(), SetupError> {
            // TODO: Confirm that the selected clock source is enabled in the
            // system / rtc_cntl peripheral? Particularly relevant for clock sources
            // other than APB_CLK, this needs to be revisited once #24 and $44 have been
            // addressed!

            cfg_if::cfg_if! {
                if #[cfg(esp32)] {
                    // Configure peripheral
                    self.reg.apb_conf.modify(|_, w|
                        // Disable FIFO mode
                        w.apb_fifo_mask()
                            .set_bit()
                            // Enable wrap mode (globally for the ESP32 and ESP32-S2 variants)
                            .mem_tx_wrap_en()
                            .set_bit()
                    );
                }
                else {
                    // Configure peripheral
                    self.reg.apb_conf.modify(|_, w|
                        // Enable clock
                        w.clk_en()
                            .set_bit()
                            // Force Clock on
                            .mem_clk_force_on()
                            .set_bit()
                            // Disable forced power down of the peripheral (just to be sure)
                            .mem_force_pd()
                            .clear_bit()
                            // Disable FIFO mode
                            .apb_fifo_mask()
                            .set_bit()
                            // Enable wrap mode (globally for the ESP32 and ESP32-S2 variants)
                            .mem_tx_wrap_en()
                            .set_bit()
                    );
                }
            };

            // Disable all interrupts
            self.reg.int_ena.write(|w| unsafe { w.bits(0) });

            // Clear all interrupts
            self.reg.int_clr.write(|w| unsafe { w.bits(0) });

            Ok(())
        }
    }
    $(
        channel_instance!($num, $cxi, $output_signal);
        output_channel!($num, $cxi, $output_signal);
    )+
 };
}

#[cfg(any(esp32c3, esp32c6, esp32h2))]
rmt!(
    sys_conf,
    (0, Channel0, channel0, OutputSignal::RMT_SIG_0),
    (1, Channel1, channel1, OutputSignal::RMT_SIG_1),
);

#[cfg(esp32s2)]
rmt!(
    apb_conf,
    (0, Channel0, channel0, OutputSignal::RMT_SIG_OUT0),
    (1, Channel1, channel1, OutputSignal::RMT_SIG_OUT1),
    (2, Channel2, channel2, OutputSignal::RMT_SIG_OUT2),
    (3, Channel3, channel3, OutputSignal::RMT_SIG_OUT3),
);

#[cfg(esp32)]
rmt!(
    apb_conf,
    (0, Channel0, channel0, OutputSignal::RMT_SIG_0),
    (1, Channel1, channel1, OutputSignal::RMT_SIG_1),
    (2, Channel2, channel2, OutputSignal::RMT_SIG_2),
    (3, Channel3, channel3, OutputSignal::RMT_SIG_3),
    (4, Channel4, channel4, OutputSignal::RMT_SIG_4),
    (5, Channel5, channel5, OutputSignal::RMT_SIG_5),
    (6, Channel6, channel6, OutputSignal::RMT_SIG_6),
    (7, Channel7, channel7, OutputSignal::RMT_SIG_7),
);

#[cfg(esp32s3)]
rmt!(
    sys_conf,
    (0, Channel0, channel0, OutputSignal::RMT_SIG_OUT0),
    (1, Channel1, channel1, OutputSignal::RMT_SIG_OUT1),
    (2, Channel2, channel2, OutputSignal::RMT_SIG_OUT2),
    (3, Channel3, channel3, OutputSignal::RMT_SIG_OUT3),
);
