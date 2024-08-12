//! # Two-wire Automotive Interface (TWAI)
//!
//! ## Overview
//! The TWAI is a multi-master, multi-cast communication protocol with error
//! detection and signaling and inbuilt message priorities and arbitration. The
//! TWAI protocol is suited for automotive and industrial applications.
//!
//! See ESP-IDF's
#![doc = concat!("[TWAI documentation](https://docs.espressif.com/projects/esp-idf/en/latest/", crate::soc::chip!(), "/api-reference/peripherals/twai.html#twai-protocol-summary)")]
//! for a summary on the protocol.
//!
//! ## Configuration
//! The driver  offers functions for initializing the TWAI peripheral, setting
//! up the timing parameters, configuring acceptance filters, handling
//! interrupts, and transmitting/receiving messages on the TWAI bus.
//!
//! This driver manages the ISO 11898-1 (CAN Specification 2.0) compatible TWAI
//! controllers. It supports Standard Frame Format (11-bit) and Extended Frame
//! Format (29-bit) frame identifiers.
//!
//! ## Example
//! ### Transmitting and Receiving Messages
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::twai;
//! # use embedded_can::Id;
//! # use esp_hal::twai::filter::SingleStandardFilter;
//! # use esp_hal::twai::filter;
//! # use esp_hal::twai::TwaiConfiguration;
//! # use esp_hal::twai::BaudRate;
//! # use esp_hal::twai::TwaiMode;
//! # use esp_hal::gpio::Io;
//! # use embedded_can::Frame;
//! # use core::option::Option::None;
//! # use nb::block;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! // Use GPIO pins 2 and 3 to connect to the respective pins on the CAN
//! // transceiver.
//! let can_tx_pin = io.pins.gpio2;
//! let can_rx_pin = io.pins.gpio3;
//!
//! // The speed of the CAN bus.
//! const CAN_BAUDRATE: twai::BaudRate = BaudRate::B1000K;
//!
//! // Begin configuring the TWAI peripheral. The peripheral is in a reset like
//! // state that prevents transmission but allows configuration.
//! let mut can_config = twai::TwaiConfiguration::new(
//!     peripherals.TWAI0,
//!     can_tx_pin,
//!     can_rx_pin,
//!     &clocks,
//!     CAN_BAUDRATE,
//!     TwaiMode::Normal
//! );
//!
//! // Partially filter the incoming messages to reduce overhead of receiving
//! // undesired messages
//! const FILTER: twai::filter::SingleStandardFilter =
//!     SingleStandardFilter::new(b"xxxxxxxxxx0", b"x", [b"xxxxxxxx",
//! b"xxxxxxxx"]); can_config.set_filter(FILTER);
//!
//! // Start the peripheral. This locks the configuration settings of the
//! // peripheral and puts it into operation mode, allowing packets to be sent
//! // and received.
//! let mut can = can_config.start();
//!
//! loop {
//!     // Wait for a frame to be received.
//!     let frame = block!(can.receive()).unwrap();
//!
//!     // Transmit the frame back.
//!     let _result = block!(can.transmit(&frame)).unwrap();
//! }
//! # }
//! ```
//! ### Self-testing (self reception of transmitted messages)
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::twai;
//! # use embedded_can::Id;
//! # use esp_hal::twai::filter::SingleStandardFilter;
//! # use esp_hal::twai::filter;
//! # use esp_hal::twai::TwaiConfiguration;
//! # use esp_hal::twai::BaudRate;
//! # use esp_hal::twai::EspTwaiFrame;
//! # use esp_hal::twai::StandardId;
//! # use esp_hal::twai::TwaiMode;
//! # use esp_hal::gpio::Io;
//! # use embedded_can::Frame;
//! # use core::option::Option::None;
//! # use nb::block;
//! # let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
//! // Use GPIO pins 2 and 3 to connect to the respective pins on the CAN
//! // transceiver.
//! let can_tx_pin = io.pins.gpio2;
//! let can_rx_pin = io.pins.gpio3;
//!
//! // The speed of the CAN bus.
//! const CAN_BAUDRATE: twai::BaudRate = BaudRate::B1000K;
//!
//! // Begin configuring the TWAI peripheral.
//! let mut can_config = twai::TwaiConfiguration::new(
//!     peripherals.TWAI0,
//!     can_tx_pin,
//!     can_rx_pin,
//!     &clocks,
//!     CAN_BAUDRATE,
//!     TwaiMode::SelfTest
//! );
//!
//! // Partially filter the incoming messages to reduce overhead of receiving
//! // undesired messages
//! const FILTER: twai::filter::SingleStandardFilter =
//!     SingleStandardFilter::new(b"xxxxxxxxxx0", b"x",
//!         [b"xxxxxxxx", b"xxxxxxxx"]);
//! can_config.set_filter(FILTER);
//!
//! // Start the peripheral. This locks the configuration settings of the
//! // peripheral and puts it into operation mode, allowing packets to be sent
//! // and received.
//! let mut can = can_config.start();
//!
//! let frame = EspTwaiFrame::new_self_reception(StandardId::ZERO.into(),
//!     &[1, 2, 3]).unwrap();
//! // Wait for a frame to be received.
//! let frame = block!(can.receive()).unwrap();
//!
//! loop {}
//! # }
//! ```

#![allow(missing_docs)] // TODO: Remove when able

use core::marker::PhantomData;

use self::filter::{Filter, FilterType};
use crate::{
    clock::Clocks,
    gpio::{InputPin, InputSignal, OutputPin, OutputSignal},
    interrupt::InterruptHandler,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::twai0::RegisterBlock,
    system::{self, PeripheralClockControl},
    InterruptConfigurable,
};

pub mod filter;

/// CAN error kind
///
/// This represents a common set of CAN operation errors. HAL implementations
/// are free to define more specific or additional error types. However, by
/// providing a mapping to these common CAN errors, generic code can still react
/// to them.
#[derive(Debug, Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash)]
#[non_exhaustive]
pub enum ErrorKind {
    /// The peripheral receive buffer was overrun.
    Overrun,
    // MAC sublayer errors
    /// A bit error is detected at that bit time when the bit value that is
    /// monitored differs from the bit value sent.
    Bit,
    /// A stuff error is detected at the bit time of the sixth consecutive
    /// equal bit level in a frame field that shall be coded by the method
    /// of bit stuffing.
    Stuff,
    /// Calculated CRC sequence does not equal the received one.
    Crc,
    /// A form error shall be detected when a fixed-form bit field contains
    /// one or more illegal bits.
    Form,
    /// An ACK  error shall be detected by a transmitter whenever it does not
    /// monitor a dominant bit during the ACK slot.
    Acknowledge,
    /// A different error occurred. The original error may contain more
    /// information.
    Other,
}

impl core::fmt::Display for ErrorKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Overrun => write!(f, "The peripheral receive buffer was overrun"),
            Self::Bit => write!(
                f,
                "Bit value that is monitored differs from the bit value sent"
            ),
            Self::Stuff => write!(f, "Sixth consecutive equal bits detected"),
            Self::Crc => write!(f, "Calculated CRC sequence does not equal the received one"),
            Self::Form => write!(
                f,
                "A fixed-form bit field contains one or more illegal bits"
            ),
            Self::Acknowledge => write!(f, "Transmitted frame was not acknowledged"),
            Self::Other => write!(
                f,
                "A different error occurred. The original error may contain more information"
            ),
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl From<ErrorKind> for embedded_hal_02::can::ErrorKind {
    fn from(value: ErrorKind) -> Self {
        match value {
            ErrorKind::Overrun => embedded_hal_02::can::ErrorKind::Overrun,
            ErrorKind::Bit => embedded_hal_02::can::ErrorKind::Bit,
            ErrorKind::Stuff => embedded_hal_02::can::ErrorKind::Stuff,
            ErrorKind::Crc => embedded_hal_02::can::ErrorKind::Crc,
            ErrorKind::Form => embedded_hal_02::can::ErrorKind::Form,
            ErrorKind::Acknowledge => embedded_hal_02::can::ErrorKind::Acknowledge,
            ErrorKind::Other => embedded_hal_02::can::ErrorKind::Other,
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::can::Error for ErrorKind {
    fn kind(&self) -> embedded_hal_02::can::ErrorKind {
        (*self).into()
    }
}

#[cfg(feature = "embedded-hal")]
impl From<ErrorKind> for embedded_can::ErrorKind {
    fn from(value: ErrorKind) -> Self {
        match value {
            ErrorKind::Overrun => embedded_can::ErrorKind::Overrun,
            ErrorKind::Bit => embedded_can::ErrorKind::Bit,
            ErrorKind::Stuff => embedded_can::ErrorKind::Stuff,
            ErrorKind::Crc => embedded_can::ErrorKind::Crc,
            ErrorKind::Form => embedded_can::ErrorKind::Form,
            ErrorKind::Acknowledge => embedded_can::ErrorKind::Acknowledge,
            ErrorKind::Other => embedded_can::ErrorKind::Other,
        }
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_can::Error for ErrorKind {
    fn kind(&self) -> embedded_can::ErrorKind {
        (*self).into()
    }
}

/// Specifies in which mode the TWAI controller will operate.
pub enum TwaiMode {
    /// Normal operating mode
    Normal,
    /// Self-test mode (no acknowledgement required for a successful message
    /// transmission)
    SelfTest,
    /// Listen only operating mode
    ListenOnly,
}

/// Standard 11-bit CAN Identifier (`0..=0x7FF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct StandardId(u16);

impl StandardId {
    /// CAN ID `0`, the highest priority.
    pub const ZERO: Self = StandardId(0);

    /// CAN ID `0x7FF`, the lowest priority.
    pub const MAX: Self = StandardId(0x7FF);

    /// Tries to create a `StandardId` from a raw 16-bit integer.
    ///
    /// This will return `None` if `raw` is out of range of an 11-bit integer
    /// (`> 0x7FF`).
    #[inline]
    pub fn new(raw: u16) -> Option<Self> {
        if raw <= 0x7FF {
            Some(StandardId(raw))
        } else {
            None
        }
    }

    /// Creates a new `StandardId` without checking if it is inside the valid
    /// range.
    ///
    /// # Safety
    /// Using this method can create an invalid ID and is thus marked as unsafe.
    #[inline]
    pub const unsafe fn new_unchecked(raw: u16) -> Self {
        StandardId(raw)
    }

    /// Returns this CAN Identifier as a raw 16-bit integer.
    #[inline]
    pub fn as_raw(&self) -> u16 {
        self.0
    }
}

#[cfg(feature = "embedded-hal-02")]
impl From<StandardId> for embedded_hal_02::can::StandardId {
    fn from(value: StandardId) -> Self {
        embedded_hal_02::can::StandardId::new(value.as_raw()).unwrap()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl From<embedded_hal_02::can::StandardId> for StandardId {
    fn from(value: embedded_hal_02::can::StandardId) -> Self {
        StandardId::new(value.as_raw()).unwrap()
    }
}

#[cfg(feature = "embedded-hal")]
impl From<StandardId> for embedded_can::StandardId {
    fn from(value: StandardId) -> Self {
        embedded_can::StandardId::new(value.as_raw()).unwrap()
    }
}

#[cfg(feature = "embedded-hal")]
impl From<embedded_can::StandardId> for StandardId {
    fn from(value: embedded_can::StandardId) -> Self {
        StandardId::new(value.as_raw()).unwrap()
    }
}

/// Extended 29-bit CAN Identifier (`0..=1FFF_FFFF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct ExtendedId(u32);

impl ExtendedId {
    /// CAN ID `0`, the highest priority.
    pub const ZERO: Self = ExtendedId(0);

    /// CAN ID `0x1FFFFFFF`, the lowest priority.
    pub const MAX: Self = ExtendedId(0x1FFF_FFFF);

    /// Tries to create a `ExtendedId` from a raw 32-bit integer.
    ///
    /// This will return `None` if `raw` is out of range of an 29-bit integer
    /// (`> 0x1FFF_FFFF`).
    #[inline]
    pub fn new(raw: u32) -> Option<Self> {
        if raw <= 0x1FFF_FFFF {
            Some(ExtendedId(raw))
        } else {
            None
        }
    }

    /// Creates a new `ExtendedId` without checking if it is inside the valid
    /// range.
    ///
    /// # Safety
    /// Using this method can create an invalid ID and is thus marked as unsafe.
    #[inline]
    pub const unsafe fn new_unchecked(raw: u32) -> Self {
        ExtendedId(raw)
    }

    /// Returns this CAN Identifier as a raw 32-bit integer.
    #[inline]
    pub fn as_raw(&self) -> u32 {
        self.0
    }

    /// Returns the Base ID part of this extended identifier.
    pub fn standard_id(&self) -> StandardId {
        // ID-28 to ID-18
        StandardId((self.0 >> 18) as u16)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl From<ExtendedId> for embedded_hal_02::can::ExtendedId {
    fn from(value: ExtendedId) -> Self {
        embedded_hal_02::can::ExtendedId::new(value.0).unwrap()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl From<embedded_hal_02::can::ExtendedId> for ExtendedId {
    fn from(value: embedded_hal_02::can::ExtendedId) -> Self {
        ExtendedId::new(value.as_raw()).unwrap()
    }
}

#[cfg(feature = "embedded-hal")]
impl From<ExtendedId> for embedded_can::ExtendedId {
    fn from(value: ExtendedId) -> Self {
        embedded_can::ExtendedId::new(value.0).unwrap()
    }
}

#[cfg(feature = "embedded-hal")]
impl From<embedded_can::ExtendedId> for ExtendedId {
    fn from(value: embedded_can::ExtendedId) -> Self {
        ExtendedId::new(value.as_raw()).unwrap()
    }
}

/// A CAN Identifier (standard or extended).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Id {
    /// Standard 11-bit Identifier (`0..=0x7FF`).
    Standard(StandardId),
    /// Extended 29-bit Identifier (`0..=0x1FFF_FFFF`).
    Extended(ExtendedId),
}

impl From<StandardId> for Id {
    #[inline]
    fn from(id: StandardId) -> Self {
        Id::Standard(id)
    }
}

impl From<ExtendedId> for Id {
    #[inline]
    fn from(id: ExtendedId) -> Self {
        Id::Extended(id)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl From<Id> for embedded_hal_02::can::Id {
    fn from(value: Id) -> Self {
        match value {
            Id::Standard(id) => embedded_hal_02::can::Id::Standard(id.into()),
            Id::Extended(id) => embedded_hal_02::can::Id::Extended(id.into()),
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl From<embedded_hal_02::can::Id> for Id {
    fn from(value: embedded_hal_02::can::Id) -> Self {
        match value {
            embedded_hal_02::can::Id::Standard(id) => Id::Standard(id.into()),
            embedded_hal_02::can::Id::Extended(id) => Id::Extended(id.into()),
        }
    }
}

#[cfg(feature = "embedded-hal")]
impl From<Id> for embedded_can::Id {
    fn from(value: Id) -> Self {
        match value {
            Id::Standard(id) => embedded_can::Id::Standard(id.into()),
            Id::Extended(id) => embedded_can::Id::Extended(id.into()),
        }
    }
}

#[cfg(feature = "embedded-hal")]
impl From<embedded_can::Id> for Id {
    fn from(value: embedded_can::Id) -> Self {
        match value {
            embedded_can::Id::Standard(id) => Id::Standard(id.into()),
            embedded_can::Id::Extended(id) => Id::Extended(id.into()),
        }
    }
}

/// Structure backing the embedded_hal_02::can::Frame/embedded_can::Frame trait.
#[derive(Clone, Copy, Debug)]
pub struct EspTwaiFrame {
    id: Id,
    dlc: usize,
    data: [u8; 8],
    is_remote: bool,
    self_reception: bool,
}

impl EspTwaiFrame {
    pub fn new(id: Id, data: &[u8]) -> Option<Self> {
        // CAN2.0 frames cannot contain more than 8 bytes of data.
        if data.len() > 8 {
            return None;
        }

        let mut d: [u8; 8] = [0; 8];
        let (left, _unused) = d.split_at_mut(data.len());
        left.clone_from_slice(data);

        Some(EspTwaiFrame {
            id,
            data: d,
            dlc: data.len(),
            is_remote: false,
            self_reception: false,
        })
    }

    pub fn new_remote(id: Id, dlc: usize) -> Option<Self> {
        // CAN2.0 frames cannot have more than 8 bytes.
        if dlc > 8 {
            return None;
        }

        Some(EspTwaiFrame {
            id,
            data: [0; 8],
            dlc,
            is_remote: true,
            self_reception: false,
        })
    }

    pub fn new_self_reception(id: Id, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }

        let mut d: [u8; 8] = [0; 8];
        let (left, _unused) = d.split_at_mut(data.len());
        left.clone_from_slice(data);

        Some(EspTwaiFrame {
            id,
            data: d,
            dlc: data.len(),
            is_remote: false,
            self_reception: true,
        })
    }

    /// Make a new frame from an id, pointer to the TWAI_DATA_x_REG registers,
    /// and the length of the data payload (dlc).
    ///
    /// # Safety
    /// This is unsafe because it directly accesses peripheral registers.
    unsafe fn new_from_data_registers(
        id: impl Into<Id>,
        registers: *const u32,
        dlc: usize,
    ) -> Self {
        let mut data: [u8; 8] = [0; 8];

        // Copy the data from the memory mapped peripheral into actual memory.
        copy_from_data_register(&mut data[..dlc], registers);

        Self {
            id: id.into(),
            data,
            dlc,
            is_remote: false,
            self_reception: true,
        }
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::can::Frame for EspTwaiFrame {
    fn new(id: impl Into<embedded_hal_02::can::Id>, data: &[u8]) -> Option<Self> {
        let id: embedded_hal_02::can::Id = id.into();
        Self::new(id.into(), data)
    }

    fn new_remote(id: impl Into<embedded_hal_02::can::Id>, dlc: usize) -> Option<Self> {
        let id: embedded_hal_02::can::Id = id.into();
        Self::new_remote(id.into(), dlc)
    }

    fn is_extended(&self) -> bool {
        match self.id {
            Id::Standard(_) => false,
            Id::Extended(_) => true,
        }
    }

    fn is_remote_frame(&self) -> bool {
        self.is_remote
    }

    fn id(&self) -> embedded_hal_02::can::Id {
        self.id.into()
    }

    fn dlc(&self) -> usize {
        self.dlc
    }

    fn data(&self) -> &[u8] {
        // Remote frames do not contain data, yet have a value for the dlc so return
        // an empty slice for remote frames.
        match self.is_remote {
            true => &[],
            false => &self.data[0..self.dlc],
        }
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_can::Frame for EspTwaiFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        let id: embedded_can::Id = id.into();
        Self::new(id.into(), data)
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        let id: embedded_can::Id = id.into();
        Self::new_remote(id.into(), dlc)
    }

    fn is_extended(&self) -> bool {
        match self.id {
            Id::Standard(_) => false,
            Id::Extended(_) => true,
        }
    }

    fn is_remote_frame(&self) -> bool {
        self.is_remote
    }

    fn id(&self) -> embedded_can::Id {
        self.id.into()
    }

    fn dlc(&self) -> usize {
        self.dlc
    }

    fn data(&self) -> &[u8] {
        // Remote frames do not contain data, yet have a value for the dlc so return
        // an empty slice for remote frames.
        match self.is_remote {
            true => &[],
            false => &self.data[0..self.dlc],
        }
    }
}

/// The underlying timings for the TWAI peripheral.
pub struct TimingConfig {
    pub baud_rate_prescaler: u16,
    pub sync_jump_width: u8,
    pub tseg_1: u8,
    pub tseg_2: u8,
    pub triple_sample: bool,
}

/// A selection of pre-determined baudrates for the TWAI driver.
/// Currently these timings are sourced from the ESP IDF C driver which assumes
/// an APB clock of 80MHz.
pub enum BaudRate {
    B125K,
    B250K,
    B500K,
    B1000K,
    Custom(TimingConfig),
}

impl BaudRate {
    /// Convert the BaudRate into the timings that the peripheral needs.
    // These timings are copied from the ESP IDF C driver.
    // #define TWAI_TIMING_CONFIG_25KBITS()    {.brp = 128, .tseg_1 = 16, .tseg_2 =
    // 8, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_50KBITS()    {.brp = 80, .tseg_1 = 15, .tseg_2 =
    // 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_100KBITS()   {.brp = 40, .tseg_1 = 15, .tseg_2 =
    // 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_125KBITS()   {.brp = 32, .tseg_1 = 15, .tseg_2 =
    // 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_250KBITS()   {.brp = 16, .tseg_1 = 15, .tseg_2 =
    // 4, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_500KBITS()   {.brp = 8, .tseg_1 = 15, .tseg_2 = 4,
    // .sjw = 3, .triple_sampling = false} #define TWAI_TIMING_CONFIG_800KBITS()
    // {.brp = 4, .tseg_1 = 16, .tseg_2 = 8, .sjw = 3, .triple_sampling = false}
    // #define TWAI_TIMING_CONFIG_1MBITS()     {.brp = 4, .tseg_1 = 15, .tseg_2 = 4,
    // .sjw = 3, .triple_sampling = false}
    //
    // see https://github.com/espressif/esp-idf/tree/master/components/hal/include/hal/twai_types.h
    const fn timing(self) -> TimingConfig {
        #[allow(unused_mut)]
        let mut timing = match self {
            Self::B125K => TimingConfig {
                baud_rate_prescaler: 32,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B250K => TimingConfig {
                baud_rate_prescaler: 16,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B500K => TimingConfig {
                baud_rate_prescaler: 8,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B1000K => TimingConfig {
                baud_rate_prescaler: 4,
                sync_jump_width: 3,
                tseg_1: 15,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::Custom(timing_config) => timing_config,
        };

        #[cfg(esp32c6)]
        {
            // clock source on ESP32-C6 is xtal (40MHz)
            timing.baud_rate_prescaler /= 2;
        }

        timing
    }
}

/// An inactive TWAI peripheral in the "Reset"/configuration state.
pub struct TwaiConfiguration<'d, T, DM: crate::Mode> {
    peripheral: PhantomData<&'d PeripheralRef<'d, T>>,
    phantom: PhantomData<DM>,
}

impl<'d, T, DM> TwaiConfiguration<'d, T, DM>
where
    T: Instance,
    DM: crate::Mode,
{
    fn new_internal<TX: OutputPin, RX: InputPin>(
        _peripheral: impl Peripheral<P = T> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        rx_pin: impl Peripheral<P = RX> + 'd,
        clocks: &Clocks<'d>,
        baud_rate: BaudRate,
        no_transceiver: bool,
        mode: TwaiMode,
    ) -> Self {
        // Enable the peripheral clock for the TWAI peripheral.
        T::enable_peripheral();

        // Set RESET bit to 1
        T::register_block()
            .mode()
            .write(|w| w.reset_mode().set_bit());

        // Set up the GPIO pins.
        crate::into_ref!(tx_pin, rx_pin);
        if no_transceiver {
            tx_pin.set_to_open_drain_output(crate::private::Internal);
        }
        tx_pin.set_to_push_pull_output(crate::private::Internal);
        tx_pin.connect_peripheral_to_output(T::OUTPUT_SIGNAL, crate::private::Internal);
        rx_pin.set_to_input(crate::private::Internal);
        rx_pin.connect_input_to_peripheral(T::INPUT_SIGNAL, crate::private::Internal);

        // Set the operating mode based on provided option
        match mode {
            TwaiMode::Normal => {
                // Do nothing special, the default state is Normal mode.
            }
            TwaiMode::SelfTest => {
                // Set the self-test mode (no acknowledgement required)
                T::register_block()
                    .mode()
                    .modify(|_, w| w.self_test_mode().set_bit());
            }
            TwaiMode::ListenOnly => {
                // Set listen-only mode
                T::register_block()
                    .mode()
                    .modify(|_, w| w.listen_only_mode().set_bit());
            }
        }

        // Set TEC to 0
        T::register_block()
            .tx_err_cnt()
            .write(|w| unsafe { w.tx_err_cnt().bits(0) });

        // Set REC to 0
        T::register_block()
            .rx_err_cnt()
            .write(|w| unsafe { w.rx_err_cnt().bits(0) });

        // Set EWL to 96
        T::register_block()
            .err_warning_limit()
            .write(|w| unsafe { w.err_warning_limit().bits(96) });

        let mut cfg = TwaiConfiguration {
            peripheral: PhantomData,
            phantom: PhantomData,
        };

        cfg.set_baud_rate(baud_rate, clocks);
        cfg
    }

    fn internal_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(T::INTERRUPT, handler.handler());
            crate::interrupt::enable(T::INTERRUPT, handler.priority()).unwrap();
        }
    }

    /// Set the bitrate of the bus.
    ///
    /// Note: The timings currently assume a APB_CLK of 80MHz.
    fn set_baud_rate(&mut self, baud_rate: BaudRate, _clocks: &Clocks<'d>) {
        // TWAI is clocked from the APB_CLK according to Table 6-4 [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf)
        // Included timings are all for 80MHz so assert that we are running at 80MHz.
        #[cfg(not(esp32c6))]
        assert!(_clocks.apb_clock == fugit::HertzU32::MHz(80));

        // Unpack the baud rate timings and convert them to the values needed for the
        // register. Many of the registers have a minimum value of 1 which is
        // represented by having zero bits set, therefore many values need to
        // have 1 subtracted from them before being stored into the register.
        let timing = baud_rate.timing();

        let prescale = (timing.baud_rate_prescaler / 2) - 1;
        let sjw = timing.sync_jump_width - 1;
        let tseg_1 = timing.tseg_1 - 1;
        let tseg_2 = timing.tseg_2 - 1;
        let triple_sample = timing.triple_sample;

        #[cfg(esp32)]
        let prescale = prescale as u8;

        // Set up the prescaler and sync jump width.
        T::register_block()
            .bus_timing_0()
            .modify(|_, w| unsafe { w.baud_presc().bits(prescale).sync_jump_width().bits(sjw) });

        // Set up the time segment 1, time segment 2, and triple sample.
        T::register_block().bus_timing_1().modify(|_, w| unsafe {
            w.time_seg1()
                .bits(tseg_1)
                .time_seg2()
                .bits(tseg_2)
                .time_samp()
                .bit(triple_sample)
        });
    }

    /// Set up the acceptance filter on the device.
    ///
    /// NOTE: On a bus with mixed 11-bit and 29-bit packet id's, you may
    /// experience an 11-bit filter match against a 29-bit frame and vice
    /// versa. Your application should check the id again once a frame has
    /// been received to make sure it is the expected value.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6)
    pub fn set_filter(&mut self, filter: impl Filter) {
        // Set or clear the rx filter mode bit depending on the filter type.
        let filter_mode_bit = filter.filter_type() == FilterType::Single;
        T::register_block()
            .mode()
            .modify(|_, w| w.rx_filter_mode().bit(filter_mode_bit));

        // Convert the filter into values for the registers and store them to the
        // registers.
        let registers = filter.to_registers();

        // Copy the filter to the peripheral.
        unsafe {
            copy_to_data_register(T::register_block().data_0().as_ptr(), &registers);
        }
    }

    /// Set the error warning threshold.
    ///
    /// In the case when any of an error counter value exceeds the threshold, or
    /// all the error counter values are below the threshold, an error
    /// warning interrupt will be triggered (given the enable signal is
    /// valid).
    pub fn set_error_warning_limit(&mut self, limit: u8) {
        T::register_block()
            .err_warning_limit()
            .write(|w| unsafe { w.err_warning_limit().bits(limit) });
    }

    /// Put the peripheral into Operation Mode, allowing the transmission and
    /// reception of packets using the new object.
    pub fn start(self) -> Twai<'d, T, DM> {
        // Put the peripheral into operation mode by clearing the reset mode bit.
        T::register_block()
            .mode()
            .modify(|_, w| w.reset_mode().clear_bit());

        Twai {
            tx: TwaiTx {
                _peripheral: PhantomData,
                phantom: PhantomData,
            },
            rx: TwaiRx {
                _peripheral: PhantomData,
                phantom: PhantomData,
            },
            phantom: PhantomData,
        }
    }
}

impl<'d, T> TwaiConfiguration<'d, T, crate::Blocking>
where
    T: Instance,
{
    /// Create a new instance of [TwaiConfiguration]
    ///
    /// You will need to use a transceiver to connect to the TWAI bus
    pub fn new<TX: OutputPin, RX: InputPin>(
        peripheral: impl Peripheral<P = T> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        rx_pin: impl Peripheral<P = RX> + 'd,
        clocks: &Clocks<'d>,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        Self::new_internal(peripheral, tx_pin, rx_pin, clocks, baud_rate, false, mode)
    }

    /// Create a new instance of [TwaiConfiguration] meant to connect two ESP32s
    /// directly
    ///
    /// You don't need a transceiver by following the description in the
    /// `twai.rs` example
    pub fn new_no_transceiver<TX: OutputPin, RX: InputPin>(
        peripheral: impl Peripheral<P = T> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        rx_pin: impl Peripheral<P = RX> + 'd,
        clocks: &Clocks<'d>,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        Self::new_internal(peripheral, tx_pin, rx_pin, clocks, baud_rate, true, mode)
    }
}

impl<'d, T> crate::private::Sealed for TwaiConfiguration<'d, T, crate::Blocking> where T: Instance {}

impl<'d, T> InterruptConfigurable for TwaiConfiguration<'d, T, crate::Blocking>
where
    T: Instance,
{
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.internal_set_interrupt_handler(handler);
    }
}

#[cfg(feature = "async")]
impl<'d, T> TwaiConfiguration<'d, T, crate::Async>
where
    T: Instance,
{
    /// Create a new instance of [TwaiConfiguration] in async mode
    ///
    /// You will need to use a transceiver to connect to the TWAI bus
    pub fn new_async<TX: OutputPin, RX: InputPin>(
        peripheral: impl Peripheral<P = T> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        rx_pin: impl Peripheral<P = RX> + 'd,
        clocks: &Clocks<'d>,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        let mut this =
            Self::new_internal(peripheral, tx_pin, rx_pin, clocks, baud_rate, false, mode);
        this.internal_set_interrupt_handler(T::async_handler());
        this
    }

    /// Create a new instance of [TwaiConfiguration] meant to connect two ESP32s
    /// directly in async mode
    ///
    /// You don't need a transceiver by following the description in the
    /// `twai.rs` example
    pub fn new_async_no_transceiver<TX: OutputPin, RX: InputPin>(
        peripheral: impl Peripheral<P = T> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        rx_pin: impl Peripheral<P = RX> + 'd,
        clocks: &Clocks<'d>,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        let mut this =
            Self::new_internal(peripheral, tx_pin, rx_pin, clocks, baud_rate, true, mode);
        this.internal_set_interrupt_handler(T::async_handler());
        this
    }
}

/// An active TWAI peripheral in Normal Mode.
///
/// In this mode, the TWAI controller can transmit and receive messages
/// including error signals (such as error and overload frames).
pub struct Twai<'d, T, DM: crate::Mode> {
    tx: TwaiTx<'d, T, DM>,
    rx: TwaiRx<'d, T, DM>,
    phantom: PhantomData<DM>,
}

impl<'d, T, DM> Twai<'d, T, DM>
where
    T: OperationInstance,
    DM: crate::Mode,
{
    /// Stop the peripheral, putting it into reset mode and enabling
    /// reconfiguration.
    pub fn stop(self) -> TwaiConfiguration<'d, T, DM> {
        // Put the peripheral into reset/configuration mode by setting the reset mode
        // bit.
        T::register_block()
            .mode()
            .modify(|_, w| w.reset_mode().set_bit());

        TwaiConfiguration {
            peripheral: PhantomData,
            phantom: PhantomData,
        }
    }

    pub fn receive_error_count(&self) -> u8 {
        T::register_block().rx_err_cnt().read().rx_err_cnt().bits()
    }

    pub fn transmit_error_count(&self) -> u8 {
        T::register_block().tx_err_cnt().read().tx_err_cnt().bits()
    }

    /// Check if the controller is in a bus off state.
    pub fn is_bus_off(&self) -> bool {
        T::register_block()
            .status()
            .read()
            .bus_off_st()
            .bit_is_set()
    }

    /// Get the number of messages that the peripheral has available in the
    /// receive FIFO.
    ///
    /// Note that this may not be the number of valid messages in the receive
    /// FIFO due to fifo overflow/overrun.
    pub fn num_available_messages(&self) -> u8 {
        T::register_block()
            .rx_message_cnt()
            .read()
            .rx_message_counter()
            .bits()
    }

    /// Clear the receive FIFO, discarding any valid, partial, or invalid
    /// packets.
    ///
    /// This is typically used to clear an overrun receive FIFO.
    ///
    /// TODO: Not sure if this needs to be guarded against Bus Off or other
    /// error states.
    pub fn clear_receive_fifo(&self) {
        while self.num_available_messages() > 0 {
            T::release_receive_fifo();
        }
    }

    pub fn transmit(&mut self, frame: &EspTwaiFrame) -> nb::Result<(), EspTwaiError> {
        self.tx.transmit(frame)
    }

    pub fn receive(&mut self) -> nb::Result<EspTwaiFrame, EspTwaiError> {
        self.rx.receive()
    }

    /// Consumes this `Twai` instance and splits it into transmitting and
    /// receiving halves.
    pub fn split(self) -> (TwaiTx<'d, T, DM>, TwaiRx<'d, T, DM>) {
        (self.tx, self.rx)
    }
}

/// Interface to the CAN transmitter part.
pub struct TwaiTx<'d, T, DM: crate::Mode> {
    _peripheral: PhantomData<&'d T>,
    phantom: PhantomData<DM>,
}

impl<'d, T, DM> TwaiTx<'d, T, DM>
where
    T: OperationInstance,
    DM: crate::Mode,
{
    /// Transmit a frame.
    ///
    /// Because of how the TWAI registers are set up, we have to do some
    /// assembly of bytes. Note that these registers serve a filter
    /// configuration role when the device is in configuration mode so
    /// patching the svd files to improve this may be non-trivial.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.4.2)
    ///
    /// NOTE: TODO: This may not work if using the self reception/self test
    /// functionality. See notes 1 and 2 in the "Frame Identifier" section
    /// of the reference manual.
    pub fn transmit(&mut self, frame: &EspTwaiFrame) -> nb::Result<(), EspTwaiError> {
        let register_block = T::register_block();
        let status = register_block.status().read();

        // Check that the peripheral is not in a bus off state.
        if status.bus_off_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(EspTwaiError::BusOff));
        }
        // Check that the peripheral is not already transmitting a packet.
        if !status.tx_buf_st().bit_is_set() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        T::write_frame(frame);

        Ok(())
    }
}

/// Interface to the CAN receiver part.
pub struct TwaiRx<'d, T, DM: crate::Mode> {
    _peripheral: PhantomData<&'d T>,
    phantom: PhantomData<DM>,
}

impl<'d, T, DM> TwaiRx<'d, T, DM>
where
    T: OperationInstance,
    DM: crate::Mode,
{
    // Receive a frame
    pub fn receive(&mut self) -> nb::Result<EspTwaiFrame, EspTwaiError> {
        let register_block = T::register_block();
        let status = register_block.status().read();

        // Check that the peripheral is not in a bus off state.
        if status.bus_off_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(EspTwaiError::BusOff));
        }

        // Check that we actually have packets to receive.
        if !status.rx_buf_st().bit_is_set() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        // Check if the packet in the receive buffer is valid or overrun.
        if status.miss_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(EspTwaiError::EmbeddedHAL(
                ErrorKind::Overrun,
            )));
        }

        // Safety:
        // - We have a `&mut self` and have unique access to the peripheral.
        // - There is a message in the FIFO because we checked status
        let frame = T::read_frame();

        Ok(frame)
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum EspTwaiError {
    BusOff,
    EmbeddedHAL(ErrorKind),
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::can::Error for EspTwaiError {
    fn kind(&self) -> embedded_hal_02::can::ErrorKind {
        match self {
            Self::BusOff => embedded_hal_02::can::ErrorKind::Other,
            Self::EmbeddedHAL(kind) => (*kind).into(),
        }
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_can::Error for EspTwaiError {
    fn kind(&self) -> embedded_can::ErrorKind {
        match self {
            Self::BusOff => embedded_can::ErrorKind::Other,
            Self::EmbeddedHAL(kind) => (*kind).into(),
        }
    }
}

/// Copy data from multiple TWAI_DATA_x_REG registers, packing the source into
/// the destination.
///
/// # Safety
/// This function is marked unsafe because it reads arbitrarily from
/// memory-mapped registers. Specifically, this function is used with the
/// TWAI_DATA_x_REG registers which has different results based on the mode of
/// the peripheral.
#[inline(always)]
unsafe fn copy_from_data_register(dest: &mut [u8], src: *const u32) {
    for (i, dest) in dest.iter_mut().enumerate() {
        // Perform a volatile read to avoid compiler optimizations.
        *dest = src.add(i).read_volatile() as u8;
    }
}

/// Copy data to multiple TWAI_DATA_x_REG registers, unpacking the source into
/// the destination.
///
/// # Safety
/// This function is marked unsafe because it writes arbitrarily to
/// memory-mapped registers. Specifically, this function is used with the
/// TWAI_DATA_x_REG registers which has different results based on the mode of
/// the peripheral.
#[inline(always)]
unsafe fn copy_to_data_register(dest: *mut u32, src: &[u8]) {
    for (i, src) in src.iter().enumerate() {
        // Perform a volatile write to avoid compiler optimizations.
        dest.add(i).write_volatile(*src as u32);
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<'d, T, DM> embedded_hal_02::can::Can for Twai<'d, T, DM>
where
    T: OperationInstance,
    DM: crate::Mode,
{
    type Frame = EspTwaiFrame;
    type Error = EspTwaiError;

    /// Transmit a frame.
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        self.tx.transmit(frame)?;

        // Success in readying packet for transmit. No packets can be replaced in the
        // transmit buffer so return None in accordance with the
        // embedded-can/embedded-hal trait.
        nb::Result::Ok(None)
    }

    /// Return a received frame if there are any available.
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.rx.receive()
    }
}

#[cfg(feature = "embedded-hal")]
impl<'d, T, DM> embedded_can::nb::Can for Twai<'d, T, DM>
where
    T: OperationInstance,
    DM: crate::Mode,
{
    type Frame = EspTwaiFrame;
    type Error = EspTwaiError;

    /// Transmit a frame.
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        self.tx.transmit(frame)?;

        // Success in readying packet for transmit. No packets can be replaced in the
        // transmit buffer so return None in accordance with the
        // embedded-can/embedded-hal trait.
        nb::Result::Ok(None)
    }

    /// Return a received frame if there are any available.
    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.rx.receive()
    }
}

pub trait Instance: crate::private::Sealed {
    const SYSTEM_PERIPHERAL: system::Peripheral;
    const NUMBER: usize;

    const INPUT_SIGNAL: InputSignal;
    const OUTPUT_SIGNAL: OutputSignal;

    const INTERRUPT: crate::peripherals::Interrupt;
    #[cfg(feature = "async")]
    fn async_handler() -> InterruptHandler;

    fn register_block() -> &'static RegisterBlock;

    fn enable_peripheral();

    fn enable_interrupts();
}

pub trait OperationInstance: Instance {
    #[cfg(feature = "async")]
    fn async_state() -> &'static asynch::TwaiAsyncState {
        &asynch::TWAI_STATE[Self::NUMBER]
    }

    /// Release the message in the buffer. This will decrement the received
    /// message counter and prepare the next message in the FIFO for
    /// reading.
    fn release_receive_fifo() {
        Self::register_block()
            .cmd()
            .write(|w| w.release_buf().set_bit());
    }

    /// Write a frame to the peripheral.
    fn write_frame(frame: &EspTwaiFrame) {
        // Assemble the frame information into the data_0 byte.
        let frame_format: u8 = matches!(frame.id, Id::Extended(_)) as u8;
        let rtr_bit: u8 = frame.is_remote as u8;
        let dlc_bits: u8 = frame.dlc as u8 & 0b1111;

        let data_0: u8 = frame_format << 7 | rtr_bit << 6 | dlc_bits;

        let register_block = Self::register_block();

        register_block
            .data_0()
            .write(|w| unsafe { w.tx_byte_0().bits(data_0) });

        // Assemble the identifier information of the packet.
        match frame.id {
            Id::Standard(id) => {
                let id = id.as_raw();

                register_block
                    .data_1()
                    .write(|w| unsafe { w.tx_byte_1().bits((id >> 3) as u8) });

                register_block
                    .data_2()
                    .write(|w| unsafe { w.tx_byte_2().bits((id << 5) as u8) });
            }
            Id::Extended(id) => {
                let id = id.as_raw();

                register_block
                    .data_1()
                    .write(|w| unsafe { w.tx_byte_1().bits((id >> 21) as u8) });
                register_block
                    .data_2()
                    .write(|w| unsafe { w.tx_byte_2().bits((id >> 13) as u8) });
                register_block
                    .data_3()
                    .write(|w| unsafe { w.tx_byte_3().bits((id >> 5) as u8) });
                register_block
                    .data_4()
                    .write(|w| unsafe { w.tx_byte_4().bits((id << 3) as u8) });
            }
        }

        // Store the data portion of the packet into the transmit buffer.
        if !frame.is_remote {
            match frame.id {
                Id::Standard(_) => unsafe {
                    copy_to_data_register(
                        register_block.data_3().as_ptr(),
                        match frame.is_remote {
                            true => &[],
                            false => &frame.data[0..frame.dlc],
                        },
                    )
                },
                Id::Extended(_) => unsafe {
                    copy_to_data_register(
                        register_block.data_5().as_ptr(),
                        match frame.is_remote {
                            true => &[],
                            false => &frame.data[0..frame.dlc],
                        },
                    )
                },
            }
        } else {
            // Is RTR frame, so no data is included.
        }

        // Trigger the appropriate transmission request based on self_reception flag
        if frame.self_reception {
            #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
            register_block.cmd().write(|w| w.self_rx_req().set_bit());
            #[cfg(not(any(esp32, esp32c3, esp32s2, esp32s3)))]
            register_block
                .cmd()
                .write(|w| w.self_rx_request().set_bit());
        } else {
            // Set the transmit request command, this will lock the transmit buffer until
            // the transmission is complete or aborted.
            register_block.cmd().write(|w| w.tx_req().set_bit());
        }
    }

    /// Read a frame from the peripheral.
    fn read_frame() -> EspTwaiFrame {
        let register_block = Self::register_block();

        // Read the frame information and extract the frame id format and dlc.
        let data_0 = register_block.data_0().read().tx_byte_0().bits();

        let is_standard_format = data_0 & 0b1 << 7 == 0;
        let is_data_frame = data_0 & 0b1 << 6 == 0;
        let dlc = (data_0 & 0b1111) as usize;

        // Read the payload from the packet and construct a frame.
        let frame = if is_standard_format {
            // Frame uses standard 11 bit id.
            let data_1 = register_block.data_1().read().tx_byte_1().bits();

            let data_2 = register_block.data_2().read().tx_byte_2().bits();

            let raw_id: u16 = ((data_1 as u16) << 3) | ((data_2 as u16) >> 5);

            let id = StandardId::new(raw_id).unwrap();

            if is_data_frame {
                // Create a new frame from the contents of the appropriate TWAI_DATA_x_REG
                // registers.
                unsafe {
                    EspTwaiFrame::new_from_data_registers(id, register_block.data_3().as_ptr(), dlc)
                }
            } else {
                EspTwaiFrame::new_remote(id.into(), dlc).unwrap()
            }
        } else {
            // Frame uses extended 29 bit id.
            let data_1 = register_block.data_1().read().tx_byte_1().bits();

            let data_2 = register_block.data_2().read().tx_byte_2().bits();

            let data_3 = register_block.data_3().read().tx_byte_3().bits();

            let data_4 = register_block.data_4().read().tx_byte_4().bits();

            let raw_id: u32 = (data_1 as u32) << 21
                | (data_2 as u32) << 13
                | (data_3 as u32) << 5
                | (data_4 as u32) >> 3;

            let id = ExtendedId::new(raw_id).unwrap();

            if is_data_frame {
                unsafe {
                    EspTwaiFrame::new_from_data_registers(id, register_block.data_5().as_ptr(), dlc)
                }
            } else {
                EspTwaiFrame::new_remote(id.into(), dlc).unwrap()
            }
        };

        // Release the packet we read from the FIFO, allowing the peripheral to prepare
        // the next packet.
        register_block.cmd().write(|w| w.release_buf().set_bit());

        frame
    }
}

#[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
impl Instance for crate::peripherals::TWAI0 {
    const SYSTEM_PERIPHERAL: system::Peripheral = system::Peripheral::Twai0;
    const NUMBER: usize = 0;

    const INPUT_SIGNAL: InputSignal = InputSignal::TWAI_RX;
    const OUTPUT_SIGNAL: OutputSignal = OutputSignal::TWAI_TX;

    const INTERRUPT: crate::peripherals::Interrupt = crate::peripherals::Interrupt::TWAI0;

    #[cfg(feature = "async")]
    fn async_handler() -> InterruptHandler {
        asynch::twai0
    }

    #[inline(always)]
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*crate::peripherals::TWAI0::PTR }
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Twai0);
    }

    fn enable_interrupts() {
        let register_block = Self::register_block();
        register_block.int_ena().modify(|_, w| {
            w.rx_int_ena()
                .set_bit()
                .tx_int_ena()
                .set_bit()
                .bus_err_int_ena()
                .set_bit()
                .arb_lost_int_ena()
                .set_bit()
                .err_passive_int_ena()
                .set_bit()
        });
    }
}

#[cfg(any(esp32, esp32c3, esp32s2, esp32s3))]
impl OperationInstance for crate::peripherals::TWAI0 {}

#[cfg(esp32c6)]
impl Instance for crate::peripherals::TWAI0 {
    const SYSTEM_PERIPHERAL: system::Peripheral = system::Peripheral::Twai0;
    const NUMBER: usize = 0;

    const INPUT_SIGNAL: InputSignal = InputSignal::TWAI0_RX;
    const OUTPUT_SIGNAL: OutputSignal = OutputSignal::TWAI0_TX;

    const INTERRUPT: crate::peripherals::Interrupt = crate::peripherals::Interrupt::TWAI0;

    #[cfg(feature = "async")]
    fn async_handler() -> InterruptHandler {
        asynch::twai0
    }

    #[inline(always)]
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*crate::peripherals::TWAI0::PTR }
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Twai0);
    }

    fn enable_interrupts() {
        let register_block = Self::register_block();
        register_block.interrupt_enable().modify(|_, w| {
            w.ext_receive_int_ena()
                .set_bit()
                .ext_transmit_int_ena()
                .set_bit()
                .bus_err_int_ena()
                .set_bit()
                .arbitration_lost_int_ena()
                .set_bit()
                .err_passive_int_ena()
                .set_bit()
        });
    }
}

#[cfg(esp32c6)]
impl OperationInstance for crate::peripherals::TWAI0 {}

#[cfg(esp32c6)]
impl Instance for crate::peripherals::TWAI1 {
    const SYSTEM_PERIPHERAL: system::Peripheral = system::Peripheral::Twai1;
    const NUMBER: usize = 1;

    const INPUT_SIGNAL: InputSignal = InputSignal::TWAI1_RX;
    const OUTPUT_SIGNAL: OutputSignal = OutputSignal::TWAI1_TX;

    const INTERRUPT: crate::peripherals::Interrupt = crate::peripherals::Interrupt::TWAI1;

    #[cfg(feature = "async")]
    fn async_handler() -> InterruptHandler {
        asynch::twai1
    }

    #[inline(always)]
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*crate::peripherals::TWAI1::PTR }
    }

    fn enable_peripheral() {
        PeripheralClockControl::enable(crate::system::Peripheral::Twai1);
    }

    fn enable_interrupts() {
        let register_block = Self::register_block();
        register_block.interrupt_enable().modify(|_, w| {
            w.ext_receive_int_ena()
                .set_bit()
                .ext_transmit_int_ena()
                .set_bit()
                .bus_err_int_ena()
                .set_bit()
                .arbitration_lost_int_ena()
                .set_bit()
                .err_passive_int_ena()
                .set_bit()
        });
    }
}

#[cfg(esp32c6)]
impl OperationInstance for crate::peripherals::TWAI1 {}

#[cfg(feature = "async")]
mod asynch {
    use core::{future::poll_fn, task::Poll};

    use embassy_sync::{
        blocking_mutex::raw::CriticalSectionRawMutex,
        channel::Channel,
        waitqueue::AtomicWaker,
    };
    use procmacros::handler;

    use super::*;
    use crate::peripherals::TWAI0;
    #[cfg(esp32c6)]
    use crate::peripherals::TWAI1;

    pub struct TwaiAsyncState {
        pub tx_waker: AtomicWaker,
        pub err_waker: AtomicWaker,
        pub rx_queue: Channel<CriticalSectionRawMutex, Result<EspTwaiFrame, EspTwaiError>, 32>,
    }

    impl Default for TwaiAsyncState {
        fn default() -> Self {
            Self::new()
        }
    }

    impl TwaiAsyncState {
        pub const fn new() -> Self {
            Self {
                tx_waker: AtomicWaker::new(),
                err_waker: AtomicWaker::new(),
                rx_queue: Channel::new(),
            }
        }
    }

    const NUM_TWAI: usize = 2;
    #[allow(clippy::declare_interior_mutable_const)]
    const NEW_STATE: TwaiAsyncState = TwaiAsyncState::new();
    pub(crate) static TWAI_STATE: [TwaiAsyncState; NUM_TWAI] = [NEW_STATE; NUM_TWAI];

    impl<T> Twai<'_, T, crate::Async>
    where
        T: OperationInstance,
    {
        pub async fn transmit_async(&mut self, frame: &EspTwaiFrame) -> Result<(), EspTwaiError> {
            self.tx.transmit_async(frame).await
        }

        pub async fn receive_async(&mut self) -> Result<EspTwaiFrame, EspTwaiError> {
            self.rx.receive_async().await
        }
    }

    impl<'d, T> TwaiTx<'d, T, crate::Async>
    where
        T: OperationInstance,
    {
        pub async fn transmit_async(&mut self, frame: &EspTwaiFrame) -> Result<(), EspTwaiError> {
            T::enable_interrupts();
            poll_fn(|cx| {
                T::async_state().tx_waker.register(cx.waker());

                let register_block = T::register_block();
                let status = register_block.status().read();

                // Check that the peripheral is not in a bus off state.
                if status.bus_off_st().bit_is_set() {
                    return Poll::Ready(Err(EspTwaiError::BusOff));
                }
                // Check that the peripheral is not already transmitting a packet.
                if !status.tx_buf_st().bit_is_set() {
                    return Poll::Pending;
                }

                T::write_frame(frame);

                Poll::Ready(Ok(()))
            })
            .await
        }
    }

    impl<'d, T> TwaiRx<'d, T, crate::Async>
    where
        T: OperationInstance,
    {
        pub async fn receive_async(&mut self) -> Result<EspTwaiFrame, EspTwaiError> {
            T::enable_interrupts();
            poll_fn(|cx| {
                T::async_state().err_waker.register(cx.waker());

                if let Poll::Ready(result) = T::async_state().rx_queue.poll_receive(cx) {
                    return Poll::Ready(result);
                } else {
                    let register_block = T::register_block();
                    let status = register_block.status().read();

                    // Check that the peripheral is not in a bus off state.
                    if status.bus_off_st().bit_is_set() {
                        return Poll::Ready(Err(EspTwaiError::BusOff));
                    }

                    // Check if the packet in the receive buffer is valid or overrun.
                    if status.miss_st().bit_is_set() {
                        return Poll::Ready(Err(EspTwaiError::EmbeddedHAL(ErrorKind::Overrun)));
                    }
                }

                Poll::Pending
            })
            .await
        }
    }

    #[cfg(any(esp32c3, esp32, esp32s2, esp32s3))]
    #[handler]
    pub(super) fn twai0() {
        let register_block = TWAI0::register_block();

        let intr_enable = register_block.int_ena().read();
        let intr_status = register_block.int_raw().read();

        let async_state = TWAI0::async_state();

        if intr_status.tx_int_st().bit_is_set() {
            async_state.tx_waker.wake();
        }

        if intr_status.rx_int_st().bit_is_set() {
            let status = register_block.status().read();

            let rx_queue = &async_state.rx_queue;

            if status.bus_off_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::BusOff));
            }

            if status.miss_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::EmbeddedHAL(ErrorKind::Overrun)));
            }

            let frame = TWAI0::read_frame();

            let _ = rx_queue.try_send(Ok(frame));

            register_block.cmd().write(|w| w.release_buf().set_bit());
        }

        if intr_status.bits() & 0b11111100 > 0 {
            async_state.err_waker.wake();
        }

        unsafe {
            register_block
                .int_ena()
                .modify(|_, w| w.bits(intr_enable.bits() & (!intr_status.bits() | 1)));
        }
    }

    #[cfg(esp32c6)]
    #[handler]
    pub(super) fn twai0() {
        let register_block = TWAI0::register_block();

        let intr_enable = register_block.interrupt_enable().read();
        let intr_status = register_block.interrupt().read();

        let async_state = TWAI0::async_state();

        if intr_status.transmit_int_st().bit_is_set() {
            async_state.tx_waker.wake();
        }

        if intr_status.receive_int_st().bit_is_set() {
            let status = register_block.status().read();

            let rx_queue = &async_state.rx_queue;

            if status.bus_off_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::BusOff));
            }

            if status.miss_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::EmbeddedHAL(ErrorKind::Overrun)));
            }

            let frame = TWAI0::read_frame();

            let _ = rx_queue.try_send(Ok(frame));

            register_block.cmd().write(|w| w.release_buf().set_bit());
        }

        if intr_status.bits() & 0b11111100 > 0 {
            async_state.err_waker.wake();
        }

        unsafe {
            register_block
                .interrupt_enable()
                .modify(|_, w| w.bits(intr_enable.bits() & (!intr_status.bits() | 1)));
        }
    }

    #[cfg(esp32c6)]
    #[handler]
    pub(super) fn twai1() {
        let register_block = TWAI1::register_block();

        let intr_enable = register_block.interrupt_enable().read();
        let intr_status = register_block.interrupt().read();

        let async_state = TWAI1::async_state();

        if intr_status.transmit_int_st().bit_is_set() {
            async_state.tx_waker.wake();
        }

        if intr_status.receive_int_st().bit_is_set() {
            let status = register_block.status().read();

            let rx_queue = &async_state.rx_queue;

            if status.bus_off_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::BusOff));
            }

            if status.miss_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::EmbeddedHAL(ErrorKind::Overrun)));
            }

            let frame = TWAI1::read_frame();

            let _ = rx_queue.try_send(Ok(frame));

            register_block.cmd().write(|w| w.release_buf().set_bit());
        }

        if intr_status.bits() & 0b11111100 > 0 {
            async_state.err_waker.wake();
        }

        unsafe {
            register_block
                .interrupt_enable()
                .modify(|_, w| w.bits(intr_enable.bits() & (!intr_status.bits() | 1)));
        }
    }
}
