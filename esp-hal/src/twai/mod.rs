//! # Two-wire Automotive Interface (TWAI)
//!
//! ## Overview
//!
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
//! This driver manages the ISO 11898-1 compatible TWAI
//! controllers. It supports Standard Frame Format (11-bit) and Extended Frame
//! Format (29-bit) frame identifiers.
//!
//! ## Examples
//!
//! ### Transmitting and Receiving Messages
//!
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::twai;
//! # use esp_hal::twai::filter;
//! # use esp_hal::twai::filter::SingleStandardFilter;
//! # use esp_hal::twai::TwaiConfiguration;
//! # use esp_hal::twai::BaudRate;
//! # use esp_hal::twai::TwaiMode;
//! # use nb::block;
//! // Use GPIO pins 2 and 3 to connect to the respective pins on the TWAI
//! // transceiver.
//! let twai_rx_pin = peripherals.GPIO3;
//! let twai_tx_pin = peripherals.GPIO2;
//!
//! // The speed of the TWAI bus.
//! const TWAI_BAUDRATE: twai::BaudRate = BaudRate::B1000K;
//!
//! // Begin configuring the TWAI peripheral. The peripheral is in a reset like
//! // state that prevents transmission but allows configuration.
//! let mut twai_config = twai::TwaiConfiguration::new(
//!     peripherals.TWAI0,
//!     twai_rx_pin,
//!     twai_tx_pin,
//!     TWAI_BAUDRATE,
//!     TwaiMode::Normal
//! );
//!
//! // Partially filter the incoming messages to reduce overhead of receiving
//! // undesired messages
//! twai_config.set_filter(const { SingleStandardFilter::new(b"xxxxxxxxxx0",
//! b"x", [b"xxxxxxxx", b"xxxxxxxx"]) });
//!
//! // Start the peripheral. This locks the configuration settings of the
//! // peripheral and puts it into operation mode, allowing packets to be sent
//! // and received.
//! let mut twai = twai_config.start();
//!
//! loop {
//!     // Wait for a frame to be received.
//!     let frame = block!(twai.receive())?;
//!
//!     // Transmit the frame back.
//!     let _result = block!(twai.transmit(&frame))?;
//! }
//! # }
//! ```
//! 
//! ### Self-testing (self reception of transmitted messages)
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::twai;
//! # use esp_hal::twai::filter;
//! # use esp_hal::twai::filter::SingleStandardFilter;
//! # use esp_hal::twai::TwaiConfiguration;
//! # use esp_hal::twai::BaudRate;
//! # use esp_hal::twai::EspTwaiFrame;
//! # use esp_hal::twai::StandardId;
//! # use esp_hal::twai::TwaiMode;
//! # use nb::block;
//! // Use GPIO pins 2 and 3 to connect to the respective pins on the TWAI
//! // transceiver.
//! let can_rx_pin = peripherals.GPIO3;
//! let can_tx_pin = peripherals.GPIO2;
//!
//! // The speed of the TWAI bus.
//! const TWAI_BAUDRATE: twai::BaudRate = BaudRate::B1000K;
//!
//! // Begin configuring the TWAI peripheral.
//! let mut can_config = twai::TwaiConfiguration::new(
//!     peripherals.TWAI0,
//!     can_rx_pin,
//!     can_tx_pin,
//!     TWAI_BAUDRATE,
//!     TwaiMode::SelfTest
//! );
//!
//! // Partially filter the incoming messages to reduce overhead of receiving
//! // undesired messages
//! can_config.set_filter(const { SingleStandardFilter::new(b"xxxxxxxxxx0",
//! b"x", [b"xxxxxxxx", b"xxxxxxxx"]) });
//!
//! // Start the peripheral. This locks the configuration settings of the
//! // peripheral and puts it into operation mode, allowing packets to be sent
//! // and received.
//! let mut can = can_config.start();
//!
//! # // TODO: `new_*` should return Result not Option
//! let frame = EspTwaiFrame::new_self_reception(StandardId::ZERO,
//!     &[1, 2, 3]).unwrap(); // Wait for a frame to be received.
//! let frame = block!(can.receive())?;
//!
//! # loop {}
//! # }
//! ```

use core::marker::PhantomData;

use self::filter::{Filter, FilterType};
use crate::{
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        InputSignal,
        OutputSignal,
        Pull,
    },
    interrupt::{InterruptConfigurable, InterruptHandler},
    pac::twai0::RegisterBlock,
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralGuard,
    twai::filter::SingleStandardFilter,
    Async,
    Blocking,
    DriverMode,
};

pub mod filter;

/// TWAI error kind
///
/// This represents a common set of TWAI operation errors. HAL implementations
/// are free to define more specific or additional error types. However, by
/// providing a mapping to these common TWAI errors, generic code can still
/// react to them.
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

macro_rules! impl_display {
    ($($kind:ident => $msg:expr),* $(,)?) => {
        impl core::fmt::Display for ErrorKind {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                match self {
                    $(Self::$kind => write!(f, $msg)),*
                }
            }
        }

        #[cfg(feature = "defmt")]
        impl defmt::Format for ErrorKind {
            fn format(&self, f: defmt::Formatter<'_>) {
                match self {
                    $(Self::$kind => defmt::write!(f, $msg)),*
                }
            }
        }
    };
}

impl_display! {
    Overrun => "The peripheral receive buffer was overrun",
    Bit => "Bit value that is monitored differs from the bit value sent",
    Stuff => "Sixth consecutive equal bits detected",
    Crc => "Calculated CRC sequence does not equal the received one",
    Form => "A fixed-form bit field contains one or more illegal bits",
    Acknowledge => "Transmitted frame was not acknowledged",
    Other => "A different error occurred. The original error may contain more information",
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
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

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl embedded_can::Error for ErrorKind {
    fn kind(&self) -> embedded_can::ErrorKind {
        (*self).into()
    }
}

/// Specifies in which mode the TWAI controller will operate.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TwaiMode {
    /// Normal operating mode
    Normal,
    /// Self-test mode (no acknowledgement required for a successful message
    /// transmission)
    SelfTest,
    /// Listen only operating mode
    ListenOnly,
}

/// Standard 11-bit TWAI Identifier (`0..=0x7FF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StandardId(u16);

impl StandardId {
    /// TWAI ID `0`, the highest priority.
    pub const ZERO: Self = StandardId(0);

    /// TWAI ID `0x7FF`, the lowest priority.
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

    /// Returns TWAI Identifier as a raw 16-bit integer.
    #[inline]
    pub fn as_raw(&self) -> u16 {
        self.0
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl From<StandardId> for embedded_can::StandardId {
    fn from(value: StandardId) -> Self {
        embedded_can::StandardId::new(value.as_raw()).unwrap()
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl From<embedded_can::StandardId> for StandardId {
    fn from(value: embedded_can::StandardId) -> Self {
        StandardId::new(value.as_raw()).unwrap()
    }
}

/// Extended 29-bit TWAI Identifier (`0..=1FFF_FFFF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ExtendedId(u32);

impl ExtendedId {
    /// TWAI ID `0`, the highest priority.
    pub const ZERO: Self = ExtendedId(0);

    /// TWAI ID `0x1FFFFFFF`, the lowest priority.
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

    /// Returns TWAI Identifier as a raw 32-bit integer.
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

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl From<ExtendedId> for embedded_can::ExtendedId {
    fn from(value: ExtendedId) -> Self {
        embedded_can::ExtendedId::new(value.0).unwrap()
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl From<embedded_can::ExtendedId> for ExtendedId {
    fn from(value: embedded_can::ExtendedId) -> Self {
        ExtendedId::new(value.as_raw()).unwrap()
    }
}

/// A TWAI Identifier (standard or extended).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl From<Id> for embedded_can::Id {
    fn from(value: Id) -> Self {
        match value {
            Id::Standard(id) => embedded_can::Id::Standard(id.into()),
            Id::Extended(id) => embedded_can::Id::Extended(id.into()),
        }
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl From<embedded_can::Id> for Id {
    fn from(value: embedded_can::Id) -> Self {
        match value {
            embedded_can::Id::Standard(id) => Id::Standard(id.into()),
            embedded_can::Id::Extended(id) => Id::Extended(id.into()),
        }
    }
}

/// A TWAI Frame.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EspTwaiFrame {
    id: Id,
    dlc: usize,
    data: [u8; 8],
    is_remote: bool,
    self_reception: bool,
}

impl EspTwaiFrame {
    /// Creates a new `EspTwaiFrame` with the specified ID and data payload.
    pub fn new(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        // TWAI frames cannot contain more than 8 bytes of data.
        if data.len() > 8 {
            return None;
        }

        let mut d: [u8; 8] = [0; 8];
        d[..data.len()].copy_from_slice(data);

        Some(EspTwaiFrame {
            id: id.into(),
            data: d,
            dlc: data.len(),
            is_remote: false,
            self_reception: false,
        })
    }

    /// Creates a new `EspTwaiFrame` for a transmission request with the
    /// specified ID and data length (DLC).
    pub fn new_remote(id: impl Into<Id>, dlc: usize) -> Option<Self> {
        // TWAI frames cannot have more than 8 bytes.
        if dlc > 8 {
            return None;
        }

        Some(EspTwaiFrame {
            id: id.into(),
            data: [0; 8],
            dlc,
            is_remote: true,
            self_reception: false,
        })
    }

    /// Creates a new `EspTwaiFrame` ready for self-reception with the specified
    /// ID and data payload.
    pub fn new_self_reception(id: impl Into<Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }

        let mut d: [u8; 8] = [0; 8];
        d[..data.len()].copy_from_slice(data);

        Some(EspTwaiFrame {
            id: id.into(),
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
            self_reception: false,
        }
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl embedded_can::Frame for EspTwaiFrame {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        Self::new(id.into(), data)
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        Self::new_remote(id.into(), dlc)
    }

    fn is_extended(&self) -> bool {
        matches!(self.id, Id::Extended(_))
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
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TimingConfig {
    /// The baudrate prescaler is used to determine the period of each time
    /// quantum by dividing the TWAI controller's source clock.
    pub baud_rate_prescaler: u16,

    /// The synchronization jump width is used to determine the maximum number
    /// of time quanta a single bit time can be lengthened/shortened for
    /// synchronization purposes.
    pub sync_jump_width: u8,

    /// Timing segment 1 consists of 1 to 16 time quanta before sample point.
    pub tseg_1: u8,

    /// Timing Segment 2 consists of 1 to 8 time quanta after sample point.
    pub tseg_2: u8,

    /// Enabling triple sampling causes 3 time quanta to be sampled per bit
    /// instead of 1.
    pub triple_sample: bool,
}

/// A selection of pre-determined baudrates for the TWAI driver.
/// Currently these timings are sourced from the ESP IDF C driver which assumes
/// an APB clock of 80MHz.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BaudRate {
    /// A baud rate of 125 Kbps.
    B125K,
    /// A baud rate of 250 Kbps.
    B250K,
    /// A baud rate of 500 Kbps.
    B500K,
    /// A baud rate of 1 Mbps.
    B1000K,
    /// A custom baud rate defined by the user.
    ///
    /// This variant allows users to specify their own timing configuration
    /// using a `TimingConfig` struct.
    Custom(TimingConfig),
}

impl BaudRate {
    /// Convert the BaudRate into the timings that the peripheral needs.
    // See: https://github.com/espressif/esp-idf/tree/master/components/hal/include/hal/twai_types.h
    const fn timing(self) -> TimingConfig {
        #[cfg(not(esp32h2))]
        let timing = match self {
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

        #[cfg(esp32h2)]
        let timing = match self {
            Self::B125K => TimingConfig {
                baud_rate_prescaler: 16,
                sync_jump_width: 3,
                tseg_1: 11,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B250K => TimingConfig {
                baud_rate_prescaler: 8,
                sync_jump_width: 3,
                tseg_1: 11,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B500K => TimingConfig {
                baud_rate_prescaler: 4,
                sync_jump_width: 3,
                tseg_1: 11,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::B1000K => TimingConfig {
                baud_rate_prescaler: 2,
                sync_jump_width: 3,
                tseg_1: 11,
                tseg_2: 4,
                triple_sample: false,
            },
            Self::Custom(timing_config) => timing_config,
        };

        // clock source on ESP32-C6 is xtal (40MHz)
        #[cfg(esp32c6)]
        let timing = TimingConfig {
            baud_rate_prescaler: timing.baud_rate_prescaler / 2,
            ..timing
        };

        timing
    }
}

/// An inactive TWAI peripheral in the "Reset"/configuration state.
pub struct TwaiConfiguration<'d, Dm: DriverMode> {
    twai: PeripheralRef<'d, AnyTwai>,
    filter: Option<(FilterType, [u8; 8])>,
    phantom: PhantomData<Dm>,
    mode: TwaiMode,
    _guard: PeripheralGuard,
}

impl<'d, Dm> TwaiConfiguration<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_internal<TX: PeripheralOutput, RX: PeripheralInput>(
        twai: PeripheralRef<'d, AnyTwai>,
        rx_pin: impl Peripheral<P = RX> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        baud_rate: BaudRate,
        no_transceiver: bool,
        mode: TwaiMode,
    ) -> Self {
        crate::into_mapped_ref!(tx_pin, rx_pin);

        let guard = PeripheralGuard::new(twai.peripheral());

        let mut this = TwaiConfiguration {
            twai,
            filter: None, // We'll immediately call `set_filter`
            phantom: PhantomData,
            mode,
            _guard: guard,
        };

        // Accept all messages by default.
        this.set_filter(
            const { SingleStandardFilter::new(b"xxxxxxxxxxx", b"x", [b"xxxxxxxx", b"xxxxxxxx"]) },
        );

        // Set RESET bit to 1
        this.regs().mode().write(|w| w.reset_mode().set_bit());

        // Enable extended register layout
        #[cfg(esp32)]
        this.regs()
            .clock_divider()
            .modify(|_, w| w.ext_mode().set_bit());

        // Set up the GPIO pins.
        let rx_pull = if no_transceiver {
            tx_pin.set_to_open_drain_output();
            tx_pin.pull_direction(Pull::Up);
            Pull::Up
        } else {
            tx_pin.set_to_push_pull_output();
            Pull::None
        };
        this.twai.output_signal().connect_to(tx_pin);

        // Setting up RX pin later allows us to use a single pin in tests.
        // `set_to_push_pull_output` disables input, here we re-enable it if rx_pin
        // uses the same GPIO.
        rx_pin.init_input(rx_pull);
        this.twai.input_signal().connect_to(rx_pin);

        // Freeze REC by changing to LOM mode
        this.set_mode(TwaiMode::ListenOnly);

        // Set TEC to 0
        this.regs()
            .tx_err_cnt()
            .write(|w| unsafe { w.tx_err_cnt().bits(0) });

        // Set REC to 0
        this.regs()
            .rx_err_cnt()
            .write(|w| unsafe { w.rx_err_cnt().bits(0) });

        // Set EWL to 96
        this.regs()
            .err_warning_limit()
            .write(|w| unsafe { w.err_warning_limit().bits(96) });

        this.set_baud_rate(baud_rate);
        this
    }

    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

    fn internal_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, self.twai.interrupt());
        }
        unsafe { crate::interrupt::bind_interrupt(self.twai.interrupt(), handler.handler()) };

        unwrap!(crate::interrupt::enable(
            self.twai.interrupt(),
            handler.priority()
        ));
    }

    /// Set the bitrate of the bus.
    ///
    /// Note: The timings currently assume a APB_CLK of 80MHz.
    fn set_baud_rate(&mut self, baud_rate: BaudRate) {
        // TWAI is clocked from the APB_CLK according to Table 6-4 [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf)
        // Included timings are all for 80MHz so assert that we are running at 80MHz.
        #[cfg(not(any(esp32h2, esp32c6)))]
        {
            let apb_clock = crate::clock::Clocks::get().apb_clock;
            assert!(apb_clock == fugit::HertzU32::MHz(80));
        }

        // Unpack the baud rate timings and convert them to the values needed for the
        // register. Many of the registers have a minimum value of 1 which is
        // represented by having zero bits set, therefore many values need to
        // have 1 subtracted from them before being stored into the register.
        let timing = baud_rate.timing();

        #[cfg_attr(not(esp32), allow(unused_mut))]
        let mut prescaler = timing.baud_rate_prescaler;

        #[cfg(esp32)]
        {
            // From <https://github.com/espressif/esp-idf/blob/6e5a178b3120dced7fa5c29c655cc22ea182df3d/components/soc/esp32/register/soc/twai_struct.h#L79>
            // and <https://github.com/espressif/esp-idf/blob/6e5a178b3120dced7fa5c29c655cc22ea182df3d/components/hal/esp32/include/hal/twai_ll.h#L528-L534>:
            if timing.baud_rate_prescaler > 128 {
                // Enable /2 baudrate divider by setting `brp_div`.
                // `brp_div` is not an interrupt, it will prescale BRP by 2. Only available on
                // ESP32 Revision 2 or later. Reserved otherwise.
                self.regs().int_ena().modify(|_, w| w.brp_div().set_bit());
                prescaler = timing.baud_rate_prescaler / 2;
            } else {
                // Disable /2 baudrate divider by clearing brp_div.
                self.regs().int_ena().modify(|_, w| w.brp_div().clear_bit());
            }
        }

        let prescale = (prescaler / 2) - 1;
        let sjw = timing.sync_jump_width - 1;
        let tseg_1 = timing.tseg_1 - 1;
        let tseg_2 = timing.tseg_2 - 1;
        let triple_sample = timing.triple_sample;

        // Set up the prescaler and sync jump width.
        self.regs().bus_timing_0().modify(|_, w| unsafe {
            w.baud_presc().bits(prescale as _);
            w.sync_jump_width().bits(sjw)
        });

        // Set up the time segment 1, time segment 2, and triple sample.
        self.regs().bus_timing_1().modify(|_, w| unsafe {
            w.time_seg1().bits(tseg_1);
            w.time_seg2().bits(tseg_2);
            w.time_samp().bit(triple_sample)
        });

        // disable CLKOUT
        self.regs()
            .clock_divider()
            .modify(|_, w| w.clock_off().set_bit());
    }

    /// Set up the acceptance filter on the device.
    ///
    /// NOTE: On a bus with mixed 11-bit and 29-bit packet id's, you may
    /// experience an 11-bit filter match against a 29-bit frame and vice
    /// versa. Your application should check the id again once a frame has
    /// been received to make sure it is the expected value.
    ///
    /// You may use a `const {}` block to ensure that the filter is parsed
    /// during program compilation.
    ///
    /// The filter is not applied to the peripheral until [`Self::start`] is
    /// called.
    ///
    /// [ESP32C3 Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf#subsubsection.29.4.6)
    pub fn set_filter(&mut self, filter: impl Filter) {
        // Convert the filter into values for the registers and store them for later
        // use.
        self.filter = Some((filter.filter_type(), filter.to_registers()));
    }

    fn apply_filter(&self) {
        let Some((filter_type, registers)) = self.filter.as_ref() else {
            return;
        };

        // Set or clear the rx filter mode bit depending on the filter type.
        self.regs()
            .mode()
            .modify(|_, w| w.rx_filter_mode().bit(*filter_type == FilterType::Single));

        // Copy the filter to the peripheral.
        unsafe {
            copy_to_data_register(self.regs().data_0().as_ptr(), registers);
        }
    }

    /// Set the error warning threshold.
    ///
    /// In the case when any of an error counter value exceeds the threshold, or
    /// all the error counter values are below the threshold, an error
    /// warning interrupt will be triggered (given the enable signal is
    /// valid).
    pub fn set_error_warning_limit(&mut self, limit: u8) {
        self.regs()
            .err_warning_limit()
            .write(|w| unsafe { w.err_warning_limit().bits(limit) });
    }

    /// Set the operating mode based on provided option
    fn set_mode(&self, mode: TwaiMode) {
        self.regs().mode().modify(|_, w| {
            // self-test mode turns off acknowledgement requirement
            w.self_test_mode().bit(mode == TwaiMode::SelfTest);
            w.listen_only_mode().bit(mode == TwaiMode::ListenOnly)
        });
    }

    /// Put the peripheral into Operation Mode, allowing the transmission and
    /// reception of packets using the new object.
    pub fn start(self) -> Twai<'d, Dm> {
        self.apply_filter();
        self.set_mode(self.mode);

        // Clear the TEC and REC
        self.regs()
            .tx_err_cnt()
            .write(|w| unsafe { w.tx_err_cnt().bits(0) });

        let rec =
            if cfg!(any(esp32, esp32s2, esp32s3, esp32c3)) && self.mode == TwaiMode::ListenOnly {
                // Errata workaround: Prevent transmission of dominant error frame while in
                // listen only mode by setting REC to 128 before exiting reset mode.
                // This forces the controller to be error passive (thus only transmits recessive
                // bits). The TEC/REC remain frozen in listen only mode thus
                // ensuring we remain error passive.
                128
            } else {
                0
            };
        self.regs()
            .rx_err_cnt()
            .write(|w| unsafe { w.rx_err_cnt().bits(rec) });

        // Clear any interrupts by reading the status register
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))] {
                let _ = self.regs().int_raw().read();
            } else {
                let _ = self.regs().interrupt().read();
            }
        }

        // Put the peripheral into operation mode by clearing the reset mode bit.
        self.regs().mode().modify(|_, w| w.reset_mode().clear_bit());

        Twai {
            rx: TwaiRx {
                twai: unsafe { self.twai.clone_unchecked() },
                phantom: PhantomData,
                _guard: PeripheralGuard::new(self.twai.peripheral()),
            },
            tx: TwaiTx {
                twai: unsafe { self.twai.clone_unchecked() },
                phantom: PhantomData,
                _guard: PeripheralGuard::new(self.twai.peripheral()),
            },
            twai: unsafe { self.twai.clone_unchecked() },
            phantom: PhantomData,
        }
    }
}

impl<'d> TwaiConfiguration<'d, Blocking> {
    /// Create a new instance of [TwaiConfiguration]
    ///
    /// You will need to use a transceiver to connect to the TWAI bus
    pub fn new<RX: PeripheralInput, TX: PeripheralOutput>(
        peripheral: impl Peripheral<P = impl Instance> + 'd,
        rx_pin: impl Peripheral<P = RX> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        crate::into_mapped_ref!(peripheral);
        Self::new_internal(peripheral, rx_pin, tx_pin, baud_rate, false, mode)
    }

    /// Create a new instance of [TwaiConfiguration] meant to connect two ESP32s
    /// directly
    ///
    /// You don't need a transceiver by following the description in the
    /// `twai.rs` example
    pub fn new_no_transceiver<RX: PeripheralInput, TX: PeripheralOutput>(
        peripheral: impl Peripheral<P = impl Instance> + 'd,
        rx_pin: impl Peripheral<P = RX> + 'd,
        tx_pin: impl Peripheral<P = TX> + 'd,
        baud_rate: BaudRate,
        mode: TwaiMode,
    ) -> Self {
        crate::into_mapped_ref!(peripheral);
        Self::new_internal(peripheral, rx_pin, tx_pin, baud_rate, true, mode)
    }

    /// Convert the configuration into an async configuration.
    pub fn into_async(mut self) -> TwaiConfiguration<'d, Async> {
        self.set_interrupt_handler(self.twai.async_handler());
        TwaiConfiguration {
            twai: self.twai,
            filter: self.filter,
            phantom: PhantomData,
            mode: self.mode,
            _guard: self._guard,
        }
    }
}

impl<'d> TwaiConfiguration<'d, Async> {
    /// Convert the configuration into a blocking configuration.
    pub fn into_blocking(self) -> TwaiConfiguration<'d, Blocking> {
        use crate::{interrupt, Cpu};

        interrupt::disable(Cpu::current(), self.twai.interrupt());

        // Re-create in  blocking mode
        TwaiConfiguration {
            twai: self.twai,
            filter: self.filter,
            phantom: PhantomData,
            mode: self.mode,
            _guard: self._guard,
        }
    }
}

impl crate::private::Sealed for TwaiConfiguration<'_, Blocking> {}

impl InterruptConfigurable for TwaiConfiguration<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.internal_set_interrupt_handler(handler);
    }
}

/// An active TWAI peripheral in Normal Mode.
///
/// In this mode, the TWAI controller can transmit and receive messages
/// including error signals (such as error and overload frames).
pub struct Twai<'d, Dm: DriverMode> {
    twai: PeripheralRef<'d, AnyTwai>,
    tx: TwaiTx<'d, Dm>,
    rx: TwaiRx<'d, Dm>,
    phantom: PhantomData<Dm>,
}

impl<'d, Dm> Twai<'d, Dm>
where
    Dm: DriverMode,
{
    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

    fn mode(&self) -> TwaiMode {
        let mode = self.regs().mode().read();

        if mode.self_test_mode().bit_is_set() {
            TwaiMode::SelfTest
        } else if mode.listen_only_mode().bit_is_set() {
            TwaiMode::ListenOnly
        } else {
            TwaiMode::Normal
        }
    }

    /// Stop the peripheral, putting it into reset mode and enabling
    /// reconfiguration.
    pub fn stop(self) -> TwaiConfiguration<'d, Dm> {
        // Put the peripheral into reset/configuration mode by setting the reset mode
        // bit.
        self.regs().mode().modify(|_, w| w.reset_mode().set_bit());

        let mode = self.mode();

        let guard = PeripheralGuard::new(self.twai.peripheral());
        TwaiConfiguration {
            twai: self.twai,
            filter: None, // filter already applied, no need to restore it
            phantom: PhantomData,
            mode,
            _guard: guard,
        }
    }

    /// Returns the value of the receive error counter.
    pub fn receive_error_count(&self) -> u8 {
        self.regs().rx_err_cnt().read().rx_err_cnt().bits()
    }

    /// Returns the value of the transmit error counter.
    pub fn transmit_error_count(&self) -> u8 {
        self.regs().tx_err_cnt().read().tx_err_cnt().bits()
    }

    /// Check if the controller is in a bus off state.
    pub fn is_bus_off(&self) -> bool {
        self.regs().status().read().bus_off_st().bit_is_set()
    }

    /// Get the number of messages that the peripheral has available in the
    /// receive FIFO.
    ///
    /// Note that this may not be the number of valid messages in the receive
    /// FIFO due to fifo overflow/overrun.
    pub fn num_available_messages(&self) -> u8 {
        self.regs()
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
            release_receive_fifo(self.regs());
        }
    }

    /// Sends the specified `EspTwaiFrame` over the TWAI bus.
    pub fn transmit(&mut self, frame: &EspTwaiFrame) -> nb::Result<(), EspTwaiError> {
        self.tx.transmit(frame)
    }

    /// Receives a TWAI frame from the TWAI bus.
    pub fn receive(&mut self) -> nb::Result<EspTwaiFrame, EspTwaiError> {
        self.rx.receive()
    }

    /// Consumes this `Twai` instance and splits it into transmitting and
    /// receiving halves.
    pub fn split(self) -> (TwaiRx<'d, Dm>, TwaiTx<'d, Dm>) {
        (self.rx, self.tx)
    }
}

/// Interface to the TWAI transmitter part.
pub struct TwaiTx<'d, Dm: DriverMode> {
    twai: PeripheralRef<'d, AnyTwai>,
    phantom: PhantomData<Dm>,
    _guard: PeripheralGuard,
}

impl<Dm> TwaiTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

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
        let status = self.regs().status().read();

        // Check that the peripheral is not in a bus off state.
        if status.bus_off_st().bit_is_set() {
            return nb::Result::Err(nb::Error::Other(EspTwaiError::BusOff));
        }
        // Check that the peripheral is not already transmitting a packet.
        if !status.tx_buf_st().bit_is_set() {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        write_frame(self.regs(), frame);

        Ok(())
    }
}

/// Interface to the TWAI receiver part.
pub struct TwaiRx<'d, Dm: DriverMode> {
    twai: PeripheralRef<'d, AnyTwai>,
    phantom: PhantomData<Dm>,
    _guard: PeripheralGuard,
}

impl<Dm> TwaiRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn regs(&self) -> &RegisterBlock {
        self.twai.register_block()
    }

    /// Receive a frame
    pub fn receive(&mut self) -> nb::Result<EspTwaiFrame, EspTwaiError> {
        let status = self.regs().status().read();

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

        Ok(read_frame(self.regs())?)
    }
}

/// Represents errors that can occur in the TWAI driver.
/// This enum defines the possible errors that can be encountered when
/// interacting with the TWAI peripheral.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EspTwaiError {
    /// TWAI peripheral has entered a bus-off state.
    BusOff,
    /// The received frame contains an invalid DLC.
    NonCompliantDlc(u8),
    /// Encapsulates errors defined by the embedded-hal crate.
    EmbeddedHAL(ErrorKind),
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl embedded_can::Error for EspTwaiError {
    fn kind(&self) -> embedded_can::ErrorKind {
        if let Self::EmbeddedHAL(kind) = self {
            (*kind).into()
        } else {
            embedded_can::ErrorKind::Other
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

#[cfg(any(doc, feature = "unstable"))]
impl<Dm> embedded_can::nb::Can for Twai<'_, Dm>
where
    Dm: DriverMode,
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

/// TWAI peripheral instance.
#[doc(hidden)]
pub trait Instance: Peripheral<P = Self> + Into<AnyTwai> + 'static {
    /// The identifier number for this TWAI instance.
    fn number(&self) -> usize;

    /// Returns the system peripheral marker for this instance.
    fn peripheral(&self) -> crate::system::Peripheral;

    /// Input signal.
    fn input_signal(&self) -> InputSignal;
    /// Output signal.
    fn output_signal(&self) -> OutputSignal;
    /// The interrupt associated with this TWAI instance.
    fn interrupt(&self) -> crate::peripherals::Interrupt;

    /// Provides an asynchronous interrupt handler for TWAI instance.
    fn async_handler(&self) -> InterruptHandler;

    /// Returns a reference to the register block for TWAI instance.
    fn register_block(&self) -> &RegisterBlock;

    /// Enables interrupts for the TWAI peripheral.
    fn enable_interrupts(&self) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))] {
                self.register_block().int_ena().modify(|_, w| {
                    w.rx_int_ena().set_bit();
                    w.tx_int_ena().set_bit();
                    w.bus_err_int_ena().set_bit();
                    w.arb_lost_int_ena().set_bit();
                    w.err_passive_int_ena().set_bit()
                });
            } else {
                self.register_block().interrupt_enable().modify(|_, w| {
                    w.ext_receive_int_ena().set_bit();
                    w.ext_transmit_int_ena().set_bit();
                    w.bus_err_int_ena().set_bit();
                    w.arbitration_lost_int_ena().set_bit();
                    w.err_passive_int_ena().set_bit()
                });
            }
        }
    }

    /// Returns a reference to the asynchronous state for this TWAI instance.
    fn async_state(&self) -> &asynch::TwaiAsyncState;
}

/// Read a frame from the peripheral.
fn read_frame(register_block: &RegisterBlock) -> Result<EspTwaiFrame, EspTwaiError> {
    // Read the frame information and extract the frame id format and dlc.
    let data_0 = register_block.data_0().read().tx_byte_0().bits();

    let is_standard_format = data_0 & (0b1 << 7) == 0;
    let is_data_frame = data_0 & (0b1 << 6) == 0;
    let self_reception = data_0 & (0b1 << 4) != 0;
    let dlc = data_0 & 0b1111;

    if dlc > 8 {
        // Release the packet we read from the FIFO, allowing the peripheral to prepare
        // the next packet.
        release_receive_fifo(register_block);

        return Err(EspTwaiError::NonCompliantDlc(dlc));
    }
    let dlc = dlc as usize;

    // Read the payload from the packet and construct a frame.
    let (id, data_ptr) = if is_standard_format {
        // Frame uses standard 11 bit id.
        let data_1 = register_block.data_1().read().tx_byte_1().bits();
        let data_2 = register_block.data_2().read().tx_byte_2().bits();

        let raw_id: u16 = ((data_1 as u16) << 3) | ((data_2 as u16) >> 5);

        let id = Id::from(StandardId::new(raw_id).unwrap());
        (id, register_block.data_3().as_ptr())
    } else {
        // Frame uses extended 29 bit id.
        let data_1 = register_block.data_1().read().tx_byte_1().bits();
        let data_2 = register_block.data_2().read().tx_byte_2().bits();
        let data_3 = register_block.data_3().read().tx_byte_3().bits();
        let data_4 = register_block.data_4().read().tx_byte_4().bits();

        let raw_id: u32 = ((data_1 as u32) << 21)
            | ((data_2 as u32) << 13)
            | ((data_3 as u32) << 5)
            | ((data_4 as u32) >> 3);

        let id = Id::from(ExtendedId::new(raw_id).unwrap());
        (id, register_block.data_5().as_ptr())
    };

    let mut frame = if is_data_frame {
        unsafe { EspTwaiFrame::new_from_data_registers(id, data_ptr, dlc) }
    } else {
        EspTwaiFrame::new_remote(id, dlc).unwrap()
    };
    frame.self_reception = self_reception;

    // Release the packet we read from the FIFO, allowing the peripheral to prepare
    // the next packet.
    release_receive_fifo(register_block);

    Ok(frame)
}

/// Release the message in the buffer. This will decrement the received
/// message counter and prepare the next message in the FIFO for
/// reading.
fn release_receive_fifo(register_block: &RegisterBlock) {
    register_block.cmd().write(|w| w.release_buf().set_bit());
}

/// Write a frame to the peripheral.
fn write_frame(register_block: &RegisterBlock, frame: &EspTwaiFrame) {
    // Assemble the frame information into the data_0 byte.
    let frame_format: u8 = matches!(frame.id, Id::Extended(_)) as u8;
    let self_reception: u8 = frame.self_reception as u8;
    let rtr_bit: u8 = frame.is_remote as u8;
    let dlc_bits: u8 = frame.dlc as u8 & 0b1111;

    let data_0: u8 = (frame_format << 7) | (rtr_bit << 6) | (self_reception << 4) | dlc_bits;

    register_block
        .data_0()
        .write(|w| unsafe { w.tx_byte_0().bits(data_0) });

    // Assemble the identifier information of the packet and return where the data
    // buffer starts.
    let data_ptr = match frame.id {
        Id::Standard(id) => {
            let id = id.as_raw();

            register_block
                .data_1()
                .write(|w| unsafe { w.tx_byte_1().bits((id >> 3) as u8) });

            register_block
                .data_2()
                .write(|w| unsafe { w.tx_byte_2().bits((id << 5) as u8) });

            register_block.data_3().as_ptr()
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

            register_block.data_5().as_ptr()
        }
    };

    // Store the data portion of the packet into the transmit buffer.
    unsafe {
        copy_to_data_register(
            data_ptr,
            match frame.is_remote {
                true => &[], // RTR frame, so no data is included.
                false => &frame.data[0..frame.dlc],
            },
        )
    }

    // Trigger the appropriate transmission request based on self_reception flag
    if frame.self_reception {
        register_block.cmd().write(|w| w.self_rx_req().set_bit());
    } else {
        // Set the transmit request command, this will lock the transmit buffer until
        // the transmission is complete or aborted.
        register_block.cmd().write(|w| w.tx_req().set_bit());
    }
}

impl Instance for crate::peripherals::TWAI0 {
    fn number(&self) -> usize {
        0
    }

    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Twai0
    }

    fn input_signal(&self) -> InputSignal {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))] {
                InputSignal::TWAI_RX
            } else {
                InputSignal::TWAI0_RX
            }
        }
    }

    fn output_signal(&self) -> OutputSignal {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))] {
                OutputSignal::TWAI_TX
            } else {
                OutputSignal::TWAI0_TX
            }
        }
    }

    fn interrupt(&self) -> crate::peripherals::Interrupt {
        crate::peripherals::Interrupt::TWAI0
    }

    fn async_handler(&self) -> InterruptHandler {
        asynch::twai0
    }

    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        crate::peripherals::TWAI0::regs()
    }

    fn async_state(&self) -> &asynch::TwaiAsyncState {
        static STATE: asynch::TwaiAsyncState = asynch::TwaiAsyncState::new();
        &STATE
    }
}

#[cfg(twai1)]
impl Instance for crate::peripherals::TWAI1 {
    fn number(&self) -> usize {
        1
    }

    fn peripheral(&self) -> crate::system::Peripheral {
        crate::system::Peripheral::Twai1
    }

    fn input_signal(&self) -> InputSignal {
        InputSignal::TWAI1_RX
    }

    fn output_signal(&self) -> OutputSignal {
        OutputSignal::TWAI1_TX
    }

    fn interrupt(&self) -> crate::peripherals::Interrupt {
        crate::peripherals::Interrupt::TWAI1
    }

    fn async_handler(&self) -> InterruptHandler {
        asynch::twai1
    }

    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        crate::peripherals::TWAI1::regs()
    }

    fn async_state(&self) -> &asynch::TwaiAsyncState {
        static STATE: asynch::TwaiAsyncState = asynch::TwaiAsyncState::new();
        &STATE
    }
}

crate::any_peripheral! {
    /// Any TWAI peripheral.
    pub peripheral AnyTwai {
        #[cfg(twai0)]
        Twai0(crate::peripherals::TWAI0),
        #[cfg(twai1)]
        Twai1(crate::peripherals::TWAI1),
    }
}

impl Instance for AnyTwai {
    delegate::delegate! {
        to match &self.0 {
            #[cfg(twai0)]
            AnyTwaiInner::Twai0(twai) => twai,
            #[cfg(twai1)]
            AnyTwaiInner::Twai1(twai) => twai,
        } {
            fn number(&self) -> usize;
            fn peripheral(&self) -> crate::system::Peripheral;
            fn input_signal(&self) -> InputSignal;
            fn output_signal(&self) -> OutputSignal;
            fn interrupt(&self) -> crate::peripherals::Interrupt;
            fn async_handler(&self) -> InterruptHandler;
            fn register_block(&self) -> &RegisterBlock;
            fn async_state(&self) -> &asynch::TwaiAsyncState;
        }
    }
}

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
    #[cfg(twai1)]
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

    impl Twai<'_, Async> {
        /// Transmits an `EspTwaiFrame` asynchronously over the TWAI bus.
        pub async fn transmit_async(&mut self, frame: &EspTwaiFrame) -> Result<(), EspTwaiError> {
            self.tx.transmit_async(frame).await
        }
        /// Receives an `EspTwaiFrame` asynchronously over the TWAI bus.
        pub async fn receive_async(&mut self) -> Result<EspTwaiFrame, EspTwaiError> {
            self.rx.receive_async().await
        }
    }

    impl TwaiTx<'_, Async> {
        /// Transmits an `EspTwaiFrame` asynchronously over the TWAI bus.
        pub async fn transmit_async(&mut self, frame: &EspTwaiFrame) -> Result<(), EspTwaiError> {
            self.twai.enable_interrupts();
            poll_fn(|cx| {
                self.twai.async_state().tx_waker.register(cx.waker());

                let status = self.regs().status().read();

                // Check that the peripheral is not in a bus off state.
                if status.bus_off_st().bit_is_set() {
                    return Poll::Ready(Err(EspTwaiError::BusOff));
                }
                // Check that the peripheral is not already transmitting a packet.
                if !status.tx_buf_st().bit_is_set() {
                    return Poll::Pending;
                }

                write_frame(self.regs(), frame);

                Poll::Ready(Ok(()))
            })
            .await
        }
    }

    impl TwaiRx<'_, Async> {
        /// Receives an `EspTwaiFrame` asynchronously over the TWAI bus.
        pub async fn receive_async(&mut self) -> Result<EspTwaiFrame, EspTwaiError> {
            self.twai.enable_interrupts();
            poll_fn(|cx| {
                self.twai.async_state().err_waker.register(cx.waker());

                if let Poll::Ready(result) = self.twai.async_state().rx_queue.poll_receive(cx) {
                    return Poll::Ready(result);
                }

                let status = self.regs().status().read();

                // Check that the peripheral is not in a bus off state.
                if status.bus_off_st().bit_is_set() {
                    return Poll::Ready(Err(EspTwaiError::BusOff));
                }

                // Check if the packet in the receive buffer is valid or overrun.
                if status.miss_st().bit_is_set() {
                    return Poll::Ready(Err(EspTwaiError::EmbeddedHAL(ErrorKind::Overrun)));
                }

                Poll::Pending
            })
            .await
        }
    }

    fn handle_interrupt(register_block: &RegisterBlock, async_state: &TwaiAsyncState) {
        cfg_if::cfg_if! {
            if #[cfg(any(esp32, esp32c3, esp32s2, esp32s3))] {
                let intr_enable = register_block.int_ena().read();
                let intr_status = register_block.int_raw().read();

                let int_ena_reg = register_block.int_ena();

                let tx_int_status = intr_status.tx_int_st();
                let rx_int_status = intr_status.rx_int_st();
            } else {
                let intr_enable = register_block.interrupt_enable().read();
                let intr_status = register_block.interrupt().read();

                let int_ena_reg = register_block.interrupt_enable();

                let tx_int_status = intr_status.transmit_int_st();
                let rx_int_status = intr_status.receive_int_st();
            }
        }

        if tx_int_status.bit_is_set() {
            async_state.tx_waker.wake();
        }

        if rx_int_status.bit_is_set() {
            let status = register_block.status().read();

            let rx_queue = &async_state.rx_queue;

            if status.bus_off_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::BusOff));
            }

            if status.miss_st().bit_is_set() {
                let _ = rx_queue.try_send(Err(EspTwaiError::EmbeddedHAL(ErrorKind::Overrun)));
            }

            match read_frame(register_block) {
                Ok(frame) => {
                    let _ = rx_queue.try_send(Ok(frame));
                }
                Err(e) => warn!("Error reading frame: {:?}", e),
            }
        }

        if intr_status.bits() & 0b11111100 > 0 {
            let err_capture = register_block.err_code_cap().read();
            let status = register_block.status().read();

            // Read error code direction (transmitting or receiving)
            let ecc_direction = err_capture.ecc_direction().bit_is_set();

            // If the error comes from Tx and Tx request is pending
            if !ecc_direction && !status.tx_buf_st().bit_is_set() {
                // Cancel a pending transmission request
                register_block.cmd().write(|w| w.abort_tx().set_bit());
            }

            async_state.err_waker.wake();
        }

        // Clear interrupt request bits
        unsafe {
            int_ena_reg.modify(|_, w| w.bits(intr_enable.bits() & (!intr_status.bits() | 1)));
        }
    }

    #[handler]
    pub(super) fn twai0() {
        let twai = unsafe { TWAI0::steal() };
        handle_interrupt(twai.register_block(), twai.async_state());
    }

    #[cfg(twai1)]
    #[handler]
    pub(super) fn twai1() {
        let twai = unsafe { TWAI1::steal() };
        handle_interrupt(twai.register_block(), twai.async_state());
    }
}
