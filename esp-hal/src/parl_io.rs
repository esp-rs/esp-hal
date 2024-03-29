//! # Parallel IO
//!
//! ## Overview
//! The Parallel IO peripheral is a general purpose parallel interface that can
//! be used to connect to external devices such as LED matrix, LCD display,
//! Printer and Camera. The peripheral has independent TX and RX units. Each
//! unit can have up to 8 or 16 data signals (depending on your target hardware)
//! plus 1 or 2 clock signals.
//!
//! The driver uses DMA (Direct Memory Access) for efficient data transfer.
//!
//! ## Examples
//!
//! ### Initialization for TX
//! ```no_run
//! // configure the data pins to use
//! let tx_pins = TxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);
//!
//! // configure the valid pin which will be driven high during a TX transfer
//! let pin_conf = TxPinConfigWithValidPin::new(tx_pins, io.pins.gpio5);
//!
//! let mut parl_io = ParlIoTxOnly::new(
//!     peripherals.PARL_IO,
//!     dma_channel.configure(
//!         false,
//!         &mut tx_descriptors,
//!         &mut rx_descriptors,
//!         DmaPriority::Priority0,
//!     ),
//!     1.MHz(),
//!     &clocks,
//! )
//! .unwrap();
//!
//! // configure a pin for the clock signal
//! let cp = ClkOutPin::new(io.pins.gpio6);
//!
//! let mut parl_io_tx = parl_io
//!     .tx
//!     .with_config(pin_conf, cp, 0, SampleEdge::Normal, BitPackOrder::Msb)
//!     .unwrap();
//! ```
//!
//! ### Start TX transfer
//! ```no_run
//! let mut transfer = parl_io_tx.write_dma(buffer).unwrap();
//!
//! transfer.wait().unwrap();
//! ```
//!
//! ### Initialization for RX
//! ```no_run
//! let rx_pins = RxFourBits::new(io.pins.gpio1, io.pins.gpio2, io.pins.gpio3, io.pins.gpio4);
//!
//! let parl_io = ParlIoRxOnly::new(
//!     peripherals.PARL_IO,
//!     dma_channel.configure(
//!         false,
//!         &mut tx_descriptors,
//!         &mut rx_descriptors,
//!         DmaPriority::Priority0,
//!     ),
//!     1.MHz(),
//!     &clocks,
//! )
//! .unwrap();
//!
//! let mut parl_io_rx = parl_io
//!     .rx
//!     .with_config(rx_pins, NoClkPin, BitPackOrder::Msb, Some(0xfff))
//!     .unwrap();
//! ```
//!
//! ### Start RX transfer
//! ```no_run
//! let mut transfer = parl_io_rx.read_dma(buffer).unwrap();
//! transfer.wait().unwrap();
//! ```

use core::marker::PhantomData;

use embedded_dma::{ReadBuffer, WriteBuffer};
use enumset::{EnumSet, EnumSetType};
use fugit::HertzU32;
use peripheral::PeripheralRef;
use private::*;

use crate::{
    clock::Clocks,
    dma::{Channel, ChannelTypes, DmaError, DmaPeripheral, ParlIoPeripheral, RxPrivate, TxPrivate},
    gpio::{InputPin, OutputPin},
    interrupt::InterruptHandler,
    peripheral::{self, Peripheral},
    peripherals,
    system::PeripheralClockControl,
    Blocking,
    Mode,
};

#[allow(unused)]
const MAX_DMA_SIZE: usize = 32736;

#[derive(EnumSetType)]
pub enum ParlIoInterrupt {
    TxFifoReEmpty,
    RxFifoWOvf,
    TxEof,
}

/// Parallel IO errors
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// General DMA error
    DmaError(DmaError),
    /// Maximum transfer size (32736) exceeded
    MaxDmaTransferSizeExceeded,
    /// Trying to use an impossible clock rate
    UnreachableClockRate,
}

impl From<DmaError> for Error {
    fn from(value: DmaError) -> Self {
        Error::DmaError(value)
    }
}

/// Parallel IO sample edge
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleEdge {
    /// Positive edge
    Normal = 0,
    /// Negative edge
    Invert = 1,
}

/// Parallel IO bit packing order
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BitPackOrder {
    /// Bit pack order: MSB
    Msb = 0,
    /// Bit pack order: LSB
    Lsb = 1,
}

#[cfg(esp32c6)]
/// Enable Mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EnableMode {
    /// Enable at high level
    HighLevel,
    /// Enable at low level
    LowLevel,
    /// Positive pulse start (data bit included) & Positive pulse end (data bit
    /// included)
    PulseMode1,
    /// Positive pulse start (data bit included) & Positive pulse end (data bit
    /// excluded)
    PulseMode2,
    /// Positive pulse start (data bit excluded) & Positive pulse end (data bit
    /// included)
    PulseMode3,
    /// Positive pulse start (data bit excluded) & Positive pulse end (data bit
    /// excluded)
    PulseMode4,
    /// Positive pulse start (data bit included) & Length end
    PulseMode5,
    /// Positive pulse start (data bit excluded) & Length end
    PulseMode6,
    /// Negative pulse start (data bit included) & Negative pulse end(data bit
    /// included)
    PulseMode7,
    /// Negative pulse start (data bit included) & Negative pulse end (data bit
    /// excluded)
    PulseMode8,
    /// Negative pulse start (data bit excluded) & Negative pulse end (data bit
    /// included)
    PulseMode9,
    /// Negative pulse start (data bit excluded) & Negative pulse end (data bit
    /// excluded)
    PulseMode10,
    /// Negative pulse start (data bit included) & Length end
    PulseMode11,
    /// Negative pulse start (data bit excluded) & Length end
    PulseMode12,
}

#[cfg(esp32c6)]
impl EnableMode {
    fn pulse_submode_sel(&self) -> Option<u8> {
        match self {
            EnableMode::PulseMode1 => Some(0),
            EnableMode::PulseMode2 => Some(1),
            EnableMode::PulseMode3 => Some(2),
            EnableMode::PulseMode4 => Some(3),
            EnableMode::PulseMode5 => Some(4),
            EnableMode::PulseMode6 => Some(5),
            EnableMode::PulseMode7 => Some(6),
            EnableMode::PulseMode8 => Some(7),
            EnableMode::PulseMode9 => Some(8),
            EnableMode::PulseMode10 => Some(9),
            EnableMode::PulseMode11 => Some(10),
            EnableMode::PulseMode12 => Some(11),
            _ => None,
        }
    }

    fn level_submode_sel(&self) -> Option<u8> {
        match self {
            EnableMode::HighLevel => Some(0),
            EnableMode::LowLevel => Some(1),
            _ => None,
        }
    }

    fn smp_model_sel(&self) -> Option<self::private::SampleMode> {
        match self {
            EnableMode::HighLevel => Some(self::private::SampleMode::ExternalLevel),
            EnableMode::LowLevel => Some(self::private::SampleMode::ExternalLevel),
            _ => Some(self::private::SampleMode::ExternalPulse),
        }
    }
}

#[cfg(esp32h2)]
/// Enable Mode
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EnableMode {
    /// Enable at high level
    HighLevel,
    /// Positive pulse start (data bit included) & Positive pulse end (data bit
    /// included)
    PulseMode1,
    /// Positive pulse start (data bit included) & Positive pulse end (data bit
    /// excluded)
    PulseMode2,
    /// Positive pulse start (data bit excluded) & Positive pulse end (data bit
    /// included)
    PulseMode3,
    /// Positive pulse start (data bit excluded) & Positive pulse end (data bit
    /// excluded)
    PulseMode4,
    /// Positive pulse start (data bit included) & Length end
    PulseMode5,
    /// Positive pulse start (data bit excluded) & Length end
    PulseMode6,
}

#[cfg(esp32h2)]
impl EnableMode {
    fn pulse_submode_sel(&self) -> Option<u8> {
        match self {
            EnableMode::PulseMode1 => Some(0),
            EnableMode::PulseMode2 => Some(1),
            EnableMode::PulseMode3 => Some(2),
            EnableMode::PulseMode4 => Some(3),
            EnableMode::PulseMode5 => Some(4),
            EnableMode::PulseMode6 => Some(5),
            _ => None,
        }
    }

    fn level_submode_sel(&self) -> Option<u8> {
        match self {
            EnableMode::HighLevel => Some(0),
            _ => None,
        }
    }

    fn smp_model_sel(&self) -> Option<self::private::SampleMode> {
        match self {
            EnableMode::HighLevel => Some(self::private::SampleMode::ExternalLevel),
            _ => Some(self::private::SampleMode::ExternalPulse),
        }
    }
}

/// Generation of GDMA SUC EOF
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EofMode {
    /// Generate GDMA SUC EOF by data byte length
    ByteLen,
    /// Generate GDMA SUC EOF by the external enable signal
    EnableSignal,
}

/// Used to configure no pin as clock output
pub struct NoClkPin;
impl TxClkPin for NoClkPin {
    fn configure(&mut self) {
        // nothing
    }
}
impl RxClkPin for NoClkPin {
    fn configure(&mut self) {
        // nothing
    }
}

/// Wraps a GPIO pin which will be used as the clock output signal
pub struct ClkOutPin<'d, P>
where
    P: OutputPin,
{
    pin: PeripheralRef<'d, P>,
}
impl<'d, P> ClkOutPin<'d, P>
where
    P: OutputPin,
{
    pub fn new(pin: impl Peripheral<P = P> + 'd) -> Self {
        crate::into_ref!(pin);
        Self { pin }
    }
}
impl<'d, P> TxClkPin for ClkOutPin<'d, P>
where
    P: OutputPin,
{
    fn configure(&mut self) {
        self.pin
            .set_to_push_pull_output()
            .connect_peripheral_to_output(crate::gpio::OutputSignal::PARL_TX_CLK);
    }
}

/// Wraps a GPIO pin which will be used as the TX clock input signal
pub struct ClkInPin<'d, P>
where
    P: InputPin,
{
    pin: PeripheralRef<'d, P>,
}
impl<'d, P> ClkInPin<'d, P>
where
    P: InputPin,
{
    pub fn new(pin: impl Peripheral<P = P> + 'd) -> Self {
        crate::into_ref!(pin);
        Self { pin }
    }
}
impl<'d, P> TxClkPin for ClkInPin<'d, P>
where
    P: InputPin,
{
    fn configure(&mut self) {
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };
        pcr.parl_clk_tx_conf().modify(|_, w| {
            w.parl_clk_tx_sel()
                .variant(3)
                .parl_clk_tx_div_num()
                .variant(0)
        }); // PAD_CLK_TX, no divider

        self.pin
            .set_to_input()
            .connect_input_to_peripheral(crate::gpio::InputSignal::PARL_TX_CLK);
    }
}

/// Wraps a GPIO pin which will be used as the RX clock input signal
pub struct RxClkInPin<'d, P>
where
    P: InputPin,
{
    pin: PeripheralRef<'d, P>,
    sample_edge: SampleEdge,
}
impl<'d, P> RxClkInPin<'d, P>
where
    P: InputPin,
{
    pub fn new(pin: impl Peripheral<P = P> + 'd, sample_edge: SampleEdge) -> Self {
        crate::into_ref!(pin);
        Self { pin, sample_edge }
    }
}
impl<'d, P> RxClkPin for RxClkInPin<'d, P>
where
    P: InputPin,
{
    fn configure(&mut self) {
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };
        pcr.parl_clk_rx_conf().modify(|_, w| {
            w.parl_clk_rx_sel()
                .variant(3)
                .parl_clk_rx_div_num()
                .variant(0)
        }); // PAD_CLK_TX, no divider

        self.pin
            .set_to_input()
            .connect_input_to_peripheral(crate::gpio::InputSignal::PARL_RX_CLK);

        Instance::set_rx_clk_edge_sel(self.sample_edge);
    }
}

/// Pin configuration with an additional pin for the valid signal.
pub struct TxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + TxPins + ConfigurePins,
    VP: OutputPin,
{
    tx_pins: P,
    valid_pin: PeripheralRef<'d, VP>,
}

impl<'d, P, VP> TxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + TxPins + ConfigurePins,
    VP: OutputPin,
{
    pub fn new(tx_pins: P, valid_pin: impl Peripheral<P = VP> + 'd) -> Self {
        crate::into_ref!(valid_pin);
        Self { tx_pins, valid_pin }
    }
}

impl<'d, P, VP> TxPins for TxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + TxPins + ConfigurePins,
    VP: OutputPin,
{
}

impl<'d, P, VP> ConfigurePins for TxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + TxPins + ConfigurePins,
    VP: OutputPin,
{
    fn configure(&mut self) -> Result<(), Error> {
        self.tx_pins.configure()?;
        self.valid_pin
            .set_to_push_pull_output()
            .connect_peripheral_to_output(Instance::tx_valid_pin_signal());
        Instance::set_tx_hw_valid_en(true);
        Ok(())
    }
}

/// Pin configuration where the pin for the valid signal is the MSB pin.
pub struct TxPinConfigIncludingValidPin<P>
where
    P: ContainsValidSignalPin + TxPins + ConfigurePins,
{
    tx_pins: P,
}

impl<P> TxPinConfigIncludingValidPin<P>
where
    P: ContainsValidSignalPin + TxPins + ConfigurePins,
{
    pub fn new(tx_pins: P) -> Self {
        Self { tx_pins }
    }
}

impl<P> TxPins for TxPinConfigIncludingValidPin<P> where
    P: ContainsValidSignalPin + TxPins + ConfigurePins
{
}

impl<P> ConfigurePins for TxPinConfigIncludingValidPin<P>
where
    P: ContainsValidSignalPin + TxPins + ConfigurePins,
{
    fn configure(&mut self) -> Result<(), Error> {
        self.tx_pins.configure()?;
        Instance::set_tx_hw_valid_en(true);
        Ok(())
    }
}

macro_rules! tx_pins {
    ($name:ident, $width:literal, $($pin:ident = $signal:ident),+ ) => {
        paste::paste! {
            #[doc = "Data pin configuration for "]
            #[doc = stringify!($width)]
            #[doc = "bit output mode"]
            pub struct $name<'d, $($pin),+> {
                $(
                    [< pin_ $pin:lower >] : PeripheralRef<'d, $pin>,
                )+
            }

            impl<'d, $($pin),+> $name<'d, $($pin),+>
            where
                $($pin: OutputPin),+
            {
                #[allow(clippy::too_many_arguments)]
                pub fn new(
                    $(
                        [< pin_ $pin:lower >] : impl Peripheral<P = $pin> + 'd,
                    )+
                ) -> Self {
                    crate::into_ref!($( [< pin_ $pin:lower >] ),+);
                    Self { $( [< pin_ $pin:lower >] ),+ }
                }
            }

            impl<'d, $($pin),+> ConfigurePins for $name<'d, $($pin),+>
            where
                $($pin: OutputPin),+
            {
                fn configure(&mut self) -> Result<(), Error>{
                    $(
                        self.[< pin_ $pin:lower >]
                            .set_to_push_pull_output()
                            .connect_peripheral_to_output(crate::gpio::OutputSignal::$signal);
                    )+

                    private::Instance::set_tx_bit_width( private::WidSel::[< Bits $width >]);
                    Ok(())
                }
            }

            impl<'d, $($pin),+> TxPins for $name<'d, $($pin),+> {}
        }
    };
}

tx_pins!(TxOneBit, 1, P0 = PARL_TX_DATA0);
tx_pins!(TxTwoBits, 2, P0 = PARL_TX_DATA0, P1 = PARL_TX_DATA1);
tx_pins!(
    TxFourBits,
    4,
    P0 = PARL_TX_DATA0,
    P1 = PARL_TX_DATA1,
    P2 = PARL_TX_DATA2,
    P3 = PARL_TX_DATA3
);
tx_pins!(
    TxEightBits,
    8,
    P0 = PARL_TX_DATA0,
    P1 = PARL_TX_DATA1,
    P2 = PARL_TX_DATA2,
    P3 = PARL_TX_DATA3,
    P4 = PARL_TX_DATA4,
    P5 = PARL_TX_DATA5,
    P6 = PARL_TX_DATA6,
    P7 = PARL_TX_DATA7
);
#[cfg(esp32c6)]
tx_pins!(
    TxSixteenBits,
    16,
    P0 = PARL_TX_DATA0,
    P1 = PARL_TX_DATA1,
    P2 = PARL_TX_DATA2,
    P3 = PARL_TX_DATA3,
    P4 = PARL_TX_DATA4,
    P5 = PARL_TX_DATA5,
    P6 = PARL_TX_DATA6,
    P7 = PARL_TX_DATA7,
    P8 = PARL_TX_DATA8,
    P9 = PARL_TX_DATA9,
    P10 = PARL_TX_DATA10,
    P11 = PARL_TX_DATA11,
    P12 = PARL_TX_DATA12,
    P13 = PARL_TX_DATA13,
    P14 = PARL_TX_DATA14,
    P15 = PARL_TX_DATA15
);

impl<'d, P0> FullDuplex for TxOneBit<'d, P0> {}

impl<'d, P0, P1> FullDuplex for TxTwoBits<'d, P0, P1> {}

impl<'d, P0, P1, P2, P3> FullDuplex for TxFourBits<'d, P0, P1, P2, P3> {}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> FullDuplex
    for TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
{
}

impl<'d, P0> NotContainsValidSignalPin for TxOneBit<'d, P0> {}

impl<'d, P0, P1> NotContainsValidSignalPin for TxTwoBits<'d, P0, P1> {}

impl<'d, P0, P1, P2, P3> NotContainsValidSignalPin for TxFourBits<'d, P0, P1, P2, P3> {}

#[cfg(esp32c6)]
impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> NotContainsValidSignalPin
    for TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
{
}

#[cfg(esp32h2)]
impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> ContainsValidSignalPin
    for TxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
{
}

#[cfg(esp32c6)]
impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
    ContainsValidSignalPin
    for TxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
{
}

/// Pin configuration with an additional pin for the valid signal.
pub struct RxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + RxPins + ConfigurePins,
    VP: InputPin,
{
    rx_pins: P,
    valid_pin: PeripheralRef<'d, VP>,
    enable_mode: EnableMode,
    eof_mode: EofMode,
}

impl<'d, P, VP> RxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + RxPins + ConfigurePins,
    VP: InputPin,
{
    pub fn new(
        rx_pins: P,
        valid_pin: impl Peripheral<P = VP> + 'd,
        enable_mode: EnableMode,
        eof_mode: EofMode,
    ) -> Self {
        crate::into_ref!(valid_pin);
        Self {
            rx_pins,
            valid_pin,
            enable_mode,
            eof_mode,
        }
    }
}

impl<'d, P, VP> RxPins for RxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + RxPins + ConfigurePins,
    VP: InputPin,
{
}

impl<'d, P, VP> ConfigurePins for RxPinConfigWithValidPin<'d, P, VP>
where
    P: NotContainsValidSignalPin + RxPins + ConfigurePins,
    VP: InputPin,
{
    fn configure(&mut self) -> Result<(), Error> {
        self.rx_pins.configure()?;
        self.valid_pin
            .set_to_input()
            .connect_input_to_peripheral(Instance::rx_valid_pin_signal());
        Instance::set_rx_sw_en(false);
        if let Some(sel) = self.enable_mode.pulse_submode_sel() {
            Instance::set_rx_pulse_submode_sel(sel);
        }
        if let Some(sel) = self.enable_mode.level_submode_sel() {
            Instance::set_rx_level_submode_sel(sel);
        }
        if let Some(sel) = self.enable_mode.smp_model_sel() {
            Instance::set_rx_sample_mode(sel);
        }
        Instance::set_eof_gen_sel(self.eof_mode);

        Ok(())
    }
}

/// Pin configuration where the pin for the valid signal is the MSB pin.
pub struct RxPinConfigIncludingValidPin<P>
where
    P: ContainsValidSignalPin + RxPins + ConfigurePins,
{
    rx_pins: P,
    enable_mode: EnableMode,
    eof_mode: EofMode,
}

impl<P> RxPinConfigIncludingValidPin<P>
where
    P: ContainsValidSignalPin + RxPins + ConfigurePins,
{
    pub fn new(rx_pins: P, enable_mode: EnableMode, eof_mode: EofMode) -> Self {
        Self {
            rx_pins,
            enable_mode,
            eof_mode,
        }
    }
}

impl<P> RxPins for RxPinConfigIncludingValidPin<P> where
    P: ContainsValidSignalPin + RxPins + ConfigurePins
{
}

impl<P> ConfigurePins for RxPinConfigIncludingValidPin<P>
where
    P: ContainsValidSignalPin + RxPins + ConfigurePins,
{
    fn configure(&mut self) -> Result<(), Error> {
        self.rx_pins.configure()?;
        Instance::set_rx_sw_en(false);
        if let Some(sel) = self.enable_mode.pulse_submode_sel() {
            Instance::set_rx_pulse_submode_sel(sel);
        }
        if let Some(sel) = self.enable_mode.level_submode_sel() {
            Instance::set_rx_level_submode_sel(sel);
        }
        if let Some(sel) = self.enable_mode.smp_model_sel() {
            Instance::set_rx_sample_mode(sel);
        }
        Instance::set_eof_gen_sel(self.eof_mode);

        Ok(())
    }
}

macro_rules! rx_pins {
    ($name:ident, $width:literal, $($pin:ident = $signal:ident),+ ) => {
        paste::paste! {
            #[doc = "Data pin configuration for "]
            #[doc = stringify!($width)]
            #[doc = "bit input mode"]
            pub struct $name<'d, $($pin),+> {
                $(
                    [< pin_ $pin:lower >] : PeripheralRef<'d, $pin>,
                )+
            }

            impl<'d, $($pin),+> $name<'d, $($pin),+>
            where
                $($pin: InputPin),+
            {
                #[allow(clippy::too_many_arguments)]
                pub fn new(
                    $(
                        [< pin_ $pin:lower >] : impl Peripheral<P = $pin> + 'd,
                    )+
                ) -> Self {
                    crate::into_ref!($( [< pin_ $pin:lower >] ),+);
                    Self { $( [< pin_ $pin:lower >] ),+ }
                }
            }

            impl<'d, $($pin),+> ConfigurePins for $name<'d, $($pin),+>
            where
                $($pin: InputPin),+
            {
                fn configure(&mut self)  -> Result<(), Error> {
                    $(
                        self.[< pin_ $pin:lower >]
                            .set_to_input()
                            .connect_input_to_peripheral(crate::gpio::InputSignal::$signal);
                    )+

                    private::Instance::set_rx_bit_width( private::WidSel::[< Bits $width >]);
                    Ok(())
                }
            }

            impl<'d, $($pin),+> RxPins for $name<'d, $($pin),+> {}
        }
    };
}

rx_pins!(RxOneBit, 1, P0 = PARL_RX_DATA0);
rx_pins!(RxTwoBits, 2, P0 = PARL_RX_DATA0, P1 = PARL_RX_DATA1);
rx_pins!(
    RxFourBits,
    4,
    P0 = PARL_RX_DATA0,
    P1 = PARL_RX_DATA1,
    P2 = PARL_RX_DATA2,
    P3 = PARL_RX_DATA3
);
rx_pins!(
    RxEightBits,
    8,
    P0 = PARL_RX_DATA0,
    P1 = PARL_RX_DATA1,
    P2 = PARL_RX_DATA2,
    P3 = PARL_RX_DATA3,
    P4 = PARL_RX_DATA4,
    P5 = PARL_RX_DATA5,
    P6 = PARL_RX_DATA6,
    P7 = PARL_RX_DATA7
);
#[cfg(esp32c6)]
rx_pins!(
    RxSixteenBits,
    16,
    P0 = PARL_RX_DATA0,
    P1 = PARL_RX_DATA1,
    P2 = PARL_RX_DATA2,
    P3 = PARL_RX_DATA3,
    P4 = PARL_RX_DATA4,
    P5 = PARL_RX_DATA5,
    P6 = PARL_RX_DATA6,
    P7 = PARL_RX_DATA7,
    P8 = PARL_RX_DATA8,
    P9 = PARL_RX_DATA9,
    P10 = PARL_RX_DATA10,
    P11 = PARL_RX_DATA11,
    P12 = PARL_RX_DATA12,
    P13 = PARL_RX_DATA13,
    P14 = PARL_RX_DATA14,
    P15 = PARL_RX_DATA15
);

impl<'d, P0> FullDuplex for RxOneBit<'d, P0> {}

impl<'d, P0, P1> FullDuplex for RxTwoBits<'d, P0, P1> {}

impl<'d, P0, P1, P2, P3> FullDuplex for RxFourBits<'d, P0, P1, P2, P3> {}

impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> FullDuplex
    for RxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
{
}

impl<'d, P0> NotContainsValidSignalPin for RxOneBit<'d, P0> {}

impl<'d, P0, P1> NotContainsValidSignalPin for RxTwoBits<'d, P0, P1> {}

impl<'d, P0, P1, P2, P3> NotContainsValidSignalPin for RxFourBits<'d, P0, P1, P2, P3> {}

#[cfg(esp32c6)]
impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> NotContainsValidSignalPin
    for RxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
{
}

#[cfg(esp32h2)]
impl<'d, P0, P1, P2, P3, P4, P5, P6, P7> ContainsValidSignalPin
    for RxEightBits<'d, P0, P1, P2, P3, P4, P5, P6, P7>
{
}

#[cfg(esp32c6)]
impl<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
    ContainsValidSignalPin
    for RxSixteenBits<'d, P0, P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11, P12, P13, P14, P15>
{
}

impl<'d, CH, DM> TxCreatorFullDuplex<'d, CH, DM>
where
    CH: ChannelTypes,
    DM: Mode,
{
    pub fn with_config<P, CP>(
        self,
        mut tx_pins: P,
        mut clk_pin: CP,
        idle_value: u16,
        sample_edge: SampleEdge,
        bit_order: BitPackOrder,
    ) -> Result<ParlIoTx<'d, CH, P, CP, DM>, Error>
    where
        P: FullDuplex + TxPins + ConfigurePins,
        CP: TxClkPin,
    {
        tx_pins.configure()?;
        clk_pin.configure();

        Instance::set_tx_idle_value(idle_value);
        Instance::set_tx_sample_edge(sample_edge);
        Instance::set_tx_bit_order(bit_order);

        Ok(ParlIoTx {
            tx_channel: self.tx_channel,
            _pins: tx_pins,
            _clk_pin: clk_pin,
            phantom: PhantomData,
        })
    }
}

impl<'d, CH, DM> TxCreator<'d, CH, DM>
where
    CH: ChannelTypes,
    DM: Mode,
{
    pub fn with_config<P, CP>(
        self,
        mut tx_pins: P,
        mut clk_pin: CP,
        idle_value: u16,
        sample_edge: SampleEdge,
        bit_order: BitPackOrder,
    ) -> Result<ParlIoTx<'d, CH, P, CP, DM>, Error>
    where
        P: TxPins + ConfigurePins,
        CP: TxClkPin,
    {
        tx_pins.configure()?;
        clk_pin.configure();

        Instance::set_tx_idle_value(idle_value);
        Instance::set_tx_sample_edge(sample_edge);
        Instance::set_tx_bit_order(bit_order);

        Ok(ParlIoTx {
            tx_channel: self.tx_channel,
            _pins: tx_pins,
            _clk_pin: clk_pin,
            phantom: PhantomData,
        })
    }
}

/// Parallel IO TX channel
pub struct ParlIoTx<'d, CH, P, CP, DM>
where
    CH: ChannelTypes,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
    DM: Mode,
{
    tx_channel: CH::Tx<'d>,
    _pins: P,
    _clk_pin: CP,
    phantom: PhantomData<DM>,
}

impl<'d, CH, P, CP, DM> core::fmt::Debug for ParlIoTx<'d, CH, P, CP, DM>
where
    CH: ChannelTypes,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
    DM: Mode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ParlIoTx").finish()
    }
}

impl<'d, CH, DM> RxCreatorFullDuplex<'d, CH, DM>
where
    CH: ChannelTypes,
    DM: Mode,
{
    pub fn with_config<P, CP>(
        self,
        mut rx_pins: P,
        mut clk_pin: CP,
        bit_order: BitPackOrder,
        timeout_ticks: Option<u16>,
    ) -> Result<ParlIoRx<'d, CH, P, CP, DM>, Error>
    where
        P: FullDuplex + RxPins + ConfigurePins,
        CP: RxClkPin,
    {
        rx_pins.configure()?;
        clk_pin.configure();

        Instance::set_rx_bit_order(bit_order);
        Instance::set_rx_timeout_ticks(timeout_ticks);

        Ok(ParlIoRx {
            rx_channel: self.rx_channel,
            _pins: rx_pins,
            _clk_pin: clk_pin,
            phantom: PhantomData,
        })
    }
}

impl<'d, CH, DM> RxCreator<'d, CH, DM>
where
    CH: ChannelTypes,
    DM: Mode,
{
    pub fn with_config<P, CP>(
        self,
        mut rx_pins: P,
        mut clk_pin: CP,
        bit_order: BitPackOrder,
        timeout_ticks: Option<u16>,
    ) -> Result<ParlIoRx<'d, CH, P, CP, DM>, Error>
    where
        P: RxPins + ConfigurePins,
        CP: RxClkPin,
    {
        rx_pins.configure()?;
        clk_pin.configure();

        Instance::set_rx_bit_order(bit_order);
        Instance::set_rx_timeout_ticks(timeout_ticks);

        Ok(ParlIoRx {
            rx_channel: self.rx_channel,
            _pins: rx_pins,
            _clk_pin: clk_pin,
            phantom: PhantomData,
        })
    }
}

/// Parallel IO RX channel
pub struct ParlIoRx<'d, CH, P, CP, DM>
where
    CH: ChannelTypes,
    P: RxPins + ConfigurePins,
    CP: RxClkPin,
    DM: Mode,
{
    rx_channel: CH::Rx<'d>,
    _pins: P,
    _clk_pin: CP,
    phantom: PhantomData<DM>,
}

impl<'d, CH, P, CP, DM> core::fmt::Debug for ParlIoRx<'d, CH, P, CP, DM>
where
    CH: ChannelTypes,
    P: RxPins + ConfigurePins,
    CP: RxClkPin,
    DM: Mode,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ParlIoTx").finish()
    }
}

fn internal_set_interrupt_handler(handler: InterruptHandler) {
    #[cfg(esp32c6)]
    {
        unsafe { crate::peripherals::PARL_IO::steal() }.bind_parl_io_interrupt(handler.handler());

        crate::interrupt::enable(crate::peripherals::Interrupt::PARL_IO, handler.priority())
            .unwrap();
    }
    #[cfg(esp32h2)]
    {
        unsafe { crate::peripherals::PARL_IO::steal() }
            .bind_parl_io_tx_interrupt(handler.handler());
        unsafe { crate::peripherals::PARL_IO::steal() }
            .bind_parl_io_rx_interrupt(handler.handler());

        crate::interrupt::enable(
            crate::peripherals::Interrupt::PARL_IO_TX,
            handler.priority(),
        )
        .unwrap();
        crate::interrupt::enable(
            crate::peripherals::Interrupt::PARL_IO_RX,
            handler.priority(),
        )
        .unwrap();
    }
}

fn internal_listen(interrupts: EnumSet<ParlIoInterrupt>) {
    let parl_io = unsafe { crate::peripherals::PARL_IO::steal() };
    for interrupt in interrupts {
        match interrupt {
            ParlIoInterrupt::TxFifoReEmpty => parl_io
                .int_ena()
                .modify(|_, w| w.tx_fifo_rempty().set_bit()),
            ParlIoInterrupt::RxFifoWOvf => {
                parl_io.int_ena().modify(|_, w| w.rx_fifo_wovf().set_bit())
            }
            ParlIoInterrupt::TxEof => parl_io.int_ena().write(|w| w.tx_eof().set_bit()),
        }
    }
}

fn internal_unlisten(interrupts: EnumSet<ParlIoInterrupt>) {
    let parl_io = unsafe { crate::peripherals::PARL_IO::steal() };
    for interrupt in interrupts {
        match interrupt {
            ParlIoInterrupt::TxFifoReEmpty => parl_io
                .int_ena()
                .modify(|_, w| w.tx_fifo_rempty().clear_bit()),
            ParlIoInterrupt::RxFifoWOvf => parl_io
                .int_ena()
                .modify(|_, w| w.rx_fifo_wovf().clear_bit()),
            ParlIoInterrupt::TxEof => parl_io.int_ena().write(|w| w.tx_eof().clear_bit()),
        }
    }
}

fn internal_interrupts() -> EnumSet<ParlIoInterrupt> {
    let mut res = EnumSet::new();
    let parl_io = unsafe { crate::peripherals::PARL_IO::steal() };
    let ints = parl_io.int_st().read();
    if ints.tx_fifo_rempty().bit() {
        res.insert(ParlIoInterrupt::TxFifoReEmpty);
    }
    if ints.rx_fifo_wovf().bit() {
        res.insert(ParlIoInterrupt::RxFifoWOvf);
    }
    if ints.tx_eof().bit() {
        res.insert(ParlIoInterrupt::TxEof);
    }

    res
}

fn internal_clear_interrupts(interrupts: EnumSet<ParlIoInterrupt>) {
    let parl_io = unsafe { crate::peripherals::PARL_IO::steal() };
    for interrupt in interrupts {
        match interrupt {
            ParlIoInterrupt::TxFifoReEmpty => parl_io
                .int_clr()
                .write(|w| w.tx_fifo_rempty().clear_bit_by_one()),
            ParlIoInterrupt::RxFifoWOvf => parl_io
                .int_clr()
                .write(|w| w.rx_fifo_wovf().clear_bit_by_one()),
            ParlIoInterrupt::TxEof => parl_io.int_clr().write(|w| w.tx_eof().clear_bit_by_one()),
        }
    }
}

/// Parallel IO in full duplex mode
///
/// Full duplex mode might limit the maximum possible bit width.
pub struct ParlIoFullDuplex<'d, CH, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    DM: Mode,
{
    pub tx: TxCreatorFullDuplex<'d, CH, DM>,
    pub rx: RxCreatorFullDuplex<'d, CH, DM>,
}

impl<'d, CH, DM> ParlIoFullDuplex<'d, CH, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    DM: Mode,
{
    pub fn new(
        _parl_io: impl Peripheral<P = peripherals::PARL_IO> + 'd,
        mut dma_channel: Channel<'d, CH, DM>,
        frequency: HertzU32,
        _clocks: &Clocks,
    ) -> Result<Self, Error> {
        internal_init(&mut dma_channel, frequency)?;

        Ok(Self {
            tx: TxCreatorFullDuplex {
                tx_channel: dma_channel.tx,
                phantom: PhantomData,
            },
            rx: RxCreatorFullDuplex {
                rx_channel: dma_channel.rx,
                phantom: PhantomData,
            },
        })
    }
}

impl<'d, CH> ParlIoFullDuplex<'d, CH, Blocking>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    /// Sets the interrupt handler, enables it with
    /// [crate::interrupt::Priority::min()]
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        internal_set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_listen(interrupts);
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_unlisten(interrupts);
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<ParlIoInterrupt> {
        internal_interrupts()
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_clear_interrupts(interrupts);
    }
}

/// Parallel IO in half duplex / TX only mode
pub struct ParlIoTxOnly<'d, CH, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    DM: Mode,
{
    pub tx: TxCreator<'d, CH, DM>,
}

impl<'d, CH, DM> ParlIoTxOnly<'d, CH, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    DM: Mode,
{
    pub fn new(
        _parl_io: impl Peripheral<P = peripherals::PARL_IO> + 'd,
        mut dma_channel: Channel<'d, CH, DM>,
        frequency: HertzU32,
        _clocks: &Clocks,
    ) -> Result<Self, Error> {
        internal_init(&mut dma_channel, frequency)?;

        Ok(Self {
            tx: TxCreator {
                tx_channel: dma_channel.tx,
                phantom: PhantomData,
            },
        })
    }
}

impl<'d, CH> ParlIoTxOnly<'d, CH, Blocking>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    /// Sets the interrupt handler, enables it with
    /// [crate::interrupt::Priority::min()]
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        internal_set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_listen(interrupts);
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_unlisten(interrupts);
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<ParlIoInterrupt> {
        internal_interrupts()
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_clear_interrupts(interrupts);
    }
}

/// Parallel IO in half duplex / RX only mode
pub struct ParlIoRxOnly<'d, CH, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    DM: Mode,
{
    pub rx: RxCreator<'d, CH, DM>,
}

impl<'d, CH, DM> ParlIoRxOnly<'d, CH, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    DM: Mode,
{
    pub fn new(
        _parl_io: impl Peripheral<P = peripherals::PARL_IO> + 'd,
        mut dma_channel: Channel<'d, CH, DM>,
        frequency: HertzU32,
        _clocks: &Clocks,
    ) -> Result<Self, Error> {
        internal_init(&mut dma_channel, frequency)?;

        Ok(Self {
            rx: RxCreator {
                rx_channel: dma_channel.rx,
                phantom: PhantomData,
            },
        })
    }
}

impl<'d, CH> ParlIoRxOnly<'d, CH, Blocking>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    /// Sets the interrupt handler, enables it with
    /// [crate::interrupt::Priority::min()]
    ///
    /// Interrupts are not enabled at the peripheral level here.
    pub fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        internal_set_interrupt_handler(handler);
    }

    /// Listen for the given interrupts
    pub fn listen(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_listen(interrupts);
    }

    /// Unlisten the given interrupts
    pub fn unlisten(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_unlisten(interrupts);
    }

    /// Gets asserted interrupts
    pub fn interrupts(&mut self) -> EnumSet<ParlIoInterrupt> {
        internal_interrupts()
    }

    /// Resets asserted interrupts
    pub fn clear_interrupts(&mut self, interrupts: EnumSet<ParlIoInterrupt>) {
        internal_clear_interrupts(interrupts);
    }
}

fn internal_init<CH, DM>(
    dma_channel: &mut Channel<'_, CH, DM>,
    frequency: HertzU32,
) -> Result<(), Error>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    DM: Mode,
{
    if frequency.raw() > 40_000_000 {
        return Err(Error::UnreachableClockRate);
    }

    PeripheralClockControl::enable(crate::system::Peripheral::ParlIo);

    let pcr = unsafe { &*crate::peripherals::PCR::PTR };

    let divider = crate::soc::constants::PARL_IO_SCLK / frequency.raw();
    if divider > 0xffff {
        return Err(Error::UnreachableClockRate);
    }
    let divider = divider as u16;

    pcr.parl_clk_tx_conf().modify(|_, w| {
        w.parl_clk_tx_en()
            .set_bit()
            .parl_clk_tx_sel()
            .variant(1) // PLL
            .parl_clk_tx_div_num()
            .variant(divider)
    });

    pcr.parl_clk_rx_conf().modify(|_, w| {
        w.parl_clk_rx_en()
            .set_bit()
            .parl_clk_rx_sel()
            .variant(1) // PLL
            .parl_clk_rx_div_num()
            .variant(divider)
    });
    Instance::set_rx_sw_en(true);
    Instance::set_rx_sample_mode(SampleMode::InternalSoftwareEnable);

    dma_channel.tx.init_channel();
    dma_channel.rx.init_channel();
    Ok(())
}

impl<'d, CH, P, CP, DM> ParlIoTx<'d, CH, P, CP, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
    DM: Mode,
{
    /// Perform a DMA write.
    ///
    /// This will return a [DmaTransfer] owning the buffer(s) and the driver
    /// instance.
    ///
    /// The maximum amount of data to be sent is 32736 bytes.
    pub fn write_dma<'t, TXBUF>(
        &'t mut self,
        words: &'t TXBUF,
    ) -> Result<DmaTransfer<'t, 'd, CH, P, CP, DM>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { words.read_buffer() };

        if len > MAX_DMA_SIZE {
            return Err(Error::MaxDmaTransferSizeExceeded);
        }

        self.start_write_bytes_dma(ptr, len)?;

        Ok(DmaTransfer { instance: self })
    }

    fn start_write_bytes_dma(&mut self, ptr: *const u8, len: usize) -> Result<(), Error> {
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };
        pcr.parl_clk_tx_conf()
            .modify(|_, w| w.parl_tx_rst_en().set_bit());
        pcr.parl_clk_tx_conf()
            .modify(|_, w| w.parl_tx_rst_en().clear_bit());

        Instance::clear_tx_interrupts();
        Instance::set_tx_bytes(len as u16);

        self.tx_channel.is_done();

        self.tx_channel
            .prepare_transfer_without_start(DmaPeripheral::ParlIo, false, ptr, len)
            .and_then(|_| self.tx_channel.start_transfer())?;

        loop {
            if Instance::is_tx_ready() {
                break;
            }
        }

        Instance::set_tx_start(true);
        Ok(())
    }
}

/// An in-progress DMA transfer.
#[must_use]
pub struct DmaTransfer<'t, 'd, C, P, CP, DM>
where
    C: ChannelTypes,
    C::P: ParlIoPeripheral,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
    DM: Mode,
{
    instance: &'t mut ParlIoTx<'d, C, P, CP, DM>,
}

impl<'t, 'd, C, P, CP, DM> DmaTransfer<'t, 'd, C, P, CP, DM>
where
    C: ChannelTypes,
    C::P: ParlIoPeripheral,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
    DM: Mode,
{
    /// Wait for the DMA transfer to complete
    #[allow(clippy::type_complexity)]
    pub fn wait(self) -> Result<(), DmaError> {
        // Waiting for the DMA transfer is not enough. We need to wait for the
        // peripheral to finish flushing its buffers, too.
        while !Instance::is_tx_eof() {}

        Instance::set_tx_start(false);

        if self.instance.tx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    /// Check if the DMA transfer is complete
    pub fn is_done(&self) -> bool {
        let ch = &self.instance.tx_channel;
        ch.is_done()
    }
}

impl<'d, CH, P, CP, DM> ParlIoRx<'d, CH, P, CP, DM>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    P: RxPins + ConfigurePins,
    CP: RxClkPin,
    DM: Mode,
{
    /// Perform a DMA read.
    ///
    /// This will return a [RxDmaTransfer] owning the buffer(s) and the driver
    /// instance.
    ///
    /// The maximum amount of data is 32736 bytes when using [EofMode::ByteLen].
    ///
    /// It's only limited by the size of the DMA buffer when using
    /// [EofMode::EnableSignal].
    pub fn read_dma<'t, RXBUF>(
        &'t mut self,
        words: &'t mut RXBUF,
    ) -> Result<RxDmaTransfer<'t, 'd, CH, P, CP, DM>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { words.write_buffer() };

        if !Instance::is_suc_eof_generated_externally() && len > MAX_DMA_SIZE {
            return Err(Error::MaxDmaTransferSizeExceeded);
        }

        self.start_receive_bytes_dma(ptr, len)?;

        Ok(RxDmaTransfer { instance: self })
    }

    fn start_receive_bytes_dma(&mut self, ptr: *mut u8, len: usize) -> Result<(), Error> {
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };
        pcr.parl_clk_rx_conf()
            .modify(|_, w| w.parl_rx_rst_en().set_bit());
        pcr.parl_clk_rx_conf()
            .modify(|_, w| w.parl_rx_rst_en().clear_bit());

        Instance::clear_rx_interrupts();
        Instance::set_rx_bytes(len as u16);

        self.rx_channel
            .prepare_transfer_without_start(false, DmaPeripheral::ParlIo, ptr, len)
            .and_then(|_| self.rx_channel.start_transfer())?;

        Instance::set_rx_reg_update();

        Instance::set_rx_start(true);
        Ok(())
    }
}

/// An in-progress DMA transfer.
pub struct RxDmaTransfer<'t, 'd, C, P, CP, DM>
where
    C: ChannelTypes,
    C::P: ParlIoPeripheral,
    P: RxPins + ConfigurePins,
    CP: RxClkPin,
    DM: Mode,
{
    instance: &'t mut ParlIoRx<'d, C, P, CP, DM>,
}

impl<'t, 'd, C, P, CP, DM> RxDmaTransfer<'t, 'd, C, P, CP, DM>
where
    C: ChannelTypes,
    C::P: ParlIoPeripheral,
    P: RxPins + ConfigurePins,
    CP: RxClkPin,
    DM: Mode,
{
    /// Wait for the DMA transfer to complete
    #[allow(clippy::type_complexity)]
    pub fn wait(self) -> Result<(), DmaError> {
        loop {
            if self.is_done() || self.is_eof_error() {
                break;
            }
        }

        Instance::set_rx_start(false);

        if self.instance.rx_channel.has_error() {
            Err(DmaError::DescriptorError)
        } else {
            Ok(())
        }
    }

    /// Check if the DMA transfer is complete
    pub fn is_done(&self) -> bool {
        let ch = &self.instance.rx_channel;
        ch.is_done()
    }

    /// Check if the DMA transfer is completed by buffer full or source EOF
    /// error
    pub fn is_eof_error(&self) -> bool {
        let ch = &self.instance.rx_channel;
        ch.has_eof_error() || ch.has_dscr_empty_error()
    }
}

#[cfg(feature = "async")]
pub mod asynch {
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use super::{
        private::{ConfigurePins, Instance, RxClkPin, RxPins, TxClkPin, TxPins},
        Error,
        ParlIoRx,
        ParlIoTx,
        MAX_DMA_SIZE,
    };
    use crate::{
        dma::{asynch::DmaRxDoneChFuture, ChannelTypes, ParlIoPeripheral},
        peripherals::Interrupt,
    };

    static TX_WAKER: AtomicWaker = AtomicWaker::new();

    pub struct TxDoneFuture {}

    impl TxDoneFuture {
        pub fn new() -> Self {
            Instance::listen_tx_done();
            Self {}
        }
    }

    impl core::future::Future for TxDoneFuture {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> Poll<Self::Output> {
            let mut parl_io = unsafe { crate::peripherals::PARL_IO::steal() };

            #[cfg(esp32c6)]
            {
                parl_io.bind_parl_io_interrupt(interrupt_handler.handler());
                crate::interrupt::enable(Interrupt::PARL_IO, interrupt_handler.priority()).unwrap();
            }
            #[cfg(esp32h2)]
            {
                parl_io.bind_parl_io_tx_interrupt(interrupt_handler.handler());
                crate::interrupt::enable(Interrupt::PARL_IO_TX, interrupt_handler.priority())
                    .unwrap();
            }

            TX_WAKER.register(cx.waker());
            if Instance::is_listening_tx_done() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        }
    }

    #[handler]
    fn interrupt_handler() {
        if Instance::is_tx_done_set() {
            Instance::clear_is_tx_done();
            Instance::unlisten_tx_done();
            TX_WAKER.wake()
        }
    }

    impl<'d, CH, P, CP> ParlIoTx<'d, CH, P, CP, crate::Async>
    where
        CH: ChannelTypes,
        CH::P: ParlIoPeripheral,
        P: TxPins + ConfigurePins,
        CP: TxClkPin,
    {
        /// Perform a DMA write.
        ///
        /// The maximum amount of data to be sent is 32736 bytes.
        pub async fn write_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            let (ptr, len) = (words.as_ptr(), words.len());

            if len > MAX_DMA_SIZE {
                return Err(Error::MaxDmaTransferSizeExceeded);
            }

            self.start_write_bytes_dma(ptr, len)?;

            TxDoneFuture::new().await;

            Ok(())
        }
    }

    impl<'d, CH, P, CP> ParlIoRx<'d, CH, P, CP, crate::Async>
    where
        CH: ChannelTypes,
        CH::P: ParlIoPeripheral,
        P: RxPins + ConfigurePins,
        CP: RxClkPin,
    {
        /// Perform a DMA write.
        ///
        /// The maximum amount of data to be sent is 32736 bytes.
        pub async fn read_dma_async(&mut self, words: &mut [u8]) -> Result<(), Error> {
            let (ptr, len) = (words.as_mut_ptr(), words.len());

            if !Instance::is_suc_eof_generated_externally() {
                if len > MAX_DMA_SIZE {
                    return Err(Error::MaxDmaTransferSizeExceeded);
                }
            }

            self.start_receive_bytes_dma(ptr, len)?;

            DmaRxDoneChFuture::new(&mut self.rx_channel).await;

            Ok(())
        }
    }
}

mod private {
    use core::marker::PhantomData;

    use super::{BitPackOrder, EofMode, Error, SampleEdge};
    use crate::{dma::ChannelTypes, Mode};

    pub trait FullDuplex {}

    pub trait NotContainsValidSignalPin {}

    pub trait ContainsValidSignalPin {}

    pub trait TxPins {}

    pub trait RxPins {}

    pub trait TxClkPin {
        fn configure(&mut self);
    }

    pub trait RxClkPin {
        fn configure(&mut self);
    }

    pub trait ConfigurePins {
        fn configure(&mut self) -> Result<(), Error>;
    }

    pub struct TxCreator<'d, CH, DM>
    where
        CH: ChannelTypes,
        DM: Mode,
    {
        pub tx_channel: CH::Tx<'d>,
        pub(crate) phantom: PhantomData<DM>,
    }

    pub struct RxCreator<'d, CH, DM>
    where
        CH: ChannelTypes,
        DM: Mode,
    {
        pub rx_channel: CH::Rx<'d>,
        pub(crate) phantom: PhantomData<DM>,
    }

    pub struct TxCreatorFullDuplex<'d, CH, DM>
    where
        CH: ChannelTypes,
        DM: Mode,
    {
        pub tx_channel: CH::Tx<'d>,
        pub(crate) phantom: PhantomData<DM>,
    }

    pub struct RxCreatorFullDuplex<'d, CH, DM>
    where
        CH: ChannelTypes,
        DM: Mode,
    {
        pub rx_channel: CH::Rx<'d>,
        pub(crate) phantom: PhantomData<DM>,
    }

    #[cfg(esp32c6)]
    pub(super) enum WidSel {
        Bits16 = 0,
        Bits8  = 1,
        Bits4  = 2,
        Bits2  = 3,
        Bits1  = 4,
    }

    #[cfg(esp32h2)]
    pub(super) enum WidSel {
        Bits8 = 3,
        Bits4 = 2,
        Bits2 = 1,
        Bits1 = 0,
    }

    pub(super) enum SampleMode {
        ExternalLevel          = 0,
        ExternalPulse          = 1,
        InternalSoftwareEnable = 2,
    }

    pub(super) struct Instance;

    #[cfg(esp32c6)]
    impl Instance {
        pub fn set_tx_bit_width(width: WidSel) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_cfg0()
                .modify(|_, w| w.tx_bus_wid_sel().variant(width as u8));
        }

        pub fn set_tx_idle_value(value: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_cfg1()
                .modify(|_, w| w.tx_idle_value().variant(value));
        }

        pub fn set_tx_sample_edge(value: SampleEdge) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_cfg0()
                .modify(|_, w| w.tx_smp_edge_sel().variant(value as u8 == 1));
        }

        pub fn set_tx_bit_order(value: BitPackOrder) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_cfg0()
                .modify(|_, w| w.tx_bit_unpack_order().variant(value as u8 == 1));
        }

        pub fn clear_tx_interrupts() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_clr().write(|w| {
                w.tx_fifo_rempty()
                    .clear_bit_by_one()
                    .tx_eof()
                    .clear_bit_by_one()
            });
        }

        pub fn set_tx_bytes(len: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_cfg0()
                .modify(|_, w| w.tx_bytelen().variant(len));
        }

        pub fn is_tx_ready() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.st().read().tx_ready().bit_is_set()
        }

        pub fn set_tx_start(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.tx_cfg0().modify(|_, w| w.tx_start().bit(value));
        }

        pub fn is_tx_eof() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_raw().read().tx_eof().bit_is_set()
        }

        pub fn tx_valid_pin_signal() -> crate::gpio::OutputSignal {
            crate::gpio::OutputSignal::PARL_TX_DATA15
        }

        pub fn set_tx_hw_valid_en(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_cfg0()
                .modify(|_, w| w.tx_hw_valid_en().bit(value));
        }

        pub fn set_rx_bit_width(width: WidSel) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_bus_wid_sel().variant(width as u8));
        }

        pub fn rx_valid_pin_signal() -> crate::gpio::InputSignal {
            crate::gpio::InputSignal::PARL_RX_DATA15
        }

        pub fn set_rx_sw_en(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.rx_cfg0().modify(|_, w| w.rx_sw_en().bit(value));
        }

        pub fn clear_rx_interrupts() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .int_clr()
                .write(|w| w.rx_fifo_wovf().clear_bit_by_one());
        }

        pub fn set_rx_bytes(len: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_data_bytelen().variant(len));
        }

        pub fn set_rx_sample_mode(sample_mode: SampleMode) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_smp_mode_sel().variant(sample_mode as u8));
        }

        pub fn set_eof_gen_sel(mode: EofMode) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_eof_gen_sel().variant(mode == EofMode::EnableSignal));
        }

        pub fn set_rx_pulse_submode_sel(sel: u8) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_pulse_submode_sel().variant(sel));
        }

        pub fn set_rx_level_submode_sel(sel: u8) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_level_submode_sel().variant(sel == 1));
        }

        pub fn set_rx_clk_edge_sel(edge: SampleEdge) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_clk_edge_sel().variant(edge as u8 == 1));
        }

        pub fn set_rx_start(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.rx_cfg0().modify(|_, w| w.rx_start().bit(value));
        }

        pub fn set_rx_reg_update() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_cfg1()
                .modify(|_, w| w.rx_reg_update().bit(true));
        }

        pub fn set_rx_bit_order(value: BitPackOrder) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .rx_cfg0()
                .modify(|_, w| w.rx_bit_pack_order().variant(value as u8 == 1));
        }

        pub fn set_rx_timeout_ticks(value: Option<u16>) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block.rx_cfg1().modify(|_, w| {
                w.rx_timeout_en()
                    .bit(value.is_some())
                    .rx_timeout_threshold()
                    .variant(value.unwrap_or(0xfff))
            });
        }

        pub fn is_suc_eof_generated_externally() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.rx_cfg0().read().rx_eof_gen_sel().bit_is_set()
        }

        #[cfg(feature = "async")]
        pub fn listen_tx_done() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_ena().modify(|_, w| w.tx_eof().set_bit());
        }

        #[cfg(feature = "async")]
        pub fn unlisten_tx_done() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_ena().modify(|_, w| w.tx_eof().clear_bit());
        }

        #[cfg(feature = "async")]
        pub fn is_listening_tx_done() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_ena().read().tx_eof().bit()
        }

        #[cfg(feature = "async")]
        pub fn is_tx_done_set() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_raw().read().tx_eof().bit()
        }

        #[cfg(feature = "async")]
        pub fn clear_is_tx_done() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_clr().write(|w| w.tx_eof().clear_bit_by_one());
        }
    }

    #[cfg(esp32h2)]
    impl Instance {
        pub fn set_tx_bit_width(width: WidSel) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_data_cfg()
                .modify(|_, w| w.tx_bus_wid_sel().variant(width as u8));
        }

        pub fn set_tx_idle_value(value: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_genrl_cfg()
                .modify(|_, w| w.tx_idle_value().variant(value));
        }

        pub fn set_tx_sample_edge(value: SampleEdge) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block.tx_clk_cfg().modify(|_, w| {
                w.tx_clk_i_inv()
                    .bit(value == SampleEdge::Invert)
                    .tx_clk_o_inv()
                    .bit(value == SampleEdge::Invert)
            });
        }

        pub fn set_tx_bit_order(value: BitPackOrder) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_data_cfg()
                .modify(|_, w| w.tx_data_order_inv().variant(value as u8 == 1));
        }

        pub fn clear_tx_interrupts() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_clr().write(|w| {
                w.tx_fifo_rempty()
                    .clear_bit_by_one()
                    .tx_eof()
                    .clear_bit_by_one()
            });
        }

        pub fn set_tx_bytes(len: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_data_cfg()
                .modify(|_, w| w.tx_bitlen().variant((len as u32) * 8));
        }

        pub fn is_tx_ready() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.st().read().tx_ready().bit_is_set()
        }

        pub fn set_tx_start(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_start_cfg()
                .modify(|_, w| w.tx_start().bit(value));
        }

        pub fn is_tx_eof() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_raw().read().tx_eof().bit_is_set()
        }

        pub fn tx_valid_pin_signal() -> crate::gpio::OutputSignal {
            crate::gpio::OutputSignal::PARL_TX_DATA7
        }

        pub fn set_tx_hw_valid_en(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_genrl_cfg()
                .modify(|_, w| w.tx_valid_output_en().bit(value));
        }

        pub fn set_rx_bit_width(width: WidSel) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_data_cfg()
                .modify(|_, w| w.rx_bus_wid_sel().variant(width as u8));
        }

        pub fn rx_valid_pin_signal() -> crate::gpio::InputSignal {
            crate::gpio::InputSignal::PARL_RX_DATA7
        }

        pub fn set_rx_sw_en(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_mode_cfg()
                .modify(|_, w| w.rx_sw_en().bit(value));
        }

        pub fn clear_rx_interrupts() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .int_clr()
                .write(|w| w.rx_fifo_wovf().clear_bit_by_one());
        }

        pub fn set_rx_bytes(len: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_data_cfg()
                .modify(|_, w| w.rx_bitlen().variant((len as u32) * 8));
        }

        pub fn set_rx_sample_mode(sample_mode: SampleMode) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_mode_cfg()
                .modify(|_, w| w.rx_smp_mode_sel().variant(sample_mode as u8));
        }

        pub fn set_eof_gen_sel(mode: EofMode) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_genrl_cfg()
                .modify(|_, w| w.rx_eof_gen_sel().variant(mode == EofMode::EnableSignal));
        }

        pub fn set_rx_pulse_submode_sel(sel: u8) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_mode_cfg()
                .modify(|_, w| w.rx_pulse_submode_sel().variant(sel));
        }

        pub fn set_rx_level_submode_sel(_sel: u8) {
            // unsupported, always high
        }

        pub fn set_rx_clk_edge_sel(value: SampleEdge) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.rx_clk_cfg().modify(|_, w| {
                w.rx_clk_i_inv()
                    .bit(value == SampleEdge::Invert)
                    .rx_clk_o_inv()
                    .bit(value == SampleEdge::Invert)
            });
        }

        pub fn set_rx_start(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_start_cfg()
                .modify(|_, w| w.rx_start().bit(value));
        }

        pub fn set_rx_reg_update() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .reg_update()
                .write(|w| w.rx_reg_update().bit(true));
        }

        pub fn set_rx_bit_order(value: BitPackOrder) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .rx_data_cfg()
                .modify(|_, w| w.rx_data_order_inv().variant(value as u8 == 1));
        }

        pub fn set_rx_timeout_ticks(value: Option<u16>) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block.rx_genrl_cfg().modify(|_, w| {
                w.rx_timeout_en()
                    .bit(value.is_some())
                    .rx_timeout_thres()
                    .variant(value.unwrap_or(0xfff))
            });
        }

        pub fn is_suc_eof_generated_externally() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .rx_genrl_cfg()
                .read()
                .rx_eof_gen_sel()
                .bit_is_set()
        }

        #[cfg(feature = "async")]
        pub fn listen_tx_done() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_ena().modify(|_, w| w.tx_eof().set_bit());
        }

        #[cfg(feature = "async")]
        pub fn unlisten_tx_done() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_ena().modify(|_, w| w.tx_eof().clear_bit());
        }

        #[cfg(feature = "async")]
        pub fn is_listening_tx_done() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_ena().read().tx_eof().bit()
        }

        #[cfg(feature = "async")]
        pub fn is_tx_done_set() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_raw().read().tx_eof().bit()
        }

        #[cfg(feature = "async")]
        pub fn clear_is_tx_done() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_clr().write(|w| w.tx_eof().clear_bit_by_one());
        }
    }
}
