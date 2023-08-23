//! # Parallel IO
//!
//! ## Overview
//! The Parallel IO peripheral is a general purpose parallel interface that can
//! be used to connect to external devices such as LED matrix, LCD display,
//! Printer and Camera. The peripheral has independent TX and RX units. Each
//! unit can have up to 8 or 16 data signals (depending on your target hardware)
//! plus 1 or 2 clock signals.
//!
//! At the moment, the Parallel IO driver only supports TX mode. The RX feature
//! is still working in progress.
//!
//! The driver uses DMA (Direct Memory Access) for efficient data transfer.
//!
//! ## Examples
//!
//! ### Initialization
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
//!     1u32.MHz(),
//!     &mut system.peripheral_clock_control,
//!     &clocks,
//! )
//! .unwrap();
//!
//! // configure a pin for the clock signal
//! let cp = ClkOutPin::new(io.pins.gpio6);
//!
//! let mut parl_io_tx =
//!     parl_io
//!         .tx
//!         .with_config(pin_conf, cp, 0, SampleEdge::Normal, BitPackOrder::Msb);
//! ```

use core::mem;

use embedded_dma::ReadBuffer;
use fugit::HertzU32;
use peripheral::PeripheralRef;
use private::*;

use crate::{
    clock::Clocks,
    dma::{Channel, ChannelTypes, DmaError, DmaPeripheral, ParlIoPeripheral, TxPrivate},
    gpio::{InputPin, OutputPin},
    peripheral::{self, Peripheral},
    peripherals,
    system::PeripheralClockControl,
};

#[allow(unused)]
const MAX_DMA_SIZE: usize = 32736;

/// Parallel IO errors
#[derive(Debug, Clone, Copy)]
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
pub enum SampleEdge {
    /// Positive edge
    Normal = 0,
    /// Negative edge
    Invert = 1,
}

/// Parallel IO bit packing order
#[derive(Debug, Clone, Copy)]
pub enum BitPackOrder {
    /// Bit pack order: MSB
    Msb = 0,
    /// Bit pack order: LSB
    Lsb = 1,
}

/// Used to configure no pin as clock output
pub struct NoClkPin;
impl TxClkPin for NoClkPin {
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

/// Wraps a GPIO pin which will be used as the clock input signal
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
        pcr.parl_clk_tx_conf.modify(|_, w| {
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
    fn configure(&mut self) {
        self.tx_pins.configure();
        self.valid_pin
            .set_to_push_pull_output()
            .connect_peripheral_to_output(Instance::tx_valid_pin_signal());
        Instance::set_tx_hw_valid_en(true);
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
    fn configure(&mut self) {
        self.tx_pins.configure();
        Instance::set_tx_hw_valid_en(true);
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
                fn configure(&mut self) {
                    $(
                        self.[< pin_ $pin:lower >]
                            .set_to_push_pull_output()
                            .connect_peripheral_to_output(crate::gpio::OutputSignal::$signal);
                    )+

                    private::Instance::set_tx_bit_width( private::WidSel::[< Bits $width >]);
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

trait RegisterAccess {}

impl<'d, CH> TxCreatorFullDuplex<'d, CH>
where
    CH: ChannelTypes,
{
    pub fn with_config<P, CP>(
        self,
        mut tx_pins: P,
        mut clk_pin: CP,
        idle_value: u16,
        sample_edge: SampleEdge,
        bit_order: BitPackOrder,
    ) -> ParlIoTx<'d, CH, P, CP>
    where
        P: FullDuplex + TxPins + ConfigurePins,
        CP: TxClkPin,
    {
        tx_pins.configure();
        clk_pin.configure();

        Instance::set_tx_idle_value(idle_value);
        Instance::set_tx_sample_edge(sample_edge);
        Instance::set_tx_bit_order(bit_order);

        ParlIoTx {
            tx_channel: self.tx_channel,
            _pins: tx_pins,
            _clk_pin: clk_pin,
        }
    }
}

impl<'d, CH> TxCreator<'d, CH>
where
    CH: ChannelTypes,
{
    pub fn with_config<P, CP>(
        self,
        mut tx_pins: P,
        mut clk_pin: CP,
        idle_value: u16,
        sample_edge: SampleEdge,
        bit_order: BitPackOrder,
    ) -> ParlIoTx<'d, CH, P, CP>
    where
        P: TxPins + ConfigurePins,
        CP: TxClkPin,
    {
        tx_pins.configure();
        clk_pin.configure();

        Instance::set_tx_idle_value(idle_value);
        Instance::set_tx_sample_edge(sample_edge);
        Instance::set_tx_bit_order(bit_order);

        ParlIoTx {
            tx_channel: self.tx_channel,
            _pins: tx_pins,
            _clk_pin: clk_pin,
        }
    }
}

/// Parallel IO TX channel
pub struct ParlIoTx<'d, CH, P, CP>
where
    CH: ChannelTypes,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
{
    tx_channel: CH::Tx<'d>,
    _pins: P,
    _clk_pin: CP,
}

impl<'d, CH, P, CP> core::fmt::Debug for ParlIoTx<'d, CH, P, CP>
where
    CH: ChannelTypes,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ParlIoTx").finish()
    }
}

/// Parallel IO in full duplex mode
///
/// Full duplex mode might limit the maximum possible bit width.
pub struct ParlIoFullDuplex<'d, CH>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    _parl_io: PeripheralRef<'d, peripherals::PARL_IO>,
    pub tx: TxCreatorFullDuplex<'d, CH>,
    pub rx: RxCreatorFullDuplex<'d, CH>,
}

impl<'d, CH> ParlIoFullDuplex<'d, CH>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    pub fn new(
        parl_io: impl Peripheral<P = peripherals::PARL_IO> + 'd,
        mut dma_channel: Channel<'d, CH>,
        frequency: HertzU32,
        peripheral_clock_control: &mut PeripheralClockControl,
        _clocks: &Clocks,
    ) -> Result<Self, Error> {
        crate::into_ref!(parl_io);
        internal_init(&mut dma_channel, frequency, peripheral_clock_control)?;

        Ok(Self {
            _parl_io: parl_io,
            tx: TxCreatorFullDuplex {
                tx_channel: dma_channel.tx,
            },
            rx: RxCreatorFullDuplex {
                rx_channel: dma_channel.rx,
            },
        })
    }
}

/// Parallel IO in half duplex / TX only mode
pub struct ParlIoTxOnly<'d, CH>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    _parl_io: PeripheralRef<'d, peripherals::PARL_IO>,
    pub tx: TxCreator<'d, CH>,
}

impl<'d, CH> ParlIoTxOnly<'d, CH>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    pub fn new(
        parl_io: impl Peripheral<P = peripherals::PARL_IO> + 'd,
        mut dma_channel: Channel<'d, CH>,
        frequency: HertzU32,
        peripheral_clock_control: &mut PeripheralClockControl,
        _clocks: &Clocks,
    ) -> Result<Self, Error> {
        crate::into_ref!(parl_io);
        internal_init(&mut dma_channel, frequency, peripheral_clock_control)?;

        Ok(Self {
            _parl_io: parl_io,
            tx: TxCreator {
                tx_channel: dma_channel.tx,
            },
        })
    }
}

fn internal_init<'d, CH>(
    dma_channel: &mut Channel<'d, CH>,
    frequency: HertzU32,
    peripheral_clock_control: &mut PeripheralClockControl,
) -> Result<(), Error>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
{
    if frequency.raw() > 40_000_000 {
        return Err(Error::UnreachableClockRate);
    }

    peripheral_clock_control.enable(crate::system::Peripheral::ParlIo);

    let pcr = unsafe { &*crate::peripherals::PCR::PTR };

    let divider = crate::soc::constants::PARL_IO_SCLK / frequency.raw();
    if divider > 0xffff {
        return Err(Error::UnreachableClockRate);
    }
    let divider = divider as u16;

    pcr.parl_clk_tx_conf.modify(|_, w| {
        w.parl_clk_tx_en()
            .set_bit()
            .parl_clk_tx_sel()
            .variant(1) // PLL
            .parl_clk_tx_div_num()
            .variant(divider)
    });

    dma_channel.tx.init_channel();
    Ok(())
}

impl<'d, CH, P, CP> ParlIoTx<'d, CH, P, CP>
where
    CH: ChannelTypes,
    CH::P: ParlIoPeripheral,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
{
    /// Perform a DMA write.
    ///
    /// This will return a [DmaTransfer] owning the buffer(s) and the driver
    /// instance. The maximum amount of data to be sent is 32736
    /// bytes.
    pub fn write_dma<TXBUF>(
        mut self,
        words: TXBUF,
    ) -> Result<DmaTransfer<'d, CH, TXBUF, P, CP>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { words.read_buffer() };

        if len > MAX_DMA_SIZE {
            return Err(Error::MaxDmaTransferSizeExceeded);
        }

        self.start_write_bytes_dma(ptr, len)?;

        Ok(DmaTransfer {
            instance: self,
            buffer: words,
        })
    }

    fn start_write_bytes_dma<'w>(&mut self, ptr: *const u8, len: usize) -> Result<(), Error> {
        let pcr = unsafe { &*crate::peripherals::PCR::PTR };
        pcr.parl_clk_tx_conf
            .modify(|_, w| w.parl_tx_rst_en().set_bit());
        pcr.parl_clk_tx_conf
            .modify(|_, w| w.parl_tx_rst_en().clear_bit());

        Instance::clear_tx_interrupts();
        Instance::set_tx_bytes(len as u16);

        self.tx_channel.is_done();

        self.tx_channel
            .prepare_transfer(DmaPeripheral::ParlIo, false, ptr, len)?;

        loop {
            if Instance::is_tx_ready() {
                break;
            }
        }

        Instance::set_tx_start(true);
        return Ok(());
    }
}

/// An in-progress DMA transfer.
pub struct DmaTransfer<'d, C, BUFFER, P, CP>
where
    C: ChannelTypes,
    C::P: ParlIoPeripheral,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
{
    instance: ParlIoTx<'d, C, P, CP>,
    buffer: BUFFER,
}

impl<'d, C, BUFFER, P, CP> DmaTransfer<'d, C, BUFFER, P, CP>
where
    C: ChannelTypes,
    C::P: ParlIoPeripheral,
    P: TxPins + ConfigurePins,
    CP: TxClkPin,
{
    /// Wait for the DMA transfer to complete and return the buffers and the
    /// SPI instance.
    pub fn wait(
        self,
    ) -> Result<(BUFFER, ParlIoTx<'d, C, P, CP>), (DmaError, BUFFER, ParlIoTx<'d, C, P, CP>)> {
        loop {
            if Instance::is_tx_eof() {
                break;
            }
        }

        Instance::set_tx_start(false);

        // `DmaTransfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `DmaTransfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = core::ptr::read(&self.buffer);
            let payload = core::ptr::read(&self.instance);
            let err = (&self).instance.tx_channel.has_error();
            mem::forget(self);
            if err {
                Err((DmaError::DescriptorError, buffer, payload))
            } else {
                Ok((buffer, payload))
            }
        }
    }

    /// Check if the DMA transfer is complete
    pub fn is_done(&self) -> bool {
        let ch = &self.instance.tx_channel;
        ch.is_done()
    }
}

mod private {
    use super::{BitPackOrder, SampleEdge};
    use crate::dma::ChannelTypes;

    pub trait FullDuplex {}

    pub trait NotContainsValidSignalPin {}

    pub trait ContainsValidSignalPin {}

    pub trait ValidSignalPin {}

    pub trait TxPins {}

    pub trait TxClkPin {
        fn configure(&mut self);
    }

    pub trait ConfigurePins {
        fn configure(&mut self);
    }

    pub struct TxCreator<'d, CH>
    where
        CH: ChannelTypes,
    {
        pub tx_channel: CH::Tx<'d>,
    }

    pub struct TxCreatorFullDuplex<'d, CH>
    where
        CH: ChannelTypes,
    {
        pub tx_channel: CH::Tx<'d>,
    }

    pub struct RxCreatorFullDuplex<'d, CH>
    where
        CH: ChannelTypes,
    {
        pub rx_channel: CH::Rx<'d>,
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

    pub(super) struct Instance;

    #[cfg(esp32c6)]
    impl Instance {
        pub fn set_tx_bit_width(width: WidSel) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_cfg0
                .modify(|_, w| w.tx_bus_wid_sel().variant(width as u8));
        }

        pub fn set_tx_idle_value(value: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_cfg1
                .modify(|_, w| w.tx_idle_value().variant(value));
        }

        pub fn set_tx_sample_edge(value: SampleEdge) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_cfg0
                .modify(|_, w| w.tx_smp_edge_sel().variant(value as u8 == 1));
        }

        pub fn set_tx_bit_order(value: BitPackOrder) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_cfg0
                .modify(|_, w| w.tx_bit_unpack_order().variant(value as u8 == 1));
        }

        pub fn clear_tx_interrupts() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_clr.write(|w| {
                w.tx_fifo_rempty_int_clr()
                    .set_bit()
                    .tx_eof_int_clr()
                    .set_bit()
            });
        }

        pub fn set_tx_bytes(len: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_cfg0
                .modify(|_, w| w.tx_bytelen().variant(len as u16));
        }

        pub fn is_tx_ready() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.st.read().tx_ready().bit_is_set()
        }

        pub fn set_tx_start(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.tx_cfg0.modify(|_, w| w.tx_start().bit(value));
        }

        pub fn is_tx_eof() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_raw.read().tx_eof_int_raw().bit_is_set()
        }

        pub fn tx_valid_pin_signal() -> crate::gpio::OutputSignal {
            crate::gpio::OutputSignal::PARL_TX_DATA15
        }

        pub fn set_tx_hw_valid_en(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_cfg0
                .modify(|_, w| w.tx_hw_valid_en().bit(value));
        }
    }

    #[cfg(esp32h2)]
    impl Instance {
        pub fn set_tx_bit_width(width: WidSel) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_data_cfg
                .modify(|_, w| w.tx_bus_wid_sel().variant(width as u8));
        }

        pub fn set_tx_idle_value(value: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block
                .tx_genrl_cfg
                .modify(|_, w| w.tx_idle_value().variant(value));
        }

        pub fn set_tx_sample_edge(value: SampleEdge) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };
            reg_block.tx_clk_cfg.modify(|_, w| {
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
                .tx_data_cfg
                .modify(|_, w| w.tx_data_order_inv().variant(value as u8 == 1));
        }

        pub fn clear_tx_interrupts() {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_clr.write(|w| {
                w.tx_fifo_rempty_int_clr()
                    .set_bit()
                    .tx_eof_int_clr()
                    .set_bit()
            });
        }

        pub fn set_tx_bytes(len: u16) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_data_cfg
                .modify(|_, w| w.tx_bitlen().variant((len as u32) * 8));
        }

        pub fn is_tx_ready() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.st.read().tx_ready().bit_is_set()
        }

        pub fn set_tx_start(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_start_cfg
                .modify(|_, w| w.tx_start().bit(value));
        }

        pub fn is_tx_eof() -> bool {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block.int_raw.read().tx_eof_int_raw().bit_is_set()
        }

        pub fn tx_valid_pin_signal() -> crate::gpio::OutputSignal {
            crate::gpio::OutputSignal::PARL_TX_DATA7
        }

        pub fn set_tx_hw_valid_en(value: bool) {
            let reg_block: crate::peripherals::PARL_IO =
                unsafe { crate::peripherals::PARL_IO::steal() };

            reg_block
                .tx_genrl_cfg
                .modify(|_, w| w.tx_valid_output_en().bit(value));
        }
    }
}
