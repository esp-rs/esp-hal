//! I2S Master

use embedded_dma::{ReadBuffer, WriteBuffer};
use private::*;

#[cfg(any(esp32s3))]
use crate::dma::I2s1Peripheral;
use crate::{
    clock::Clocks,
    dma::{
        Channel,
        ChannelTypes,
        DmaError,
        DmaTransfer,
        I2s0Peripheral,
        I2sPeripheral,
        RxPrivate,
        TxPrivate,
    },
    gpio::{InputPin, OutputPin},
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
};

#[cfg(any(esp32, esp32s2, esp32s3))]
const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 6;

#[cfg(any(esp32c3, esp32c6, esp32h2))]
const I2S_LL_MCLK_DIVIDER_BIT_WIDTH: usize = 9;

const I2S_LL_MCLK_DIVIDER_MAX: usize = (1 << I2S_LL_MCLK_DIVIDER_BIT_WIDTH) - 1;

trait AcceptedWord {}
impl AcceptedWord for u8 {}
impl AcceptedWord for u16 {}
impl AcceptedWord for u32 {}
impl AcceptedWord for i8 {}
impl AcceptedWord for i16 {}
impl AcceptedWord for i32 {}

/// I2S Error
#[derive(Debug, Clone, Copy)]
pub enum Error {
    Unknown,
    DmaError(DmaError),
    IllegalArgument,
}

impl From<DmaError> for Error {
    fn from(value: DmaError) -> Self {
        Error::DmaError(value)
    }
}

/// Supported standards.
pub enum Standard {
    Philips,
    // Tdm,
    // Pdm,
}

/// Supported data formats
#[cfg(not(any(esp32, esp32s2)))]
pub enum DataFormat {
    Data32Channel32,
    Data32Channel24,
    Data32Channel16,
    Data32Channel8,
    Data16Channel16,
    Data16Channel8,
    Data8Channel8,
}

/// Supported data formats
#[cfg(any(esp32, esp32s2))]
pub enum DataFormat {
    Data32Channel32,
    Data16Channel16,
}

#[cfg(not(any(esp32, esp32s2)))]
impl DataFormat {
    pub fn data_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data32Channel24 => 32,
            DataFormat::Data32Channel16 => 32,
            DataFormat::Data32Channel8 => 32,
            DataFormat::Data16Channel16 => 16,
            DataFormat::Data16Channel8 => 16,
            DataFormat::Data8Channel8 => 8,
        }
    }

    pub fn channel_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data32Channel24 => 24,
            DataFormat::Data32Channel16 => 16,
            DataFormat::Data32Channel8 => 8,
            DataFormat::Data16Channel16 => 16,
            DataFormat::Data16Channel8 => 8,
            DataFormat::Data8Channel8 => 8,
        }
    }
}

#[cfg(any(esp32, esp32s2))]
impl DataFormat {
    pub fn data_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data16Channel16 => 16,
        }
    }

    pub fn channel_bits(&self) -> u8 {
        match self {
            DataFormat::Data32Channel32 => 32,
            DataFormat::Data16Channel16 => 16,
        }
    }
}

/// Pins to use for I2S tx
pub struct PinsBclkWsDout<'d, B, W, DO> {
    bclk: PeripheralRef<'d, B>,
    ws: PeripheralRef<'d, W>,
    dout: PeripheralRef<'d, DO>,
}

impl<'d, B, W, DO> I2sPins for PinsBclkWsDout<'d, B, W, DO>
where
    B: OutputPin,
    W: OutputPin,
    DO: OutputPin,
{
}

impl<'d, B, W, DO> PinsBclkWsDout<'d, B, W, DO>
where
    B: OutputPin,
    W: OutputPin,
    DO: OutputPin,
{
    pub fn new(
        bclk: impl Peripheral<P = B> + 'd,
        ws: impl Peripheral<P = W> + 'd,
        dout: impl Peripheral<P = DO> + 'd,
    ) -> Self {
        crate::into_ref!(bclk, ws, dout);
        Self { bclk, ws, dout }
    }
}

impl<'d, B, W, DO> I2sTxPins for PinsBclkWsDout<'d, B, W, DO>
where
    B: OutputPin,
    W: OutputPin,
    DO: OutputPin,
{
    fn configure<I>(&mut self, instance: &mut I)
    where
        I: RegisterAccess,
    {
        self.bclk
            .set_to_push_pull_output()
            .connect_peripheral_to_output(instance.bclk_signal());

        self.ws
            .set_to_push_pull_output()
            .connect_peripheral_to_output(instance.ws_signal());

        self.dout
            .set_to_push_pull_output()
            .connect_peripheral_to_output(instance.dout_signal());
    }
}

/// Pins to use for I2S rx
pub struct PinsBclkWsDin<'d, B, W, DI> {
    bclk: PeripheralRef<'d, B>,
    ws: PeripheralRef<'d, W>,
    din: PeripheralRef<'d, DI>,
}

impl<'d, B, W, DI> PinsBclkWsDin<'d, B, W, DI>
where
    B: OutputPin,
    W: OutputPin,
    DI: InputPin,
{
    pub fn new(
        bclk: impl Peripheral<P = B> + 'd,
        ws: impl Peripheral<P = W> + 'd,
        din: impl Peripheral<P = DI> + 'd,
    ) -> Self {
        crate::into_ref!(bclk, ws, din);
        Self { bclk, ws, din }
    }
}

impl<'d, B, W, DI> I2sPins for PinsBclkWsDin<'d, B, W, DI>
where
    B: OutputPin,
    W: OutputPin,
    DI: InputPin,
{
}

impl<'d, B, W, DI> I2sRxPins for PinsBclkWsDin<'d, B, W, DI>
where
    B: OutputPin,
    W: OutputPin,
    DI: InputPin,
{
    fn configure<I>(&mut self, instance: &mut I)
    where
        I: RegisterAccess,
    {
        self.bclk
            .set_to_push_pull_output()
            .connect_peripheral_to_output(instance.bclk_rx_signal());

        self.ws
            .set_to_push_pull_output()
            .connect_peripheral_to_output(instance.ws_rx_signal());

        self.din
            .set_to_input()
            .connect_input_to_peripheral(instance.din_signal());
    }
}

/// MCLK pin to use
#[cfg(not(esp32))]
pub struct MclkPin<'d, M: OutputPin> {
    mclk: PeripheralRef<'d, M>,
}

#[cfg(not(esp32))]
impl<'d, M> I2sPins for MclkPin<'d, M> where M: OutputPin {}

#[cfg(not(esp32))]
impl<'d, M: OutputPin> MclkPin<'d, M> {
    pub fn new(pin: impl Peripheral<P = M> + 'd) -> Self {
        Self {
            mclk: pin.into_ref(),
        }
    }
}

#[cfg(not(esp32))]
impl<'d, M> I2sMclkPin for MclkPin<'d, M>
where
    M: OutputPin,
{
    fn configure<I>(&mut self, instance: &mut I)
    where
        I: RegisterAccess,
    {
        self.mclk
            .set_to_push_pull_output()
            .connect_peripheral_to_output(instance.mclk_signal());
    }
}

/// No MCLK pin
pub struct NoMclk {}

impl I2sPins for NoMclk {}

impl I2sMclkPin for NoMclk {
    fn configure<I>(&mut self, _instance: &mut I)
    where
        I: RegisterAccess,
    {
        // nothing to do
    }
}

/// An in-progress DMA write transfer.
pub struct I2sWriteDmaTransfer<'d, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    i2s_tx: I2sTx<'d, T, P, CH>,
    buffer: BUFFER,
}

impl<'d, T, P, CH, BUFFER> I2sWriteDmaTransfer<'d, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    /// Amount of bytes which can be pushed.
    /// Only useful for circular DMA transfers
    pub fn available(&mut self) -> usize {
        self.i2s_tx.tx_channel.available()
    }

    /// Push bytes into the DMA buffer.
    /// Only useful for circular DMA transfers
    pub fn push(&mut self, data: &[u8]) -> Result<usize, Error> {
        Ok(self.i2s_tx.tx_channel.push(data)?)
    }
}

impl<'d, T, P, CH, BUFFER> DmaTransfer<BUFFER, I2sTx<'d, T, P, CH>>
    for I2sWriteDmaTransfer<'d, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    /// Wait for the DMA transfer to complete and return the buffers and the
    /// I2sTx instance.
    fn wait(self) -> (BUFFER, I2sTx<'d, T, P, CH>) {
        self.i2s_tx.wait_tx_dma_done().ok(); // waiting for the DMA transfer is not enough

        // `DmaTransfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `DmaTransfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = core::ptr::read(&self.buffer);
            let payload = core::ptr::read(&self.i2s_tx);
            core::mem::forget(self);
            (buffer, payload)
        }
    }

    /// Check if the DMA transfer is complete
    fn is_done(&self) -> bool {
        self.i2s_tx.tx_channel.is_done()
    }
}

impl<'d, T, P, CH, BUFFER> Drop for I2sWriteDmaTransfer<'d, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    fn drop(&mut self) {
        self.i2s_tx.wait_tx_dma_done().ok();
    }
}

/// Blocking I2s Write
pub trait I2sWrite<W> {
    fn write(&mut self, words: &[W]) -> Result<(), Error>;
}

/// Initiate a DMA tx transfer
pub trait I2sWriteDma<'d, T, P, CH, TXBUF>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    /// Write I2S.
    /// Returns [I2sWriteDmaTransfer] which represents the in-progress DMA
    /// transfer
    fn write_dma(self, words: TXBUF) -> Result<I2sWriteDmaTransfer<'d, T, P, CH, TXBUF>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>;

    /// Continuously write to I2S. Returns [I2sWriteDmaTransfer] which
    /// represents the in-progress DMA transfer
    fn write_dma_circular(
        self,
        words: TXBUF,
    ) -> Result<I2sWriteDmaTransfer<'d, T, P, CH, TXBUF>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>;
}

/// An in-progress DMA read transfer.
pub struct I2sReadDmaTransfer<'d, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    i2s_rx: I2sRx<'d, T, P, CH>,
    buffer: BUFFER,
}

impl<'d, T, P, CH, BUFFER> I2sReadDmaTransfer<'d, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    /// Amount of bytes which can be popped
    pub fn available(&mut self) -> usize {
        self.i2s_rx.rx_channel.available()
    }

    pub fn pop(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        Ok(self.i2s_rx.rx_channel.pop(data)?)
    }

    /// Wait for the DMA transfer to complete and return the buffers and the
    /// I2sTx instance after copying the read data to the given buffer.
    /// Length of the received data is returned at the third element of the
    /// tuple.
    pub fn wait_receive(mut self, dst: &mut [u8]) -> (BUFFER, I2sRx<'d, T, P, CH>, usize) {
        self.i2s_rx.wait_rx_dma_done().ok(); // waiting for the DMA transfer is not enough

        let len = self.i2s_rx.rx_channel.drain_buffer(dst).unwrap();

        // `DmaTransfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `DmaTransfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = core::ptr::read(&self.buffer);
            let payload = core::ptr::read(&self.i2s_rx);
            core::mem::forget(self);
            (buffer, payload, len)
        }
    }
}

impl<'d, T, P, CH, BUFFER> DmaTransfer<BUFFER, I2sRx<'d, T, P, CH>>
    for I2sReadDmaTransfer<'d, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    /// Wait for the DMA transfer to complete and return the buffers and the
    /// I2sTx instance.
    fn wait(self) -> (BUFFER, I2sRx<'d, T, P, CH>) {
        self.i2s_rx.wait_rx_dma_done().ok(); // waiting for the DMA transfer is not enough

        // `DmaTransfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `DmaTransfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = core::ptr::read(&self.buffer);
            let payload = core::ptr::read(&self.i2s_rx);
            core::mem::forget(self);
            (buffer, payload)
        }
    }

    /// Check if the DMA transfer is complete
    fn is_done(&self) -> bool {
        self.i2s_rx.rx_channel.is_done()
    }
}

impl<T, P, CH, BUFFER> Drop for I2sReadDmaTransfer<'_, T, P, CH, BUFFER>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    fn drop(&mut self) {
        self.i2s_rx.wait_rx_dma_done().ok();
    }
}

/// Blocking I2S Read
pub trait I2sRead<W> {
    fn read(&mut self, words: &mut [W]) -> Result<(), Error>;
}

/// Initiate a DMA rx transfer
pub trait I2sReadDma<'d, T, P, CH, RXBUF>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    /// Read I2S.
    /// Returns [I2sReadDmaTransfer] which represents the in-progress DMA
    /// transfer
    fn read_dma(self, words: RXBUF) -> Result<I2sReadDmaTransfer<'d, T, P, CH, RXBUF>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>;

    /// Continuously read from I2S.
    /// Returns [I2sReadDmaTransfer] which represents the in-progress DMA
    /// transfer
    fn read_dma_circular(
        self,
        words: RXBUF,
    ) -> Result<I2sReadDmaTransfer<'d, T, P, CH, RXBUF>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>;
}

/// Instance of the I2S peripheral driver
pub struct I2s<'d, I, P, CH>
where
    I: Instance,
    P: I2sMclkPin,
    CH: ChannelTypes,
{
    _peripheral: PeripheralRef<'d, I>,
    _pins: P,
    pub i2s_tx: TxCreator<'d, I::Peripheral, CH>,
    pub i2s_rx: RxCreator<'d, I::Peripheral, CH>,
}

impl<'d, I, P, CH> I2s<'d, I, P, CH>
where
    I: Instance,
    P: I2sMclkPin,
    CH: ChannelTypes,
{
    fn new_internal(
        i2s: impl Peripheral<P = I> + 'd,
        mut pins: P,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        mut channel: Channel<'d, CH>,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        // on ESP32-C3 / ESP32-S3 and later RX and TX are independent and
        // could be configured totally independently but for now handle all
        // the targets the same and force same configuration for both, TX and RX

        crate::into_ref!(i2s);
        let mut register_access = i2s.register_access();

        channel.tx.init_channel();
        peripheral_clock_control.enable(register_access.get_peripheral());
        pins.configure(&mut register_access);
        register_access.set_clock(calculate_clock(
            sample_rate,
            2,
            data_format.channel_bits(),
            clocks,
        ));
        register_access.configure(&standard, &data_format);
        register_access.set_master();
        register_access.update();

        Self {
            _peripheral: i2s,
            _pins: pins,
            i2s_tx: TxCreator {
                register_access: register_access.clone(),
                tx_channel: channel.tx,
            },
            i2s_rx: RxCreator {
                register_access,
                rx_channel: channel.rx,
            },
        }
    }
}

/// Construct a new I2S peripheral driver instance for the first I2S peripheral
pub trait I2s0New<'d, I, P, CH>
where
    I: Instance,
    P: I2sMclkPin,
    CH: ChannelTypes,
    CH::P: I2sPeripheral + I2s0Peripheral,
{
    fn new(
        i2s: impl Peripheral<P = I> + 'd,
        pins: P,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH>,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self;
}

impl<'d, I, P, CH> I2s0New<'d, I, P, CH> for I2s<'d, I, P, CH>
where
    I: Instance + I2s0Instance,
    P: I2sMclkPin,
    CH: ChannelTypes,
    CH::P: I2sPeripheral + I2s0Peripheral,
{
    fn new(
        i2s: impl Peripheral<P = I> + 'd,
        pins: P,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH>,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        Self::new_internal(
            i2s,
            pins,
            standard,
            data_format,
            sample_rate,
            channel,
            peripheral_clock_control,
            clocks,
        )
    }
}

/// Construct a new I2S peripheral driver instance for the second I2S peripheral
#[cfg(any(esp32s3))]
pub trait I2s1New<'d, I, P, CH>
where
    I: Instance,
    P: I2sMclkPin,
    CH: ChannelTypes,
    CH::P: I2sPeripheral + I2s1Peripheral,
{
    fn new(
        i2s: impl Peripheral<P = I> + 'd,
        pins: P,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH>,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self;
}

#[cfg(any(esp32s3))]
impl<'d, I, P, CH> I2s1New<'d, I, P, CH> for I2s<'d, I, P, CH>
where
    I: Instance + I2s1Instance,
    P: I2sMclkPin,
    CH: ChannelTypes,
    CH::P: I2sPeripheral + I2s1Peripheral,
{
    fn new(
        i2s: impl Peripheral<P = I> + 'd,
        pins: P,
        standard: Standard,
        data_format: DataFormat,
        sample_rate: impl Into<fugit::HertzU32>,
        channel: Channel<'d, CH>,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        Self::new_internal(
            i2s,
            pins,
            standard,
            data_format,
            sample_rate,
            channel,
            peripheral_clock_control,
            clocks,
        )
    }
}

/// I2S TX channel
pub struct I2sTx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    register_access: T,
    _pins: P,
    tx_channel: CH::Tx<'d>,
}

impl<'d, T, P, CH> I2sTx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    fn new(mut register_access: T, mut pins: P, tx_channel: CH::Tx<'d>) -> Self {
        pins.configure(&mut register_access);

        Self {
            register_access,
            _pins: pins,
            tx_channel,
        }
    }

    fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        let ptr = data as *const _ as *const u8;

        // Reset TX unit and TX FIFO
        self.register_access.reset_tx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.tx_channel.prepare_transfer(
            self.register_access.get_dma_peripheral(),
            false,
            ptr,
            data.len(),
        )?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        self.register_access.tx_start();

        // wait until I2S_TX_IDLE is 1
        self.register_access.wait_for_tx_done();

        Ok(())
    }

    fn start_tx_transfer<TXBUF>(
        mut self,
        words: TXBUF,
        circular: bool,
    ) -> Result<I2sWriteDmaTransfer<'d, T, P, CH, TXBUF>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { words.read_buffer() };

        // Reset TX unit and TX FIFO
        self.register_access.reset_tx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.tx_channel.prepare_transfer(
            self.register_access.get_dma_peripheral(),
            circular,
            ptr,
            len,
        )?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        self.register_access.tx_start();

        Ok(I2sWriteDmaTransfer {
            i2s_tx: self,
            buffer: words,
        })
    }

    fn wait_tx_dma_done(&self) -> Result<(), Error> {
        // wait until I2S_TX_IDLE is 1
        self.register_access.wait_for_tx_done();

        Ok(())
    }
}

impl<'d, T, P, W, CH> I2sWrite<W> for I2sTx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
    W: AcceptedWord,
{
    fn write(&mut self, words: &[W]) -> Result<(), Error> {
        self.write_bytes(unsafe {
            core::slice::from_raw_parts(
                words as *const _ as *const u8,
                words.len() * core::mem::size_of::<W>(),
            )
        })
    }
}

impl<'d, T, P, CH, TXBUF> I2sWriteDma<'d, T, P, CH, TXBUF> for I2sTx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sTxPins,
{
    fn write_dma(self, words: TXBUF) -> Result<I2sWriteDmaTransfer<'d, T, P, CH, TXBUF>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        self.start_tx_transfer(words, false)
    }

    fn write_dma_circular(
        self,
        words: TXBUF,
    ) -> Result<I2sWriteDmaTransfer<'d, T, P, CH, TXBUF>, Error>
    where
        TXBUF: ReadBuffer<Word = u8>,
    {
        self.start_tx_transfer(words, true)
    }
}

/// I2S RX channel
pub struct I2sRx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    register_access: T,
    _pins: P,
    rx_channel: CH::Rx<'d>,
}

impl<'d, T, P, CH> I2sRx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    fn new(mut register_access: T, mut pins: P, rx_channel: CH::Rx<'d>) -> Self {
        pins.configure(&mut register_access);

        Self {
            register_access,
            _pins: pins,
            rx_channel,
        }
    }

    fn read_bytes(&mut self, data: &mut [u8]) -> Result<(), Error> {
        let ptr = data as *mut _ as *mut u8;

        // Reset RX unit and RX FIFO
        self.register_access.reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.rx_channel.prepare_transfer(
            false,
            self.register_access.get_dma_peripheral(),
            ptr,
            data.len(),
        )?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_TX_START
        self.register_access.rx_start(data.len() - 1);

        // wait until I2S_TX_IDLE is 1
        self.register_access.wait_for_rx_done();

        Ok(())
    }

    fn start_rx_transfer<RXBUF>(
        mut self,
        mut words: RXBUF,
        circular: bool,
    ) -> Result<I2sReadDmaTransfer<'d, T, P, CH, RXBUF>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        let (ptr, len) = unsafe { words.write_buffer() };

        if len % 4 != 0 {
            return Err(Error::IllegalArgument);
        }

        // Reset TX unit and TX FIFO
        self.register_access.reset_rx();

        // Enable corresponding interrupts if needed

        // configure DMA outlink
        self.rx_channel.prepare_transfer(
            circular,
            self.register_access.get_dma_peripheral(),
            ptr,
            len,
        )?;

        // set I2S_TX_STOP_EN if needed

        // start: set I2S_RX_START
        #[cfg(not(esp32))]
        self.register_access.rx_start(len - 1);

        #[cfg(esp32)]
        self.register_access.rx_start(len);

        Ok(I2sReadDmaTransfer {
            i2s_rx: self,
            buffer: words,
        })
    }

    fn wait_rx_dma_done(&self) -> Result<(), Error> {
        self.register_access.wait_for_rx_done();

        Ok(())
    }
}

impl<'d, W, T, P, CH> I2sRead<W> for I2sRx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
    W: AcceptedWord,
{
    fn read(&mut self, words: &mut [W]) -> Result<(), Error> {
        if words.len() * core::mem::size_of::<W>() > 4096 || words.len() == 0 {
            return Err(Error::IllegalArgument);
        }

        self.read_bytes(unsafe {
            core::slice::from_raw_parts_mut(
                words as *mut _ as *mut u8,
                words.len() * core::mem::size_of::<W>(),
            )
        })
    }
}

impl<'d, T, P, CH, RXBUF> I2sReadDma<'d, T, P, CH, RXBUF> for I2sRx<'d, T, P, CH>
where
    T: RegisterAccess,
    CH: ChannelTypes,
    P: I2sRxPins,
{
    fn read_dma(self, words: RXBUF) -> Result<I2sReadDmaTransfer<'d, T, P, CH, RXBUF>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        self.start_rx_transfer(words, false)
    }

    fn read_dma_circular(
        self,
        words: RXBUF,
    ) -> Result<I2sReadDmaTransfer<'d, T, P, CH, RXBUF>, Error>
    where
        RXBUF: WriteBuffer<Word = u8>,
    {
        self.start_rx_transfer(words, true)
    }
}

pub trait RegisterAccess: RegisterAccessPrivate {}

pub trait I2sTxPins: I2sPins {
    fn configure<I>(&mut self, instance: &mut I)
    where
        I: RegisterAccess;
}

pub trait I2sRxPins: I2sPins {
    fn configure<I>(&mut self, instance: &mut I)
    where
        I: RegisterAccess;
}

pub trait I2sMclkPin: I2sPins {
    fn configure<I>(&mut self, instance: &mut I)
    where
        I: RegisterAccess;
}

mod private {
    use fugit::HertzU32;

    use super::{DataFormat, I2sRx, I2sTx, RegisterAccess, Standard, I2S_LL_MCLK_DIVIDER_MAX};
    #[cfg(not(any(esp32, esp32s3)))]
    use crate::peripherals::i2s0::RegisterBlock;
    // on ESP32-S3 I2S1 doesn't support all features - use that to avoid using those features
    // by accident
    #[cfg(any(esp32, esp32s3))]
    use crate::peripherals::i2s1::RegisterBlock;
    use crate::{
        clock::Clocks,
        dma::{ChannelTypes, DmaPeripheral},
        gpio::{InputSignal, OutputSignal},
        peripherals::I2S0,
        system::Peripheral,
    };

    pub trait I2sPins {}

    pub struct TxCreator<'d, T, CH>
    where
        T: RegisterAccess + Clone,
        CH: ChannelTypes,
    {
        pub register_access: T,
        pub tx_channel: CH::Tx<'d>,
    }

    impl<'d, T, CH> TxCreator<'d, T, CH>
    where
        T: RegisterAccess + Clone,
        CH: ChannelTypes,
    {
        pub fn with_pins<P>(self, pins: P) -> I2sTx<'d, T, P, CH>
        where
            P: super::I2sTxPins,
        {
            I2sTx::new(self.register_access, pins, self.tx_channel)
        }
    }

    pub struct RxCreator<'d, T, CH>
    where
        T: RegisterAccess + Clone,
        CH: ChannelTypes,
    {
        pub register_access: T,
        pub rx_channel: CH::Rx<'d>,
    }

    impl<'d, T, CH> RxCreator<'d, T, CH>
    where
        T: RegisterAccess + Clone,
        CH: ChannelTypes,
    {
        pub fn with_pins<P>(self, pins: P) -> I2sRx<'d, T, P, CH>
        where
            P: super::I2sRxPins,
        {
            I2sRx::new(self.register_access, pins, self.rx_channel)
        }
    }

    pub trait I2s0Instance {}

    pub trait I2s1Instance {}

    pub trait Instance {
        type Peripheral: RegisterAccess + Clone;

        fn register_access(&self) -> Self::Peripheral;
    }

    impl Instance for I2S0 {
        type Peripheral = I2sPeripheral0;

        fn register_access(&self) -> I2sPeripheral0 {
            I2sPeripheral0 {}
        }
    }

    impl I2s0Instance for I2S0 {}

    #[cfg(esp32s3)]
    impl Instance for crate::peripherals::I2S1 {
        type Peripheral = I2sPeripheral1;
        fn register_access(&self) -> I2sPeripheral1 {
            I2sPeripheral1 {}
        }
    }

    #[cfg(esp32s3)]
    impl I2s1Instance for crate::peripherals::I2S1 {}

    pub trait Signals {
        fn get_peripheral(&self) -> Peripheral;

        fn get_dma_peripheral(&self) -> DmaPeripheral;

        fn mclk_signal(&self) -> OutputSignal;

        fn bclk_signal(&self) -> OutputSignal;

        fn ws_signal(&self) -> OutputSignal;

        fn dout_signal(&self) -> OutputSignal;

        fn bclk_rx_signal(&self) -> OutputSignal;

        fn ws_rx_signal(&self) -> OutputSignal;

        fn din_signal(&self) -> InputSignal;
    }

    pub trait RegBlock {
        fn register_block(&self) -> &'static RegisterBlock;
    }

    #[cfg(any(esp32, esp32s2))]
    pub trait RegisterAccessPrivate: Signals + RegBlock {
        fn set_clock(&self, clock_settings: I2sClockDividers) {
            let i2s = self.register_block();

            i2s.clkm_conf.modify(|r, w| unsafe {
                w.bits(r.bits() | (crate::soc::constants::I2S_DEFAULT_CLK_SRC << 21))
                // select PLL_160M
            });

            #[cfg(esp32)]
            i2s.clkm_conf.modify(|_, w| w.clka_ena().clear_bit());

            i2s.clkm_conf.modify(|_, w| {
                w.clk_en()
                    .set_bit()
                    .clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
            });

            i2s.clkm_conf.modify(|_, w| {
                w.clkm_div_a()
                    .variant(clock_settings.denominator as u8)
                    .clkm_div_b()
                    .variant(clock_settings.numerator as u8)
            });

            i2s.sample_rate_conf.modify(|_, w| {
                w.tx_bck_div_num()
                    .variant(clock_settings.bclk_divider as u8)
                    .rx_bck_div_num()
                    .variant(clock_settings.bclk_divider as u8)
            });
        }

        fn configure(&self, _standard: &Standard, data_format: &DataFormat) {
            let i2s = self.register_block();

            let fifo_mod = match data_format {
                DataFormat::Data32Channel32 => 2,
                DataFormat::Data16Channel16 => 0,
            };

            i2s.sample_rate_conf
                .modify(|_, w| w.tx_bits_mod().variant(data_format.channel_bits()));
            i2s.sample_rate_conf
                .modify(|_, w| w.rx_bits_mod().variant(data_format.channel_bits()));

            i2s.conf.modify(|_, w| {
                w.tx_slave_mod()
                    .clear_bit()
                    .rx_slave_mod()
                    .clear_bit()
                    .tx_msb_shift()
                    .set_bit() // ?
                    .rx_msb_shift()
                    .set_bit() // ?
                    .tx_short_sync()
                    .variant(false) //??
                    .rx_short_sync()
                    .variant(false) //??
                    .tx_msb_right()
                    .clear_bit()
                    .rx_msb_right()
                    .clear_bit()
                    .tx_right_first()
                    .clear_bit()
                    .rx_right_first()
                    .clear_bit()
                    .tx_mono()
                    .clear_bit()
                    .rx_mono()
                    .clear_bit()
                    .sig_loopback()
                    .clear_bit()
            });

            i2s.fifo_conf.modify(|_, w| {
                w.tx_fifo_mod()
                    .variant(fifo_mod)
                    .tx_fifo_mod_force_en()
                    .set_bit()
                    .dscr_en()
                    .set_bit()
                    .rx_fifo_mod()
                    .variant(fifo_mod)
                    .rx_fifo_mod_force_en()
                    .set_bit()
            });

            i2s.conf_chan
                .modify(|_, w| w.tx_chan_mod().variant(0).rx_chan_mod().variant(0)); // for now only stereo

            i2s.conf1
                .modify(|_, w| w.tx_pcm_bypass().set_bit().rx_pcm_bypass().set_bit());

            i2s.pd_conf
                .modify(|_, w| w.fifo_force_pu().set_bit().fifo_force_pd().clear_bit());

            i2s.conf2
                .modify(|_, w| w.camera_en().clear_bit().lcd_en().clear_bit());
        }

        fn set_master(&self) {
            let i2s = self.register_block();
            i2s.conf
                .modify(|_, w| w.rx_slave_mod().clear_bit().tx_slave_mod().clear_bit());
        }

        fn update(&self) {
            // nothing to do
        }

        fn reset_tx(&self) {
            let i2s = self.register_block();
            i2s.conf
                .modify(|_, w| w.tx_reset().set_bit().tx_fifo_reset().set_bit());
            i2s.conf
                .modify(|_, w| w.tx_reset().clear_bit().tx_fifo_reset().clear_bit());

            i2s.lc_conf.modify(|_, w| w.out_rst().set_bit());
            i2s.lc_conf.modify(|_, w| w.out_rst().clear_bit());

            i2s.int_clr.write(|w| {
                w.out_done_int_clr()
                    .set_bit()
                    .out_total_eof_int_clr()
                    .set_bit()
            });
        }

        fn tx_start(&self) {
            let i2s = self.register_block();
            i2s.conf.modify(|_, w| w.tx_start().set_bit());
        }

        fn wait_for_tx_done(&self) {
            let i2s = self.register_block();
            while i2s.state.read().tx_idle().bit_is_clear() {
                // wait
            }

            i2s.conf.modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx(&self) {
            let i2s = self.register_block();
            i2s.conf
                .modify(|_, w| w.rx_reset().set_bit().rx_fifo_reset().set_bit());
            i2s.conf
                .modify(|_, w| w.rx_reset().clear_bit().rx_fifo_reset().clear_bit());

            i2s.lc_conf.modify(|_, w| w.in_rst().set_bit());
            i2s.lc_conf.modify(|_, w| w.in_rst().clear_bit());

            i2s.int_clr
                .write(|w| w.in_done_int_clr().set_bit().in_suc_eof_int_clr().set_bit());
        }

        fn rx_start(&self, len: usize) {
            let i2s = self.register_block();

            i2s.int_clr.write(|w| w.in_suc_eof_int_clr().set_bit());

            #[cfg(not(esp32))]
            i2s.rxeof_num
                .modify(|_, w| w.rx_eof_num().variant(len as u32));

            // On ESP32, the eof_num count in words.
            #[cfg(esp32)]
            i2s.rxeof_num
                .modify(|_, w| w.rx_eof_num().variant((len / 4) as u32));

            i2s.conf.modify(|_, w| w.rx_start().set_bit());
        }

        fn wait_for_rx_done(&self) {
            let i2s = self.register_block();
            while i2s.int_raw.read().in_suc_eof_int_raw().bit_is_clear() {
                // wait
            }

            i2s.int_clr.write(|w| w.in_suc_eof_int_clr().set_bit());
        }
    }

    #[cfg(any(esp32c3, esp32c6, esp32h2, esp32s3))]
    pub trait RegisterAccessPrivate: Signals + RegBlock {
        #[cfg(any(esp32c3, esp32s3))]
        fn set_clock(&self, clock_settings: I2sClockDividers) {
            let i2s = self.register_block();

            let clkm_div_x: u32;
            let clkm_div_y: u32;
            let clkm_div_z: u32;
            let clkm_div_yn1: u32;

            if clock_settings.denominator == 0 || clock_settings.numerator == 0 {
                clkm_div_x = 0;
                clkm_div_y = 0;
                clkm_div_z = 0;
                clkm_div_yn1 = 1;
            } else {
                if clock_settings.numerator > clock_settings.denominator / 2 {
                    clkm_div_x = clock_settings
                        .denominator
                        .overflowing_div(
                            clock_settings
                                .denominator
                                .overflowing_sub(clock_settings.numerator)
                                .0,
                        )
                        .0
                        .overflowing_sub(1)
                        .0;
                    clkm_div_y = clock_settings.denominator
                        % (clock_settings
                            .denominator
                            .overflowing_sub(clock_settings.numerator)
                            .0);
                    clkm_div_z = clock_settings
                        .denominator
                        .overflowing_sub(clock_settings.numerator)
                        .0;
                    clkm_div_yn1 = 1;
                } else {
                    clkm_div_x = clock_settings.denominator / clock_settings.numerator - 1;
                    clkm_div_y = clock_settings.denominator % clock_settings.numerator;
                    clkm_div_z = clock_settings.numerator;
                    clkm_div_yn1 = 0;
                }
            }

            i2s.tx_clkm_div_conf.modify(|_, w| {
                w.tx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .tx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .tx_clkm_div_yn1()
                    .variant(if clkm_div_yn1 != 0 { true } else { false })
                    .tx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            i2s.tx_clkm_conf.modify(|_, w| {
                w.clk_en()
                    .set_bit()
                    .tx_clk_active()
                    .set_bit()
                    .tx_clk_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz
                    .tx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
            });

            i2s.tx_conf1.modify(|_, w| {
                w.tx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });

            i2s.rx_clkm_div_conf.modify(|_, w| {
                w.rx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .rx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .rx_clkm_div_yn1()
                    .variant(if clkm_div_yn1 != 0 { true } else { false })
                    .rx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            i2s.rx_clkm_conf.modify(|_, w| {
                w.rx_clk_active()
                    .set_bit()
                    .rx_clk_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz
                    .rx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
                    .mclk_sel()
                    .variant(true)
            });

            i2s.rx_conf1.modify(|_, w| {
                w.rx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
        }

        #[cfg(any(esp32c6, esp32h2))]
        fn set_clock(&self, clock_settings: I2sClockDividers) {
            let i2s = self.register_block();
            let pcr = unsafe { &*crate::peripherals::PCR::PTR }; // I2S clocks are configured via PCR

            let clkm_div_x: u32;
            let clkm_div_y: u32;
            let clkm_div_z: u32;
            let clkm_div_yn1: u32;

            if clock_settings.denominator == 0 || clock_settings.numerator == 0 {
                clkm_div_x = 0;
                clkm_div_y = 0;
                clkm_div_z = 0;
                clkm_div_yn1 = 1;
            } else {
                if clock_settings.numerator > clock_settings.denominator / 2 {
                    clkm_div_x = clock_settings
                        .denominator
                        .overflowing_div(
                            clock_settings
                                .denominator
                                .overflowing_sub(clock_settings.numerator)
                                .0,
                        )
                        .0
                        .overflowing_sub(1)
                        .0;
                    clkm_div_y = clock_settings.denominator
                        % (clock_settings
                            .denominator
                            .overflowing_sub(clock_settings.numerator)
                            .0);
                    clkm_div_z = clock_settings
                        .denominator
                        .overflowing_sub(clock_settings.numerator)
                        .0;
                    clkm_div_yn1 = 1;
                } else {
                    clkm_div_x = clock_settings.denominator / clock_settings.numerator - 1;
                    clkm_div_y = clock_settings.denominator % clock_settings.numerator;
                    clkm_div_z = clock_settings.numerator;
                    clkm_div_yn1 = 0;
                }
            }

            pcr.i2s_tx_clkm_div_conf.modify(|_, w| {
                w.i2s_tx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .i2s_tx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .i2s_tx_clkm_div_yn1()
                    .variant(if clkm_div_yn1 != 0 { true } else { false })
                    .i2s_tx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            pcr.i2s_tx_clkm_conf.modify(|_, w| {
                w.i2s_tx_clkm_en()
                    .set_bit()
                    .i2s_tx_clkm_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz for C6 and 96MHz for H2
                    .i2s_tx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
            });

            #[cfg(not(esp32h2))]
            i2s.tx_conf1.modify(|_, w| {
                w.tx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            i2s.tx_conf.modify(|_, w| {
                w.tx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });

            pcr.i2s_rx_clkm_div_conf.modify(|_, w| {
                w.i2s_rx_clkm_div_x()
                    .variant(clkm_div_x as u16)
                    .i2s_rx_clkm_div_y()
                    .variant(clkm_div_y as u16)
                    .i2s_rx_clkm_div_yn1()
                    .variant(if clkm_div_yn1 != 0 { true } else { false })
                    .i2s_rx_clkm_div_z()
                    .variant(clkm_div_z as u16)
            });

            pcr.i2s_rx_clkm_conf.modify(|_, w| {
                w.i2s_rx_clkm_en()
                    .set_bit()
                    .i2s_rx_clkm_sel()
                    .variant(crate::soc::constants::I2S_DEFAULT_CLK_SRC) // for now fixed at 160MHz for C6 and 96MHz for H2
                    .i2s_rx_clkm_div_num()
                    .variant(clock_settings.mclk_divider as u8)
                    .i2s_mclk_sel()
                    .variant(true)
            });
            #[cfg(not(esp32h2))]
            i2s.rx_conf1.modify(|_, w| {
                w.rx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
            #[cfg(esp32h2)]
            i2s.rx_conf.modify(|_, w| {
                w.rx_bck_div_num()
                    .variant((clock_settings.bclk_divider - 1) as u8)
            });
        }

        fn configure(&self, _standard: &Standard, data_format: &DataFormat) {
            let i2s = self.register_block();
            i2s.tx_conf1.modify(|_, w| {
                w.tx_tdm_ws_width()
                    .variant((data_format.channel_bits() - 1).into())
                    .tx_bits_mod()
                    .variant(data_format.data_bits() - 1)
                    .tx_tdm_chan_bits()
                    .variant(data_format.channel_bits() - 1)
                    .tx_half_sample_bits()
                    .variant(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            i2s.tx_conf1.modify(|_, w| w.tx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            i2s.tx_conf.modify(|_, w| w.tx_msb_shift().set_bit());
            i2s.tx_conf.modify(|_, w| {
                w.tx_mono()
                    .clear_bit()
                    .tx_mono_fst_vld()
                    .set_bit()
                    .tx_stop_en()
                    .set_bit()
                    .tx_chan_equal()
                    .clear_bit()
                    .tx_tdm_en()
                    .set_bit()
                    .tx_pdm_en()
                    .clear_bit()
                    .tx_pcm_bypass()
                    .set_bit()
                    .tx_big_endian()
                    .clear_bit()
                    .tx_bit_order()
                    .clear_bit()
                    .tx_chan_mod()
                    .variant(0)
            });

            i2s.tx_tdm_ctrl.modify(|_, w| {
                w.tx_tdm_tot_chan_num()
                    .variant(1)
                    .tx_tdm_chan0_en()
                    .set_bit()
                    .tx_tdm_chan1_en()
                    .set_bit()
                    .tx_tdm_chan2_en()
                    .clear_bit()
                    .tx_tdm_chan3_en()
                    .clear_bit()
                    .tx_tdm_chan4_en()
                    .clear_bit()
                    .tx_tdm_chan5_en()
                    .clear_bit()
                    .tx_tdm_chan6_en()
                    .clear_bit()
                    .tx_tdm_chan7_en()
                    .clear_bit()
                    .tx_tdm_chan8_en()
                    .clear_bit()
                    .tx_tdm_chan9_en()
                    .clear_bit()
                    .tx_tdm_chan10_en()
                    .clear_bit()
                    .tx_tdm_chan11_en()
                    .clear_bit()
                    .tx_tdm_chan12_en()
                    .clear_bit()
                    .tx_tdm_chan13_en()
                    .clear_bit()
                    .tx_tdm_chan14_en()
                    .clear_bit()
                    .tx_tdm_chan15_en()
                    .clear_bit()
            });

            i2s.rx_conf1.modify(|_, w| {
                w.rx_tdm_ws_width()
                    .variant((data_format.channel_bits() - 1).into())
                    .rx_bits_mod()
                    .variant(data_format.data_bits() - 1)
                    .rx_tdm_chan_bits()
                    .variant(data_format.channel_bits() - 1)
                    .rx_half_sample_bits()
                    .variant(data_format.channel_bits() - 1)
            });
            #[cfg(not(esp32h2))]
            i2s.rx_conf1.modify(|_, w| w.rx_msb_shift().set_bit());
            #[cfg(esp32h2)]
            i2s.rx_conf.modify(|_, w| w.rx_msb_shift().set_bit());

            i2s.rx_conf.modify(|_, w| {
                w.rx_mono()
                    .clear_bit()
                    .rx_mono_fst_vld()
                    .set_bit()
                    .rx_stop_mode()
                    .variant(2)
                    .rx_tdm_en()
                    .set_bit()
                    .rx_pdm_en()
                    .clear_bit()
                    .rx_pcm_bypass()
                    .set_bit()
                    .rx_big_endian()
                    .clear_bit()
                    .rx_bit_order()
                    .clear_bit()
            });

            i2s.rx_tdm_ctrl.modify(|_, w| {
                w.rx_tdm_tot_chan_num()
                    .variant(1)
                    .rx_tdm_pdm_chan0_en()
                    .set_bit()
                    .rx_tdm_pdm_chan1_en()
                    .set_bit()
                    .rx_tdm_pdm_chan2_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan3_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan4_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan5_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan6_en()
                    .clear_bit()
                    .rx_tdm_pdm_chan7_en()
                    .clear_bit()
                    .rx_tdm_chan8_en()
                    .clear_bit()
                    .rx_tdm_chan9_en()
                    .clear_bit()
                    .rx_tdm_chan10_en()
                    .clear_bit()
                    .rx_tdm_chan11_en()
                    .clear_bit()
                    .rx_tdm_chan12_en()
                    .clear_bit()
                    .rx_tdm_chan13_en()
                    .clear_bit()
                    .rx_tdm_chan14_en()
                    .clear_bit()
                    .rx_tdm_chan15_en()
                    .clear_bit()
            });
        }

        fn set_master(&self) {
            let i2s = self.register_block();
            i2s.tx_conf.modify(|_, w| w.tx_slave_mod().clear_bit());
            i2s.rx_conf.modify(|_, w| w.rx_slave_mod().clear_bit());
        }

        fn update(&self) {
            let i2s = self.register_block();
            i2s.tx_conf.modify(|_, w| w.tx_update().clear_bit());
            i2s.tx_conf.modify(|_, w| w.tx_update().set_bit());

            i2s.rx_conf.modify(|_, w| w.rx_update().clear_bit());
            i2s.rx_conf.modify(|_, w| w.rx_update().set_bit());
        }

        fn reset_tx(&self) {
            let i2s = self.register_block();
            i2s.tx_conf
                .modify(|_, w| w.tx_reset().set_bit().tx_fifo_reset().set_bit());
            i2s.tx_conf
                .modify(|_, w| w.tx_reset().clear_bit().tx_fifo_reset().clear_bit());

            i2s.int_clr
                .write(|w| w.tx_done_int_clr().set_bit().tx_hung_int_clr().set_bit());
        }

        fn tx_start(&self) {
            let i2s = self.register_block();
            i2s.tx_conf.modify(|_, w| w.tx_start().set_bit());
        }

        fn wait_for_tx_done(&self) {
            let i2s = self.register_block();
            while i2s.state.read().tx_idle().bit_is_clear() {
                // wait
            }

            i2s.tx_conf.modify(|_, w| w.tx_start().clear_bit());
        }

        fn reset_rx(&self) {
            let i2s = self.register_block();
            i2s.rx_conf
                .modify(|_, w| w.rx_reset().set_bit().rx_fifo_reset().set_bit());
            i2s.rx_conf
                .modify(|_, w| w.rx_reset().clear_bit().rx_fifo_reset().clear_bit());

            i2s.int_clr
                .write(|w| w.rx_done_int_clr().set_bit().rx_hung_int_clr().set_bit());
        }

        fn rx_start(&self, len: usize) {
            let i2s = self.register_block();
            i2s.rxeof_num.write(|w| w.rx_eof_num().variant(len as u16));
            i2s.rx_conf.modify(|_, w| w.rx_start().set_bit());
        }

        fn wait_for_rx_done(&self) {
            let i2s = self.register_block();
            while i2s.int_raw.read().rx_done_int_raw().bit_is_clear() {
                // wait
            }

            i2s.int_clr.write(|w| w.rx_done_int_clr().set_bit());
        }
    }

    #[derive(Clone)]
    pub struct I2sPeripheral0 {}

    #[cfg(any(esp32s3, esp32))]
    #[derive(Clone)]
    pub struct I2sPeripheral1 {}

    #[cfg(any(esp32c3, esp32c6, esp32h2))]
    impl Signals for I2sPeripheral0 {
        fn get_peripheral(&self) -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S_MCLK
        }

        fn bclk_signal(&self) -> OutputSignal {
            OutputSignal::I2SO_BCK
        }

        fn ws_signal(&self) -> OutputSignal {
            OutputSignal::I2SO_WS
        }

        fn dout_signal(&self) -> OutputSignal {
            OutputSignal::I2SO_SD
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2SI_BCK
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2SI_WS
        }

        fn din_signal(&self) -> InputSignal {
            InputSignal::I2SI_SD
        }
    }

    #[cfg(esp32s3)]
    impl Signals for I2sPeripheral0 {
        fn get_peripheral(&self) -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S0_MCLK
        }

        fn bclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_BCK
        }

        fn ws_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_WS
        }

        fn dout_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_SD
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S0I_BCK
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S0I_WS
        }

        fn din_signal(&self) -> InputSignal {
            InputSignal::I2S0I_SD
        }
    }

    #[cfg(esp32s3)]
    impl Signals for I2sPeripheral1 {
        fn get_peripheral(&self) -> Peripheral {
            Peripheral::I2s1
        }

        fn get_dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::I2s1
        }

        fn mclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S1_MCLK
        }

        fn bclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_BCK
        }

        fn ws_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_WS
        }

        fn dout_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_SD
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S1I_BCK
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S1I_WS
        }

        fn din_signal(&self) -> InputSignal {
            InputSignal::I2S1I_SD
        }
    }

    #[cfg(esp32)]
    impl Signals for I2sPeripheral0 {
        fn get_peripheral(&self) -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal(&self) -> OutputSignal {
            panic!("MCLK currently not supported on ESP32");
        }

        fn bclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_BCK
        }

        fn ws_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_WS
        }

        fn dout_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_DATA_23
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S0I_BCK
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S0I_WS
        }

        fn din_signal(&self) -> InputSignal {
            InputSignal::I2S0I_DATA_15
        }
    }

    #[cfg(esp32)]
    impl Signals for I2sPeripheral1 {
        fn get_peripheral(&self) -> Peripheral {
            Peripheral::I2s1
        }

        fn get_dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::I2s1
        }

        fn mclk_signal(&self) -> OutputSignal {
            panic!("MCLK currently not supported on ESP32");
        }

        fn bclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_BCK
        }

        fn ws_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_WS
        }

        fn dout_signal(&self) -> OutputSignal {
            OutputSignal::I2S1O_DATA_23
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S1I_BCK
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S1I_WS
        }

        fn din_signal(&self) -> InputSignal {
            InputSignal::I2S1I_DATA_15
        }
    }

    #[cfg(esp32s2)]
    impl Signals for I2sPeripheral0 {
        fn get_peripheral(&self) -> Peripheral {
            Peripheral::I2s0
        }

        fn get_dma_peripheral(&self) -> DmaPeripheral {
            DmaPeripheral::I2s0
        }

        fn mclk_signal(&self) -> OutputSignal {
            OutputSignal::CLK_I2S
        }

        fn bclk_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_BCK
        }

        fn ws_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_WS
        }

        fn dout_signal(&self) -> OutputSignal {
            OutputSignal::I2S0O_DATA_OUT23
        }

        fn bclk_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S0I_BCK
        }

        fn ws_rx_signal(&self) -> OutputSignal {
            OutputSignal::I2S0I_WS
        }

        fn din_signal(&self) -> InputSignal {
            InputSignal::I2S0I_DATA_IN15
        }
    }

    impl RegBlock for I2sPeripheral0 {
        fn register_block(&self) -> &'static RegisterBlock {
            unsafe { core::mem::transmute(I2S0::PTR) }
        }
    }

    #[cfg(any(esp32s3, esp32))]
    impl RegBlock for I2sPeripheral1 {
        fn register_block(&self) -> &'static RegisterBlock {
            unsafe { core::mem::transmute(crate::peripherals::I2S1::PTR) }
        }
    }

    impl RegisterAccessPrivate for I2sPeripheral0 {}
    impl super::RegisterAccess for I2sPeripheral0 {}

    #[cfg(any(esp32s3, esp32))]
    impl RegisterAccessPrivate for I2sPeripheral1 {}
    #[cfg(any(esp32s3, esp32))]
    impl super::RegisterAccess for I2sPeripheral1 {}

    pub struct I2sClockDividers {
        mclk_divider: u32,
        bclk_divider: u32,
        denominator: u32,
        numerator: u32,
    }

    pub fn calculate_clock(
        sample_rate: impl Into<fugit::HertzU32>,
        channels: u8,
        data_bits: u8,
        _clocks: &Clocks,
    ) -> I2sClockDividers {
        // this loosely corresponds to `i2s_std_calculate_clock` and
        // `i2s_ll_tx_set_mclk` in esp-idf
        //
        // main difference is we are using fixed-point arithmetic here

        // If data_bits is a power of two, use 256 as the mclk_multiple
        // If data_bits is 24, use 192 (24 * 8) as the mclk_multiple
        let mclk_multiple = if data_bits == 24 { 192 } else { 256 };
        let sclk = crate::soc::constants::I2S_SCLK; // for now it's fixed 160MHz and 96MHz (just H2)

        let rate_hz: HertzU32 = sample_rate.into();
        let rate = rate_hz.raw();

        let bclk = rate * channels as u32 * data_bits as u32;
        let mclk = rate * mclk_multiple;
        let bclk_divider = mclk / bclk;
        let mut mclk_divider = sclk / mclk;

        let mut ma: u32;
        let mut mb: u32;
        let mut denominator: u32 = 0;
        let mut numerator: u32 = 0;

        let freq_diff = sclk.abs_diff(mclk * mclk_divider);

        if freq_diff != 0 {
            let decimal = freq_diff as u64 * 10000 / mclk as u64;

            // Carry bit if the decimal is greater than 1.0 - 1.0 / (63.0 * 2) = 125.0 /
            // 126.0
            if decimal > 1250000 / 126 {
                mclk_divider += 1;
            } else {
                let mut min: u32 = !0;

                for a in 2..=I2S_LL_MCLK_DIVIDER_MAX {
                    let b = (a as u64) * (freq_diff as u64 * 10000u64 / mclk as u64) + 5000;
                    ma = ((freq_diff as u64 * 10000u64 * a as u64) / 10000) as u32;
                    mb = (mclk as u64 * (b / 10000)) as u32;

                    if ma == mb {
                        denominator = a as u32;
                        numerator = (b / 10000) as u32;
                        break;
                    }

                    if mb.abs_diff(ma) < min {
                        denominator = a as u32;
                        numerator = b as u32;
                        min = mb.abs_diff(ma);
                    }
                }
            }
        }

        I2sClockDividers {
            mclk_divider,
            bclk_divider,
            denominator,
            numerator,
        }
    }
}
