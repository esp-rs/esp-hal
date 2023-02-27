//! # Serial Peripheral Interface
//!
//! There are multiple ways to use SPI, depending on your needs. Regardless of
//! which way you choose, you must first create an SPI instance with
//! [`Spi::new`].
//!
//! ```rust
//! let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
//! let sclk = io.pins.gpio12;
//! let miso = io.pins.gpio11;
//! let mosi = io.pins.gpio13;
//! let cs = io.pins.gpio10;
//!
//! let mut spi = hal::spi::Spi::new(
//!     peripherals.SPI2,
//!     sclk,
//!     mosi,
//!     miso,
//!     cs,
//!     100u32.kHz(),
//!     SpiMode::Mode0,
//!     &mut peripheral_clock_control,
//!     &mut clocks,
//! );
//! ```
//!
//! ## Exclusive access to the SPI bus
//!
//! If all you want to do is to communicate to a single device, and you initiate
//! transactions yourself, there are a number of ways to achieve this:
//!
//! - Use the [`FullDuplex`](embedded_hal::spi::FullDuplex) trait to read/write
//!   single bytes at a time,
//! - Use the [`SpiBus`](embedded_hal_1::spi::SpiBus) trait (requires the "eh1"
//!   feature) and its associated functions to initiate transactions with
//!   simultaneous reads and writes, or
//! - Use the [`SpiBusWrite`](embedded_hal_1::spi::SpiBusWrite) and
//!   [`SpiBusRead`](embedded_hal_1::spi::SpiBusRead) traits (requires the "eh1"
//!   feature) and their associated functions to read or write mutiple bytes at
//!   a time.
//!
//!
//! ## Shared SPI access
//!
//! If you have multiple devices on the same SPI bus that each have their own CS
//! line, you may want to have a look at the [`SpiBusController`] and
//! [`SpiBusDevice`] implemented here. These give exclusive access to the
//! underlying SPI bus by means of a Mutex. This ensures that device
//! transactions do not interfere with each other.

use fugit::HertzU32;

use crate::{
    clock::Clocks,
    dma::{DmaError, DmaPeripheral, Rx, Tx},
    peripheral::{Peripheral, PeripheralRef},
    peripherals::spi2::RegisterBlock,
    system::PeripheralClockControl,
    types::{InputSignal, OutputSignal},
    InputPin,
    OutputPin,
};

/// The size of the FIFO buffer for SPI
#[cfg(not(esp32s2))]
const FIFO_SIZE: usize = 64;
#[cfg(esp32s2)]
const FIFO_SIZE: usize = 72;
/// Padding byte for empty write transfers
const EMPTY_WRITE_PAD: u8 = 0x00u8;

#[allow(unused)]
const MAX_DMA_SIZE: usize = 32736;

#[derive(Debug, Clone, Copy)]
pub enum Error {
    DmaError(DmaError),
    MaxDmaTransferSizeExceeded,
    Unknown,
}

impl From<DmaError> for Error {
    fn from(value: DmaError) -> Self {
        Error::DmaError(value)
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_1::spi::Error for Error {
    fn kind(&self) -> embedded_hal_1::spi::ErrorKind {
        embedded_hal_1::spi::ErrorKind::Other
    }
}

#[derive(Debug, Clone, Copy)]
pub enum SpiMode {
    Mode0,
    Mode1,
    Mode2,
    Mode3,
}

pub struct Spi<'d, T> {
    spi: PeripheralRef<'d, T>,
}

impl<'d, T> Spi<'d, T>
where
    T: Instance,
{
    /// Constructs an SPI instance in 8bit dataframe mode.
    pub fn new<SCK: OutputPin, MOSI: OutputPin, MISO: InputPin, CS: OutputPin>(
        spi: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = SCK> + 'd,
        mosi: impl Peripheral<P = MOSI> + 'd,
        miso: impl Peripheral<P = MISO> + 'd,
        cs: impl Peripheral<P = CS> + 'd,
        frequency: HertzU32,
        mode: SpiMode,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        crate::into_ref!(spi, sck, mosi, miso, cs);
        sck.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.sclk_signal());

        mosi.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.mosi_signal());

        miso.set_to_input()
            .connect_input_to_peripheral(spi.miso_signal());

        cs.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.cs_signal());

        Self::new_internal(spi, frequency, mode, peripheral_clock_control, clocks)
    }

    /// Constructs an SPI instance in 8bit dataframe mode without CS pin.
    pub fn new_no_cs<SCK: OutputPin, MOSI: OutputPin, MISO: InputPin>(
        spi: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = SCK> + 'd,
        mosi: impl Peripheral<P = MOSI> + 'd,
        miso: impl Peripheral<P = MISO> + 'd,
        frequency: HertzU32,
        mode: SpiMode,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        crate::into_ref!(spi, sck, mosi, miso);
        sck.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.sclk_signal());

        mosi.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.mosi_signal());

        miso.set_to_input()
            .connect_input_to_peripheral(spi.miso_signal());

        Self::new_internal(spi, frequency, mode, peripheral_clock_control, clocks)
    }

    /// Constructs an SPI instance in 8bit dataframe mode without CS and MISO
    /// pin.
    pub fn new_no_cs_no_miso<SCK: OutputPin, MOSI: OutputPin>(
        spi: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = SCK> + 'd,
        mosi: impl Peripheral<P = MOSI> + 'd,
        frequency: HertzU32,
        mode: SpiMode,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        crate::into_ref!(spi, sck, mosi);
        sck.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.sclk_signal());

        mosi.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.mosi_signal());

        Self::new_internal(spi, frequency, mode, peripheral_clock_control, clocks)
    }

    /// Constructs an SPI instance in 8bit dataframe mode with only MOSI
    /// connected. This might be useful for (ab)using SPI to  implement
    /// other protocols by bitbanging (WS2812B, onewire, generating arbitrary
    /// waveforms…)
    pub fn new_mosi_only<MOSI: OutputPin>(
        spi: impl Peripheral<P = T> + 'd,
        mosi: impl Peripheral<P = MOSI> + 'd,
        frequency: HertzU32,
        mode: SpiMode,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        crate::into_ref!(spi, mosi);
        mosi.set_to_push_pull_output()
            .connect_peripheral_to_output(spi.mosi_signal());

        Self::new_internal(spi, frequency, mode, peripheral_clock_control, clocks)
    }

    pub(crate) fn new_internal(
        spi: PeripheralRef<'d, T>,
        frequency: HertzU32,
        mode: SpiMode,
        peripheral_clock_control: &mut PeripheralClockControl,
        clocks: &Clocks,
    ) -> Self {
        spi.enable_peripheral(peripheral_clock_control);

        let mut spi = Self { spi };
        spi.spi.setup(frequency, clocks);
        spi.spi.init();
        spi.spi.set_data_mode(mode);

        spi
    }

    pub fn change_bus_frequency(&mut self, frequency: HertzU32, clocks: &Clocks) {
        self.spi.ch_bus_freq(frequency, clocks);
    }
}

impl<T> embedded_hal::spi::FullDuplex<u8> for Spi<'_, T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.spi.read_byte()
    }

    fn send(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.spi.write_byte(word)
    }
}

impl<T> embedded_hal::blocking::spi::Transfer<u8> for Spi<'_, T>
where
    T: Instance,
{
    type Error = Error;

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        self.spi.transfer(words)
    }
}

impl<T> embedded_hal::blocking::spi::Write<u8> for Spi<'_, T>
where
    T: Instance,
{
    type Error = Error;

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.spi.write_bytes(words)?;
        self.spi.flush()?;
        Ok(())
    }
}

pub mod dma {
    use core::mem;

    use embedded_dma::{ReadBuffer, WriteBuffer};

    #[cfg(any(esp32, esp32s2))]
    use super::Spi3Instance;
    use super::{Instance, InstanceDma, Spi, Spi2Instance, MAX_DMA_SIZE};
    #[cfg(any(esp32, esp32s2))]
    use crate::dma::Spi3Peripheral;
    use crate::{
        dma::{Channel, DmaTransfer, DmaTransferRxTx, Rx, Spi2Peripheral, SpiPeripheral, Tx},
        peripheral::PeripheralRef,
    };

    pub trait WithDmaSpi2<'d, T, RX, TX, P>
    where
        T: Instance + Spi2Instance,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        fn with_dma(self, channel: Channel<TX, RX, P>) -> SpiDma<'d, T, TX, RX, P>;
    }

    #[cfg(any(esp32, esp32s2))]
    pub trait WithDmaSpi3<'d, T, RX, TX, P>
    where
        T: Instance + Spi3Instance,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        fn with_dma(self, channel: Channel<TX, RX, P>) -> SpiDma<'d, T, TX, RX, P>;
    }

    impl<'d, T, RX, TX, P> WithDmaSpi2<'d, T, RX, TX, P> for Spi<'d, T>
    where
        T: Instance + Spi2Instance,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral + Spi2Peripheral,
    {
        fn with_dma(self, mut channel: Channel<TX, RX, P>) -> SpiDma<'d, T, TX, RX, P> {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            SpiDma {
                spi: self.spi,
                channel,
            }
        }
    }

    #[cfg(any(esp32, esp32s2))]
    impl<'d, T, RX, TX, P> WithDmaSpi3<'d, T, RX, TX, P> for Spi<'d, T>
    where
        T: Instance + Spi3Instance,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral + Spi3Peripheral,
    {
        fn with_dma(self, mut channel: Channel<TX, RX, P>) -> SpiDma<'d, T, TX, RX, P> {
            channel.tx.init_channel(); // no need to call this for both, TX and RX

            SpiDma {
                spi: self.spi,
                channel,
            }
        }
    }
    /// An in-progress DMA transfer
    pub struct SpiDmaTransferRxTx<'d, T, TX, RX, P, RBUFFER, TBUFFER>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        spi_dma: SpiDma<'d, T, TX, RX, P>,
        rbuffer: RBUFFER,
        tbuffer: TBUFFER,
    }

    impl<'d, T, TX, RX, P, RXBUF, TXBUF> DmaTransferRxTx<RXBUF, TXBUF, SpiDma<'d, T, TX, RX, P>>
        for SpiDmaTransferRxTx<'d, T, TX, RX, P, RXBUF, TXBUF>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        /// Wait for the DMA transfer to complete and return the buffers and the
        /// SPI instance.
        fn wait(mut self) -> (RXBUF, TXBUF, SpiDma<'d, T, TX, RX, P>) {
            self.spi_dma.spi.flush().ok(); // waiting for the DMA transfer is not enough

            // `DmaTransfer` needs to have a `Drop` implementation, because we accept
            // managed buffers that can free their memory on drop. Because of that
            // we can't move out of the `DmaTransfer`'s fields, so we use `ptr::read`
            // and `mem::forget`.
            //
            // NOTE(unsafe) There is no panic branch between getting the resources
            // and forgetting `self`.
            unsafe {
                let rbuffer = core::ptr::read(&self.rbuffer);
                let tbuffer = core::ptr::read(&self.tbuffer);
                let payload = core::ptr::read(&self.spi_dma);
                mem::forget(self);
                (rbuffer, tbuffer, payload)
            }
        }
    }

    impl<'d, T, TX, RX, P, RXBUF, TXBUF> Drop for SpiDmaTransferRxTx<'d, T, TX, RX, P, RXBUF, TXBUF>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        fn drop(&mut self) {
            self.spi_dma.spi.flush().ok();
        }
    }

    /// An in-progress DMA transfer.
    pub struct SpiDmaTransfer<'d, T, TX, RX, P, BUFFER>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        spi_dma: SpiDma<'d, T, TX, RX, P>,
        buffer: BUFFER,
    }

    impl<'d, T, TX, RX, P, BUFFER> DmaTransfer<BUFFER, SpiDma<'d, T, TX, RX, P>>
        for SpiDmaTransfer<'d, T, TX, RX, P, BUFFER>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        /// Wait for the DMA transfer to complete and return the buffers and the
        /// SPI instance.
        fn wait(mut self) -> (BUFFER, SpiDma<'d, T, TX, RX, P>) {
            self.spi_dma.spi.flush().ok(); // waiting for the DMA transfer is not enough

            // `DmaTransfer` needs to have a `Drop` implementation, because we accept
            // managed buffers that can free their memory on drop. Because of that
            // we can't move out of the `DmaTransfer`'s fields, so we use `ptr::read`
            // and `mem::forget`.
            //
            // NOTE(unsafe) There is no panic branch between getting the resources
            // and forgetting `self`.
            unsafe {
                let buffer = core::ptr::read(&self.buffer);
                let payload = core::ptr::read(&self.spi_dma);
                mem::forget(self);
                (buffer, payload)
            }
        }
    }

    impl<'d, T, TX, RX, P, BUFFER> Drop for SpiDmaTransfer<'d, T, TX, RX, P, BUFFER>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        fn drop(&mut self) {
            self.spi_dma.spi.flush().ok();
        }
    }

    /// A DMA capable SPI instance.
    pub struct SpiDma<'d, T, TX, RX, P>
    where
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        pub(crate) spi: PeripheralRef<'d, T>,
        pub(crate) channel: Channel<TX, RX, P>,
    }

    impl<'d, T, TX, RX, P> SpiDma<'d, T, TX, RX, P>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        /// Perform a DMA write.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer(s) and the SPI
        /// instance. The maximum amount of data to be sent is 32736
        /// bytes.
        pub fn dma_write<TXBUF>(
            mut self,
            words: TXBUF,
        ) -> Result<SpiDmaTransfer<'d, T, TX, RX, P, TXBUF>, super::Error>
        where
            TXBUF: ReadBuffer<Word = u8>,
        {
            let (ptr, len) = unsafe { words.read_buffer() };

            if len > MAX_DMA_SIZE {
                return Err(super::Error::MaxDmaTransferSizeExceeded);
            }

            self.spi
                .start_write_bytes_dma(ptr, len, &mut self.channel.tx)?;
            Ok(SpiDmaTransfer {
                spi_dma: self,
                buffer: words,
            })
        }

        /// Perform a DMA read.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer(s) and the SPI
        /// instance. The maximum amount of data to be received is 32736
        /// bytes.
        pub fn dma_read<RXBUF>(
            mut self,
            mut words: RXBUF,
        ) -> Result<SpiDmaTransfer<'d, T, TX, RX, P, RXBUF>, super::Error>
        where
            RXBUF: WriteBuffer<Word = u8>,
        {
            let (ptr, len) = unsafe { words.write_buffer() };

            if len > MAX_DMA_SIZE {
                return Err(super::Error::MaxDmaTransferSizeExceeded);
            }

            self.spi
                .start_read_bytes_dma(ptr, len, &mut self.channel.rx)?;
            Ok(SpiDmaTransfer {
                spi_dma: self,
                buffer: words,
            })
        }

        /// Perform a DMA transfer.
        ///
        /// This will return a [SpiDmaTransfer] owning the buffer(s) and the SPI
        /// instance. The maximum amount of data to be sent/received is
        /// 32736 bytes.
        pub fn dma_transfer<TXBUF, RXBUF>(
            mut self,
            words: TXBUF,
            mut read_buffer: RXBUF,
        ) -> Result<SpiDmaTransferRxTx<'d, T, TX, RX, P, RXBUF, TXBUF>, super::Error>
        where
            TXBUF: ReadBuffer<Word = u8>,
            RXBUF: WriteBuffer<Word = u8>,
        {
            let (write_ptr, write_len) = unsafe { words.read_buffer() };
            let (read_ptr, read_len) = unsafe { read_buffer.write_buffer() };

            if write_len > MAX_DMA_SIZE || read_len > MAX_DMA_SIZE {
                return Err(super::Error::MaxDmaTransferSizeExceeded);
            }

            self.spi.start_transfer_dma(
                write_ptr,
                write_len,
                read_ptr,
                read_len,
                &mut self.channel.tx,
                &mut self.channel.rx,
            )?;
            Ok(SpiDmaTransferRxTx {
                spi_dma: self,
                rbuffer: read_buffer,
                tbuffer: words,
            })
        }
    }

    impl<'d, T, TX, RX, P> embedded_hal::blocking::spi::Transfer<u8> for SpiDma<'d, T, TX, RX, P>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        type Error = super::Error;

        fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
            self.spi
                .transfer_in_place_dma(words, &mut self.channel.tx, &mut self.channel.rx)
        }
    }

    impl<'d, T, TX, RX, P> embedded_hal::blocking::spi::Write<u8> for SpiDma<'d, T, TX, RX, P>
    where
        T: InstanceDma<TX, RX>,
        TX: Tx,
        RX: Rx,
        P: SpiPeripheral,
    {
        type Error = super::Error;

        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.spi.write_bytes_dma(words, &mut self.channel.tx)?;
            self.spi.flush()?;
            Ok(())
        }
    }

    #[cfg(feature = "async")]
    mod asynch {
        use super::*;

        impl<'d, T, TX, RX, P> embedded_hal_async::spi::SpiBusWrite for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            async fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
                for chunk in words.chunks(MAX_DMA_SIZE) {
                    self.spi.start_write_bytes_dma(
                        chunk.as_ptr(),
                        chunk.len(),
                        &mut self.channel.tx,
                    )?;

                    crate::dma::asynch::DmaTxFuture::new(&mut self.channel.tx).await;

                    // FIXME: in the future we should use the peripheral DMA status registers to
                    // await on both the dma transfer _and_ the peripherals status
                    self.spi.flush()?;
                }

                Ok(())
            }
        }

        impl<'d, T, TX, RX, P> embedded_hal_async::spi::SpiBusFlush for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            async fn flush(&mut self) -> Result<(), Self::Error> {
                // TODO use async flush in the future
                self.spi.flush()
            }
        }

        impl<'d, T, TX, RX, P> embedded_hal_async::spi::SpiBusRead for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.spi.start_read_bytes_dma(
                    words.as_mut_ptr(),
                    words.len(),
                    &mut self.channel.rx,
                )?;

                crate::dma::asynch::DmaRxFuture::new(&mut self.channel.rx).await;

                Ok(())
            }
        }

        impl<'d, T, TX, RX, P> embedded_hal_async::spi::SpiBus for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            async fn transfer<'a>(
                &'a mut self,
                read: &'a mut [u8],
                write: &'a [u8],
            ) -> Result<(), Self::Error> {
                let mut idx = 0;
                loop {
                    let write_idx = isize::min(idx, write.len() as isize);
                    let write_len = usize::min(write.len() - idx as usize, MAX_DMA_SIZE);

                    let read_idx = isize::min(idx, read.len() as isize);
                    let read_len = usize::min(read.len() - idx as usize, MAX_DMA_SIZE);

                    self.spi.start_transfer_dma(
                        unsafe { write.as_ptr().offset(write_idx) },
                        write_len,
                        unsafe { read.as_mut_ptr().offset(read_idx) },
                        read_len,
                        &mut self.channel.tx,
                        &mut self.channel.rx,
                    )?;

                    embassy_futures::join::join(
                        crate::dma::asynch::DmaTxFuture::new(&mut self.channel.tx),
                        crate::dma::asynch::DmaRxFuture::new(&mut self.channel.rx),
                    )
                    .await;

                    // FIXME: in the future we should use the peripheral DMA status registers to
                    // await on both the dma transfer _and_ the peripherals status
                    self.spi.flush()?;

                    idx += MAX_DMA_SIZE as isize;
                    if idx >= write.len() as isize && idx >= read.len() as isize {
                        break;
                    }
                }

                Ok(())
            }

            async fn transfer_in_place<'a>(
                &'a mut self,
                words: &'a mut [u8],
            ) -> Result<(), Self::Error> {
                for chunk in words.chunks_mut(MAX_DMA_SIZE) {
                    self.spi.start_transfer_dma(
                        chunk.as_ptr(),
                        chunk.len(),
                        chunk.as_mut_ptr(),
                        chunk.len(),
                        &mut self.channel.tx,
                        &mut self.channel.rx,
                    )?;

                    embassy_futures::join::join(
                        crate::dma::asynch::DmaTxFuture::new(&mut self.channel.tx),
                        crate::dma::asynch::DmaRxFuture::new(&mut self.channel.rx),
                    )
                    .await;

                    // FIXME: in the future we should use the peripheral DMA status registers to
                    // await on both the dma transfer _and_ the peripherals status
                    self.spi.flush()?;
                }

                Ok(())
            }
        }
    }

    #[cfg(feature = "eh1")]
    mod ehal1 {
        use embedded_hal_1::spi::{SpiBus, SpiBusFlush, SpiBusRead, SpiBusWrite};

        use super::{super::InstanceDma, SpiDma, SpiPeripheral};
        use crate::dma::{Rx, Tx};

        impl<'d, T, TX, RX, P> embedded_hal_1::spi::ErrorType for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            type Error = super::super::Error;
        }

        impl<'d, T, TX, RX, P> SpiBusWrite for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            /// See also: [`write_bytes`].
            fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
                self.spi.write_bytes_dma(words, &mut self.channel.tx)?;
                self.flush()
            }
        }

        impl<'d, T, TX, RX, P> SpiBusRead for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.spi
                    .transfer_dma(&[], words, &mut self.channel.tx, &mut self.channel.rx)?;
                self.flush()
            }
        }

        impl<'d, T, TX, RX, P> SpiBus for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            /// Write out data from `write`, read response into `read`.
            ///
            /// `read` and `write` are allowed to have different lengths. If
            /// `write` is longer, all other bytes received are
            /// discarded. If `read` is longer, [`EMPTY_WRITE_PAD`]
            /// is written out as necessary until enough bytes have
            /// been read. Reading and writing happens
            /// simultaneously.
            fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
                self.spi
                    .transfer_dma(write, read, &mut self.channel.tx, &mut self.channel.rx)?;
                self.flush()
            }

            /// Transfer data in place.
            ///
            /// Writes data from `words` out on the bus and stores the reply
            /// into `words`. A convenient wrapper around
            /// [`write`](SpiBusWrite::write), [`flush`](SpiBusFlush::flush) and
            /// [`read`](SpiBusRead::read).
            fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
                self.spi.transfer_in_place_dma(
                    words,
                    &mut self.channel.tx,
                    &mut self.channel.rx,
                )?;
                self.flush()
            }
        }

        impl<'d, T, TX, RX, P> SpiBusFlush for SpiDma<'d, T, TX, RX, P>
        where
            T: InstanceDma<TX, RX>,
            TX: Tx,
            RX: Rx,
            P: SpiPeripheral,
        {
            fn flush(&mut self) -> Result<(), Self::Error> {
                self.spi.flush()
            }
        }
    }
}

#[cfg(feature = "eh1")]
pub use ehal1::*;

#[cfg(feature = "eh1")]
mod ehal1 {
    use core::cell::RefCell;

    use embedded_hal_1::spi::{
        self,
        ErrorType,
        SpiBus,
        SpiBusFlush,
        SpiBusRead,
        SpiBusWrite,
        SpiDevice,
    };
    use embedded_hal_nb::spi::FullDuplex;

    use super::*;
    use crate::OutputPin;

    impl<T> embedded_hal_1::spi::ErrorType for Spi<'_, T> {
        type Error = super::Error;
    }

    impl<T> FullDuplex for Spi<'_, T>
    where
        T: Instance,
    {
        fn read(&mut self) -> nb::Result<u8, Self::Error> {
            self.spi.read_byte()
        }

        fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
            self.spi.write_byte(word)
        }
    }

    impl<T> SpiBusWrite for Spi<'_, T>
    where
        T: Instance,
    {
        /// See also: [`write_bytes`].
        fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
            self.spi.write_bytes(words)
        }
    }

    impl<T> SpiBusRead for Spi<'_, T>
    where
        T: Instance,
    {
        /// See also: [`read_bytes`].
        fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.spi.read_bytes(words)
        }
    }

    impl<T> SpiBus for Spi<'_, T>
    where
        T: Instance,
    {
        /// Write out data from `write`, read response into `read`.
        ///
        /// `read` and `write` are allowed to have different lengths. If `write`
        /// is longer, all other bytes received are discarded. If `read`
        /// is longer, [`EMPTY_WRITE_PAD`] is written out as necessary
        /// until enough bytes have been read. Reading and writing happens
        /// simultaneously.
        fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
            // Optimizations
            if read.len() == 0 {
                SpiBusWrite::write(self, write)?;
            } else if write.len() == 0 {
                SpiBusRead::read(self, read)?;
            }

            let mut write_from = 0;
            let mut read_from = 0;

            loop {
                // How many bytes we write in this chunk
                let write_inc = core::cmp::min(FIFO_SIZE, write.len() - write_from);
                let write_to = write_from + write_inc;
                // How many bytes we read in this chunk
                let read_inc = core::cmp::min(FIFO_SIZE, read.len() - read_from);
                let read_to = read_from + read_inc;

                if (write_inc == 0) && (read_inc == 0) {
                    break;
                }

                if write_to < read_to {
                    // Read more than we write, must pad writing part with zeros
                    let mut empty = [EMPTY_WRITE_PAD; FIFO_SIZE];
                    empty[0..write_inc].copy_from_slice(&write[write_from..write_to]);
                    SpiBusWrite::write(self, &empty)?;
                } else {
                    SpiBusWrite::write(self, &write[write_from..write_to])?;
                }

                SpiBusFlush::flush(self)?;

                if read_inc > 0 {
                    self.spi
                        .read_bytes_from_fifo(&mut read[read_from..read_to])?;
                }

                write_from = write_to;
                read_from = read_to;
            }
            Ok(())
        }

        /// Transfer data in place.
        ///
        /// Writes data from `words` out on the bus and stores the reply into
        /// `words`. A convenient wrapper around
        /// [`write`](SpiBusWrite::write), [`flush`](SpiBusFlush::flush) and
        /// [`read`](SpiBusRead::read).
        fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            // Since we have the traits so neatly implemented above, use them!
            for chunk in words.chunks_mut(FIFO_SIZE) {
                SpiBusWrite::write(self, chunk)?;
                SpiBusFlush::flush(self)?;
                self.spi.read_bytes_from_fifo(chunk)?;
            }
            Ok(())
        }
    }

    impl<T> SpiBusFlush for Spi<'_, T>
    where
        T: Instance,
    {
        fn flush(&mut self) -> Result<(), Self::Error> {
            self.spi.flush()
        }
    }

    /// SPI bus controller.
    ///
    /// Has exclusive access to an SPI bus, which is managed via a `Mutex`. Used
    /// as basis for the [`SpiBusDevice`] implementation. Note that the
    /// wrapped [`RefCell`] is used solely to achieve interior mutability.
    pub struct SpiBusController<'d, I: Instance> {
        lock: critical_section::Mutex<RefCell<Spi<'d, I>>>,
    }

    impl<'d, I: Instance> SpiBusController<'d, I> {
        /// Create a new controller from an SPI bus instance.
        ///
        /// Takes ownership of the SPI bus in the process. Afterwards, the SPI
        /// bus can only be accessed via instances of [`SpiBusDevice`].
        pub fn from_spi(bus: Spi<'d, I>) -> Self {
            SpiBusController {
                lock: critical_section::Mutex::new(RefCell::new(bus)),
            }
        }

        pub fn add_device<'a, CS: OutputPin>(&'a self, cs: CS) -> SpiBusDevice<'a, 'd, I, CS> {
            SpiBusDevice::new(self, cs)
        }
    }

    impl<'d, I: Instance> ErrorType for SpiBusController<'d, I> {
        type Error = spi::ErrorKind;
    }

    /// An SPI device on a shared SPI bus.
    ///
    /// Provides device specific access on a shared SPI bus. Enables attaching
    /// multiple SPI devices to the same bus, each with its own CS line, and
    /// performing safe transfers on them.
    pub struct SpiBusDevice<'a, 'd, I, CS>
    where
        I: Instance,
        CS: OutputPin,
    {
        bus: &'a SpiBusController<'d, I>,
        cs: CS,
    }

    impl<'a, 'd, I, CS> SpiBusDevice<'a, 'd, I, CS>
    where
        I: Instance,
        CS: OutputPin,
    {
        pub fn new(bus: &'a SpiBusController<'d, I>, mut cs: CS) -> Self {
            cs.set_to_push_pull_output().set_output_high(true);
            SpiBusDevice { bus, cs }
        }
    }

    impl<'a, 'd, I, CS> ErrorType for SpiBusDevice<'a, 'd, I, CS>
    where
        I: Instance,
        CS: OutputPin,
    {
        type Error = spi::ErrorKind;
    }

    impl<'a, 'd, I, CS> SpiDevice for SpiBusDevice<'a, 'd, I, CS>
    where
        I: Instance,
        CS: OutputPin + crate::gpio::OutputPin,
    {
        type Bus = Spi<'d, I>;

        fn transaction<R>(
            &mut self,
            f: impl FnOnce(&mut Self::Bus) -> Result<R, <Self::Bus as ErrorType>::Error>,
        ) -> Result<R, Self::Error> {
            critical_section::with(|cs| {
                let mut bus = self.bus.lock.borrow_ref_mut(cs);

                self.cs.connect_peripheral_to_output(bus.spi.cs_signal());

                // We postpone handling these errors until AFTER we raised CS again, so the bus
                // is free (Or we die trying if CS errors).
                let f_res = f(&mut bus);
                let flush_res = bus.flush();

                self.cs.disconnect_peripheral_from_output();

                let f_res = f_res.map_err(|_| spi::ErrorKind::Other)?;
                flush_res.map_err(|_| spi::ErrorKind::Other)?;

                Ok(f_res)
            })
        }
    }
}

pub trait InstanceDma<TX, RX>: Instance
where
    TX: Tx,
    RX: Rx,
{
    fn transfer_in_place_dma<'w>(
        &mut self,
        words: &'w mut [u8],
        tx: &mut TX,
        rx: &mut RX,
    ) -> Result<&'w [u8], Error> {
        for chunk in words.chunks_mut(MAX_DMA_SIZE) {
            self.start_transfer_dma(
                chunk.as_ptr(),
                chunk.len(),
                chunk.as_mut_ptr(),
                chunk.len(),
                tx,
                rx,
            )?;

            while !tx.is_done() && !rx.is_done() {}
            self.flush().unwrap();
        }

        return Ok(words);
    }

    fn transfer_dma<'w>(
        &mut self,
        write_buffer: &'w [u8],
        read_buffer: &'w mut [u8],
        tx: &mut TX,
        rx: &mut RX,
    ) -> Result<&'w [u8], Error> {
        let mut idx = 0;
        loop {
            let write_idx = isize::min(idx, write_buffer.len() as isize);
            let write_len = usize::min(write_buffer.len() - idx as usize, MAX_DMA_SIZE);

            let read_idx = isize::min(idx, read_buffer.len() as isize);
            let read_len = usize::min(read_buffer.len() - idx as usize, MAX_DMA_SIZE);

            self.start_transfer_dma(
                unsafe { write_buffer.as_ptr().offset(write_idx) },
                write_len,
                unsafe { read_buffer.as_mut_ptr().offset(read_idx) },
                read_len,
                tx,
                rx,
            )?;

            while !tx.is_done() && !rx.is_done() {}
            self.flush().unwrap();

            idx += MAX_DMA_SIZE as isize;
            if idx >= write_buffer.len() as isize && idx >= read_buffer.len() as isize {
                break;
            }
        }

        return Ok(read_buffer);
    }

    fn start_transfer_dma<'w>(
        &mut self,
        write_buffer_ptr: *const u8,
        write_buffer_len: usize,
        read_buffer_ptr: *mut u8,
        read_buffer_len: usize,
        tx: &mut TX,
        rx: &mut RX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();
        self.configure_datalen(usize::max(read_buffer_len, write_buffer_len) as u32 * 8);

        tx.is_done();
        rx.is_done();

        self.enable_dma();
        self.update();

        reset_dma_before_load_dma_dscr(reg_block);
        tx.prepare_transfer(
            self.dma_peripheral(),
            false,
            write_buffer_ptr,
            write_buffer_len,
        )?;
        rx.prepare_transfer(
            false,
            self.dma_peripheral(),
            read_buffer_ptr,
            read_buffer_len,
        )?;

        self.clear_dma_interrupts();
        reset_dma_before_usr_cmd(reg_block);

        reg_block.cmd.modify(|_, w| w.usr().set_bit());

        Ok(())
    }

    fn write_bytes_dma<'w>(&mut self, words: &'w [u8], tx: &mut TX) -> Result<&'w [u8], Error> {
        for chunk in words.chunks(MAX_DMA_SIZE) {
            self.start_write_bytes_dma(chunk.as_ptr(), chunk.len(), tx)?;

            while !tx.is_done() {}
            self.flush().unwrap(); // seems "is_done" doesn't work as intended?
        }

        return Ok(words);
    }

    fn start_write_bytes_dma<'w>(
        &mut self,
        ptr: *const u8,
        len: usize,
        tx: &mut TX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();
        self.configure_datalen(len as u32 * 8);

        tx.is_done();

        self.enable_dma();
        self.update();

        reset_dma_before_load_dma_dscr(reg_block);
        tx.prepare_transfer(self.dma_peripheral(), false, ptr, len)?;

        self.clear_dma_interrupts();
        reset_dma_before_usr_cmd(reg_block);

        reg_block.cmd.modify(|_, w| w.usr().set_bit());

        return Ok(());
    }

    fn start_read_bytes_dma<'w>(
        &mut self,
        ptr: *mut u8,
        len: usize,
        rx: &mut RX,
    ) -> Result<(), Error> {
        let reg_block = self.register_block();
        self.configure_datalen(len as u32 * 8);

        rx.is_done();

        self.enable_dma();
        self.update();

        reset_dma_before_load_dma_dscr(reg_block);
        rx.prepare_transfer(false, self.dma_peripheral(), ptr, len)?;

        self.clear_dma_interrupts();
        reset_dma_before_usr_cmd(reg_block);

        reg_block.cmd.modify(|_, w| w.usr().set_bit());

        return Ok(());
    }

    fn dma_peripheral(&self) -> DmaPeripheral {
        match self.spi_num() {
            2 => DmaPeripheral::Spi2,
            #[cfg(any(esp32, esp32s2))]
            3 => DmaPeripheral::Spi3,
            _ => panic!("Illegal SPI instance"),
        }
    }

    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    fn enable_dma(&self) {
        let reg_block = self.register_block();
        reg_block.dma_conf.modify(|_, w| w.dma_tx_ena().set_bit());
        reg_block.dma_conf.modify(|_, w| w.dma_rx_ena().set_bit());
    }

    #[cfg(any(esp32, esp32s2))]
    fn enable_dma(&self) {
        // for non GDMA this is done in `assign_tx_device` / `assign_rx_device`
    }

    #[cfg(any(esp32c2, esp32c3, esp32s3))]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.register_block();
        reg_block.dma_int_clr.write(|w| {
            w.dma_infifo_full_err_int_clr()
                .set_bit()
                .dma_outfifo_empty_err_int_clr()
                .set_bit()
                .trans_done_int_clr()
                .set_bit()
                .mst_rx_afifo_wfull_err_int_clr()
                .set_bit()
                .mst_tx_afifo_rempty_err_int_clr()
                .set_bit()
        });
    }

    #[cfg(any(esp32, esp32s2))]
    fn clear_dma_interrupts(&self) {
        let reg_block = self.register_block();
        reg_block.dma_int_clr.write(|w| {
            w.inlink_dscr_empty_int_clr()
                .set_bit()
                .outlink_dscr_error_int_clr()
                .set_bit()
                .inlink_dscr_error_int_clr()
                .set_bit()
                .in_done_int_clr()
                .set_bit()
                .in_err_eof_int_clr()
                .set_bit()
                .in_suc_eof_int_clr()
                .set_bit()
                .out_done_int_clr()
                .set_bit()
                .out_eof_int_clr()
                .set_bit()
                .out_total_eof_int_clr()
                .set_bit()
        });
    }
}

#[cfg(not(any(esp32, esp32s2)))]
fn reset_dma_before_usr_cmd(reg_block: &RegisterBlock) {
    reg_block.dma_conf.modify(|_, w| {
        w.rx_afifo_rst()
            .set_bit()
            .buf_afifo_rst()
            .set_bit()
            .dma_afifo_rst()
            .set_bit()
    });
}

#[cfg(any(esp32, esp32s2))]
fn reset_dma_before_usr_cmd(_reg_block: &RegisterBlock) {}

#[cfg(not(any(esp32, esp32s2)))]
fn reset_dma_before_load_dma_dscr(_reg_block: &RegisterBlock) {}

#[cfg(any(esp32, esp32s2))]
fn reset_dma_before_load_dma_dscr(reg_block: &RegisterBlock) {
    reg_block.dma_conf.modify(|_, w| {
        w.out_rst()
            .set_bit()
            .in_rst()
            .set_bit()
            .ahbm_fifo_rst()
            .set_bit()
            .ahbm_rst()
            .set_bit()
    });

    reg_block.dma_conf.modify(|_, w| {
        w.out_rst()
            .clear_bit()
            .in_rst()
            .clear_bit()
            .ahbm_fifo_rst()
            .clear_bit()
            .ahbm_rst()
            .clear_bit()
    });
}

impl<TX, RX> InstanceDma<TX, RX> for crate::peripherals::SPI2
where
    TX: Tx,
    RX: Rx,
{
}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl<TX, RX> InstanceDma<TX, RX> for crate::peripherals::SPI3
where
    TX: Tx,
    RX: Rx,
{
}

pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn sclk_signal(&self) -> OutputSignal;

    fn mosi_signal(&self) -> OutputSignal;

    fn miso_signal(&self) -> InputSignal;

    fn cs_signal(&self) -> OutputSignal;

    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl);

    fn spi_num(&self) -> u8;

    fn init(&mut self) {
        let reg_block = self.register_block();
        reg_block.user.modify(|_, w| {
            w.usr_miso_highpart()
                .clear_bit()
                .usr_miso_highpart()
                .clear_bit()
                .doutdin()
                .set_bit()
                .usr_miso()
                .set_bit()
                .usr_mosi()
                .set_bit()
                .cs_hold()
                .set_bit()
                .usr_dummy_idle()
                .set_bit()
                .usr_addr()
                .clear_bit()
                .usr_command()
                .clear_bit()
        });

        #[cfg(not(any(esp32, esp32s2)))]
        reg_block.clk_gate.modify(|_, w| {
            w.clk_en()
                .set_bit()
                .mst_clk_active()
                .set_bit()
                .mst_clk_sel()
                .set_bit()
        });

        reg_block.ctrl.write(|w| unsafe { w.bits(0) });

        #[cfg(not(esp32))]
        reg_block.misc.write(|w| unsafe { w.bits(0) });

        reg_block.slave.write(|w| unsafe { w.bits(0) });
    }

    // taken from https://github.com/apache/incubator-nuttx/blob/8267a7618629838231256edfa666e44b5313348e/arch/risc-v/src/esp32c3/esp32c3_spi.c#L496
    fn setup(&mut self, frequency: HertzU32, clocks: &Clocks) {
        // FIXME: this might not be always true
        let apb_clk_freq: HertzU32 = HertzU32::Hz(clocks.apb_clock.to_Hz());

        let reg_val: u32;
        let duty_cycle = 128;

        // In HW, n, h and l fields range from 1 to 64, pre ranges from 1 to 8K.
        // The value written to register is one lower than the used value.

        if frequency > ((apb_clk_freq / 4) * 3) {
            // Using APB frequency directly will give us the best result here.
            reg_val = 1 << 31;
        } else {
            /* For best duty cycle resolution, we want n to be as close to 32 as
             * possible, but we also need a pre/n combo that gets us as close as
             * possible to the intended frequency. To do this, we bruteforce n and
             * calculate the best pre to go along with that. If there's a choice
             * between pre/n combos that give the same result, use the one with the
             * higher n.
             */

            let mut pre: i32;
            let mut bestn: i32 = -1;
            let mut bestpre: i32 = -1;
            let mut besterr: i32 = 0;
            let mut errval: i32;

            /* Start at n = 2. We need to be able to set h/l so we have at least
             * one high and one low pulse.
             */

            for n in 2..64 {
                /* Effectively, this does:
                 *   pre = round((APB_CLK_FREQ / n) / frequency)
                 */

                pre = ((apb_clk_freq.raw() as i32 / n) + (frequency.raw() as i32 / 2))
                    / frequency.raw() as i32;

                if pre <= 0 {
                    pre = 1;
                }

                if pre > 16 {
                    pre = 16;
                }

                errval = (apb_clk_freq.raw() as i32 / (pre as i32 * n as i32)
                    - frequency.raw() as i32)
                    .abs();
                if bestn == -1 || errval <= besterr {
                    besterr = errval;
                    bestn = n as i32;
                    bestpre = pre as i32;
                }
            }

            let n: i32 = bestn;
            pre = bestpre as i32;
            let l: i32 = n;

            /* Effectively, this does:
             *   h = round((duty_cycle * n) / 256)
             */

            let mut h: i32 = (duty_cycle * n + 127) / 256;
            if h <= 0 {
                h = 1;
            }

            reg_val = (l as u32 - 1)
                | ((h as u32 - 1) << 6)
                | ((n as u32 - 1) << 12)
                | ((pre as u32 - 1) << 18);
        }

        self.register_block()
            .clock
            .write(|w| unsafe { w.bits(reg_val) });
    }

    #[cfg(not(esp32))]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode3 => {
                reg_block.misc.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
        }
        self
    }

    #[cfg(esp32)]
    fn set_data_mode(&mut self, data_mode: SpiMode) -> &mut Self {
        let reg_block = self.register_block();

        match data_mode {
            SpiMode::Mode0 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
            SpiMode::Mode1 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().clear_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode2 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().set_bit());
            }
            SpiMode::Mode3 => {
                reg_block.pin.modify(|_, w| w.ck_idle_edge().set_bit());
                reg_block.user.modify(|_, w| w.ck_out_edge().clear_bit());
            }
        }
        self
    }

    fn ch_bus_freq(&mut self, frequency: HertzU32, clocks: &Clocks) {
        // Disable clock source
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        self.register_block().clk_gate.modify(|_, w| {
            w.clk_en()
                .clear_bit()
                .mst_clk_active()
                .clear_bit()
                .mst_clk_sel()
                .clear_bit()
        });

        // Change clock frequency
        self.setup(frequency, clocks);

        // Enable clock source
        #[cfg(not(any(feature = "esp32", feature = "esp32s2")))]
        self.register_block().clk_gate.modify(|_, w| {
            w.clk_en()
                .set_bit()
                .mst_clk_active()
                .set_bit()
                .mst_clk_sel()
                .set_bit()
        });
    }

    fn read_byte(&mut self) -> nb::Result<u8, Error> {
        let reg_block = self.register_block();

        if reg_block.cmd.read().usr().bit_is_set() {
            return Err(nb::Error::WouldBlock);
        }

        Ok(u32::try_into(reg_block.w0.read().bits()).unwrap_or_default())
    }

    fn write_byte(&mut self, word: u8) -> nb::Result<(), Error> {
        let reg_block = self.register_block();

        if reg_block.cmd.read().usr().bit_is_set() {
            return Err(nb::Error::WouldBlock);
        }

        self.configure_datalen(8);

        reg_block.w0.write(|w| unsafe { w.bits(word.into()) });

        self.update();

        reg_block.cmd.modify(|_, w| w.usr().set_bit());

        Ok(())
    }

    /// Write bytes to SPI.
    ///
    /// Copies the content of `words` in chunks of 64 bytes into the SPI
    /// transmission FIFO. If `words` is longer than 64 bytes, multiple
    /// sequential transfers are performed. This function will return before
    /// all bytes of the last chunk to transmit have been sent to the wire. If
    /// you must ensure that the whole messages was written correctly, use
    /// [`flush`].
    // FIXME: See below.
    fn write_bytes(&mut self, words: &[u8]) -> Result<(), Error> {
        let reg_block = self.register_block();
        let num_chunks = words.len() / FIFO_SIZE;

        // The fifo has a limited fixed size, so the data must be chunked and then
        // transmitted
        for (i, chunk) in words.chunks(FIFO_SIZE).enumerate() {
            self.configure_datalen(chunk.len() as u32 * 8);

            let fifo_ptr = reg_block.w0.as_ptr();
            for i in (0..chunk.len()).step_by(4) {
                let state = if chunk.len() - i < 4 {
                    chunk.len() % 4
                } else {
                    0
                };
                let word = match state {
                    0 => {
                        (chunk[i] as u32)
                            | (chunk[i + 1] as u32) << 8
                            | (chunk[i + 2] as u32) << 16
                            | (chunk[i + 3] as u32) << 24
                    }

                    3 => {
                        (chunk[i] as u32) | (chunk[i + 1] as u32) << 8 | (chunk[i + 2] as u32) << 16
                    }

                    2 => (chunk[i] as u32) | (chunk[i + 1] as u32) << 8,

                    1 => chunk[i] as u32,

                    _ => panic!(),
                };
                unsafe {
                    fifo_ptr.add(i / 4).write_volatile(word);
                }
            }

            self.update();

            reg_block.cmd.modify(|_, w| w.usr().set_bit());

            // Wait for all chunks to complete except the last one.
            // The function is allowed to return before the bus is idle.
            // see [embedded-hal flushing](https://docs.rs/embedded-hal/1.0.0-alpha.8/embedded_hal/spi/blocking/index.html#flushing)
            //
            // THIS IS NOT TRUE FOR EH 0.2.X! MAKE SURE TO FLUSH IN EH 0.2.X TRAIT
            // IMPLEMENTATIONS!
            if i < num_chunks {
                while reg_block.cmd.read().usr().bit_is_set() {
                    // wait
                }
            }
        }
        Ok(())
    }

    /// Read bytes from SPI.
    ///
    /// Sends out a stuffing byte for every byte to read. This function doesn't
    /// perform flushing. If you want to read the response to something you
    /// have written before, consider using [`transfer`] instead.
    fn read_bytes(&mut self, words: &mut [u8]) -> Result<(), Error> {
        let empty_array = [EMPTY_WRITE_PAD; FIFO_SIZE];

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write_bytes(&empty_array[0..chunk.len()])?;
            self.flush()?;
            self.read_bytes_from_fifo(chunk)?;
        }
        Ok(())
    }

    /// Read received bytes from SPI FIFO.
    ///
    /// Copies the contents of the SPI receive FIFO into `words`. This function
    /// doesn't perform flushing. If you want to read the response to
    /// something you have written before, consider using [`transfer`]
    /// instead.
    // FIXME: Using something like `core::slice::from_raw_parts` and
    // `copy_from_slice` on the receive registers works only for the esp32 and
    // esp32c3 varaints. The reason for this is unknown.
    fn read_bytes_from_fifo(&mut self, words: &mut [u8]) -> Result<(), Error> {
        let reg_block = self.register_block();

        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.configure_datalen(chunk.len() as u32 * 8);

            let mut fifo_ptr = reg_block.w0.as_ptr();
            for index in (0..chunk.len()).step_by(4) {
                let reg_val = unsafe { *fifo_ptr };
                let bytes = reg_val.to_le_bytes();

                let len = usize::min(chunk.len(), index + 4) - index;
                chunk[index..(index + len)].clone_from_slice(&bytes[0..len]);

                unsafe {
                    fifo_ptr = fifo_ptr.offset(1);
                };
            }
        }

        Ok(())
    }

    // Check if the bus is busy and if it is wait for it to be idle
    fn flush(&mut self) -> Result<(), Error> {
        let reg_block = self.register_block();

        while reg_block.cmd.read().usr().bit_is_set() {
            // wait for bus to be clear
        }
        Ok(())
    }

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Error> {
        for chunk in words.chunks_mut(FIFO_SIZE) {
            self.write_bytes(chunk)?;
            self.flush()?;
            self.read_bytes_from_fifo(chunk)?;
        }

        Ok(words)
    }

    #[cfg(not(any(esp32, esp32s2)))]
    fn update(&self) {
        let reg_block = self.register_block();

        reg_block.cmd.modify(|_, w| w.update().set_bit());

        while reg_block.cmd.read().update().bit_is_set() {
            // wait
        }
    }

    #[cfg(any(esp32, esp32s2))]
    fn update(&self) {
        // not need/available on ESP32/ESP32S2
    }

    fn configure_datalen(&self, len: u32) {
        let reg_block = self.register_block();

        #[cfg(any(esp32c2, esp32c3, esp32s3))]
        reg_block
            .ms_dlen
            .write(|w| unsafe { w.ms_data_bitlen().bits(len - 1) });

        #[cfg(not(any(esp32c2, esp32c3, esp32s3)))]
        {
            reg_block
                .mosi_dlen
                .write(|w| unsafe { w.usr_mosi_dbitlen().bits(len - 1) });

            reg_block
                .miso_dlen
                .write(|w| unsafe { w.usr_miso_dbitlen().bits(len - 1) });
        }
    }
}

#[cfg(any(esp32c2, esp32c3))]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::FSPICLK_MUX
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(any(esp32))]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::HSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::HSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::HSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::HSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi2);
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(any(esp32))]
impl Instance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::VSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::VSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::VSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::VSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        3
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl Instance for crate::peripherals::SPI2 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::FSPICLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::FSPID
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::FSPIQ
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::FSPICS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi2)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        2
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl Instance for crate::peripherals::SPI3 {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }

    #[inline(always)]
    fn sclk_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_CLK
    }

    #[inline(always)]
    fn mosi_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_D
    }

    #[inline(always)]
    fn miso_signal(&self) -> InputSignal {
        InputSignal::SPI3_Q
    }

    #[inline(always)]
    fn cs_signal(&self) -> OutputSignal {
        OutputSignal::SPI3_CS0
    }

    #[inline(always)]
    fn enable_peripheral(&self, peripheral_clock_control: &mut PeripheralClockControl) {
        peripheral_clock_control.enable(crate::system::Peripheral::Spi3)
    }

    #[inline(always)]
    fn spi_num(&self) -> u8 {
        3
    }
}

pub trait Spi2Instance {}

#[cfg(any(esp32, esp32s2, esp32s3))]
pub trait Spi3Instance {}

impl Spi2Instance for crate::peripherals::SPI2 {}

#[cfg(any(esp32, esp32s2, esp32s3))]
impl Spi3Instance for crate::peripherals::SPI3 {}
