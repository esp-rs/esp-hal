//! USB Serial/JTAG Controller (USB_SERIAL_JTAG)
//!
//! ## Overview
//! The USB Serial/JTAG controller can be used to program the SoC's flash, read
//! program output, or attach a debugger to the running program. This is
//! possible for any computer with a USB host (hereafter referred to as 'host'),
//! without any active external components.
//!
//! This peripheral integrates the functionality of both a USB-to-serial
//! converter as well as a USB-to-JTAG adapter. As this device directly
//! interfaces with an external USB host using only the two data lines required
//! by USB 2.0, only two pins are required to be dedicated to this functionality
//! for debugging.
//!
//! The USB Serial/JTAG controller boasts the following features:
//!
//! - Hardwired for CDC-ACM (Communication Device Class - Abstract Control
//!   Model) and JTAG adapter functionality
//! - Integrates CDC-ACM adherent serial port emulation (plug-and-play on most
//!   modern OSes); supports host controllable chip reset and entry into
//!   download mode
//! - Allows fast communication with CPU debugging core using a compact
//!   representation of JTAG instructions
//! - Two OUT Endpoints and three IN Endpoints in addition to Control Endpoint
//!   0; Up to 64-byte data payload size
//! - Internal PHY means that very few or no external components needed to
//!   connect to a host computer
//!
//! ## Usage
//! The USB Serial/JTAG driver implements a number of third-party traits, with
//! the intention of making the HAL inter-compatible with various device drivers
//! from the community. This includes, but is not limited to, the [embedded-hal]
//! and [embedded-io] blocking traits, and the [embedded-hal-async] and
//! [embedded-io-async] asynchronous traits.
//!
//! In addition to the interfaces provided by these traits, native APIs are also
//! available. See the examples below for more information on how to interact
//! with this driver.
//!
//! ## Examples
//! ### Sending and Receiving Data
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::usb_serial_jtag::UsbSerialJtag;
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//!
//! // Write bytes out over the USB Serial/JTAG:
//! usb_serial.write_bytes("Hello, world!".as_bytes()).expect("write error!");
//! }
//! ```
//! 
//! ### Splitting the USB Serial/JTAG into TX and RX Components
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::usb_serial_jtag::UsbSerialJtag;
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//! // The USB Serial/JTAG can be split into separate Transmit and Receive
//! // components:
//! let (mut tx, mut rx) = usb_serial.split();
//!
//! // Each component can be used individually to interact with the USB
//! // Serial/JTAG:
//! tx.write_bytes(&[42u8]).expect("write error!");
//! let byte = rx.read_byte().expect("read error!");
//! # }
//! ```
//! 
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-io]: https://docs.rs/embedded-io/latest/embedded_io/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
//! [embedded-io-async]: https://docs.rs/embedded-io-async/latest/embedded_io_async/

use core::{convert::Infallible, marker::PhantomData};

use crate::{
    interrupt::InterruptHandler,
    peripheral::Peripheral,
    peripherals::{usb_device::RegisterBlock, Interrupt, USB_DEVICE},
    system::PeripheralClockControl,
    Blocking,
    InterruptConfigurable,
    Mode,
};

/// Custom USB serial error type
type Error = Infallible;

/// USB Serial/JTAG (Full-duplex)
pub struct UsbSerialJtag<'d, M> {
    tx: UsbSerialJtagTx<'d, M>,
    rx: UsbSerialJtagRx<'d, M>,
}

/// USB Serial/JTAG (Transmit)
pub struct UsbSerialJtagTx<'d, M> {
    phantom: PhantomData<(&'d mut USB_DEVICE, M)>,
}

/// USB Serial/JTAG (Receive)
pub struct UsbSerialJtagRx<'d, M> {
    phantom: PhantomData<(&'d mut USB_DEVICE, M)>,
}

impl<'d, M> UsbSerialJtagTx<'d, M>
where
    M: Mode,
{
    fn new_inner() -> Self {
        Self {
            phantom: PhantomData,
        }
    }

    /// Write data to the serial output in chunks of up to 64 bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        let reg_block = USB_DEVICE::register_block();

        for chunk in data.chunks(64) {
            for byte in chunk {
                reg_block
                    .ep1()
                    .write(|w| unsafe { w.rdwr_byte().bits(*byte) });
            }
            reg_block.ep1_conf().modify(|_, w| w.wr_done().set_bit());

            while reg_block.ep1_conf().read().bits() & 0b011 == 0b000 {
                // wait
            }
        }

        Ok(())
    }

    /// Write data to the serial output in a non-blocking manner
    /// Requires manual flushing (automatically flushed every 64 bytes)
    pub fn write_byte_nb(&mut self, word: u8) -> nb::Result<(), Error> {
        let reg_block = USB_DEVICE::register_block();

        if reg_block
            .ep1_conf()
            .read()
            .serial_in_ep_data_free()
            .bit_is_set()
        {
            // the FIFO is not full
            unsafe {
                reg_block.ep1().write(|w| w.rdwr_byte().bits(word));
            }

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Flush the output FIFO and block until it has been sent
    pub fn flush_tx(&mut self) -> Result<(), Error> {
        let reg_block = USB_DEVICE::register_block();
        reg_block.ep1_conf().modify(|_, w| w.wr_done().set_bit());

        while reg_block.ep1_conf().read().bits() & 0b011 == 0b000 {
            // wait
        }

        Ok(())
    }

    /// Flush the output FIFO but don't block if it isn't ready immediately
    pub fn flush_tx_nb(&mut self) -> nb::Result<(), Error> {
        let reg_block = USB_DEVICE::register_block();
        reg_block.ep1_conf().modify(|_, w| w.wr_done().set_bit());

        if reg_block.ep1_conf().read().bits() & 0b011 == 0b000 {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }
}

impl<'d, M> UsbSerialJtagRx<'d, M>
where
    M: Mode,
{
    fn new_inner() -> Self {
        Self {
            phantom: PhantomData,
        }
    }

    /// Read a byte from the UART in a non-blocking manner
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        let reg_block = USB_DEVICE::register_block();

        // Check if there are any bytes to read
        if reg_block
            .ep1_conf()
            .read()
            .serial_out_ep_data_avail()
            .bit_is_set()
        {
            let value = reg_block.ep1().read().rdwr_byte().bits();

            Ok(value)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Read all available bytes from the RX FIFO into the provided buffer and
    /// returns the number of read bytes. Never blocks. May stop early if the
    /// number of bytes in the FIFO is larger than `buf`.
    pub fn drain_rx_fifo(&mut self, buf: &mut [u8]) -> usize {
        let mut count = 0;
        while let Ok(value) = self.read_byte() {
            buf[count] = value;
            count += 1;
            if count == buf.len() {
                break;
            }
        }
        count
    }

    /// Listen for RX-PACKET-RECV interrupts
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        USB_DEVICE::register_block()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().set_bit());
    }

    /// Stop listening for RX-PACKET-RECV interrupts
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        USB_DEVICE::register_block()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().clear_bit());
    }

    /// Checks if RX-PACKET-RECV interrupt is set
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        USB_DEVICE::register_block()
            .int_st()
            .read()
            .serial_out_recv_pkt()
            .bit_is_set()
    }

    /// Reset RX-PACKET-RECV interrupt
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        USB_DEVICE::register_block()
            .int_clr()
            .write(|w| w.serial_out_recv_pkt().clear_bit_by_one())
    }
}

impl<'d> UsbSerialJtag<'d, Blocking> {
    /// Create a new USB serial/JTAG instance with defaults
    pub fn new(usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        Self::new_inner(usb_device)
    }
}

impl<'d> crate::private::Sealed for UsbSerialJtag<'d, Blocking> {}

impl<'d> InterruptConfigurable for UsbSerialJtag<'d, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.inner_set_interrupt_handler(handler);
    }
}

impl<'d, M> UsbSerialJtag<'d, M>
where
    M: Mode,
{
    fn new_inner(_usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        // Do NOT reset the peripheral. Doing so will result in a broken USB JTAG
        // connection.
        PeripheralClockControl::enable(crate::system::Peripheral::UsbDevice);

        USB_DEVICE::disable_tx_interrupts();
        USB_DEVICE::disable_rx_interrupts();

        #[cfg(any(esp32c3, esp32s3))]
        {
            use crate::soc::efuse::*;

            // On the esp32c3, and esp32s3 the USB_EXCHG_PINS efuse is bugged and
            // doesn't swap the pullups too, this works around that.
            if Efuse::read_field_le(USB_EXCHG_PINS) {
                USB_DEVICE::register_block().conf0().modify(|_, w| {
                    w.pad_pull_override()
                        .set_bit()
                        .dm_pullup()
                        .clear_bit()
                        .dp_pullup()
                        .set_bit()
                });
            }
        }

        Self {
            tx: UsbSerialJtagTx::new_inner(),
            rx: UsbSerialJtagRx::new_inner(),
        }
    }

    fn inner_set_interrupt_handler(&mut self, handler: InterruptHandler) {
        unsafe {
            crate::interrupt::bind_interrupt(Interrupt::USB_DEVICE, handler.handler());
            crate::interrupt::enable(Interrupt::USB_DEVICE, handler.priority()).unwrap();
        }
    }

    /// Split the USB Serial JTAG peripheral into a transmitter and receiver,
    /// which is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UsbSerialJtagTx<'d, M>, UsbSerialJtagRx<'d, M>) {
        (self.tx, self.rx)
    }

    /// Write data to the serial output in chunks of up to 64 bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        self.tx.write_bytes(data)
    }

    /// Write data to the serial output in a non-blocking manner
    /// Requires manual flushing (automatically flushed every 64 bytes)
    pub fn write_byte_nb(&mut self, word: u8) -> nb::Result<(), Error> {
        self.tx.write_byte_nb(word)
    }

    /// Flush the output FIFO and block until it has been sent
    pub fn flush_tx(&mut self) -> Result<(), Error> {
        self.tx.flush_tx()
    }

    /// Flush the output FIFO but don't block if it isn't ready immediately
    pub fn flush_tx_nb(&mut self) -> nb::Result<(), Error> {
        self.tx.flush_tx_nb()
    }

    /// Read a single byte but don't block if it isn't ready immediately
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        self.rx.read_byte()
    }

    /// Listen for RX-PACKET-RECV interrupts
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        self.rx.listen_rx_packet_recv_interrupt()
    }

    /// Stop listening for RX-PACKET-RECV interrupts
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        self.rx.unlisten_rx_packet_recv_interrupt()
    }

    /// Checks if RX-PACKET-RECV interrupt is set
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        self.rx.rx_packet_recv_interrupt_set()
    }

    /// Reset RX-PACKET-RECV interrupt
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        self.rx.reset_rx_packet_recv_interrupt()
    }
}

/// USB Serial/JTAG peripheral instance
pub trait Instance: crate::private::Sealed {
    /// Get a reference to the peripheral's underlying register block
    fn register_block() -> &'static RegisterBlock;

    /// Disable all transmit interrupts for the peripheral
    fn disable_tx_interrupts() {
        Self::register_block()
            .int_ena()
            .modify(|_, w| w.serial_in_empty().clear_bit());

        Self::register_block()
            .int_clr()
            .write(|w| w.serial_in_empty().clear_bit_by_one())
    }

    /// Disable all receive interrupts for the peripheral
    fn disable_rx_interrupts() {
        Self::register_block()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().clear_bit());

        Self::register_block()
            .int_clr()
            .write(|w| w.serial_out_recv_pkt().clear_bit_by_one())
    }
}

impl Instance for USB_DEVICE {
    #[inline(always)]
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*USB_DEVICE::ptr() }
    }
}

impl<M> core::fmt::Write for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        core::fmt::Write::write_str(&mut self.tx, s)
    }
}

impl<M> core::fmt::Write for UsbSerialJtagTx<'_, M>
where
    M: Mode,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

#[cfg(feature = "ufmt")]
impl<M> ufmt_write::uWrite for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        ufmt_write::uWrite::write_str(&mut self.tx, s)
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        ufmt_write::uWrite::write_char(&mut self.tx, ch)
    }
}

#[cfg(feature = "ufmt")]
impl<M> ufmt_write::uWrite for UsbSerialJtagTx<'_, M>
where
    M: Mode,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_bytes(s.as_bytes())?;
        Ok(())
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        let mut buffer = [0u8; 4];
        self.write_bytes(ch.encode_utf8(&mut buffer).as_bytes())?;

        Ok(())
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<M> embedded_hal_02::serial::Read<u8> for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_02::serial::Read::read(&mut self.rx)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<M> embedded_hal_02::serial::Read<u8> for UsbSerialJtagRx<'_, M>
where
    M: Mode,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<M> embedded_hal_02::serial::Write<u8> for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_02::serial::Write::write(&mut self.tx, word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_02::serial::Write::flush(&mut self.tx)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl<M> embedded_hal_02::serial::Write<u8> for UsbSerialJtagTx<'_, M>
where
    M: Mode,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}

#[cfg(feature = "embedded-hal")]
impl<M> embedded_hal_nb::serial::ErrorType for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl<M> embedded_hal_nb::serial::ErrorType for UsbSerialJtagTx<'_, M>
where
    M: Mode,
{
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl<M> embedded_hal_nb::serial::ErrorType for UsbSerialJtagRx<'_, M>
where
    M: Mode,
{
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl<M> embedded_hal_nb::serial::Read for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_nb::serial::Read::read(&mut self.rx)
    }
}

#[cfg(feature = "embedded-hal")]
impl<M> embedded_hal_nb::serial::Read for UsbSerialJtagRx<'_, M>
where
    M: Mode,
{
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal")]
impl<M> embedded_hal_nb::serial::Write for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::write(&mut self.tx, word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::flush(&mut self.tx)
    }
}

#[cfg(feature = "embedded-hal")]
impl<M> embedded_hal_nb::serial::Write for UsbSerialJtagTx<'_, M>
where
    M: Mode,
{
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}

#[cfg(feature = "embedded-io")]
impl<M> embedded_io::ErrorType for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl<M> embedded_io::ErrorType for UsbSerialJtagTx<'_, M>
where
    M: Mode,
{
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl<M> embedded_io::ErrorType for UsbSerialJtagRx<'_, M>
where
    M: Mode,
{
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl<M> embedded_io::Read for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx, buf)
    }
}

#[cfg(feature = "embedded-io")]
impl<M> embedded_io::Read for UsbSerialJtagRx<'_, M>
where
    M: Mode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            let count = self.drain_rx_fifo(buf);
            if count > 0 {
                return Ok(count);
            }
        }
    }
}

#[cfg(feature = "embedded-io")]
impl<M> embedded_io::Write for UsbSerialJtag<'_, M>
where
    M: Mode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io::Write::write(&mut self.tx, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}

#[cfg(feature = "embedded-io")]
impl<M> embedded_io::Write for UsbSerialJtagTx<'_, M>
where
    M: Mode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_bytes(buf)?;

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx()
    }
}

#[cfg(feature = "async")]
mod asynch {
    use core::{marker::PhantomData, task::Poll};

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::handler;

    use super::{Error, Instance, UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
    use crate::{peripheral::Peripheral, peripherals::USB_DEVICE, Async};

    // Static instance of the waker for each component of the peripheral:
    static WAKER_TX: AtomicWaker = AtomicWaker::new();
    static WAKER_RX: AtomicWaker = AtomicWaker::new();

    pub(crate) struct UsbSerialJtagWriteFuture<'d> {
        phantom: PhantomData<&'d mut USB_DEVICE>,
    }

    impl<'d> UsbSerialJtagWriteFuture<'d> {
        pub fn new() -> Self {
            // Set the interrupt enable bit for the USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT
            // interrupt
            USB_DEVICE::register_block()
                .int_ena()
                .modify(|_, w| w.serial_in_empty().set_bit());

            Self {
                phantom: PhantomData,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            USB_DEVICE::register_block()
                .int_ena()
                .read()
                .serial_in_empty()
                .bit_is_clear()
        }
    }

    impl<'d> core::future::Future for UsbSerialJtagWriteFuture<'d> {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            WAKER_TX.register(cx.waker());
            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    pub(crate) struct UsbSerialJtagReadFuture<'d> {
        phantom: PhantomData<&'d mut USB_DEVICE>,
    }

    impl<'d> UsbSerialJtagReadFuture<'d> {
        pub fn new() -> Self {
            // Set the interrupt enable bit for the USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT
            // interrupt
            USB_DEVICE::register_block()
                .int_ena()
                .modify(|_, w| w.serial_out_recv_pkt().set_bit());

            Self {
                phantom: PhantomData,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            USB_DEVICE::register_block()
                .int_ena()
                .read()
                .serial_out_recv_pkt()
                .bit_is_clear()
        }
    }

    impl<'d> core::future::Future for UsbSerialJtagReadFuture<'d> {
        type Output = ();

        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            WAKER_RX.register(cx.waker());
            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl<'d> UsbSerialJtag<'d, Async> {
        /// Create a new USB serial/JTAG instance in asynchronous mode
        pub fn new_async(usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
            let mut this = Self::new_inner(usb_device);
            this.inner_set_interrupt_handler(async_interrupt_handler);
            this
        }
    }

    impl UsbSerialJtagTx<'_, Async> {
        async fn write_bytes_async(&mut self, words: &[u8]) -> Result<(), Error> {
            let reg_block = USB_DEVICE::register_block();

            for chunk in words.chunks(64) {
                for byte in chunk {
                    reg_block
                        .ep1()
                        .write(|w| unsafe { w.rdwr_byte().bits(*byte) });
                }
                reg_block.ep1_conf().modify(|_, w| w.wr_done().set_bit());

                UsbSerialJtagWriteFuture::new().await;
            }

            Ok(())
        }

        async fn flush_tx_async(&mut self) -> Result<(), Error> {
            if USB_DEVICE::register_block()
                .jfifo_st()
                .read()
                .out_fifo_empty()
                .bit_is_clear()
            {
                UsbSerialJtagWriteFuture::new().await;
            }

            Ok(())
        }
    }

    impl UsbSerialJtagRx<'_, Async> {
        async fn read_bytes_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            if buf.is_empty() {
                return Ok(0);
            }

            loop {
                let read_bytes = self.drain_rx_fifo(buf);
                if read_bytes > 0 {
                    return Ok(read_bytes);
                }
                UsbSerialJtagReadFuture::new().await;
            }
        }
    }

    impl embedded_io_async::Write for UsbSerialJtag<'_, Async> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            embedded_io_async::Write::write(&mut self.tx, buf).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            embedded_io_async::Write::flush(&mut self.tx).await
        }
    }

    impl embedded_io_async::Write for UsbSerialJtagTx<'_, Async> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.write_bytes_async(buf).await?;

            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flush_tx_async().await
        }
    }

    impl embedded_io_async::Read for UsbSerialJtag<'_, Async> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            embedded_io_async::Read::read(&mut self.rx, buf).await
        }
    }

    impl embedded_io_async::Read for UsbSerialJtagRx<'_, Async> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_bytes_async(buf).await
        }
    }

    #[handler]
    fn async_interrupt_handler() {
        let usb = USB_DEVICE::register_block();
        let interrupts = usb.int_st().read();

        let tx = interrupts.serial_in_empty().bit_is_set();
        let rx = interrupts.serial_out_recv_pkt().bit_is_set();

        if tx {
            usb.int_ena().modify(|_, w| w.serial_in_empty().clear_bit());
        }
        if rx {
            usb.int_ena()
                .modify(|_, w| w.serial_out_recv_pkt().clear_bit());
        }

        usb.int_clr().write(|w| {
            w.serial_in_empty()
                .clear_bit_by_one()
                .serial_out_recv_pkt()
                .clear_bit_by_one()
        });

        if rx {
            WAKER_RX.wake();
        }
        if tx {
            WAKER_TX.wake();
        }
    }
}
