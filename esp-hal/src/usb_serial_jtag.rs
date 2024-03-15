//! # USB Serial JTAG peripheral driver
//!
//! ## Overview
//! The USB Serial JTAG peripheral driver provides an interface to communicate
//! with the USB Serial/JTAG peripheral on ESP chips. It enables serial
//! communication and JTAG debugging capabilities, allowing developers to
//! interact with the ESP chip for programming, debugging, and data transfer
//! purposes, can be also used to program the SoC's flash, read program output,
//! as well as attach a debugger to a running program.
//!
//! ## Example
//! ```no_run
//! let peripherals = Peripherals::take();
//! ...
//! // Initialize USB Serial/JTAG peripheral
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);

use core::{convert::Infallible, marker::PhantomData};

use crate::{
    peripheral::Peripheral,
    peripherals::{usb_device::RegisterBlock, USB_DEVICE},
    system::PeripheralClockControl,
};

/// Custom USB serial error type
type Error = Infallible;

/// USB Serial JTAG driver
pub struct UsbSerialJtag<'d> {
    tx: UsbSerialJtagTx<'d>,
    rx: UsbSerialJtagRx<'d>,
}

/// USB Serial JTAG TX driver
pub struct UsbSerialJtagTx<'d> {
    phantom: PhantomData<&'d mut USB_DEVICE>,
}

/// USB Serial JTAG RX driver
pub struct UsbSerialJtagRx<'d> {
    phantom: PhantomData<&'d mut USB_DEVICE>,
}

impl<'d> UsbSerialJtagTx<'d> {
    // If we want to implement a standalone UsbSerialJtagTx, uncomment below and
    // take care of the configuration
    // pub fn new(_usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
    //     Self::new_inner()
    // }

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
            reg_block.ep1_conf().write(|w| w.wr_done().set_bit());

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
        reg_block.ep1_conf().write(|w| w.wr_done().set_bit());

        while reg_block.ep1_conf().read().bits() & 0b011 == 0b000 {
            // wait
        }

        Ok(())
    }

    /// Flush the output FIFO but don't block if it isn't ready immediately
    pub fn flush_tx_nb(&mut self) -> nb::Result<(), Error> {
        let reg_block = USB_DEVICE::register_block();
        reg_block.ep1_conf().write(|w| w.wr_done().set_bit());

        if reg_block.ep1_conf().read().bits() & 0b011 == 0b000 {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }
}

impl<'d> UsbSerialJtagRx<'d> {
    // If we want to implement a standalone UsbSerialJtagRx, uncomment below and
    // take care of the configuration
    // pub fn new(_usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
    //     Self::new_inner()
    // }

    fn new_inner() -> Self {
        Self {
            phantom: PhantomData,
        }
    }

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
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().set_bit());
    }

    /// Stop listening for RX-PACKET-RECV interrupts
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        USB_DEVICE::register_block()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().clear_bit());
    }

    /// Checks if RX-PACKET-RECV interrupt is set
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        USB_DEVICE::register_block()
            .int_st()
            .read()
            .serial_out_recv_pkt_int_st()
            .bit_is_set()
    }

    /// Reset RX-PACKET-RECV interrupt
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        USB_DEVICE::register_block()
            .int_clr()
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }
}

impl<'d> UsbSerialJtag<'d> {
    /// Create a new USB serial/JTAG instance with defaults
    pub fn new(_usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
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

    /// Split the USB Serial JTAG peripheral into a transmitter and receiver,
    /// which is particuarly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UsbSerialJtagTx<'d>, UsbSerialJtagRx<'d>) {
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

/// USB Serial JTAG peripheral instance
pub trait Instance {
    fn register_block() -> &'static RegisterBlock;

    fn disable_tx_interrupts() {
        Self::register_block()
            .int_ena()
            .write(|w| w.serial_in_empty_int_ena().clear_bit());

        Self::register_block()
            .int_clr()
            .write(|w| w.serial_in_empty_int_clr().set_bit())
    }

    fn disable_rx_interrupts() {
        Self::register_block()
            .int_ena()
            .write(|w| w.serial_out_recv_pkt_int_ena().clear_bit());

        Self::register_block()
            .int_clr()
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }
}

impl Instance for USB_DEVICE {
    #[inline(always)]
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*USB_DEVICE::ptr() }
    }
}

impl core::fmt::Write for UsbSerialJtag<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        core::fmt::Write::write_str(&mut self.tx, s)
    }
}

impl core::fmt::Write for UsbSerialJtagTx<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

#[cfg(feature = "ufmt")]
impl ufmt_write::uWrite for UsbSerialJtag<'_> {
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
impl ufmt_write::uWrite for UsbSerialJtagTx<'_> {
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
impl embedded_hal_02::serial::Read<u8> for UsbSerialJtag<'_> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_02::serial::Read::read(&mut self.rx)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::serial::Read<u8> for UsbSerialJtagRx<'_> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::serial::Write<u8> for UsbSerialJtag<'_> {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_02::serial::Write::write(&mut self.tx, word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_02::serial::Write::flush(&mut self.tx)
    }
}

#[cfg(feature = "embedded-hal-02")]
impl embedded_hal_02::serial::Write<u8> for UsbSerialJtagTx<'_> {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::ErrorType for UsbSerialJtag<'_> {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::ErrorType for UsbSerialJtagTx<'_> {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::ErrorType for UsbSerialJtagRx<'_> {
    type Error = Error;
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Read for UsbSerialJtag<'_> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        embedded_hal_nb::serial::Read::read(&mut self.rx)
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Read for UsbSerialJtagRx<'_> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Write for UsbSerialJtag<'_> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::write(&mut self.tx, word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::flush(&mut self.tx)
    }
}

#[cfg(feature = "embedded-hal")]
impl embedded_hal_nb::serial::Write for UsbSerialJtagTx<'_> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::ErrorType for UsbSerialJtag<'_> {
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl embedded_io::ErrorType for UsbSerialJtagTx<'_> {
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl embedded_io::ErrorType for UsbSerialJtagRx<'_> {
    type Error = Error;
}

#[cfg(feature = "embedded-io")]
impl embedded_io::Read for UsbSerialJtag<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx, buf)
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::Read for UsbSerialJtagRx<'_> {
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
impl embedded_io::Write for UsbSerialJtag<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io::Write::write(&mut self.tx, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}

#[cfg(feature = "embedded-io")]
impl embedded_io::Write for UsbSerialJtagTx<'_> {
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
    use procmacros::interrupt;

    use super::{Error, Instance, UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
    use crate::peripherals::USB_DEVICE;

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
                .modify(|_, w| w.serial_in_empty_int_ena().set_bit());

            Self {
                phantom: PhantomData,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            USB_DEVICE::register_block()
                .int_ena()
                .read()
                .serial_in_empty_int_ena()
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
                .modify(|_, w| w.serial_out_recv_pkt_int_ena().set_bit());

            Self {
                phantom: PhantomData,
            }
        }

        fn event_bit_is_clear(&self) -> bool {
            USB_DEVICE::register_block()
                .int_ena()
                .read()
                .serial_out_recv_pkt_int_ena()
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

    impl UsbSerialJtagTx<'_> {
        async fn write_bytes_async(&mut self, words: &[u8]) -> Result<(), Error> {
            let reg_block = USB_DEVICE::register_block();

            for chunk in words.chunks(64) {
                for byte in chunk {
                    reg_block
                        .ep1()
                        .write(|w| unsafe { w.rdwr_byte().bits(*byte) });
                }
                reg_block.ep1_conf().write(|w| w.wr_done().set_bit());

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

    impl UsbSerialJtagRx<'_> {
        async fn read_bytes_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            if buf.len() == 0 {
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

    impl embedded_io_async::Write for UsbSerialJtag<'_> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            embedded_io_async::Write::write(&mut self.tx, buf).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            embedded_io_async::Write::flush(&mut self.tx).await
        }
    }

    impl embedded_io_async::Write for UsbSerialJtagTx<'_> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.write_bytes_async(buf).await?;

            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flush_tx_async().await
        }
    }

    impl embedded_io_async::Read for UsbSerialJtag<'_> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            embedded_io_async::Read::read(&mut self.rx, buf).await
        }
    }

    impl embedded_io_async::Read for UsbSerialJtagRx<'_> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_bytes_async(buf).await
        }
    }

    #[interrupt]
    fn USB_DEVICE() {
        let usb = USB_DEVICE::register_block();
        let interrupts = usb.int_st().read();

        let tx = interrupts.serial_in_empty_int_st().bit_is_set();
        let rx = interrupts.serial_out_recv_pkt_int_st().bit_is_set();

        if tx {
            usb.int_ena()
                .write(|w| w.serial_in_empty_int_ena().clear_bit());
        }
        if rx {
            usb.int_ena()
                .write(|w| w.serial_out_recv_pkt_int_ena().clear_bit());
        }

        usb.int_clr().write(|w| {
            w.serial_in_empty_int_clr()
                .set_bit()
                .serial_out_recv_pkt_int_clr()
                .set_bit()
        });

        if rx {
            WAKER_RX.wake();
        }
        if tx {
            WAKER_TX.wake();
        }
    }
}
