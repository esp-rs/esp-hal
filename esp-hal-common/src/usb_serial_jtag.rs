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

use core::convert::Infallible;

use crate::{
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{usb_device::RegisterBlock, USB_DEVICE},
    system::PeripheralClockControl,
};

/// Custom USB serial error type
type Error = Infallible;

/// USB Serial JTAG driver
pub struct UsbSerialJtag<'d> {
    usb_serial: PeripheralRef<'d, USB_DEVICE>,
}

impl<'d> UsbSerialJtag<'d> {
    /// Create a new USB serial/JTAG instance with defaults
    pub fn new(usb_serial: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        crate::into_ref!(usb_serial);

        PeripheralClockControl::enable(crate::system::Peripheral::UsbDevice);

        let mut dev = Self { usb_serial };
        dev.usb_serial.disable_rx_interrupts();
        dev.usb_serial.disable_tx_interrupts();

        dev
    }

    /// Write data to the serial output in chunks of up to 64 bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        let reg_block = self.usb_serial.register_block();

        for chunk in data.chunks(64) {
            for byte in chunk {
                reg_block
                    .ep1
                    .write(|w| unsafe { w.rdwr_byte().bits(*byte) });
            }
            reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

            while reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
                // wait
            }
        }

        Ok(())
    }

    /// Write data to the serial output in a non-blocking manner
    /// Requires manual flushing (automatically flushed every 64 bytes)
    pub fn write_byte_nb(&mut self, word: u8) -> nb::Result<(), Error> {
        let reg_block = self.usb_serial.register_block();

        if reg_block
            .ep1_conf
            .read()
            .serial_in_ep_data_free()
            .bit_is_set()
        {
            // the FIFO is not full
            unsafe {
                reg_block.ep1.write(|w| w.rdwr_byte().bits(word.into()));
            }

            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Flush the output FIFO and block until it has been sent
    pub fn flush_tx(&mut self) -> Result<(), Error> {
        let reg_block = self.usb_serial.register_block();
        reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

        while reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
            // wait
        }

        Ok(())
    }

    /// Flush the output FIFO but don't block if it isn't ready immediately
    pub fn flush_tx_nb(&mut self) -> nb::Result<(), Error> {
        let reg_block = self.usb_serial.register_block();
        reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

        if reg_block.ep1_conf.read().bits() & 0b011 == 0b000 {
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        let reg_block = self.usb_serial.register_block();

        // Check if there are any bytes to read
        if reg_block
            .ep1_conf
            .read()
            .serial_out_ep_data_avail()
            .bit_is_set()
        {
            let value = reg_block.ep1.read().rdwr_byte().bits();

            Ok(value)
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// Listen for RX-PACKET-RECV interrupts
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        let reg_block = self.usb_serial.register_block();
        reg_block
            .int_ena
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().set_bit());
    }

    /// Stop listening for RX-PACKET-RECV interrupts
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        let reg_block = self.usb_serial.register_block();
        reg_block
            .int_ena
            .modify(|_, w| w.serial_out_recv_pkt_int_ena().clear_bit());
    }

    /// Checks if RX-PACKET-RECV interrupt is set
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        let reg_block = unsafe { &*USB_DEVICE::PTR };
        reg_block
            .int_st
            .read()
            .serial_out_recv_pkt_int_st()
            .bit_is_set()
    }

    /// Reset RX-PACKET-RECV interrupt
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        let reg_block = unsafe { &*USB_DEVICE::PTR };

        reg_block
            .int_clr
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }

    #[cfg(feature = "async")]
    pub(crate) fn inner(&self) -> &USB_DEVICE {
        &self.usb_serial
    }
}

/// USB Serial JTAG peripheral instance
pub trait Instance {
    fn register_block(&self) -> &RegisterBlock;

    fn disable_tx_interrupts(&mut self) {
        self.register_block()
            .int_ena
            .write(|w| w.serial_in_empty_int_ena().clear_bit());

        self.register_block()
            .int_clr
            .write(|w| w.serial_in_empty_int_clr().set_bit())
    }

    fn disable_rx_interrupts(&mut self) {
        self.register_block()
            .int_ena
            .write(|w| w.serial_out_recv_pkt_int_ena().clear_bit());

        self.register_block()
            .int_clr
            .write(|w| w.serial_out_recv_pkt_int_clr().set_bit())
    }

    fn get_rx_fifo_count(&self) -> u16 {
        let ep0_state = self.register_block().in_ep0_st.read();
        let wr_addr = ep0_state.in_ep0_wr_addr().bits();
        let rd_addr = ep0_state.in_ep0_rd_addr().bits();

        (wr_addr - rd_addr).into()
    }

    fn get_tx_fifo_count(&self) -> u16 {
        let ep1_state = self.register_block().in_ep1_st.read();
        let wr_addr = ep1_state.in_ep1_wr_addr().bits();
        let rd_addr = ep1_state.in_ep1_rd_addr().bits();

        (wr_addr - rd_addr).into()
    }

    fn txfifo_empty(&self) -> bool {
        self.register_block()
            .jfifo_st
            .read()
            .out_fifo_empty()
            .bit_is_clear()
    }
}

impl Instance for USB_DEVICE {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        self
    }
}

impl core::fmt::Write for UsbSerialJtag<'_> {
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

impl embedded_hal::serial::Read<u8> for UsbSerialJtag<'_> {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl embedded_hal::serial::Write<u8> for UsbSerialJtag<'_> {
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_nb::serial::ErrorType for UsbSerialJtag<'_> {
    type Error = Error;
}

#[cfg(feature = "eh1")]
impl embedded_hal_nb::serial::Read for UsbSerialJtag<'_> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

#[cfg(feature = "eh1")]
impl embedded_hal_nb::serial::Write for UsbSerialJtag<'_> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.flush_tx_nb()
    }
}

impl embedded_io::ErrorType for UsbSerialJtag<'_> {
    type Error = Error;
}

impl embedded_io::Read for UsbSerialJtag<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut count = 0;
        loop {
            if count >= buf.len() {
                break;
            }

            match self.read_byte() {
                Ok(byte) => {
                    buf[count] = byte;
                    count += 1;
                }
                Err(nb::Error::WouldBlock) => {
                    // Block until we have read at least one byte
                    if count > 0 {
                        break;
                    }
                }
                Err(nb::Error::Other(e)) => return Err(e),
            }
        }

        Ok(count)
    }
}

impl embedded_io::Write for UsbSerialJtag<'_> {
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
    use core::task::Poll;

    use embassy_sync::waitqueue::AtomicWaker;
    use procmacros::interrupt;

    use super::{Error, Instance};
    use crate::UsbSerialJtag;

    // Single static instance of the waker
    static WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) struct UsbSerialJtagWriteFuture<'d> {
        instance: &'d crate::peripherals::USB_DEVICE,
    }

    impl<'d> UsbSerialJtagWriteFuture<'d> {
        pub fn new(instance: &'d crate::peripherals::USB_DEVICE) -> Self {
            // Set the interrupt enable bit for the USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT
            // interrupt
            instance
                .register_block()
                .int_ena
                .modify(|_, w| w.serial_in_empty_int_ena().set_bit());

            Self { instance }
        }

        fn event_bit_is_clear(&self) -> bool {
            self.instance
                .register_block()
                .int_ena
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
            WAKER.register(cx.waker());
            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    pub(crate) struct UsbSerialJtagReadFuture<'d> {
        instance: &'d crate::peripherals::USB_DEVICE,
    }

    impl<'d> UsbSerialJtagReadFuture<'d> {
        pub fn new(instance: &'d crate::peripherals::USB_DEVICE) -> Self {
            // Set the interrupt enable bit for the USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT
            // interrupt
            instance
                .register_block()
                .int_ena
                .modify(|_, w| w.serial_out_recv_pkt_int_ena().set_bit());

            Self { instance }
        }

        fn event_bit_is_clear(&self) -> bool {
            self.instance
                .register_block()
                .int_ena
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
            WAKER.register(cx.waker());
            if self.event_bit_is_clear() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        }
    }

    impl UsbSerialJtag<'_> {
        async fn write_bytes_async(&mut self, words: &[u8]) -> Result<(), Error> {
            let reg_block = self.usb_serial.register_block();

            for chunk in words.chunks(64) {
                for byte in chunk {
                    reg_block
                        .ep1
                        .write(|w| unsafe { w.rdwr_byte().bits(*byte) });
                }
                reg_block.ep1_conf.write(|w| w.wr_done().set_bit());

                UsbSerialJtagWriteFuture::new(self.inner()).await;
            }

            Ok(())
        }

        async fn read_bytes_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
            fn read_nb<'d>(
                usb: &'d mut UsbSerialJtag,
                buf: &mut [u8],
            ) -> Result<Result<usize, Error>, UsbSerialJtagReadFuture<'d>> {
                let mut count = 0;
                loop {
                    if count >= buf.len() {
                        return Ok(Ok(count));
                    }

                    match usb.read_byte() {
                        Ok(byte) => {
                            buf[count] = byte;
                            count += 1;
                        }
                        Err(nb::Error::WouldBlock) => {
                            if count > 0 {
                                return Ok(Ok(count));
                            } else {
                                return Err(UsbSerialJtagReadFuture::new(usb.inner()));
                            }
                        }
                        Err(nb::Error::Other(e)) => return Ok(Err(e)),
                    }
                }
            }

            loop {
                match critical_section::with(|_cs| read_nb(self, buf)) {
                    Ok(result) => {
                        return result;
                    }
                    Err(fut) => {
                        fut.await;
                    }
                }
            }
        }

        async fn flush_tx_async(&mut self) -> Result<(), Error> {
            if self.inner().txfifo_empty() {
                UsbSerialJtagWriteFuture::new(self.inner()).await;
            }

            Ok(())
        }
    }

    impl embedded_io_async::Write for UsbSerialJtag<'_> {
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
            self.read_bytes_async(buf).await
        }
    }

    #[interrupt]
    fn USB_DEVICE() {
        let usb = unsafe { &*crate::peripherals::USB_DEVICE::ptr() };
        let in_empty = usb.int_st.read().serial_in_empty_int_st().bit_is_set();
        let out_recv = usb.int_st.read().serial_out_recv_pkt_int_st().bit_is_set();

        if in_empty {
            usb.int_ena
                .write(|w| w.serial_in_empty_int_ena().clear_bit());
        }
        if out_recv {
            usb.int_ena
                .write(|w| w.serial_out_recv_pkt_int_ena().clear_bit());
        }
        usb.int_clr.write(|w| {
            w.serial_in_empty_int_clr()
                .set_bit()
                .serial_out_recv_pkt_int_clr()
                .set_bit()
        });
        WAKER.wake();
    }
}
