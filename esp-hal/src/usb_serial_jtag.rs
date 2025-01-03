//! USB Serial/JTAG Controller (USB_SERIAL_JTAG)
//!
//! ## Overview
//!
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
//!
//! The USB Serial/JTAG driver implements a number of third-party traits, with
//! the intention of making the HAL inter-compatible with various device drivers
//! from the community. This includes, but is not limited to, the [embedded-hal]
//! and [embedded-io] blocking traits, and the [embedded-hal-async]
//! and [embedded-io-async] asynchronous traits.
//!
//! In addition to the interfaces provided by these traits, native APIs are also
//! available. See the examples below for more information on how to interact
//! with this driver.
//!
//! ## Examples
//!
//! ### Sending and Receiving Data
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::usb_serial_jtag::UsbSerialJtag;
//!
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//!
//! // Write bytes out over the USB Serial/JTAG:
//! usb_serial.write_bytes(b"Hello, world!").expect("write error!");
//! # }
//! ```
//! 
//! ### Splitting the USB Serial/JTAG into TX and RX Components
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! use esp_hal::usb_serial_jtag::UsbSerialJtag;
//!
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//! // The USB Serial/JTAG can be split into separate Transmit and Receive
//! // components:
//! let (mut rx, mut tx) = usb_serial.split();
//!
//! // Each component can be used individually to interact with the USB
//! // Serial/JTAG:
//! tx.write_bytes(&[42u8]).expect("write error!");
//! let byte = rx.read_byte().expect("read error!");
//! # }
//! ```
//! 
//! ### How to output text using USB Serial/JTAG.
//! ```rust, no_run
#![doc = crate::before_snippet!()]
//! # use esp_hal::{delay::Delay, usb_serial_jtag::UsbSerialJtag, Blocking};
//!
//! let delay = Delay::new();
//!
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//! usb_serial.set_interrupt_handler(usb_device);
//! usb_serial.listen_rx_packet_recv_interrupt();
//!
//! critical_section::with(|cs|
//! USB_SERIAL.borrow_ref_mut(cs).replace(usb_serial));
//!
//! loop {
//!     critical_section::with(|cs| {
//!         writeln!(
//!             USB_SERIAL.borrow_ref_mut(cs).as_mut().unwrap(),
//!             "Hello world!"
//!         )
//!         .ok();
//!     });
//!
//!     delay.delay(1.secs());
//! }
//! # }
//!
//! # use critical_section::Mutex;
//! # use core::{cell::RefCell, fmt::Write};
//! # use esp_hal::usb_serial_jtag::UsbSerialJtag;
//! static USB_SERIAL:
//!     Mutex<RefCell<Option<UsbSerialJtag<'static, esp_hal::Blocking>>>> =
//!         Mutex::new(RefCell::new(None));
//!
//! #[handler]
//! fn usb_device() {
//!     critical_section::with(|cs| {
//!         let mut usb_serial = USB_SERIAL.borrow_ref_mut(cs);
//!         let usb_serial = usb_serial.as_mut().unwrap();
//!
//!         writeln!(usb_serial, "USB serial interrupt").unwrap();
//!
//!         while let Some(c) = usb_serial.read_byte() {
//!             writeln!(usb_serial, "Read byte: {:02x}", c).unwrap();
//!         }
//!
//!         usb_serial.reset_rx_packet_recv_interrupt();
//!     });
//! }
//! ```
//! 
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-io]: https://docs.rs/embedded-io/latest/embedded_io/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
//! [embedded-io-async]: https://docs.rs/embedded-io-async/latest/embedded_io_async/

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
use core::task::Poll;
use core::{convert::Infallible, marker::PhantomData};

use procmacros::handler;

use crate::{
    asynch::AtomicWaker,
    interrupt::InterruptConfigurable,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::{usb_device::RegisterBlock, Interrupt, USB_DEVICE},
    system::PeripheralClockControl,
    Async,
    Blocking,
    Cpu,
    DriverMode,
};

/// Custom USB serial error type
type Error = Infallible;

/// USB Serial/JTAG (Full-duplex)
pub struct UsbSerialJtag<'d, Dm> {
    rx: UsbSerialJtagRx<'d, Dm>,
    tx: UsbSerialJtagTx<'d, Dm>,
}

/// USB Serial/JTAG (Transmit)
pub struct UsbSerialJtagTx<'d, Dm> {
    peripheral: PeripheralRef<'d, USB_DEVICE>,
    phantom: PhantomData<Dm>,
}

/// USB Serial/JTAG (Receive)
pub struct UsbSerialJtagRx<'d, Dm> {
    peripheral: PeripheralRef<'d, USB_DEVICE>,
    phantom: PhantomData<Dm>,
}

impl<'d, Dm> UsbSerialJtagTx<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_inner(peripheral: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        crate::into_ref!(peripheral);
        Self {
            peripheral,
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
    pub fn write_byte_nb(&mut self, word: u8) -> Option<()> {
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

            Some(())
        } else {
            None
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
    pub fn flush_tx_nb(&mut self) -> Option<()> {
        let reg_block = USB_DEVICE::register_block();
        reg_block.ep1_conf().modify(|_, w| w.wr_done().set_bit());

        if reg_block.ep1_conf().read().bits() & 0b011 == 0b000 {
            None
        } else {
            Some(())
        }
    }
}

impl<'d, Dm> UsbSerialJtagRx<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_inner(peripheral: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        crate::into_ref!(peripheral);
        Self {
            peripheral,
            phantom: PhantomData,
        }
    }

    /// Read a byte from the UART in a non-blocking manner
    pub fn read_byte(&mut self) -> Option<u8> {
        let reg_block = USB_DEVICE::register_block();

        // Check if there are any bytes to read
        if reg_block
            .ep1_conf()
            .read()
            .serial_out_ep_data_avail()
            .bit_is_set()
        {
            let value = reg_block.ep1().read().rdwr_byte().bits();

            Some(value)
        } else {
            None
        }
    }

    /// Read all available bytes from the RX FIFO into the provided buffer and
    /// returns the number of read bytes. Never blocks. May stop early if the
    /// number of bytes in the FIFO is larger than `buf`.
    pub fn drain_rx_fifo(&mut self, buf: &mut [u8]) -> usize {
        let mut count = 0;
        while let Some(value) = self.read_byte() {
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
            .write(|w| w.serial_out_recv_pkt().clear_bit_by_one());
    }
}

impl<'d> UsbSerialJtag<'d, Blocking> {
    /// Create a new USB serial/JTAG instance with defaults
    pub fn new(usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        Self::new_inner(usb_device)
    }

    /// Reconfigure the USB Serial JTAG peripheral to operate in asynchronous
    /// mode.
    pub fn into_async(mut self) -> UsbSerialJtag<'d, Async> {
        self.set_interrupt_handler(async_interrupt_handler);

        UsbSerialJtag {
            rx: UsbSerialJtagRx {
                peripheral: self.rx.peripheral,
                phantom: PhantomData,
            },
            tx: UsbSerialJtagTx {
                peripheral: self.tx.peripheral,
                phantom: PhantomData,
            },
        }
    }
}

impl crate::private::Sealed for UsbSerialJtag<'_, Blocking> {}

impl InterruptConfigurable for UsbSerialJtag<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        for core in crate::Cpu::other() {
            crate::interrupt::disable(core, Interrupt::USB_DEVICE);
        }
        unsafe { crate::interrupt::bind_interrupt(Interrupt::USB_DEVICE, handler.handler()) };
        unwrap!(crate::interrupt::enable(
            Interrupt::USB_DEVICE,
            handler.priority()
        ));
    }
}

impl<'d, Dm> UsbSerialJtag<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_inner(usb_device: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
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
            if Efuse::read_bit(USB_EXCHG_PINS) {
                USB_DEVICE::register_block().conf0().modify(|_, w| {
                    w.pad_pull_override().set_bit();
                    w.dm_pullup().clear_bit();
                    w.dp_pullup().set_bit()
                });
            }
        }

        crate::into_ref!(usb_device);

        Self {
            rx: UsbSerialJtagRx::new_inner(unsafe { usb_device.clone_unchecked() }),
            tx: UsbSerialJtagTx::new_inner(usb_device),
        }
    }
    /// Split the USB Serial JTAG peripheral into a transmitter and receiver,
    /// which is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UsbSerialJtagRx<'d, Dm>, UsbSerialJtagTx<'d, Dm>) {
        (self.rx, self.tx)
    }

    /// Write data to the serial output in chunks of up to 64 bytes
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<(), Error> {
        self.tx.write_bytes(data)
    }

    /// Write data to the serial output in a non-blocking manner
    /// Requires manual flushing (automatically flushed every 64 bytes)
    pub fn write_byte_nb(&mut self, word: u8) -> Option<()> {
        self.tx.write_byte_nb(word)
    }

    /// Flush the output FIFO and block until it has been sent
    pub fn flush_tx(&mut self) -> Result<(), Error> {
        self.tx.flush_tx()
    }

    /// Flush the output FIFO but don't block if it isn't ready immediately
    pub fn flush_tx_nb(&mut self) -> Option<()> {
        self.tx.flush_tx_nb()
    }

    /// Read a single byte but don't block if it isn't ready immediately
    pub fn read_byte(&mut self) -> Option<u8> {
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
#[doc(hidden)]
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
            .write(|w| w.serial_in_empty().clear_bit_by_one());
    }

    /// Disable all receive interrupts for the peripheral
    fn disable_rx_interrupts() {
        Self::register_block()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().clear_bit());

        Self::register_block()
            .int_clr()
            .write(|w| w.serial_out_recv_pkt().clear_bit_by_one());
    }
}

impl Instance for USB_DEVICE {
    #[inline(always)]
    fn register_block() -> &'static RegisterBlock {
        unsafe { &*USB_DEVICE::ptr() }
    }
}

impl<Dm> core::fmt::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        core::fmt::Write::write_str(&mut self.tx, s)
    }
}

impl<Dm> core::fmt::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_bytes(s.as_bytes())
            .map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

impl<Dm> ufmt_write::uWrite for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
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

impl<Dm> ufmt_write::uWrite for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
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

impl<Dm> embedded_hal_nb::serial::ErrorType for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

impl<Dm> embedded_hal_nb::serial::ErrorType for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

impl<Dm> embedded_hal_nb::serial::ErrorType for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

impl<Dm> embedded_hal_nb::serial::Read for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self) -> embedded_hal_nb::nb::Result<u8, Self::Error> {
        embedded_hal_nb::serial::Read::read(&mut self.rx)
    }
}

impl<Dm> embedded_hal_nb::serial::Read for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self) -> embedded_hal_nb::nb::Result<u8, Self::Error> {
        self.read_byte()
            .ok_or(embedded_hal_nb::nb::Error::WouldBlock)
    }
}

impl<Dm> embedded_hal_nb::serial::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, word: u8) -> embedded_hal_nb::nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::write(&mut self.tx, word)
    }

    fn flush(&mut self) -> embedded_hal_nb::nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::flush(&mut self.tx)
    }
}

impl<Dm> embedded_hal_nb::serial::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, word: u8) -> embedded_hal_nb::nb::Result<(), Self::Error> {
        self.write_byte_nb(word)
            .ok_or(embedded_hal_nb::nb::Error::WouldBlock)
    }

    fn flush(&mut self) -> embedded_hal_nb::nb::Result<(), Self::Error> {
        self.flush_tx_nb()
            .ok_or(embedded_hal_nb::nb::Error::WouldBlock)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm> embedded_io::ErrorType for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm> embedded_io::ErrorType for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm> embedded_io::ErrorType for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm> embedded_io::Read for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx, buf)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm> embedded_io::Read for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
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

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm> embedded_io::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io::Write::write(&mut self.tx, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx)
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<Dm> embedded_io::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_bytes(buf)?;

        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx()
    }
}

// Static instance of the waker for each component of the peripheral:
static WAKER_TX: AtomicWaker = AtomicWaker::new();
static WAKER_RX: AtomicWaker = AtomicWaker::new();

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct UsbSerialJtagWriteFuture<'d> {
    _peripheral: PeripheralRef<'d, USB_DEVICE>,
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<'d> UsbSerialJtagWriteFuture<'d> {
    fn new(_peripheral: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        crate::into_ref!(_peripheral);
        // Set the interrupt enable bit for the USB_SERIAL_JTAG_SERIAL_IN_EMPTY_INT
        // interrupt
        USB_DEVICE::register_block()
            .int_ena()
            .modify(|_, w| w.serial_in_empty().set_bit());

        Self { _peripheral }
    }

    fn event_bit_is_clear(&self) -> bool {
        USB_DEVICE::register_block()
            .int_ena()
            .read()
            .serial_in_empty()
            .bit_is_clear()
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl core::future::Future for UsbSerialJtagWriteFuture<'_> {
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

#[must_use = "futures do nothing unless you `.await` or poll them"]
#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
struct UsbSerialJtagReadFuture<'d> {
    _peripheral: PeripheralRef<'d, USB_DEVICE>,
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl<'d> UsbSerialJtagReadFuture<'d> {
    fn new(_peripheral: impl Peripheral<P = USB_DEVICE> + 'd) -> Self {
        crate::into_ref!(_peripheral);
        // Set the interrupt enable bit for the USB_SERIAL_JTAG_SERIAL_OUT_RECV_PKT
        // interrupt
        USB_DEVICE::register_block()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().set_bit());

        Self { _peripheral }
    }

    fn event_bit_is_clear(&self) -> bool {
        USB_DEVICE::register_block()
            .int_ena()
            .read()
            .serial_out_recv_pkt()
            .bit_is_clear()
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl core::future::Future for UsbSerialJtagReadFuture<'_> {
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
    /// Reconfigure the USB Serial JTAG peripheral to operate in blocking
    /// mode.
    pub fn into_blocking(self) -> UsbSerialJtag<'d, Blocking> {
        crate::interrupt::disable(Cpu::current(), Interrupt::USB_DEVICE);
        UsbSerialJtag {
            rx: UsbSerialJtagRx {
                peripheral: self.rx.peripheral,
                phantom: PhantomData,
            },
            tx: UsbSerialJtagTx {
                peripheral: self.tx.peripheral,
                phantom: PhantomData,
            },
        }
    }
}

impl UsbSerialJtagTx<'_, Async> {
    #[cfg(any(doc, feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    async fn write_bytes_async(&mut self, words: &[u8]) -> Result<(), Error> {
        let reg_block = USB_DEVICE::register_block();

        for chunk in words.chunks(64) {
            for byte in chunk {
                reg_block
                    .ep1()
                    .write(|w| unsafe { w.rdwr_byte().bits(*byte) });
            }
            reg_block.ep1_conf().modify(|_, w| w.wr_done().set_bit());

            UsbSerialJtagWriteFuture::new(self.peripheral.reborrow()).await;
        }

        Ok(())
    }

    #[cfg(any(doc, feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    async fn flush_tx_async(&mut self) -> Result<(), Error> {
        if USB_DEVICE::register_block()
            .jfifo_st()
            .read()
            .out_fifo_empty()
            .bit_is_clear()
        {
            UsbSerialJtagWriteFuture::new(self.peripheral.reborrow()).await;
        }

        Ok(())
    }
}

impl UsbSerialJtagRx<'_, Async> {
    #[cfg(any(doc, feature = "unstable"))]
    #[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
    async fn read_bytes_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }

        loop {
            let read_bytes = self.drain_rx_fifo(buf);
            if read_bytes > 0 {
                return Ok(read_bytes);
            }
            UsbSerialJtagReadFuture::new(self.peripheral.reborrow()).await;
        }
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl embedded_io_async::Write for UsbSerialJtag<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_async::Write::write(&mut self.tx, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_async::Write::flush(&mut self.tx).await
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl embedded_io_async::Write for UsbSerialJtagTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_bytes_async(buf).await?;

        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx_async().await
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
impl embedded_io_async::Read for UsbSerialJtag<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_async::Read::read(&mut self.rx, buf).await
    }
}

#[cfg(any(doc, feature = "unstable"))]
#[cfg_attr(docsrs, doc(cfg(feature = "unstable")))]
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
