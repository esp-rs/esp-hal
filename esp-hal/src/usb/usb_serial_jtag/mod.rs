#![cfg_attr(docsrs, procmacros::doc_replace)]
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
//! - Hardwired for CDC-ACM (Communication Device Class - Abstract Control Model) and JTAG adapter
//!   functionality
//! - Integrates CDC-ACM adherent serial port emulation (plug-and-play on most modern OSes);
//!   supports host controllable chip reset and entry into download mode
//! - Allows fast communication with CPU debugging core using a compact representation of JTAG
//!   instructions
//! - Two OUT Endpoints and three IN Endpoints in addition to Control Endpoint 0; Up to 64-byte data
//!   payload size
//! - Internal PHY means that very few or no external components needed to connect to a host
//!   computer
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
//! ### Packets and flushing
//!
//! The driver sends data to the host in packets of up to 64 bytes. When the output FIFO holds 64
//! bytes, the hardware sends them as one packet. Fewer bytes stay in the FIFO until you call
//! [`UsbSerialJtagTx::flush`] or [`UsbSerialJtagTx::flush_async`]. The write functions, and the
//! [`core::fmt::Write`] and [`ufmt_write::uWrite`] implementations, do not flush.
//!
//! The host can hold back a 64-byte packet until the transfer ends. A flush ends the transfer,
//! so always flush when the host must see the data.
//!
//! The driver assumes that it is the only writer to the output FIFO. Other writers, for example
//! ROM functions or a logger on the same peripheral, can make the flush functions skip a packet.
//!
//! ## Examples
//!
//! ### Sending and Receiving Data
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
//!
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//!
//! // Write bytes out over the USB Serial/JTAG:
//! let mut data: &[u8] = b"Hello, world!";
//! while !data.is_empty() {
//!     let written = usb_serial.write(data)?;
//!     data = &data[written..];
//! }
//! usb_serial.flush()?;
//! # {after_snippet}
//! ```
//!
//! ### Splitting the USB Serial/JTAG into TX and RX Components
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
//!
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//! // The USB Serial/JTAG can be split into separate Transmit and Receive
//! // components:
//! let (mut rx, mut tx) = usb_serial.split();
//!
//! // Each component can be used individually to interact with the USB
//! // Serial/JTAG:
//! tx.write(&[42u8])?;
//! tx.flush()?;
//!
//! let mut buffer = [0u8; 64];
//! let count = rx.read(&mut buffer)?;
//! # {after_snippet}
//! ```
//!
//! ### How to output text using USB Serial/JTAG.
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::{delay::Delay, usb::usb_serial_jtag::UsbSerialJtag, Blocking};
//!
//! let delay = Delay::new();
//!
//! let mut usb_serial = UsbSerialJtag::new(peripherals.USB_DEVICE);
//! usb_serial.set_interrupt_handler(usb_device);
//! usb_serial.listen_rx_packet_recv_interrupt();
//!
//! critical_section::with(|cs| USB_SERIAL.borrow_ref_mut(cs).replace(usb_serial));
//!
//! loop {
//!     println!("Send keystrokes to see the interrupt trigger");
//!     delay.delay(Duration::from_secs(1));
//! }
//! # }
//!
//! # use critical_section::Mutex;
//! # use core::{cell::RefCell, fmt::Write};
//! # use esp_hal::usb::usb_serial_jtag::UsbSerialJtag;
//! static USB_SERIAL: Mutex<RefCell<Option<UsbSerialJtag<'static, esp_hal::Blocking>>>> =
//!     Mutex::new(RefCell::new(None));
//!
//! #[esp_hal::handler]
//! fn usb_device() {
//!     critical_section::with(|cs| {
//!         let mut usb_serial = USB_SERIAL.borrow_ref_mut(cs);
//!         if let Some(usb_serial) = usb_serial.as_mut() {
//!             println!("USB serial interrupt");
//!
//!             let mut buffer = [0u8; 64];
//!             let Ok(count) = usb_serial.read_buffered(&mut buffer);
//!             println!("Read bytes: {:02x?}", &buffer[..count]);
//!
//!             usb_serial.reset_rx_packet_recv_interrupt();
//!         }
//!     });
//! }
//! ```
//!
//! [embedded-hal]: https://docs.rs/embedded-hal/latest/embedded_hal/
//! [embedded-io]: https://docs.rs/embedded-io/latest/embedded_io/
//! [embedded-hal-async]: https://docs.rs/embedded-hal-async/latest/embedded_hal_async/
//! [embedded-io-async]: https://docs.rs/embedded-io-async/latest/embedded_io_async/

#[instability::unstable]
use core::task::Poll;
use core::{convert::Infallible, marker::PhantomData};

use esp_sync::RawMutex;
use procmacros::handler;

use crate::{
    Async,
    Blocking,
    DriverMode,
    asynch::AtomicWaker,
    pac::usb_device::RegisterBlock,
    peripherals::USB_DEVICE,
    rtc_cntl::WakeLock,
    system::{Peripheral, PeripheralClockControl},
};

mod compat;

/// Custom USB serial error type
type Error = Infallible;

/// USB Serial/JTAG (Full-duplex)
pub struct UsbSerialJtag<'d, Dm: DriverMode> {
    rx: UsbSerialJtagRx<'d, Dm>,
    tx: UsbSerialJtagTx<'d, Dm>,
}

/// USB Serial/JTAG (Transmit)
pub struct UsbSerialJtagTx<'d, Dm: DriverMode> {
    peripheral: USB_DEVICE<'d>,
    phantom: PhantomData<Dm>,
    wake_lock: WakeLock,
    /// The FIFO holds bytes, and no packet started for them.
    partial: bool,
    /// The last packet had 64 bytes, and no shorter packet ended the transfer.
    zlp_owed: bool,
}

/// USB Serial/JTAG (Receive)
pub struct UsbSerialJtagRx<'d, Dm: DriverMode> {
    peripheral: USB_DEVICE<'d>,
    phantom: PhantomData<Dm>,
    wake_lock: WakeLock,
}

impl<'d, Dm> UsbSerialJtagTx<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_inner(peripheral: USB_DEVICE<'d>) -> Self {
        Self {
            peripheral,
            phantom: PhantomData,
            wake_lock: WakeLock::new(),
            partial: false,
            zlp_owed: false,
        }
    }

    fn into_mode<Dm2: DriverMode>(self) -> UsbSerialJtagTx<'d, Dm2> {
        UsbSerialJtagTx {
            peripheral: self.peripheral,
            phantom: PhantomData,
            wake_lock: self.wake_lock,
            partial: self.partial,
            zlp_owed: self.zlp_owed,
        }
    }

    fn regs(&self) -> &RegisterBlock {
        self.peripheral.register_block()
    }

    /// Returns whether the output FIFO has room for at least one byte.
    ///
    /// If this function returns `true`, [`Self::write`] does not block.
    #[instability::unstable]
    pub fn write_ready(&self) -> bool {
        self.regs()
            .ep1_conf()
            .read()
            .serial_in_ep_data_free()
            .bit_is_set()
    }

    /// Writes bytes into the output FIFO.
    ///
    /// Blocks until the FIFO has room, then writes bytes until the data ends or the FIFO is full.
    /// Returns the number of bytes written, at most 64. Returns 0 only if `data` is empty.
    ///
    /// The hardware sends the FIFO to the host when it holds 64 bytes. Call [`Self::flush`] to
    /// send fewer bytes.
    #[instability::unstable]
    pub fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        if data.is_empty() {
            return Ok(0);
        }
        while !self.write_ready() {}

        Ok(self.write_fifo(data))
    }

    fn write_all(&mut self, mut data: &[u8]) -> Result<(), Error> {
        while !data.is_empty() {
            let written = self.write(data)?;
            data = &data[written..];
        }
        Ok(())
    }

    fn write_fifo(&mut self, data: &[u8]) -> usize {
        let mut written = 0;
        // The host can take a full packet before the next `write_ready` check.
        for &byte in data.iter().take(64) {
            if !self.write_ready() {
                break;
            }
            unsafe { self.regs().ep1().write(|w| w.rdwr_byte().bits(byte)) };
            written += 1;
        }

        if written > 0 {
            // The hardware starts a packet when the 64th byte enters the FIFO.
            if self.write_ready() {
                self.partial = true;
                self.zlp_owed = false;
            } else {
                self.partial = false;
                self.zlp_owed = true;
            }
        }

        written
    }

    /// Takes one step to send the data in the output FIFO to the host, without waiting.
    ///
    /// Returns `true` when the host has taken every written byte. Call this function until it
    /// returns `true`. Extra calls do not send extra packets.
    #[instability::unstable]
    pub fn try_flush(&mut self) -> bool {
        if !self.write_ready() {
            return false;
        }
        if self.partial || self.zlp_owed {
            // With an empty FIFO, this sends a zero-length packet, which ends the transfer.
            self.regs().ep1_conf().modify(|_, w| w.wr_done().set_bit());
            self.partial = false;
            self.zlp_owed = false;
            return false;
        }
        true
    }

    /// Sends the data in the output FIFO to the host.
    ///
    /// Blocks until the host has taken every written byte.
    #[instability::unstable]
    pub fn flush(&mut self) -> Result<(), Error> {
        while !self.try_flush() {}
        Ok(())
    }
}

impl<'d, Dm> UsbSerialJtagRx<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_inner(peripheral: USB_DEVICE<'d>) -> Self {
        Self {
            peripheral,
            phantom: PhantomData,
            wake_lock: WakeLock::new(),
        }
    }

    fn into_mode<Dm2: DriverMode>(self) -> UsbSerialJtagRx<'d, Dm2> {
        UsbSerialJtagRx {
            peripheral: self.peripheral,
            phantom: PhantomData,
            wake_lock: self.wake_lock,
        }
    }

    fn regs(&self) -> &RegisterBlock {
        self.peripheral.register_block()
    }

    /// Returns whether the input FIFO holds data from the host.
    ///
    /// If this function returns `true`, [`Self::read`] does not block.
    #[instability::unstable]
    pub fn read_ready(&self) -> bool {
        self.regs()
            .ep1_conf()
            .read()
            .serial_out_ep_data_avail()
            .bit_is_set()
    }

    /// Reads bytes from the input FIFO.
    ///
    /// Blocks until the FIFO holds data, then reads bytes until `buf` is full or the FIFO is
    /// empty. Returns the number of bytes read. Returns 0 only if `buf` is empty.
    #[instability::unstable]
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        while !self.read_ready() {}

        self.read_buffered(buf)
    }

    /// Reads the bytes that the input FIFO holds, without waiting.
    ///
    /// Returns the number of bytes read. This can be 0.
    #[instability::unstable]
    pub fn read_buffered(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let mut count = 0;
        for byte in buf.iter_mut() {
            if !self.read_ready() {
                break;
            }
            *byte = self.regs().ep1().read().rdwr_byte().bits();
            count += 1;
        }
        Ok(count)
    }

    /// Listens for RX-PACKET-RECV interrupts.
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        self.regs()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().set_bit());
    }

    /// Stops listening for RX-PACKET-RECV interrupts.
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        self.regs()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().clear_bit());
    }

    /// Returns whether RX-PACKET-RECV interrupt is set.
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        self.regs()
            .int_st()
            .read()
            .serial_out_recv_pkt()
            .bit_is_set()
    }

    /// Resets RX-PACKET-RECV interrupt.
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        self.regs()
            .int_clr()
            .write(|w| w.serial_out_recv_pkt().clear_bit_by_one());
    }
}

impl<'d> UsbSerialJtag<'d, Blocking> {
    /// Creates a new USB serial/JTAG instance with defaults.
    pub fn new(usb_device: USB_DEVICE<'d>) -> Self {
        Self::new_inner(usb_device)
    }

    /// Reconfigures the USB Serial JTAG peripheral to operate in asynchronous
    /// mode.
    pub fn into_async(mut self) -> UsbSerialJtag<'d, Async> {
        self.set_interrupt_handler(async_interrupt_handler);

        UsbSerialJtag {
            rx: self.rx.into_mode(),
            tx: self.tx.into_mode(),
        }
    }
}

impl<'d, Dm> UsbSerialJtag<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_inner(usb_device: USB_DEVICE<'d>) -> Self {
        // Do NOT reset the peripheral. Doing so will result in a broken USB JTAG
        // connection.
        if PeripheralClockControl::enable(Peripheral::UsbDevice) {
            PeripheralClockControl::reset(Peripheral::UsbDevice);
        } else {
            // Refcount was more than 0. Decrement to avoid overflow because we don't handle
            // dropping the driver.
            PeripheralClockControl::disable(Peripheral::UsbDevice);
        }

        usb_device.disable_tx_interrupts();
        usb_device.disable_rx_interrupts();

        #[cfg(any(esp32c3, esp32s3))]
        {
            use crate::efuse::USB_EXCHG_PINS;

            // On the esp32c3, and esp32s3 the USB_EXCHG_PINS efuse is bugged and
            // doesn't swap the pullups too, this works around that.
            if crate::efuse::read_bit(USB_EXCHG_PINS) {
                usb_device.register_block().conf0().modify(|_, w| {
                    w.pad_pull_override().set_bit();
                    w.dm_pullup().clear_bit();
                    w.dp_pullup().set_bit()
                });
            }
        }

        Self {
            rx: UsbSerialJtagRx::new_inner(unsafe { usb_device.clone_unchecked() }),
            tx: UsbSerialJtagTx::new_inner(usb_device),
        }
    }
    /// Splits the USB Serial JTAG peripheral into a transmitter and receiver,
    /// which is particularly useful when having two tasks correlating to
    /// transmitting and receiving.
    pub fn split(self) -> (UsbSerialJtagRx<'d, Dm>, UsbSerialJtagTx<'d, Dm>) {
        (self.rx, self.tx)
    }

    /// Returns whether the output FIFO has room for at least one byte.
    ///
    /// If this function returns `true`, [`Self::write`] does not block.
    #[instability::unstable]
    pub fn write_ready(&self) -> bool {
        self.tx.write_ready()
    }

    /// Writes bytes into the output FIFO.
    ///
    /// Blocks until the FIFO has room, then writes bytes until the data ends or the FIFO is full.
    /// Returns the number of bytes written, at most 64. Returns 0 only if `data` is empty.
    ///
    /// The hardware sends the FIFO to the host when it holds 64 bytes. Call [`Self::flush`] to
    /// send fewer bytes.
    #[instability::unstable]
    pub fn write(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write(data)
    }

    /// Takes one step to send the data in the output FIFO to the host, without waiting.
    ///
    /// Returns `true` when the host has taken every written byte. Call this function until it
    /// returns `true`. Extra calls do not send extra packets.
    #[instability::unstable]
    pub fn try_flush(&mut self) -> bool {
        self.tx.try_flush()
    }

    /// Sends the data in the output FIFO to the host.
    ///
    /// Blocks until the host has taken every written byte.
    #[instability::unstable]
    pub fn flush(&mut self) -> Result<(), Error> {
        self.tx.flush()
    }

    /// Returns whether the input FIFO holds data from the host.
    ///
    /// If this function returns `true`, [`Self::read`] does not block.
    #[instability::unstable]
    pub fn read_ready(&self) -> bool {
        self.rx.read_ready()
    }

    /// Reads bytes from the input FIFO.
    ///
    /// Blocks until the FIFO holds data, then reads bytes until `buf` is full or the FIFO is
    /// empty. Returns the number of bytes read. Returns 0 only if `buf` is empty.
    #[instability::unstable]
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read(buf)
    }

    /// Reads the bytes that the input FIFO holds, without waiting.
    ///
    /// Returns the number of bytes read. This can be 0.
    #[instability::unstable]
    pub fn read_buffered(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_buffered(buf)
    }

    /// Listens for RX-PACKET-RECV interrupts.
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        self.rx.listen_rx_packet_recv_interrupt()
    }

    /// Stops listening for RX-PACKET-RECV interrupts.
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        self.rx.unlisten_rx_packet_recv_interrupt()
    }

    /// Returns whether RX-PACKET-RECV interrupt is set.
    pub fn rx_packet_recv_interrupt_set(&mut self) -> bool {
        self.rx.rx_packet_recv_interrupt_set()
    }

    /// Resets RX-PACKET-RECV interrupt.
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        self.rx.reset_rx_packet_recv_interrupt()
    }

    /// Registers an interrupt handler for the USB Serial JTAG peripheral.
    ///
    /// Replaces any previously registered interrupt handlers.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.rx.peripheral.disable_peri_interrupt_on_all_cores();
        self.rx.peripheral.bind_peri_interrupt(handler);
    }
}

/// USB Serial/JTAG peripheral instance
#[doc(hidden)]
pub trait Instance: crate::private::Sealed {
    /// Returns a reference to the peripheral's underlying register block.
    fn register_block(&self) -> &RegisterBlock;

    /// Disables all transmit interrupts for the peripheral.
    fn disable_tx_interrupts(&self) {
        self.register_block()
            .int_ena()
            .modify(|_, w| w.serial_in_empty().clear_bit());

        self.register_block()
            .int_clr()
            .write(|w| w.serial_in_empty().clear_bit_by_one());
    }

    /// Disables all receive interrupts for the peripheral.
    fn disable_rx_interrupts(&self) {
        self.register_block()
            .int_ena()
            .modify(|_, w| w.serial_out_recv_pkt().clear_bit());

        self.register_block()
            .int_clr()
            .write(|w| w.serial_out_recv_pkt().clear_bit_by_one());
    }
}

impl Instance for USB_DEVICE<'_> {
    #[inline(always)]
    fn register_block(&self) -> &RegisterBlock {
        USB_DEVICE::regs()
    }
}

// Static instance of the waker for each component of the peripheral:
static WAKER_TX: AtomicWaker = AtomicWaker::new();
static WAKER_RX: AtomicWaker = AtomicWaker::new();
// TX and RX interrupts are enabled independently. Once either is enabled, its
// handler can preempt the other side's INT_ENA read-modify-write and cause
// stale enable bits to be restored. Serialize all INT_ENA updates.
static INT_ENA_LOCK: RawMutex = RawMutex::new();

#[derive(Clone, Copy)]
enum Event {
    /// `SERIAL_IN_EMPTY`: the host took a packet from the output FIFO.
    TxPacketSent,
    /// `SERIAL_OUT_RECV_PKT`: the host put a packet into the input FIFO.
    RxPacketReceived,
}

impl Event {
    fn waker(self) -> &'static AtomicWaker {
        match self {
            Event::TxPacketSent => &WAKER_TX,
            Event::RxPacketReceived => &WAKER_RX,
        }
    }

    fn ready(self, regs: &RegisterBlock) -> bool {
        let ep1_conf = regs.ep1_conf().read();
        match self {
            Event::TxPacketSent => ep1_conf.serial_in_ep_data_free().bit_is_set(),
            Event::RxPacketReceived => ep1_conf.serial_out_ep_data_avail().bit_is_set(),
        }
    }

    fn set_enabled(self, regs: &RegisterBlock, enable: bool) {
        INT_ENA_LOCK.lock(|| {
            regs.int_ena().modify(|_, w| match self {
                Event::TxPacketSent => w.serial_in_empty().bit(enable),
                Event::RxPacketReceived => w.serial_out_recv_pkt().bit(enable),
            });
        });
    }

    fn enabled(self, regs: &RegisterBlock) -> bool {
        let int_ena = regs.int_ena().read();
        match self {
            Event::TxPacketSent => int_ena.serial_in_empty().bit_is_set(),
            Event::RxPacketReceived => int_ena.serial_out_recv_pkt().bit_is_set(),
        }
    }

    fn clear(self, regs: &RegisterBlock) {
        regs.int_clr().write(|w| match self {
            Event::TxPacketSent => w.serial_in_empty().clear_bit_by_one(),
            Event::RxPacketReceived => w.serial_out_recv_pkt().clear_bit_by_one(),
        });
    }
}

/// Resolves when the FIFO is ready for `event`'s direction.
#[must_use = "futures do nothing unless you `.await` or poll them"]
struct UsbSerialJtagFuture<'d> {
    peripheral: USB_DEVICE<'d>,
    event: Event,
}

impl<'d> UsbSerialJtagFuture<'d> {
    fn new(peripheral: USB_DEVICE<'d>, event: Event) -> Self {
        let regs = peripheral.register_block();
        // A raw event from an earlier packet (`SERIAL_IN_EMPTY` resets to 1) wakes the future at
        // once. `poll` checks the FIFO after the clear, so no event is lost.
        event.clear(regs);
        event.set_enabled(regs, true);

        Self { peripheral, event }
    }
}

impl core::future::Future for UsbSerialJtagFuture<'_> {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        self.event.waker().register(cx.waker());
        let regs = self.peripheral.register_block();
        if self.event.ready(regs) || !self.event.enabled(regs) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Drop for UsbSerialJtagFuture<'_> {
    fn drop(&mut self) {
        self.event
            .set_enabled(self.peripheral.register_block(), false);
    }
}

impl<'d> UsbSerialJtag<'d, Async> {
    /// Reconfigures the USB Serial JTAG peripheral to operate in blocking
    /// mode.
    pub fn into_blocking(self) -> UsbSerialJtag<'d, Blocking> {
        self.rx.peripheral.disable_peri_interrupt_on_all_cores();
        UsbSerialJtag {
            rx: self.rx.into_mode(),
            tx: self.tx.into_mode(),
        }
    }

    /// Waits until the output FIFO has room for at least one byte.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe.
    #[instability::unstable]
    pub async fn wait_for_write_ready(&mut self) {
        self.tx.wait_for_write_ready().await
    }

    /// Writes bytes into the output FIFO.
    ///
    /// Waits until the FIFO has room, then writes bytes until the data ends or the FIFO is full.
    /// Returns the number of bytes written, at most 64. Returns 0 only if `data` is empty.
    ///
    /// The hardware sends the FIFO to the host when it holds 64 bytes. Call [`Self::flush_async`]
    /// to send fewer bytes.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were written.
    #[instability::unstable]
    pub async fn write_async(&mut self, data: &[u8]) -> Result<usize, Error> {
        self.tx.write_async(data).await
    }

    /// Sends the data in the output FIFO to the host.
    ///
    /// Waits until the host has taken every written byte.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. A later flush continues where the dropped future stopped.
    #[instability::unstable]
    pub async fn flush_async(&mut self) -> Result<(), Error> {
        self.tx.flush_async().await
    }

    /// Reads bytes from the input FIFO.
    ///
    /// Waits until the FIFO holds data, then reads bytes until `buf` is full or the FIFO is
    /// empty. Returns the number of bytes read. Returns 0 only if `buf` is empty.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were read.
    #[instability::unstable]
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.rx.read_async(buf).await
    }

    /// Fills `buf` with bytes from the input FIFO.
    ///
    /// Waits until the host has sent enough bytes.
    ///
    /// # Cancellation Safety
    ///
    /// **Not** cancellation safe. If the future is dropped, the caller does not know how many
    /// bytes were read.
    #[instability::unstable]
    pub async fn read_exact_async(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        self.rx.read_exact_async(buf).await
    }
}

impl UsbSerialJtagTx<'_, Async> {
    /// Waits until the output FIFO has room for at least one byte.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe.
    #[instability::unstable]
    pub async fn wait_for_write_ready(&mut self) {
        while !self.write_ready() {
            UsbSerialJtagFuture::new(self.peripheral.reborrow(), Event::TxPacketSent).await;
        }
    }

    /// Writes bytes into the output FIFO.
    ///
    /// Waits until the FIFO has room, then writes bytes until the data ends or the FIFO is full.
    /// Returns the number of bytes written, at most 64. Returns 0 only if `data` is empty.
    ///
    /// The hardware sends the FIFO to the host when it holds 64 bytes. Call [`Self::flush_async`]
    /// to send fewer bytes.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were written.
    #[instability::unstable]
    pub async fn write_async(&mut self, data: &[u8]) -> Result<usize, Error> {
        if data.is_empty() {
            return Ok(0);
        }
        self.wait_for_write_ready().await;

        Ok(self.write_fifo(data))
    }

    /// Sends the data in the output FIFO to the host.
    ///
    /// Waits until the host has taken every written byte.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. A later flush continues where the dropped future stopped.
    #[instability::unstable]
    pub async fn flush_async(&mut self) -> Result<(), Error> {
        while !self.try_flush() {
            self.wait_for_write_ready().await;
        }
        Ok(())
    }
}

impl UsbSerialJtagRx<'_, Async> {
    /// Reads bytes from the input FIFO.
    ///
    /// Waits until the FIFO holds data, then reads bytes until `buf` is full or the FIFO is
    /// empty. Returns the number of bytes read. Returns 0 only if `buf` is empty.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were read.
    #[instability::unstable]
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        while !self.read_ready() {
            UsbSerialJtagFuture::new(self.peripheral.reborrow(), Event::RxPacketReceived).await;
        }

        self.read_buffered(buf)
    }

    /// Fills `buf` with bytes from the input FIFO.
    ///
    /// Waits until the host has sent enough bytes.
    ///
    /// # Cancellation Safety
    ///
    /// **Not** cancellation safe. If the future is dropped, the caller does not know how many
    /// bytes were read.
    #[instability::unstable]
    pub async fn read_exact_async(&mut self, mut buf: &mut [u8]) -> Result<(), Error> {
        while !buf.is_empty() {
            let read = self.read_async(buf).await?;
            buf = &mut buf[read..];
        }
        Ok(())
    }
}

#[handler]
fn async_interrupt_handler() {
    let usb = USB_DEVICE::regs();
    let interrupts = usb.int_st().read();

    let tx = interrupts.serial_in_empty().bit_is_set();
    let rx = interrupts.serial_out_recv_pkt().bit_is_set();

    INT_ENA_LOCK.lock(|| {
        usb.int_ena().modify(|_, w| {
            if tx {
                w.serial_in_empty().clear_bit();
            }
            if rx {
                w.serial_out_recv_pkt().clear_bit();
            }
            w
        });
    });

    usb.int_clr().write(|w| {
        if tx {
            w.serial_in_empty().clear_bit_by_one();
        }
        if rx {
            w.serial_out_recv_pkt().clear_bit_by_one();
        }
        w
    });

    if rx {
        WAKER_RX.wake();
    }
    if tx {
        WAKER_TX.wake();
    }
}
