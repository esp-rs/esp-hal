#![cfg_attr(docsrs, procmacros::doc_replace(
    "dp_pin" => {
        cfg(not(usb_serial_jtag_phy_pads_usj)) => gpio_for_signal!(USB_FS_DP),
        _ => gpio_for_signal!(USJ_DP),
    },
    "dm_pin" => {
        cfg(not(usb_serial_jtag_phy_pads_usj)) => gpio_for_signal!(USB_FS_DM),
        _ => gpio_for_signal!(USJ_DM),
    },
))]
//! # USB Serial/JTAG Controller (USB_SERIAL_JTAG)
//!
//! ## Overview
//!
//! The USB Serial/JTAG controller lets a USB host program the flash, read the program output,
//! and connect a debugger to the running program. It needs no external active components.
//!
//! The controller is a USB-to-serial converter and a USB-to-JTAG adapter in one device. It uses
//! only the two USB 2.0 data lines, so only two pins are reserved for it.
//!
//! The controller has these features:
//!
//! - Fixed CDC-ACM (Communication Device Class, Abstract Control Model) and JTAG adapter functions
//! - A CDC-ACM serial port that works without drivers on most operating systems, with chip reset
//!   and entry into download mode from the host
//! - Fast communication with the CPU debug module through a compact form of JTAG instructions
//! - Two OUT endpoints and three IN endpoints in addition to control endpoint 0, each with a
//!   payload of up to 64 bytes
//! - An internal PHY, so the connection to the host needs few or no external components
//!
//! ## Usage
//!
//! The driver takes the D+ and D- pins of the PHY that the controller uses. On some chips, the
//! `USB_PHY_SEL` eFuse or a PHY selection register moves the controller to another PHY. Then pass
//! the pins of that PHY. [`UsbSerialJtag::new`] returns [`ConfigError::WrongPins`] for other pins.
//!
//! The driver implements these traits:
//!
//! - [`core::fmt::Write`] and [`ufmt_write::uWrite`]
//! - the blocking [embedded-io] traits `Read`, `Write`, `ReadReady`, and `WriteReady`, versions 0.6
//!   and 0.7
//! - the [embedded-io-async] traits `Read` and `Write` in [`Async`] mode, versions 0.6 and 0.7
//!
//! [embedded-io]: embedded_io_07
//! [embedded-io-async]: embedded_io_async_07
//!
//! ### Packets and Flushing
//!
//! The driver sends data to the host in packets of up to 64 bytes. When the output FIFO holds 64
//! bytes, the hardware sends them as one packet. Fewer bytes stay in the FIFO until a call to
//! [`UsbSerialJtagTx::flush`] or [`UsbSerialJtagTx::flush_async`]. The write functions and the
//! [`core::fmt::Write`] and [`ufmt_write::uWrite`] implementations do not flush.
//!
//! The host can hold back a 64-byte packet until the transfer ends. A flush ends the transfer.
//! Flush the output when the host must see the data.
//!
//! The driver must be the only writer to the output FIFO. Other writers, for example ROM
//! functions or a logger on the same peripheral, can make the flush functions skip a packet.
//!
//! ### Timeout
//!
//! The host takes a packet only while a program on the host reads the serial port. The blocking
//! write and flush functions wait for the host for at most [`Config::tx_timeout`], then return
//! [`TxError::Timeout`]. After a timeout, they return [`TxError::Timeout`] at once until the host
//! takes the packet. The blocking read functions wait for data for at most
//! [`Config::rx_timeout`], then return [`RxError::Timeout`]. The async functions do not time
//! out. Use a timer to cancel them.
//!
//! ## Examples
//!
//! ### Sending Data
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::usb::usb_serial_jtag::{Config, UsbSerialJtag};
//!
//! let mut usb_serial = UsbSerialJtag::new(
//!     peripherals.USB_DEVICE,
//!     peripherals.__dp_pin__,
//!     peripherals.__dm_pin__,
//!     Config::default(),
//! )?;
//!
//! // `write` stores at most one packet, so write in a loop.
//! let mut data: &[u8] = b"Hello, world!";
//! while !data.is_empty() {
//!     let written = usb_serial.write(data)?;
//!     data = &data[written..];
//! }
//! usb_serial.flush()?;
//! # {after_snippet}
//! ```
//!
//! ### Splitting the Driver into Transmitter and Receiver
//! ```rust, no_run
//! # {before_snippet}
//! use esp_hal::usb::usb_serial_jtag::{Config, UsbSerialJtag};
//!
//! let usb_serial = UsbSerialJtag::new(
//!     peripherals.USB_DEVICE,
//!     peripherals.__dp_pin__,
//!     peripherals.__dm_pin__,
//!     Config::default(),
//! )?;
//! let (mut rx, mut tx) = usb_serial.split();
//!
//! tx.write(&[42u8])?;
//! tx.flush()?;
//!
//! let mut buffer = [0u8; 64];
//! let count = rx.read(&mut buffer)?;
//! # {after_snippet}
//! ```
//!
//! ### Receiving Data in an Interrupt Handler
//! ```rust, no_run
//! # {before_snippet}
//! # use esp_hal::{delay::Delay, usb::usb_serial_jtag::{Config, UsbSerialJtag}, Blocking};
//!
//! let delay = Delay::new();
//!
//! let mut usb_serial = UsbSerialJtag::new(
//!     peripherals.USB_DEVICE,
//!     peripherals.__dp_pin__,
//!     peripherals.__dm_pin__,
//!     Config::default(),
//! )?;
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
//!             let count = usb_serial.read_buffered(&mut buffer).unwrap_or(0);
//!             println!("Read bytes: {:02x?}", &buffer[..count]);
//!
//!             usb_serial.reset_rx_packet_recv_interrupt();
//!         }
//!     });
//! }
//! ```

use core::{marker::PhantomData, task::Poll};

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
    time::{Duration, Instant},
};

mod compat;
mod pins;

pub use pins::UsbSerialJtagPin;

/// USB Serial/JTAG transmitter error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum TxError {
    /// The host took no packet within [`Config::tx_timeout`].
    ///
    /// No program on the host reads the serial port, or the USB cable is disconnected.
    Timeout,
}

#[instability::unstable]
impl core::error::Error for TxError {}

#[instability::unstable]
impl core::fmt::Display for TxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TxError::Timeout => write!(f, "The host took no packet within the timeout"),
        }
    }
}

/// USB Serial/JTAG receiver error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum RxError {
    /// The host sent no data within [`Config::rx_timeout`].
    Timeout,
}

#[instability::unstable]
impl core::error::Error for RxError {}

#[instability::unstable]
impl core::fmt::Display for RxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RxError::Timeout => write!(f, "The host sent no data within the timeout"),
        }
    }
}

/// USB Serial/JTAG error of the full-duplex driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum IoError {
    /// Transmitter error.
    Tx(TxError),
    /// Receiver error.
    Rx(RxError),
}

#[instability::unstable]
impl core::error::Error for IoError {}

#[instability::unstable]
impl core::fmt::Display for IoError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            IoError::Tx(e) => e.fmt(f),
            IoError::Rx(e) => e.fmt(f),
        }
    }
}

#[instability::unstable]
impl From<TxError> for IoError {
    fn from(e: TxError) -> Self {
        IoError::Tx(e)
    }
}

#[instability::unstable]
impl From<RxError> for IoError {
    fn from(e: RxError) -> Self {
        IoError::Rx(e)
    }
}

/// USB Serial/JTAG configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub enum ConfigError {
    /// The pins are not the pins of the PHY that the controller uses.
    ///
    /// The `USB_PHY_SEL` eFuse and the PHY selection registers select the PHY.
    WrongPins,
    /// The controller uses a PHY that the driver does not support, for example an external PHY.
    UnsupportedPhy,
}

#[instability::unstable]
impl core::error::Error for ConfigError {}

#[instability::unstable]
impl core::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ConfigError::WrongPins => {
                write!(
                    f,
                    "The pins are not the pins of the PHY that the controller uses"
                )
            }
            ConfigError::UnsupportedPhy => {
                write!(
                    f,
                    "The controller uses a PHY that the driver does not support"
                )
            }
        }
    }
}

/// USB Serial/JTAG driver configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, procmacros::BuilderLite)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
#[non_exhaustive]
pub struct Config {
    /// How long a blocking write or flush waits for the host to take a packet.
    ///
    /// `None` waits forever. The default is 50 ms.
    tx_timeout: Option<Duration>,
    /// How long a blocking read waits for the host to send data.
    ///
    /// `None` waits forever. The default is `None`.
    rx_timeout: Option<Duration>,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            tx_timeout: Some(Duration::from_millis(50)),
            rx_timeout: None,
        }
    }
}

/// USB Serial/JTAG driver with a transmitter and a receiver.
///
/// [`Self::split`] returns the transmitter and the receiver as separate drivers.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct UsbSerialJtag<'d, Dm: DriverMode> {
    rx: UsbSerialJtagRx<'d, Dm>,
    tx: UsbSerialJtagTx<'d, Dm>,
}

/// USB Serial/JTAG transmitter.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct UsbSerialJtagTx<'d, Dm: DriverMode> {
    peripheral: USB_DEVICE<'d>,
    phantom: PhantomData<Dm>,
    wake_lock: WakeLock,
    /// The FIFO holds bytes, and no packet started for them.
    partial: bool,
    /// The last packet had 64 bytes, and no shorter packet ended the transfer.
    zlp_owed: bool,
    tx_timeout: Option<Duration>,
    /// A blocking wait timed out, and the host has taken no packet since.
    timed_out: bool,
}

/// USB Serial/JTAG receiver.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct UsbSerialJtagRx<'d, Dm: DriverMode> {
    peripheral: USB_DEVICE<'d>,
    phantom: PhantomData<Dm>,
    wake_lock: WakeLock,
    rx_timeout: Option<Duration>,
}

impl<'d> UsbSerialJtag<'d, Blocking> {
    /// Creates a new USB Serial/JTAG driver in [`Blocking`] mode.
    ///
    /// `dp` and `dm` are the D+ and D- pins of the PHY that the controller uses. The driver
    /// borrows them, so the program cannot use them as GPIOs. The driver does not check the order
    /// of the pins, because the `USB_EXCHG_PINS` eFuse swaps D+ and D-.
    ///
    /// # Errors
    ///
    /// Returns [`ConfigError::WrongPins`] if the pins are not the pins of the PHY that the
    /// controller uses. Returns [`ConfigError::UnsupportedPhy`] if the controller uses a PHY that
    /// the driver does not support.
    #[instability::unstable]
    pub fn new(
        usb_device: USB_DEVICE<'d>,
        dp: impl UsbSerialJtagPin + 'd,
        dm: impl UsbSerialJtagPin + 'd,
        config: Config,
    ) -> Result<Self, ConfigError> {
        let pads = pins::validate_pins(&dp, &dm)?;
        pins::enable_pads(pads);
        let mut this = Self::new_inner(usb_device);
        this.apply_config(&config)?;
        Ok(this)
    }

    /// Reconfigures the driver to operate in [`Async`] mode.
    #[instability::unstable]
    pub fn into_async(mut self) -> UsbSerialJtag<'d, Async> {
        self.set_interrupt_handler(async_interrupt_handler);

        UsbSerialJtag {
            rx: self.rx.into_mode(),
            tx: self.tx.into_mode(),
        }
    }

    /// Registers an interrupt handler for the peripheral.
    ///
    /// Replaces the interrupt handler that was registered before.
    #[instability::unstable]
    pub fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.rx.peripheral.disable_peri_interrupt_on_all_cores();
        self.rx.peripheral.bind_peri_interrupt(handler);
    }

    /// Enables the interrupt for a packet received from the host.
    #[instability::unstable]
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        self.rx.listen_rx_packet_recv_interrupt()
    }

    /// Disables the interrupt for a packet received from the host.
    #[instability::unstable]
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        self.rx.unlisten_rx_packet_recv_interrupt()
    }

    /// Returns whether the interrupt for a packet received from the host is pending.
    #[instability::unstable]
    pub fn rx_packet_recv_interrupt_set(&self) -> bool {
        self.rx.rx_packet_recv_interrupt_set()
    }

    /// Clears the interrupt for a packet received from the host.
    #[instability::unstable]
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        self.rx.reset_rx_packet_recv_interrupt()
    }
}

impl<'d> UsbSerialJtag<'d, Async> {
    /// Reconfigures the driver to operate in [`Blocking`] mode.
    #[instability::unstable]
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
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were written.
    #[instability::unstable]
    pub async fn write_async(&mut self, data: &[u8]) -> Result<usize, TxError> {
        self.tx.write_async(data).await
    }

    /// Sends the data in the output FIFO to the host.
    ///
    /// Waits until the host has taken every written byte.
    ///
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. A later flush continues where the dropped future stopped.
    #[instability::unstable]
    pub async fn flush_async(&mut self) -> Result<(), TxError> {
        self.tx.flush_async().await
    }

    /// Reads bytes from the input FIFO.
    ///
    /// Waits until the FIFO holds data, then reads bytes until `buf` is full or the FIFO is
    /// empty. Returns the number of bytes read. Returns 0 only if `buf` is empty.
    ///
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were read.
    #[instability::unstable]
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.rx.read_async(buf).await
    }

    /// Fills `buf` with bytes from the input FIFO.
    ///
    /// Waits until the host has sent enough bytes.
    ///
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// **Not** cancellation safe. If the future is dropped, the caller does not know how many
    /// bytes were read.
    #[instability::unstable]
    pub async fn read_exact_async(&mut self, buf: &mut [u8]) -> Result<(), RxError> {
        self.rx.read_exact_async(buf).await
    }
}

impl<'d, Dm> UsbSerialJtag<'d, Dm>
where
    Dm: DriverMode,
{
    fn new_inner(usb_device: USB_DEVICE<'d>) -> Self {
        // A reset breaks the USB-JTAG connection. `UsbDevice` is in `Peripheral::KEEP_ENABLED`, so
        // `enable` returns `false` and the reset does not run.
        if PeripheralClockControl::enable(Peripheral::UsbDevice) {
            PeripheralClockControl::reset(Peripheral::UsbDevice);
        } else {
            // The driver has no `Drop`, so undo the count increment to prevent an overflow.
            PeripheralClockControl::disable(Peripheral::UsbDevice);
        }

        let regs = USB_DEVICE::regs();
        for event in [Event::TxPacketSent, Event::RxPacketReceived] {
            event.set_enabled(regs, false);
            event.clear(regs);
        }

        #[cfg(any(esp32c3, esp32s3))]
        {
            use crate::efuse::USB_EXCHG_PINS;

            // On ESP32-C3 and ESP32-S3, the USB_EXCHG_PINS eFuse swaps the data lines, but not
            // their pull-up resistors.
            if crate::efuse::read_bit(USB_EXCHG_PINS) {
                regs.conf0().modify(|_, w| {
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

    /// Applies a new configuration.
    ///
    /// # Errors
    ///
    /// This function does not fail.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.rx.apply_config(config)?;
        self.tx.apply_config(config)
    }

    /// Splits the driver into a receiver and a transmitter.
    ///
    /// Separate tasks can then own the receiver and the transmitter.
    #[instability::unstable]
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
    ///
    /// # Errors
    ///
    /// Returns [`TxError::Timeout`] if the FIFO has no room within [`Config::tx_timeout`]. No
    /// bytes were written then.
    #[instability::unstable]
    pub fn write(&mut self, data: &[u8]) -> Result<usize, TxError> {
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
    ///
    /// # Errors
    ///
    /// Returns [`TxError::Timeout`] if the host takes no packet within [`Config::tx_timeout`].
    /// The data stays in the FIFO, and a later flush continues.
    #[instability::unstable]
    pub fn flush(&mut self) -> Result<(), TxError> {
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
    ///
    /// # Errors
    ///
    /// Returns [`RxError::Timeout`] if the host sends no data within [`Config::rx_timeout`].
    #[instability::unstable]
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.rx.read(buf)
    }

    /// Reads the bytes that the input FIFO holds, without waiting.
    ///
    /// Returns the number of bytes read. This can be 0.
    #[instability::unstable]
    pub fn read_buffered(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        self.rx.read_buffered(buf)
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
        self.timed_out = false;
    }

    /// Writes bytes into the output FIFO.
    ///
    /// Waits until the FIFO has room, then writes bytes until the data ends or the FIFO is full.
    /// Returns the number of bytes written, at most 64. Returns 0 only if `data` is empty.
    ///
    /// The hardware sends the FIFO to the host when it holds 64 bytes. Call [`Self::flush_async`]
    /// to send fewer bytes.
    ///
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were written.
    #[instability::unstable]
    pub async fn write_async(&mut self, data: &[u8]) -> Result<usize, TxError> {
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
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. A later flush continues where the dropped future stopped.
    #[instability::unstable]
    pub async fn flush_async(&mut self) -> Result<(), TxError> {
        while !self.try_flush() {
            self.wait_for_write_ready().await;
        }
        Ok(())
    }
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
            tx_timeout: None,
            timed_out: false,
        }
    }

    fn into_mode<Dm2: DriverMode>(self) -> UsbSerialJtagTx<'d, Dm2> {
        UsbSerialJtagTx {
            peripheral: self.peripheral,
            phantom: PhantomData,
            wake_lock: self.wake_lock,
            partial: self.partial,
            zlp_owed: self.zlp_owed,
            tx_timeout: self.tx_timeout,
            timed_out: self.timed_out,
        }
    }

    fn regs(&self) -> &RegisterBlock {
        USB_DEVICE::regs()
    }

    /// Applies a new configuration.
    ///
    /// # Errors
    ///
    /// This function does not fail.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.tx_timeout = config.tx_timeout;
        self.timed_out = false;
        Ok(())
    }

    fn wait_for_write_ready_blocking(&mut self) -> Result<(), TxError> {
        if !self.write_ready() {
            let Some(timeout) = self.tx_timeout else {
                while !self.write_ready() {}
                return Ok(());
            };
            if self.timed_out {
                return Err(TxError::Timeout);
            }
            let start = Instant::now();
            while !self.write_ready() {
                if start.elapsed() >= timeout {
                    self.timed_out = true;
                    return Err(TxError::Timeout);
                }
            }
        }
        self.timed_out = false;
        Ok(())
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
    ///
    /// # Errors
    ///
    /// Returns [`TxError::Timeout`] if the FIFO has no room within [`Config::tx_timeout`]. No
    /// bytes were written then.
    #[instability::unstable]
    pub fn write(&mut self, data: &[u8]) -> Result<usize, TxError> {
        if data.is_empty() {
            return Ok(0);
        }
        self.wait_for_write_ready_blocking()?;

        Ok(self.write_fifo(data))
    }

    fn write_all(&mut self, mut data: &[u8]) -> Result<(), TxError> {
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
        self.timed_out = false;
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
    ///
    /// # Errors
    ///
    /// Returns [`TxError::Timeout`] if the host takes no packet within [`Config::tx_timeout`].
    /// The data stays in the FIFO, and a later flush continues.
    #[instability::unstable]
    pub fn flush(&mut self) -> Result<(), TxError> {
        while !self.try_flush() {
            self.wait_for_write_ready_blocking()?;
        }
        Ok(())
    }
}

impl UsbSerialJtagRx<'_, Blocking> {
    /// Enables the interrupt for a packet received from the host.
    #[instability::unstable]
    pub fn listen_rx_packet_recv_interrupt(&mut self) {
        Event::RxPacketReceived.set_enabled(self.regs(), true);
    }

    /// Disables the interrupt for a packet received from the host.
    #[instability::unstable]
    pub fn unlisten_rx_packet_recv_interrupt(&mut self) {
        Event::RxPacketReceived.set_enabled(self.regs(), false);
    }

    /// Returns whether the interrupt for a packet received from the host is pending.
    #[instability::unstable]
    pub fn rx_packet_recv_interrupt_set(&self) -> bool {
        self.regs()
            .int_st()
            .read()
            .serial_out_recv_pkt()
            .bit_is_set()
    }

    /// Clears the interrupt for a packet received from the host.
    #[instability::unstable]
    pub fn reset_rx_packet_recv_interrupt(&mut self) {
        Event::RxPacketReceived.clear(self.regs());
    }
}

impl UsbSerialJtagRx<'_, Async> {
    /// Reads bytes from the input FIFO.
    ///
    /// Waits until the FIFO holds data, then reads bytes until `buf` is full or the FIFO is
    /// empty. Returns the number of bytes read. Returns 0 only if `buf` is empty.
    ///
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// Cancellation safe. If the future is dropped, no bytes were read.
    #[instability::unstable]
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
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
    /// # Errors
    ///
    /// This function does not time out. It returns no error.
    ///
    /// # Cancellation Safety
    ///
    /// **Not** cancellation safe. If the future is dropped, the caller does not know how many
    /// bytes were read.
    #[instability::unstable]
    pub async fn read_exact_async(&mut self, mut buf: &mut [u8]) -> Result<(), RxError> {
        while !buf.is_empty() {
            let read = self.read_async(buf).await?;
            buf = &mut buf[read..];
        }
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
            rx_timeout: None,
        }
    }

    fn into_mode<Dm2: DriverMode>(self) -> UsbSerialJtagRx<'d, Dm2> {
        UsbSerialJtagRx {
            peripheral: self.peripheral,
            phantom: PhantomData,
            wake_lock: self.wake_lock,
            rx_timeout: self.rx_timeout,
        }
    }

    fn regs(&self) -> &RegisterBlock {
        USB_DEVICE::regs()
    }

    /// Applies a new configuration.
    ///
    /// # Errors
    ///
    /// This function does not fail.
    #[instability::unstable]
    pub fn apply_config(&mut self, config: &Config) -> Result<(), ConfigError> {
        self.rx_timeout = config.rx_timeout;
        Ok(())
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
    ///
    /// # Errors
    ///
    /// Returns [`RxError::Timeout`] if the host sends no data within [`Config::rx_timeout`].
    #[instability::unstable]
    pub fn read(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
        if buf.is_empty() {
            return Ok(0);
        }
        if !self.read_ready() {
            match self.rx_timeout {
                None => while !self.read_ready() {},
                Some(timeout) => {
                    let start = Instant::now();
                    while !self.read_ready() {
                        if start.elapsed() >= timeout {
                            return Err(RxError::Timeout);
                        }
                    }
                }
            }
        }

        self.read_buffered(buf)
    }

    /// Reads the bytes that the input FIFO holds, without waiting.
    ///
    /// Returns the number of bytes read. This can be 0.
    #[instability::unstable]
    pub fn read_buffered(&mut self, buf: &mut [u8]) -> Result<usize, RxError> {
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
}

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
    _peripheral: USB_DEVICE<'d>,
    event: Event,
}

impl<'d> UsbSerialJtagFuture<'d> {
    fn new(peripheral: USB_DEVICE<'d>, event: Event) -> Self {
        let regs = USB_DEVICE::regs();
        // A raw event from an earlier packet (`SERIAL_IN_EMPTY` resets to 1) wakes the future at
        // once. `poll` checks the FIFO after the clear, so no event is lost.
        event.clear(regs);
        event.set_enabled(regs, true);

        Self {
            _peripheral: peripheral,
            event,
        }
    }
}

impl core::future::Future for UsbSerialJtagFuture<'_> {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        self.event.waker().register(cx.waker());
        let regs = USB_DEVICE::regs();
        if self.event.ready(regs) || !self.event.enabled(regs) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

impl Drop for UsbSerialJtagFuture<'_> {
    fn drop(&mut self) {
        self.event.set_enabled(USB_DEVICE::regs(), false);
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
