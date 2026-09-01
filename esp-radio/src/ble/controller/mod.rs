//! BLE controller
use alloc::boxed::Box;
use core::{future::Future, task::Poll};

use bt_hci::{
    ControllerToHostPacket,
    FromHciBytes,
    FromHciBytesError,
    HostToControllerPacket,
    WriteHci,
};
use bt_hci_transport::{PacketKind, PacketToController, PacketToHost};
use docsplay::Display;
use esp_phy::PhyInitGuard;

use crate::{
    RadioRefGuard,
    asynch::AtomicWaker,
    ble::{
        Config,
        InvalidConfigError,
        have_hci_packet,
        have_hci_read_data,
        read_hci,
        read_next,
        send_hci,
        take_next,
    },
};

#[derive(Display, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error enum for BLE initialization failures.
pub enum BleInitError {
    /// Failure during initial validation of the provided configuration: {0}.
    Config(InvalidConfigError),
}

impl core::error::Error for BleInitError {}

// Implement the From trait for cleaner error mapping
impl From<InvalidConfigError> for BleInitError {
    fn from(err: InvalidConfigError) -> Self {
        BleInitError::Config(err)
    }
}

/// A blocking HCI connector
#[instability::unstable]
pub struct BleConnector<'d> {
    _phy_init_guard: PhyInitGuard<'d>,
    _device: crate::hal::peripherals::BT<'d>,
    _guard: RadioRefGuard,
}

impl Drop for BleConnector<'_> {
    fn drop(&mut self) {
        crate::ble::ble_deinit();
        crate::ble::clear_bt_state();
    }
}
impl<'d> BleConnector<'d> {
    /// Create and init a new BLE connector.
    #[instability::unstable]
    pub fn new(
        device: crate::hal::peripherals::BT<'d>,
        config: Config,
    ) -> Result<BleConnector<'d>, BleInitError> {
        let _guard = RadioRefGuard::new();

        config.validate()?;

        Ok(Self {
            _phy_init_guard: crate::ble::ble_init(&config),
            _device: device,
            _guard,
        })
    }

    /// Read the next HCI packet from the BLE controller.
    #[instability::unstable]
    pub fn next(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
        Ok(read_next(buf))
    }

    /// Read from HCI.
    #[instability::unstable]
    pub fn read(&mut self, mut buf: &mut [u8]) -> Result<usize, BleConnectorError> {
        let mut total = 0;
        while !buf.is_empty() {
            let len = read_hci(buf);
            if len == 0 {
                break;
            }

            buf = &mut buf[len..];
            total += len;
        }
        Ok(total)
    }

    /// Read from HCI.
    #[instability::unstable]
    pub async fn read_async(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
        if buf.is_empty() {
            return Ok(0);
        }

        HciAnyDataReadyEventFuture.await;

        self.read(buf)
    }

    /// Write to HCI.
    ///
    /// Returns the number of bytes written, which is at most one packet.
    #[instability::unstable]
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
        Ok(send_hci(buf))
    }
}

#[derive(Display, Debug, Copy, Clone, Eq, PartialEq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Error type for the BLE connector.
#[instability::unstable]
pub enum BleConnectorError {
    /// Unknown BLE error occurred.
    Unknown,
}

impl embedded_io_06::Error for BleConnectorError {
    fn kind(&self) -> embedded_io_06::ErrorKind {
        embedded_io_06::ErrorKind::Other
    }
}

impl embedded_io_07::Error for BleConnectorError {
    fn kind(&self) -> embedded_io_07::ErrorKind {
        embedded_io_07::ErrorKind::Other
    }
}

impl core::error::Error for BleConnectorError {}

impl embedded_io_06::ErrorType for BleConnector<'_> {
    type Error = BleConnectorError;
}

impl embedded_io_06::Read for BleConnector<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf)
    }
}

impl embedded_io_06::Write for BleConnector<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // nothing to do
        Ok(())
    }
}

impl embedded_io_07::ErrorType for BleConnector<'_> {
    type Error = BleConnectorError;
}

impl embedded_io_07::Read for BleConnector<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf)
    }
}

impl embedded_io_07::Write for BleConnector<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // nothing to do
        Ok(())
    }
}

static HCI_WAKER: AtomicWaker = AtomicWaker::new();

pub(crate) fn hci_read_data_available() {
    HCI_WAKER.wake();
}

impl embedded_io_async_06::Read for BleConnector<'_> {
    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, BleConnectorError>> {
        self.read_async(buf)
    }
}

impl embedded_io_async_06::Write for BleConnector<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
        Ok(send_hci(buf))
    }

    async fn flush(&mut self) -> Result<(), BleConnectorError> {
        // nothing to do
        Ok(())
    }
}

impl embedded_io_async_07::Read for BleConnector<'_> {
    fn read(&mut self, buf: &mut [u8]) -> impl Future<Output = Result<usize, BleConnectorError>> {
        self.read_async(buf)
    }
}

impl embedded_io_async_07::Write for BleConnector<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
        Ok(send_hci(buf))
    }

    async fn flush(&mut self) -> Result<(), BleConnectorError> {
        // nothing to do
        Ok(())
    }
}

impl From<FromHciBytesError> for BleConnectorError {
    fn from(_e: FromHciBytesError) -> Self {
        BleConnectorError::Unknown
    }
}

/// Completes once any HCI data is available, including a part-read packet.
pub(crate) struct HciAnyDataReadyEventFuture;

impl core::future::Future for HciAnyDataReadyEventFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        HCI_WAKER.register(cx.waker());

        if have_hci_read_data() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// Completes once the receive queue holds a complete packet.
pub(crate) struct HciPacketReadyEventFuture;

impl core::future::Future for HciPacketReadyEventFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        HCI_WAKER.register(cx.waker());

        if have_hci_packet() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
}

/// The HCI output of the BLE controller.
///
/// The transport implementations use this zero-sized writer to serialize packets directly into the
/// controller.
struct HciWriter;

impl embedded_io_07::ErrorType for HciWriter {
    type Error = BleConnectorError;
}

impl embedded_io_async_07::Write for HciWriter {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Ok(send_hci(buf))
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        // nothing to do
        Ok(())
    }
}

impl<E: embedded_io_07::Error> From<bt_hci_transport::ReadHciError<E>> for BleConnectorError {
    fn from(_e: bt_hci_transport::ReadHciError<E>) -> Self {
        BleConnectorError::Unknown
    }
}

fn parse_hci(data: &[u8]) -> Result<ControllerToHostPacket<'_>, BleConnectorError> {
    match ControllerToHostPacket::from_hci_bytes_complete(data) {
        Ok(p) => Ok(p),
        Err(e) => {
            warn!("[hci] error parsing packet: {:?}", e);
            Err(BleConnectorError::Unknown)
        }
    }
}

/// Waits for a packet from the controller, then removes it from the receive queue.
async fn next_packet() -> Box<[u8]> {
    loop {
        HciPacketReadyEventFuture.await;

        if let Some(packet) = take_next() {
            return packet;
        }
    }
}

impl bt_hci::transport::Transport for BleConnector<'_> {
    /// Read a complete HCI packet into the rx buffer
    async fn read<'a>(&self, rx: &'a mut [u8]) -> Result<ControllerToHostPacket<'a>, Self::Error> {
        // Workaround for borrow checker.
        // Safety: we only return a reference to x once, if parsing is successful.
        let rx = unsafe { &mut *core::ptr::slice_from_raw_parts_mut(rx.as_mut_ptr(), rx.len()) };

        // `ControllerToHostPacket` borrows `rx`, so the packet has to be copied there.
        HciPacketReadyEventFuture.await;
        let len = read_next(rx);
        parse_hci(&rx[..len])
    }

    /// Write a complete HCI packet from the tx buffer
    async fn write<T: HostToControllerPacket>(&self, val: &T) -> Result<(), Self::Error> {
        bt_hci::transport::WithIndicator::new(val)
            .write_hci_async(HciWriter)
            .await
    }
}

impl bt_hci_transport::Transport for BleConnector<'_> {
    /// Read a complete HCI packet into the rx buffer
    async fn read<'a, P: PacketToHost<'a>>(&self, rx: &'a mut [u8]) -> Result<P, Self::Error> {
        // `P::read_hci` deserializes from a reader into `rx`, so the packet must be read from a
        // buffer other than `rx`. The queued packet itself is that buffer.
        let packet = next_packet().await;

        let mut reader = &packet[..];
        let kind = PacketKind::read(&mut reader)?;
        Ok(P::read_hci(kind, &mut reader, rx)?)
    }

    /// Write a complete HCI packet from the tx buffer
    async fn write<P: PacketToController>(&self, tx: &P) -> Result<(), Self::Error> {
        bt_hci_transport::WithIndicator::new(tx)
            .write_hci_async(HciWriter)
            .await
    }
}
