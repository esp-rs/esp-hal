//! BLE controller
use core::{future::ready, task::Poll};

use bt_hci_driver::{PacketToController, PacketToHost, ReadHciError, Transport};
use docsplay::Display;
use esp_phy::PhyInitGuard;

use crate::{
    RadioRefGuard,
    asynch::AtomicWaker,
    ble::{
        BT_STATE,
        Config,
        InvalidConfigError,
        have_hci_read_data,
        read_hci,
        read_next,
        send_hci,
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

        if !have_hci_read_data() {
            HciReadyEventFuture.await;
        }

        self.read(buf)
    }

    /// Write to HCI.
    #[instability::unstable]
    pub fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
        for b in buf {
            send_hci(&[*b]);
        }
        Ok(buf.len())
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
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
        self.read_async(buf).await
    }
}

impl embedded_io_async_06::Write for BleConnector<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
        send_hci(buf);
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), BleConnectorError> {
        // nothing to do
        Ok(())
    }
}

impl embedded_io_async_07::Read for BleConnector<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
        self.read_async(buf).await
    }
}

impl embedded_io_async_07::Write for BleConnector<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
        send_hci(buf);
        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), BleConnectorError> {
        // nothing to do
        Ok(())
    }
}

impl<E: embedded_io_07::Error> From<ReadHciError<E>> for BleConnectorError {
    fn from(_e: ReadHciError<E>) -> Self {
        BleConnectorError::Unknown
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub(crate) struct HciReadyEventFuture;

impl core::future::Future for HciReadyEventFuture {
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

impl Transport for BleConnector<'_> {
    /// Read a complete HCI packet into the rx buffer
    async fn read<'a, P: PacketToHost<'a>>(&self, buf: &'a mut [u8]) -> Result<P, Self::Error> {
        if !have_hci_read_data() {
            HciReadyEventFuture.await;
        }

        if let Some(packet) = BT_STATE.with(|state| state.rx_queue.pop_front()) {
            let mut reader = &packet.data[..];
            let packet = P::read_hci(&mut reader, buf).map_err(|e| {
                warn!("[hci] error parsing packet: {:?}", e);
                BleConnectorError::Unknown
            })?;
            if !reader.is_empty() {
                warn!("[hci] packet too long: {} bytes", reader.len());
                return Err(BleConnectorError::Unknown);
            }
            Ok(packet)
        } else {
            unreachable!()
        }
    }

    /// Write a complete HCI packet from the tx buffer
    fn write<T: PacketToController>(
        &self,
        val: &T,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        const MAX_HCI_PACKET_SIZE: usize = 259;
        let mut buf: [u8; MAX_HCI_PACKET_SIZE] = [0; MAX_HCI_PACKET_SIZE];
        let mut writer = &mut buf[..];
        let result = val
            .write_hci(&mut writer)
            .map_err(|_| BleConnectorError::Unknown);

        let len = MAX_HCI_PACKET_SIZE - writer.len();
        // TODO: send_hci should be async
        send_hci(&buf[..len]);

        ready(result)
    }
}
