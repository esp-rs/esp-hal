use embedded_io::{Error, ErrorType, Read, Write};
use esp_phy::PhyInitGuard;

use super::{read_hci, read_next, send_hci};
use crate::{Controller, ble::Config};

/// A blocking HCI connector
#[instability::unstable]
pub struct BleConnector<'d> {
    _phy_init_guard: PhyInitGuard<'d>,
    _device: crate::hal::peripherals::BT<'d>,
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
        _init: &'d Controller<'d>,
        device: crate::hal::peripherals::BT<'d>,
        config: Config,
    ) -> BleConnector<'d> {
        Self {
            _phy_init_guard: crate::ble::ble_init(&config),
            _device: device,
        }
    }

    /// Read the next HCI packet from the BLE controller.
    #[instability::unstable]
    pub fn next(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
        Ok(read_next(buf))
    }
}

#[derive(Debug)]
/// Error type for the BLE connector.
#[instability::unstable]
pub enum BleConnectorError {
    Unknown,
}

impl Error for BleConnectorError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl core::error::Error for BleConnectorError {}

impl core::fmt::Display for BleConnectorError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BleConnectorError::Unknown => write!(f, "Unknown BLE error occured"),
        }
    }
}

impl ErrorType for BleConnector<'_> {
    type Error = BleConnectorError;
}

impl Read for BleConnector<'_> {
    fn read(&mut self, mut buf: &mut [u8]) -> Result<usize, Self::Error> {
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
}

impl Write for BleConnector<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        for b in buf {
            send_hci(&[*b]);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // nothing to do
        Ok(())
    }
}

/// Async Interface
pub(crate) mod asynch {
    use core::task::Poll;

    use bt_hci::{
        ControllerToHostPacket,
        FromHciBytes,
        FromHciBytesError,
        HostToControllerPacket,
        WriteHci,
        transport::{Transport, WithIndicator},
    };
    use esp_hal::asynch::AtomicWaker;

    use super::*;
    use crate::ble::have_hci_read_data;

    static HCI_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) fn hci_read_data_available() {
        HCI_WAKER.wake();
    }

    impl embedded_io_async::Read for BleConnector<'_> {
        async fn read(&mut self, mut buf: &mut [u8]) -> Result<usize, BleConnectorError> {
            if buf.is_empty() {
                return Ok(0);
            }

            let mut total = 0;
            if !have_hci_read_data() {
                HciReadyEventFuture.await;
            }
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
    }

    impl embedded_io_async::Write for BleConnector<'_> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
            send_hci(buf);
            Ok(buf.len())
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

    fn parse_hci(data: &[u8]) -> Result<Option<ControllerToHostPacket<'_>>, BleConnectorError> {
        match ControllerToHostPacket::from_hci_bytes_complete(data) {
            Ok(p) => Ok(Some(p)),
            Err(e) => {
                warn!("[hci] error parsing packet: {:?}", e);
                Err(BleConnectorError::Unknown)
            }
        }
    }

    impl Transport for BleConnector<'_> {
        /// Read a complete HCI packet into the rx buffer
        async fn read<'a>(
            &self,
            rx: &'a mut [u8],
        ) -> Result<ControllerToHostPacket<'a>, Self::Error> {
            loop {
                if !have_hci_read_data() {
                    HciReadyEventFuture.await;
                }

                // Workaround for borrow checker.
                // Safety: we only return a reference to x once, if parsing is successful.
                let rx =
                    unsafe { &mut *core::ptr::slice_from_raw_parts_mut(rx.as_mut_ptr(), rx.len()) };

                let len = crate::ble::read_next(rx);
                if let Some(packet) = parse_hci(&rx[..len])? {
                    return Ok(packet);
                }
            }
        }

        /// Write a complete HCI packet from the tx buffer
        async fn write<T: HostToControllerPacket>(&self, val: &T) -> Result<(), Self::Error> {
            let mut buf: [u8; 259] = [0; 259];
            let w = WithIndicator::new(val);
            let len = w.size();
            w.write_hci(&mut buf[..])
                .map_err(|_| BleConnectorError::Unknown)?;
            send_hci(&buf[..len]);
            Ok(())
        }
    }
}
