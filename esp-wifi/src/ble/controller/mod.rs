use embedded_io::{
    blocking::{Read, Write},
    Error, Io,
};
use esp_hal_common::peripheral::{Peripheral, PeripheralRef};

use super::{read_hci, send_hci};

pub struct BleConnector<'d> {
    _device: PeripheralRef<'d, esp_hal_common::radio::Bluetooth>,
}

impl<'d> BleConnector<'d> {
    pub fn new(
        device: impl Peripheral<P = esp_hal_common::radio::Bluetooth> + 'd,
    ) -> BleConnector<'d> {
        Self {
            _device: device.into_ref(),
        }
    }
}

#[derive(Debug)]
pub enum BleConnectorError {
    Unknown,
}

impl Error for BleConnectorError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl Io for BleConnector<'_> {
    type Error = BleConnectorError;
}

impl Read for BleConnector<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let mut total = 0;
        for b in buf {
            let mut buffer = [0u8];
            let len = read_hci(&mut buffer);

            if len == 1 {
                *b = buffer[0];
                total += 1;
            } else {
                return Ok(total);
            }
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

#[cfg(feature = "async")]
pub mod asynch {
    use core::task::Poll;

    use crate::ble::ble::have_hci_read_data;

    use super::BleConnectorError;
    use super::{read_hci, send_hci};
    use embassy_sync::waitqueue::AtomicWaker;
    use embedded_io::asynch;
    use embedded_io::Io;
    use esp_hal_common::peripheral::{Peripheral, PeripheralRef};

    static HCI_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) fn hci_read_data_available() {
        HCI_WAKER.wake();
    }

    pub struct BleConnector<'d> {
        _device: PeripheralRef<'d, esp_hal_common::radio::Bluetooth>,
    }

    impl<'d> BleConnector<'d> {
        pub fn new(
            device: impl Peripheral<P = esp_hal_common::radio::Bluetooth> + 'd,
        ) -> BleConnector<'d> {
            Self {
                _device: device.into_ref(),
            }
        }
    }

    impl Io for BleConnector<'_> {
        type Error = BleConnectorError;
    }

    impl asynch::Read for BleConnector<'_> {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
            if !have_hci_read_data() {
                HciReadyEventFuture.await;
            }

            let mut total = 0;
            for b in buf {
                let mut buffer = [0u8];
                let len = read_hci(&mut buffer);

                if len == 1 {
                    *b = buffer[0];
                    total += 1;
                } else {
                    return Ok(total);
                }
            }

            Ok(total)
        }
    }

    impl asynch::Write for BleConnector<'_> {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, BleConnectorError> {
            send_hci(buf);
            Ok(buf.len())
        }

        async fn flush(&mut self) -> Result<(), BleConnectorError> {
            // nothing to do
            Ok(())
        }
    }

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
}
