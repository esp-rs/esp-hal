use embedded_io::{Error, ErrorType, Read, Write};

use super::{read_hci, read_next, send_hci};
use crate::{
    hal::peripheral::{Peripheral, PeripheralRef},
    EspWifiInitialization,
};

/// A blocking HCI connector
pub struct BleConnector<'d> {
    _device: PeripheralRef<'d, crate::hal::peripherals::BT>,
}

impl<'d> BleConnector<'d> {
    pub fn new(
        init: &EspWifiInitialization,
        device: impl Peripheral<P = crate::hal::peripherals::BT> + 'd,
    ) -> BleConnector<'d> {
        if !init.is_ble() {
            panic!("Not initialized for BLE use");
        }

        Self {
            _device: device.into_ref(),
        }
    }

    pub fn get_next(&mut self, buf: &mut [u8]) -> Result<usize, BleConnectorError> {
        Ok(read_next(buf))
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

impl ErrorType for BleConnector<'_> {
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

/// Async Interface
#[cfg(feature = "async")]
pub mod asynch {
    use core::task::Poll;

    use bt_hci::{
        transport::{Transport, WithIndicator},
        ControllerToHostPacket,
        FromHciBytes,
        HostToControllerPacket,
        WriteHci,
    };
    use embassy_sync::waitqueue::AtomicWaker;
    use embedded_io::ErrorType;

    use super::{read_hci, send_hci, BleConnectorError};
    use crate::{
        ble::ble::have_hci_read_data,
        hal::peripheral::{Peripheral, PeripheralRef},
        EspWifiInitialization,
    };

    static HCI_WAKER: AtomicWaker = AtomicWaker::new();

    pub(crate) fn hci_read_data_available() {
        HCI_WAKER.wake();
    }

    /// Async HCI connector
    pub struct BleConnector<'d> {
        _device: PeripheralRef<'d, crate::hal::peripherals::BT>,
    }

    impl<'d> BleConnector<'d> {
        pub fn new(
            init: &EspWifiInitialization,
            device: impl Peripheral<P = crate::hal::peripherals::BT> + 'd,
        ) -> BleConnector<'d> {
            if !init.is_ble() {
                panic!("Not initialized for BLE use");
            }

            Self {
                _device: device.into_ref(),
            }
        }
    }

    impl ErrorType for BleConnector<'_> {
        type Error = BleConnectorError;
    }

    impl embedded_io_async::Read for BleConnector<'_> {
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

    fn parse_hci<'m>(
        data: &'m [u8],
    ) -> Result<Option<ControllerToHostPacket<'m>>, BleConnectorError> {
        match ControllerToHostPacket::from_hci_bytes_complete(data) {
            Ok(p) => Ok(Some(p)),
            Err(e) => {
                if e == bt_hci::FromHciBytesError::InvalidSize {
                    use bt_hci::{event::EventPacketHeader, PacketKind};

                    // Some controllers emit a suprious command complete event at startup.
                    let (kind, data) =
                        PacketKind::from_hci_bytes(data).map_err(|_| BleConnectorError::Unknown)?;
                    if kind == PacketKind::Event {
                        let (header, _) = EventPacketHeader::from_hci_bytes(data)
                            .map_err(|_| BleConnectorError::Unknown)?;
                        const COMMAND_COMPLETE: u8 = 0x0E;
                        if header.code == COMMAND_COMPLETE && header.params_len < 4 {
                            return Ok(None);
                        }
                    }
                }
                warn!("[hci] error parsing packet: {:?}", e);
                Err(BleConnectorError::Unknown)
            }
        }
    }

    impl<'d> Transport for BleConnector<'d> {
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
                let rx = unsafe { &mut *core::ptr::slice_from_raw_parts_mut(rx.as_mut_ptr(), rx.len()) };

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
