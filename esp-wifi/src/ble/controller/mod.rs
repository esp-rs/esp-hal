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
