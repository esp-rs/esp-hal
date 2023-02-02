use embedded_io::{
    blocking::{Read, Write},
    Error, Io,
};

use super::{read_hci, send_hci};

pub struct BleConnector {}

#[derive(Debug)]
pub enum BleConnectorError {
    Unknown,
}

impl Error for BleConnectorError {
    fn kind(&self) -> embedded_io::ErrorKind {
        embedded_io::ErrorKind::Other
    }
}

impl Io for BleConnector {
    type Error = BleConnectorError;
}

impl Read for BleConnector {
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

impl Write for BleConnector {
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
