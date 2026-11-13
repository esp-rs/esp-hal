use super::{Error, UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};
use crate::{Async, Blocking, DriverMode};

impl crate::private::Sealed for UsbSerialJtag<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for UsbSerialJtag<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.set_interrupt_handler(handler);
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
        self.write(s.as_bytes()).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

#[instability::unstable]
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

#[instability::unstable]
impl<Dm> ufmt_write::uWrite for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write(s.as_bytes())?;
        Ok(())
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        let mut buffer = [0u8; 4];
        self.write(ch.encode_utf8(&mut buffer).as_bytes())?;

        Ok(())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::ErrorType for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io_06::ErrorType for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io_06::ErrorType for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io_06::Read for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_06::Read::read(&mut self.rx, buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Read for UsbSerialJtagRx<'_, Dm>
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

#[instability::unstable]
impl<Dm> embedded_io_06::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_06::Write::write(&mut self.tx, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_06::Write::flush(&mut self.tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_packet(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx()
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::ReadReady for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx.read_ready())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::ReadReady for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UsbSerialJtagRx::read_ready(self))
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::WriteReady for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.tx.write_ready())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::WriteReady for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UsbSerialJtagTx::write_ready(self))
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::ErrorType for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io_07::ErrorType for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io_07::ErrorType for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = Error;
}

#[instability::unstable]
impl<Dm> embedded_io_07::Read for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_07::Read::read(&mut self.rx, buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Read for UsbSerialJtagRx<'_, Dm>
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

#[instability::unstable]
impl<Dm> embedded_io_07::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_07::Write::write(&mut self.tx, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_07::Write::flush(&mut self.tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_packet(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx()
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::ReadReady for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx.read_ready())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::ReadReady for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UsbSerialJtagRx::read_ready(self))
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::WriteReady for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.tx.write_ready())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::WriteReady for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UsbSerialJtagTx::write_ready(self))
    }
}

#[instability::unstable]
impl embedded_io_async_06::Write for UsbSerialJtag<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_async_06::Write::write(&mut self.tx, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_async_06::Write::flush(&mut self.tx).await
    }
}

#[instability::unstable]
impl embedded_io_async_06::Write for UsbSerialJtagTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await?;

        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx_async().await
    }
}

#[instability::unstable]
impl embedded_io_async_06::Read for UsbSerialJtag<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_async_06::Read::read(&mut self.rx, buf).await
    }
}

#[instability::unstable]
impl embedded_io_async_06::Read for UsbSerialJtagRx<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }
}

#[instability::unstable]
impl embedded_io_async_07::Write for UsbSerialJtag<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_async_07::Write::write(&mut self.tx, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_async_07::Write::flush(&mut self.tx).await
    }
}

#[instability::unstable]
impl embedded_io_async_07::Write for UsbSerialJtagTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await?;

        Ok(buf.len())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_tx_async().await
    }
}

#[instability::unstable]
impl embedded_io_async_07::Read for UsbSerialJtag<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_async_07::Read::read(&mut self.rx, buf).await
    }
}

#[instability::unstable]
impl embedded_io_async_07::Read for UsbSerialJtagRx<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }
}
