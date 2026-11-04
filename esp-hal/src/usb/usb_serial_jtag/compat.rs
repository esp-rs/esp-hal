use super::{
    Config,
    ConfigError,
    IoError,
    RxError,
    TxError,
    UsbSerialJtag,
    UsbSerialJtagRx,
    UsbSerialJtagTx,
};
use crate::{Async, Blocking, DriverMode};

impl crate::private::Sealed for UsbSerialJtag<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for UsbSerialJtag<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: crate::interrupt::InterruptHandler) {
        self.set_interrupt_handler(handler);
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

#[instability::unstable]
impl embedded_io_06::Error for RxError {
    fn kind(&self) -> embedded_io_06::ErrorKind {
        match self {
            RxError::Timeout => embedded_io_06::ErrorKind::TimedOut,
        }
    }
}

#[instability::unstable]
impl embedded_io_07::Error for RxError {
    fn kind(&self) -> embedded_io_07::ErrorKind {
        match self {
            RxError::Timeout => embedded_io_07::ErrorKind::TimedOut,
        }
    }
}

#[instability::unstable]
impl embedded_io_06::Error for IoError {
    fn kind(&self) -> embedded_io_06::ErrorKind {
        match self {
            IoError::Tx(e) => embedded_io_06::Error::kind(e),
            IoError::Rx(e) => embedded_io_06::Error::kind(e),
        }
    }
}

#[instability::unstable]
impl embedded_io_07::Error for IoError {
    fn kind(&self) -> embedded_io_07::ErrorKind {
        match self {
            IoError::Tx(e) => embedded_io_07::Error::kind(e),
            IoError::Rx(e) => embedded_io_07::Error::kind(e),
        }
    }
}

#[instability::unstable]
impl embedded_io_06::Error for TxError {
    fn kind(&self) -> embedded_io_06::ErrorKind {
        match self {
            TxError::Timeout => embedded_io_06::ErrorKind::TimedOut,
        }
    }
}

#[instability::unstable]
impl embedded_io_07::Error for TxError {
    fn kind(&self) -> embedded_io_07::ErrorKind {
        match self {
            TxError::Timeout => embedded_io_07::ErrorKind::TimedOut,
        }
    }
}

#[instability::unstable]
impl<Dm> core::fmt::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        core::fmt::Write::write_str(&mut self.tx, s)
    }
}

#[instability::unstable]
impl<Dm> core::fmt::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_all(s.as_bytes()).map_err(|_| core::fmt::Error)?;
        Ok(())
    }
}

#[instability::unstable]
impl<Dm> ufmt_write::uWrite for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = TxError;

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
    type Error = TxError;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_all(s.as_bytes())?;
        Ok(())
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        let mut buffer = [0u8; 4];
        self.write_all(ch.encode_utf8(&mut buffer).as_bytes())?;

        Ok(())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::ErrorType for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = IoError;
}

#[instability::unstable]
impl<Dm> embedded_io_06::ErrorType for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = TxError;
}

#[instability::unstable]
impl<Dm> embedded_io_06::ErrorType for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = RxError;
}

#[instability::unstable]
impl<Dm> embedded_io_06::Read for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_06::Read::read(&mut self.rx, buf).map_err(IoError::Rx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Read for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        UsbSerialJtagRx::read(self, buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_06::Write::write(&mut self.tx, buf).map_err(IoError::Tx)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_06::Write::flush(&mut self.tx).map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        UsbSerialJtagTx::write(self, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        UsbSerialJtagTx::flush(self)
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
    type Error = IoError;
}

#[instability::unstable]
impl<Dm> embedded_io_07::ErrorType for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = TxError;
}

#[instability::unstable]
impl<Dm> embedded_io_07::ErrorType for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = RxError;
}

#[instability::unstable]
impl<Dm> embedded_io_07::Read for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_07::Read::read(&mut self.rx, buf).map_err(IoError::Rx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Read for UsbSerialJtagRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        UsbSerialJtagRx::read(self, buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Write for UsbSerialJtag<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io_07::Write::write(&mut self.tx, buf).map_err(IoError::Tx)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_07::Write::flush(&mut self.tx).map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Write for UsbSerialJtagTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        UsbSerialJtagTx::write(self, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        UsbSerialJtagTx::flush(self)
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
        embedded_io_async_06::Write::write(&mut self.tx, buf)
            .await
            .map_err(IoError::Tx)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_async_06::Write::flush(&mut self.tx)
            .await
            .map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl embedded_io_async_06::Write for UsbSerialJtagTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
    }
}

#[instability::unstable]
impl embedded_io_async_06::Read for UsbSerialJtag<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_async_06::Read::read(&mut self.rx, buf)
            .await
            .map_err(IoError::Rx)
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
        embedded_io_async_07::Write::write(&mut self.tx, buf)
            .await
            .map_err(IoError::Tx)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io_async_07::Write::flush(&mut self.tx)
            .await
            .map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl embedded_io_async_07::Write for UsbSerialJtagTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
    }
}

#[instability::unstable]
impl embedded_io_async_07::Read for UsbSerialJtag<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io_async_07::Read::read(&mut self.rx, buf)
            .await
            .map_err(IoError::Rx)
    }
}

#[instability::unstable]
impl embedded_io_async_07::Read for UsbSerialJtagRx<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }
}
