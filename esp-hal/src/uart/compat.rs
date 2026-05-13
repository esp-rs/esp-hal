#[cfg(feature = "unstable")]
use super::{Async, Config, ConfigError, IoError, RxError, TxError, UartRx};
use super::{Blocking, DriverMode, Uart, UartTx};
#[cfg(feature = "unstable")]
use crate::interrupt::InterruptHandler;

#[instability::unstable]
impl embedded_io_06::Error for RxError {
    fn kind(&self) -> embedded_io_06::ErrorKind {
        embedded_io_06::ErrorKind::Other
    }
}

#[instability::unstable]
impl embedded_io_07::Error for RxError {
    fn kind(&self) -> embedded_io_07::ErrorKind {
        embedded_io_07::ErrorKind::Other
    }
}

#[instability::unstable]
impl embedded_io_06::Error for TxError {
    fn kind(&self) -> embedded_io_06::ErrorKind {
        embedded_io_06::ErrorKind::Other
    }
}
#[instability::unstable]
impl embedded_io_07::Error for TxError {
    fn kind(&self) -> embedded_io_07::ErrorKind {
        embedded_io_07::ErrorKind::Other
    }
}

#[instability::unstable]
impl<Dm> embassy_embedded_hal::SetConfig for Uart<'_, Dm>
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
impl<Dm> embassy_embedded_hal::SetConfig for UartRx<'_, Dm>
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
impl<Dm> embassy_embedded_hal::SetConfig for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Config = Config;
    type ConfigError = ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.baudrate = config.baudrate;
        self.apply_config(config)
    }
}

impl crate::private::Sealed for Uart<'_, Blocking> {}

#[instability::unstable]
impl crate::interrupt::InterruptConfigurable for Uart<'_, Blocking> {
    fn set_interrupt_handler(&mut self, handler: InterruptHandler) {
        // `self.tx.uart` and `self.rx.uart` are the same
        self.tx.uart.set_interrupt_handler(handler);
    }
}

#[instability::unstable]
impl<Dm> ufmt_write::uWrite for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = TxError;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.tx.write_str(s)
    }

    #[inline]
    fn write_char(&mut self, ch: char) -> Result<(), Self::Error> {
        self.tx.write_char(ch)
    }
}

#[instability::unstable]
impl<Dm> ufmt_write::uWrite for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    type Error = TxError;

    #[inline]
    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.write_all(s.as_bytes())
    }
}

impl<Dm> core::fmt::Write for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

impl<Dm> core::fmt::Write for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    #[inline]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_all(s.as_bytes()).map_err(|_| core::fmt::Error)
    }
}

#[instability::unstable]
impl embedded_io_06::Error for IoError {
    fn kind(&self) -> embedded_io_06::ErrorKind {
        embedded_io_06::ErrorKind::Other
    }
}

#[instability::unstable]
impl embedded_io_07::Error for IoError {
    fn kind(&self) -> embedded_io_07::ErrorKind {
        embedded_io_07::ErrorKind::Other
    }
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io_06::ErrorType for Uart<'_, Dm> {
    type Error = IoError;
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io_06::ErrorType for UartTx<'_, Dm> {
    type Error = TxError;
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io_06::ErrorType for UartRx<'_, Dm> {
    type Error = RxError;
}

#[instability::unstable]
impl<Dm> embedded_io_06::Read for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).map_err(IoError::Rx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Read for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::ReadReady for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx.read_ready())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::ReadReady for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UartRx::read_ready(self))
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Write for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf).map_err(IoError::Tx)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush().map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::Write for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush()
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::WriteReady for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UartTx::write_ready(self))
    }
}

#[instability::unstable]
impl<Dm> embedded_io_06::WriteReady for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.tx.write_ready())
    }
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io_07::ErrorType for Uart<'_, Dm> {
    type Error = IoError;
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io_07::ErrorType for UartTx<'_, Dm> {
    type Error = TxError;
}

#[instability::unstable]
impl<Dm: DriverMode> embedded_io_07::ErrorType for UartRx<'_, Dm> {
    type Error = RxError;
}

#[instability::unstable]
impl<Dm> embedded_io_07::Read for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx.read(buf).map_err(IoError::Rx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Read for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read(buf)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::ReadReady for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx.read_ready())
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::ReadReady for UartRx<'_, Dm>
where
    Dm: DriverMode,
{
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UartRx::read_ready(self))
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Write for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.tx.write(buf).map_err(IoError::Tx)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.tx.flush().map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::Write for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write(buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush()
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::WriteReady for UartTx<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(UartTx::write_ready(self))
    }
}

#[instability::unstable]
impl<Dm> embedded_io_07::WriteReady for Uart<'_, Dm>
where
    Dm: DriverMode,
{
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.tx.write_ready())
    }
}

#[instability::unstable]
impl embedded_io_async_06::Read for Uart<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await.map_err(IoError::Rx)
    }

    async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), embedded_io_06::ReadExactError<Self::Error>> {
        self.read_exact_async(buf)
            .await
            .map_err(|e| embedded_io_06::ReadExactError::Other(IoError::Rx(e)))
    }
}

#[instability::unstable]
impl embedded_io_async_06::Read for UartRx<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }

    async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), embedded_io_06::ReadExactError<Self::Error>> {
        self.read_exact_async(buf)
            .await
            .map_err(embedded_io_06::ReadExactError::Other)
    }
}

#[instability::unstable]
impl embedded_io_async_06::Write for Uart<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await.map_err(IoError::Tx)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await.map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl embedded_io_async_06::Write for UartTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
    }
}

#[instability::unstable]
impl embedded_io_async_07::Read for Uart<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await.map_err(IoError::Rx)
    }

    async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), embedded_io_07::ReadExactError<Self::Error>> {
        self.read_exact_async(buf)
            .await
            .map_err(|e| embedded_io_07::ReadExactError::Other(IoError::Rx(e)))
    }
}

#[instability::unstable]
impl embedded_io_async_07::Read for UartRx<'_, Async> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.read_async(buf).await
    }

    async fn read_exact(
        &mut self,
        buf: &mut [u8],
    ) -> Result<(), embedded_io_07::ReadExactError<Self::Error>> {
        self.read_exact_async(buf)
            .await
            .map_err(embedded_io_07::ReadExactError::Other)
    }
}

#[instability::unstable]
impl embedded_io_async_07::Write for Uart<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await.map_err(IoError::Tx)
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await.map_err(IoError::Tx)
    }
}

#[instability::unstable]
impl embedded_io_async_07::Write for UartTx<'_, Async> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.write_async(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.flush_async().await
    }
}
