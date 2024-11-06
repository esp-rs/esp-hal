use crate::{
    i2c::master::{Error, I2c, Instance, Operation},
    Async,
    Mode,
};

impl<'a, 'b> From<&'a mut embedded_hal::i2c::Operation<'b>> for Operation<'a> {
    fn from(value: &'a mut embedded_hal::i2c::Operation<'b>) -> Self {
        match value {
            embedded_hal::i2c::Operation::Write(buffer) => Operation::Write(buffer),
            embedded_hal::i2c::Operation::Read(buffer) => Operation::Read(buffer),
        }
    }
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::{ErrorKind, NoAcknowledgeSource};

        match self {
            Self::ExceedingFifo => ErrorKind::Overrun,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::AckCheckFailed => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
            _ => ErrorKind::Other,
        }
    }
}

impl<T, DM: Mode> embedded_hal::i2c::ErrorType for I2c<'_, DM, T> {
    type Error = Error;
}

impl<T, DM: Mode> embedded_hal::i2c::I2c for I2c<'_, DM, T>
where
    T: Instance,
{
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl(address, operations.iter_mut().map(Into::into))
    }
}

impl<T> embedded_hal_async::i2c::I2c for I2c<'_, Async, T>
where
    T: Instance,
{
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl_async(address, operations.iter_mut().map(Into::into))
            .await
    }
}
