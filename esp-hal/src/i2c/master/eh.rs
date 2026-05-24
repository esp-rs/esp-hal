use embedded_hal::i2c::Operation as EhalOperation;

use super::{Async, DriverMode, Error, I2c, I2cAddress, Operation};

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        use embedded_hal::i2c::ErrorKind;

        match self {
            Self::FifoExceeded => ErrorKind::Overrun,
            Self::ArbitrationLost => ErrorKind::ArbitrationLoss,
            Self::AcknowledgeCheckFailed(reason) => ErrorKind::NoAcknowledge(reason.into()),
            _ => ErrorKind::Other,
        }
    }
}

#[instability::unstable]
impl<Dm: DriverMode> embassy_embedded_hal::SetConfig for I2c<'_, Dm> {
    type Config = super::Config;
    type ConfigError = super::ConfigError;

    fn set_config(&mut self, config: &Self::Config) -> Result<(), Self::ConfigError> {
        self.apply_config(config)
    }
}

impl<Dm: DriverMode> embedded_hal::i2c::ErrorType for I2c<'_, Dm> {
    type Error = Error;
}

impl<Dm: DriverMode> embedded_hal::i2c::I2c for I2c<'_, Dm> {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.driver()
            .transaction_impl(
                I2cAddress::SevenBit(address),
                operations.iter_mut().map(Operation::from),
            )
            .inspect_err(|error| self.internal_recover(error))
    }
}

impl embedded_hal_async::i2c::I2c for I2c<'_, Async> {
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [EhalOperation<'_>],
    ) -> Result<(), Self::Error> {
        self.driver()
            .transaction_impl_async(address.into(), operations.iter_mut().map(Operation::from))
            .await
            .inspect_err(|error| self.internal_recover(error))
    }
}
