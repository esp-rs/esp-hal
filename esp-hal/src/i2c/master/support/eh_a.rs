use embedded_hal::i2c::Operation;

use crate::{
    i2c::master::{I2c, Instance},
    Async,
};

impl<T> embedded_hal_async::i2c::I2c for I2c<'_, Async, T>
where
    T: Instance,
{
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_impl_async(address, operations.iter_mut().map(Into::into))
            .await
    }
}
