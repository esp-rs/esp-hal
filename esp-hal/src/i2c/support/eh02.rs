use crate::{
    i2c::{Error, I2c, Instance},
    Blocking,
};

impl<T> embedded_hal_02::blocking::i2c::Read for I2c<'_, Blocking, T>
where
    T: Instance,
{
    type Error = Error;

    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.read(address, buffer)
    }
}

impl<T> embedded_hal_02::blocking::i2c::Write for I2c<'_, Blocking, T>
where
    T: Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write(addr, bytes)
    }
}

impl<T> embedded_hal_02::blocking::i2c::WriteRead for I2c<'_, Blocking, T>
where
    T: Instance,
{
    type Error = Error;

    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.write_read(address, bytes, buffer)
    }
}
