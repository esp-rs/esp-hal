use super::{Input, Level, Output, OutputOpenDrain};

impl<const PIN: u8> embedded_hal::digital::ErrorType for Input<PIN> {
    type Error = core::convert::Infallible;
}

impl<const PIN: u8> embedded_hal::digital::ErrorType for Output<PIN> {
    type Error = core::convert::Infallible;
}

impl<const PIN: u8> embedded_hal::digital::ErrorType for OutputOpenDrain<PIN> {
    type Error = core::convert::Infallible;
}

impl<const PIN: u8> embedded_hal::digital::InputPin for Input<PIN> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::High)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::Low)
    }
}

impl<const PIN: u8> embedded_hal::digital::OutputPin for Output<PIN> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High);
        Ok(())
    }
}

impl<const PIN: u8> embedded_hal::digital::StatefulOutputPin for Output<PIN> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::High)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::Low)
    }
}

impl<const PIN: u8> embedded_hal::digital::InputPin for OutputOpenDrain<PIN> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::High)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.level() == Level::Low)
    }
}

impl<const PIN: u8> embedded_hal::digital::OutputPin for OutputOpenDrain<PIN> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::Low);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_level(Level::High);
        Ok(())
    }
}

impl<const PIN: u8> embedded_hal::digital::StatefulOutputPin for OutputOpenDrain<PIN> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::High)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self.output_level() == Level::Low)
    }
}
