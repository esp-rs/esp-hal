use embedded_hal::digital;
use embedded_hal_async::digital::Wait;

#[cfg(feature = "unstable")]
use super::Flex;
use super::{Input, Output};

impl digital::ErrorType for Input<'_> {
    type Error = core::convert::Infallible;
}

impl digital::InputPin for Input<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_low(self))
    }
}

impl digital::ErrorType for Output<'_> {
    type Error = core::convert::Infallible;
}

impl digital::OutputPin for Output<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Self::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Self::set_high(self);
        Ok(())
    }
}

impl digital::StatefulOutputPin for Output<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_low(self))
    }
}

#[instability::unstable]
impl digital::InputPin for Flex<'_> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_high(self))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_low(self))
    }
}

#[instability::unstable]
impl digital::ErrorType for Flex<'_> {
    type Error = core::convert::Infallible;
}

#[instability::unstable]
impl digital::OutputPin for Flex<'_> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Self::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Self::set_high(self);
        Ok(())
    }
}

#[instability::unstable]
impl digital::StatefulOutputPin for super::Flex<'_> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_high(self))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(Self::is_set_low(self))
    }
}

#[instability::unstable]
impl Wait for Flex<'_> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_high(self).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_low(self).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_rising_edge(self).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_falling_edge(self).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_any_edge(self).await;
        Ok(())
    }
}

impl Wait for Input<'_> {
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_high(self).await;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_low(self).await;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_rising_edge(self).await;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_falling_edge(self).await;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        Self::wait_for_any_edge(self).await;
        Ok(())
    }
}
