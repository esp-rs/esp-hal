//! Pulse Counter peripheral driver

use self::unit::Unit;
use crate::{
    peripheral::{Peripheral, PeripheralRef},
    system::PeripheralClockControl,
};

pub mod channel;
pub mod unit;

pub struct PCNT<'d> {
    _instance: PeripheralRef<'d, crate::peripherals::PCNT>,
}

impl<'d> PCNT<'d> {
    /// Return a new PCNT
    pub fn new(
        _instance: impl Peripheral<P = crate::peripherals::PCNT> + 'd,
        peripheral_clock_control: &mut PeripheralClockControl,
    ) -> Self {
        crate::into_ref!(_instance);
        // Enable the PCNT peripherals clock in the system peripheral
        peripheral_clock_control.enable(crate::system::Peripheral::Pcnt);
        PCNT { _instance }
    }

    /// Return a unit
    pub fn get_unit(&self, number: unit::Number) -> Unit {
        Unit::new(number)
    }
}
